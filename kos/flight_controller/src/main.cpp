/**
 * \file
 * \~English \brief Implementation of the security module FlightController component main loop.
 * \~Russian \brief Реализация основного цикла компонента FlightController модуля безопасности.
 */

#include "../include/flight_controller.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_navigation_system.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"
#include "../../shared/include/ipc_messages_logger.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <thread>
#include <atomic>
#include <cmath>
#include <chrono>


/** \cond */
#define RETRY_DELAY_SEC 1
#define RETRY_REQUEST_DELAY_SEC 5
#define FLY_ACCEPT_PERIOD_US 500000

constexpr double REACH_RADIUS_M     = 5.0;   // считаем точку достигнутой
constexpr double BASE_MARGIN_M      = 2.0;   // минимальный зазор для "ближайшая не та"
constexpr double REL_MARGIN_K       = 0.20;  // доля от dcur для динамического порога
constexpr double TREND_STEP_MIN_M   = 0.2;   // минимальный шаг роста dcur, считаем "удалением"
constexpr int    TREND_CONFIRM_N    = 6;     // подряд тиков "удаляемся"
constexpr int    HIJACK_CONFIRM_N   = 2;     // подряд тиков "ближайшая другая" для срабатывания
constexpr int    ESCALATE_AFTER_K   = 3;     // сколько коррекций подряд до эскалации
constexpr double STARTUP_MOVE_ARM_M         = 0; // сколько пройти по земле, чтобы "вооружить" детектор
constexpr int    NO_MISSION_COOLDOWN_MS     = 3000; // кулдаун после неудачного changeWaypoint()
constexpr double ALT_ARM_MIN_M        = 15.0;  // начнём анализ, когда выше этой высоты
//constexpr int    LOCK_WINDOW_TICKS    = 10;    // ≈ 3 с при периоде 300 мс
constexpr double LOCK_MIN_GAIN_M      = 3.0;   // минимум убывания дистанции, чтобы считать "идём на этот WP"
constexpr int    GUARD_CONFIRM_TICKS  = 3;    
constexpr double NEAR_LOCK_DIST_M      = 30.0;   // прямой лок по близости
constexpr double LOCK_STEP_MIN_M       = 0.2;    // минимальный «шаг» убыли между тиками
constexpr int    LOCK_DECR_TICKS       = 6;      // минимум тиков с убывшей дистанцией
constexpr int    LOCK_WINDOW_TICKS     = 12;     // ~3.6 c при 300 мс
constexpr double LOCK_MIN_GAIN_ABS_M   = 1.0;    // порог абсолютной убыли за окно
constexpr double LOCK_MIN_GAIN_REL_K   = 0.08;   // относительный порог: 8% от d0
constexpr int    LOCK_MAX_WINDOWS      = 3;      // столько окон подряд терпим «inconclusive»
constexpr int    LOCK_MAX_AHEAD        = 6;      // при fallback смотрим первые N точек
constexpr double LOCK_FALLBACK_DIST_M  = 80.0;   // и не дальше этого порога

char boardId[32] = {0};
uint32_t sessionDelay;
std::thread sessionThread, updateThread;
/** \endcond */
static std::atomic<bool> g_stopRouteGuard{false};

enum class RGMode { WAIT, LOCK, GUARD };
static RGMode g_mode = RGMode::WAIT;

struct LockBuf {
    std::vector<double> d0;      // дистанции в начале окна
    std::vector<double> prev;    // дистанции на прошлом тике
    std::vector<int>    decr;    // сколько раз уменьшилась за окно
    int ticks = 0;
    int windows = 0;
} g_lock;
struct RGState {
    bool   armed = false;           // детектор готов к работе
    double movedAccum = 0.0;        // накопленное горизонтальное перемещение, м
    int32_t lastLat = 0, lastLon = 0;
    bool   haveLast = false;
    int64_t cooldownUntilMs = 0;    // монотонное время (мс), до которого сторож спит
};
static RGState g_rg;
static int64_t monoMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

static double toRad(int32_t vdeg1e7) {
    return (static_cast<double>(vdeg1e7) / 1e7) * M_PI / 180.0;
}
static double haversineMeters(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
    constexpr double R = 6371000.0; // Earth radius [m]
    const double dLat = toRad(lat2 - lat1);
    const double dLon = toRad(lon2 - lon1);
    const double a = std::sin(dLat/2)*std::sin(dLat/2) +
                     std::cos(toRad(lat1))*std::cos(toRad(lat2))*
                     std::sin(dLon/2)*std::sin(dLon/2);
    const double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
    return R * c;
}
static size_t pickLockIndexByGain(const std::vector<double>& d0, const std::vector<double>& d1, double minGain) {
    size_t best = SIZE_MAX; double bestGain = 0.0;
    for (size_t i = 0; i < d0.size() && i < d1.size(); ++i) {
        double gain = d0[i] - d1[i]; // насколько дистанция уменьшилась
        if (gain > bestGain && gain >= minGain) { bestGain = gain; best = i; }
    }
    return best;
}
static size_t pickByGain(const std::vector<double>& d0,
                         const std::vector<double>& d1,
                         double absMin, double relK)
{
    size_t best = SIZE_MAX; double bestGain = 0.0;
    for (size_t i=0;i<d0.size() && i<d1.size();++i){
        double gain = d0[i] - d1[i];
        double need = std::max(absMin, relK * d0[i]);
        if (gain >= need && gain > bestGain) { bestGain = gain; best = i; }
    }
    return best;
}

static void routeGuardThreadFunc() {
    logEntry((char*)"RouteGuard: starting route monitoring", ENTITY_NAME, LogLevel::LOG_INFO);

    // snapshot original mission and build raw bytes
    int numCmds = 0;
    MissionCommand* cmds = getMissionCommands(numCmds);
    if (!cmds || numCmds <= 0) {
        logEntry((char*)"RouteGuard: no mission loaded", ENTITY_NAME, LogLevel::LOG_WARNING);
        return;
    }

    const uint32_t bytesSize = getMissionBytesSize(cmds, (uint8_t)numCmds);
    std::vector<uint8_t> missionBytes(bytesSize);
    if (!missionToBytes(cmds, (uint8_t)numCmds, missionBytes.data())) {
        logEntry((char*)"RouteGuard: missionToBytes failed", ENTITY_NAME, LogLevel::LOG_WARNING);
        return;
    }

    // collect target waypoints
    struct WP { int32_t lat, lon, alt; };
    std::vector<WP> targets;
    targets.reserve(numCmds);

    logEntry((char*)"RouteGuard: parsing mission waypoints:", ENTITY_NAME, LogLevel::LOG_INFO);
    for (int i=0;i<numCmds;i++) {
        if (cmds[i].type == CommandType::WAYPOINT) {
            targets.push_back({cmds[i].content.waypoint.latitude,
                               cmds[i].content.waypoint.longitude,
                               cmds[i].content.waypoint.altitude});
            char buf[128];
            snprintf(buf, sizeof(buf), "  WP #%d: lat=%d lon=%d alt=%d",
                     (int)targets.size()-1,
                     cmds[i].content.waypoint.latitude,
                     cmds[i].content.waypoint.longitude,
                     cmds[i].content.waypoint.altitude);
            logEntry(buf, ENTITY_NAME, LogLevel::LOG_INFO);
        } else if (cmds[i].type == CommandType::LAND) {
            if (cmds[i].content.waypoint.latitude != 0 || cmds[i].content.waypoint.longitude != 0) {
                targets.push_back({cmds[i].content.waypoint.latitude,
                                   cmds[i].content.waypoint.longitude,
                                   cmds[i].content.waypoint.altitude});
                char buf[128];
                snprintf(buf, sizeof(buf), "  LAND-WP #%d: lat=%d lon=%d alt=%d",
                         (int)targets.size()-1,
                         cmds[i].content.waypoint.latitude,
                         cmds[i].content.waypoint.longitude,
                         cmds[i].content.waypoint.altitude);
                logEntry(buf, ENTITY_NAME, LogLevel::LOG_INFO);
            }
        }
    }
    if (targets.empty()) {
        logEntry((char*)"RouteGuard: no WAYPOINTs found", ENTITY_NAME, LogLevel::LOG_WARNING);
        return;
    }

    size_t idx = 0;
    double prevDist = 1e18;
    int trendUp = 0;          // подряд тиков с ростом dcur
    int hijackTicks = 0;      // подряд тиков "ближайшая другая"
    int corrections = 0;      // сколько раз подряд чинили (для эскалации)

    while (!g_stopRouteGuard.load()) {
        // 0) кулдаун как раньше
        int64_t nowMs = monoMs();
        if (nowMs < g_rg.cooldownUntilMs) { std::this_thread::sleep_for(std::chrono::milliseconds(200)); continue; }

        // 1) координаты
        int32_t lat=0, lon=0, alt=0;
        if (!getCoords(lat, lon, alt)) { logEntry((char*)"RouteGuard: failed to get coords", ENTITY_NAME, LogLevel::LOG_WARNING); std::this_thread::sleep_for(std::chrono::milliseconds(200)); continue; }

        // 2) движение для арминга (как у тебя)
        if (!g_rg.haveLast) { g_rg.lastLat = lat; g_rg.lastLon = lon; g_rg.haveLast = true; }
        else { g_rg.movedAccum += haversineMeters(lat, lon, g_rg.lastLat, g_rg.lastLon); g_rg.lastLat = lat; g_rg.lastLon = lon; }

        // 3) высота и движение: ждём реального старта
        if (!g_rg.armed) {
            char buf[160];
            snprintf(buf, sizeof(buf), "RouteGuard: arming check — movedAccum=%.2f m / %.2f m, alt=%.1f m",
                    g_rg.movedAccum, STARTUP_MOVE_ARM_M, (double)alt);
            logEntry(buf, ENTITY_NAME, LogLevel::LOG_INFO);

            if (g_rg.movedAccum >= STARTUP_MOVE_ARM_M && alt >= ALT_ARM_MIN_M) {
                g_rg.armed = true; g_mode = RGMode::LOCK; g_lock = {}; // стартуем LOCK-ON окно
                logEntry((char*)"RouteGuard: ARMED → LOCK mode", ENTITY_NAME, LogLevel::LOG_INFO);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        // === режим LOCK-ON: собираем окно, выбираем индекс по наибольшему убыванию дистанции ===
        if (g_mode == RGMode::LOCK) {
            // посчитать дистанции до всех целей
            std::vector<double> dcur_all(targets.size());
            for (size_t j=0;j<targets.size();++j)
                dcur_all[j] = haversineMeters(lat, lon, targets[j].lat, targets[j].lon);

            if (g_lock.ticks == 0) {
                g_lock.d0   = dcur_all;
                g_lock.prev = dcur_all;
                g_lock.decr.assign(targets.size(), 0);
                logEntry((char*)"RouteGuard: LOCK start window", ENTITY_NAME, LogLevel::LOG_INFO);
            } else {
                // мажоритарная убыль помежтиково
                for (size_t j=0;j<targets.size();++j) {
                    if (g_lock.prev[j] - dcur_all[j] >= LOCK_STEP_MIN_M) g_lock.decr[j]++;
                }
                g_lock.prev = dcur_all;
            }
            g_lock.ticks++;

            // 1) быстрый lock по близости
            size_t proxIdx = SIZE_MAX;
            for (size_t j=0;j<targets.size();++j) {
                if (dcur_all[j] < NEAR_LOCK_DIST_M) { proxIdx = j; break; }
            }
            if (proxIdx != SIZE_MAX) {
                char buf[160];
                snprintf(buf, sizeof(buf), "RouteGuard: LOCK by proximity → idx=%zu (%.1f m)",
                        proxIdx, dcur_all[proxIdx]);
                logEntry(buf, ENTITY_NAME, LogLevel::LOG_INFO);
                idx = proxIdx; g_mode = RGMode::GUARD;
                prevDist = 1e18; trendUp = 0; hijackTicks = 0;
                logEntry((char*)"RouteGuard: GUARD mode", ENTITY_NAME, LogLevel::LOG_INFO);
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                continue;
            }

            // ждём завершения окна
            if (g_lock.ticks < LOCK_WINDOW_TICKS) {
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                continue;
            }

            // 2) lock по относительной/абсолютной убыли
            size_t gainIdx = pickByGain(g_lock.d0, dcur_all, LOCK_MIN_GAIN_ABS_M, LOCK_MIN_GAIN_REL_K);

            // 3) lock по «мажоритарной убыли»
            size_t majIdx = SIZE_MAX; int bestDecr = 0;
            for (size_t j=0;j<g_lock.decr.size();++j) {
                if (g_lock.decr[j] >= LOCK_DECR_TICKS && g_lock.decr[j] > bestDecr) {
                    bestDecr = g_lock.decr[j]; majIdx = j;
                }
            }

            if (gainIdx != SIZE_MAX || majIdx != SIZE_MAX) {
                size_t lockIdx = (gainIdx != SIZE_MAX ? gainIdx : majIdx);
                char buf[200];
                snprintf(buf, sizeof(buf),
                        "RouteGuard: LOCK resolved → idx=%zu (%s)",
                        lockIdx, gainIdx != SIZE_MAX ? "by gain" : "by majority");
                logEntry(buf, ENTITY_NAME, LogLevel::LOG_INFO);
                idx = lockIdx; g_mode = RGMode::GUARD;
                prevDist = 1e18; trendUp = 0; hijackTicks = 0;
                logEntry((char*)"RouteGuard: GUARD mode", ENTITY_NAME, LogLevel::LOG_INFO);
                // сброс окна
                g_lock = {};
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                continue;
            }

            // 4) fallback после нескольких «inconclusive»
            g_lock.windows++;
            if (g_lock.windows >= LOCK_MAX_WINDOWS) {
                size_t limit = std::min((size_t)LOCK_MAX_AHEAD, targets.size()-1);
                size_t fbIdx = SIZE_MAX; double fbDist = 1e18;
                for (size_t j=0; j<=limit; ++j) {
                    if (dcur_all[j] < LOCK_FALLBACK_DIST_M && dcur_all[j] < fbDist) {
                        fbDist = dcur_all[j]; fbIdx = j;
                    }
                }
                if (fbIdx != SIZE_MAX) {
                    char buf[200];
                    snprintf(buf, sizeof(buf),
                            "RouteGuard: LOCK fallback(first %d) → idx=%zu (%.1f m)",
                            LOCK_MAX_AHEAD, fbIdx, fbDist);
                    logEntry(buf, ENTITY_NAME, LogLevel::LOG_WARNING);
                    idx = fbIdx; g_mode = RGMode::GUARD;
                    prevDist = 1e18; trendUp = 0; hijackTicks = 0;
                    logEntry((char*)"RouteGuard: GUARD mode", ENTITY_NAME, LogLevel::LOG_INFO);
                    g_lock = {};
                    std::this_thread::sleep_for(std::chrono::milliseconds(300));
                    continue;
                }
            }

            // 5) окно не срослось — начать новое
            logEntry((char*)"RouteGuard: LOCK inconclusive — extend window", ENTITY_NAME, LogLevel::LOG_WARNING);
            g_lock.ticks = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            continue;
        }

        // === режим GUARD: контроль только «нелегальных прыжков» индексов ===
        double dcur = haversineMeters(lat, lon, targets[idx].lat, targets[idx].lon);

        // нормальный прогресс: достигли текущий → idx++
        if (dcur < REACH_RADIUS_M && idx + 1 < targets.size()) {
            char buf[128];
            snprintf(buf, sizeof(buf), "RouteGuard: reached idx=%zu (%.1f m) → %zu", idx, dcur, idx+1);
            logEntry(buf, ENTITY_NAME, LogLevel::LOG_INFO);
            idx++; prevDist = 1e18; trendUp = 0; hijackTicks = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        // найдём ближайший по дистанции
        size_t best = idx; double bestDist = dcur;
        for (size_t j = idx+1; j < targets.size(); ++j) {
            double dj = haversineMeters(lat, lon, targets[j].lat, targets[j].lon);
            if (dj < bestDist) { bestDist = dj; best = j; }
        }

        // лог метрик
        {
            char buf[256];
            snprintf(buf, sizeof(buf),
                "Pos: (%d,%d) alt=%d | idx=%zu dcur=%.1f | best=%zu bestDist=%.1f | mode=GUARD",
                lat, lon, alt, idx, dcur, best, bestDist);
            logEntry(buf, ENTITY_NAME, LogLevel::LOG_INFO);
        }

        // --- ключ: допускаем только best == idx или best == idx+1 ---
        // если best далеко вперёд (>= idx+2) или назад (< idx), считаем подозрительно
        bool illegalJump = (best + 1 < idx) || (best >= idx + 2);
        if (illegalJump) {
            hijackTicks++;
            char buf[80]; snprintf(buf, sizeof(buf), "RouteGuard: illegalJump best=%zu vs idx=%zu (tick %d)", best, idx, hijackTicks);
            logEntry(buf, ENTITY_NAME, LogLevel::LOG_WARNING);
        } else if (hijackTicks > 0) {
            hijackTicks--;
        }

        // подтверждение атаки
        if (hijackTicks >= GUARD_CONFIRM_TICKS) {
            char buf[160];
            snprintf(buf, sizeof(buf),
                "RouteGuard: HIJACK CONFIRMED (best=%zu, idx=%zu) → changeWaypoint to idx=%zu",
                best, idx, idx);
            logEntry(buf, ENTITY_NAME, LogLevel::LOG_ERROR);

            if (!changeWaypoint(targets[idx].lat, targets[idx].lon, targets[idx].alt)) {
                logEntry((char*)"RouteGuard: changeWaypoint FAILED → backoff & LOCK again", ENTITY_NAME, LogLevel::LOG_WARNING);
                // вернуться в LOCK, если миссия/режим сбились
                g_mode = RGMode::LOCK; g_lock = {};
                g_rg.cooldownUntilMs = nowMs + NO_MISSION_COOLDOWN_MS;
                g_rg.armed = false; g_rg.movedAccum = 0.0; g_rg.haveLast = false;
                hijackTicks = 0; trendUp = 0; prevDist = 1e18;
            } else {
                logEntry((char*)"RouteGuard: changeWaypoint OK", ENTITY_NAME, LogLevel::LOG_INFO);
                hijackTicks = 0; trendUp = 0; prevDist = 1e18;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(300));

    }

}
/**
 * \~English Procedure that checks connection to the ATM server.
 * \~Russian Процедура, проверяющая наличие соединения с сервером ОРВД.
 */
void pingSession() {
    sleep(sessionDelay);

    char pingMessage[1024] = {0};
    while (true) {
        if (!receiveSubscription("ping/", pingMessage, 1024)) {
            logEntry("Failed to receive ping through Server Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
            continue;
        }

        if (strcmp(pingMessage, "")) {
            uint8_t authenticity = 0;
            if (!checkSignature(pingMessage, authenticity) || !authenticity) {
                logEntry("Failed to check signature of ping received through Server Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
                continue;
            }

            //Processing delay until next session
            sessionDelay = parseDelay(strstr(pingMessage, "$Delay "));
        }
        else {
            //No response from the server
            //If server does not respond for 3 more seconds, flight must be paused until the response is received
        }

        sleep(sessionDelay);
    }
}

/**
 * \~English Procedure that tracks flight status and no flight areas changes.
 * \~Russian Процедура, отслеживающая изменение статуса полета и запретных зон.
 */
void serverUpdateCheck() {
    char message[4096] = {0};

    while (true) {
        if (receiveSubscription("api/flight_status/", message, 4096)) {
            if (strcmp(message, "")) {
                uint8_t authenticity = 0;
                if (checkSignature(message, authenticity) || !authenticity) {
                    if (strstr(message, "$Flight -1#")) {
                        logEntry("Emergency stop request is received. Disabling motors", ENTITY_NAME, LogLevel::LOG_INFO);
                        if (!enableBuzzer())
                            logEntry("Failed to enable buzzer", ENTITY_NAME, LogLevel::LOG_WARNING);
                        while (!setKillSwitch(false)) {
                            logEntry("Failed to forbid motor usage. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
                            sleep(1);
                        }
                    }
                    //The message has two other possible options:
                    //  "$Flight 1#" that requires to pause flight and remain landed
                    //  "$Flight 0#" that requires to resume flight and keep flying
                    //Implementation is required to be done
                }
                else
                    logEntry("Failed to check signature of flight status received through Server Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
            }
        }
        else
            logEntry("Failed to receive flight status through Server Connector", ENTITY_NAME, LogLevel::LOG_WARNING);

        if (receiveSubscription("api/forbidden_zones", message, 4096)) {
            if (strcmp(message, "")) {
                uint8_t authenticity = 0;
                if (checkSignature(message, authenticity) || !authenticity) {
                    deleteNoFlightAreas();
                    loadNoFlightAreas(message);
                    logEntry("New no-flight areas are received from the server", ENTITY_NAME, LogLevel::LOG_INFO);
                    printNoFlightAreas();
                    //Path recalculation must be done if current path crosses new no-flight areas
                }
                else
                    logEntry("Failed to check signature of no-flight areas received through Server Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
            }
        }
        else
            logEntry("Failed to receive no-flight areas through Server Connector", ENTITY_NAME, LogLevel::LOG_WARNING);

        sleep(1);
    }
}

/**
 * \~English Auxiliary procedure. Asks the ATM server to approve new mission and parses its response.
 * \param[in] mission New mission in string format.
 * \param[out] result ATM server response: 1 if mission approved, 0 otherwise.
 * \return Returns 1 on successful send, 0 otherwise.
 * \~Russian Вспомогательная процедура. Просит у сервера ОРВД одобрения новой миссии и обрабатывает ответ.
 * \param[in] mission Новая миссия в виде строки.
 * \param[out] result Ответ сервера ОРВД: 1 при одобрении миссии, иначе -- 0.
 * \return Возвращает 1 при успешной отправке, иначе -- 0.
 */
int askForMissionApproval(char* mission, int& result) {
    int messageSize = 512 + strlen(mission);
    char *message = (char*)malloc(messageSize);
    char signature[257] = {0};

    snprintf(message, messageSize, "/api/nmission?id=%s&mission=%s", boardId, mission);
    if (!signMessage(message, signature, 257)) {
        logEntry("Failed to sign new mission at Credential Manager", ENTITY_NAME, LogLevel::LOG_WARNING);
        free(message);
        return 0;
    }

    snprintf(message, messageSize, "mission=%s&sig=0x%s", mission, signature);
    if (!publishMessage("api/nmission/request", message)) {
        logEntry("Failed to publish new mission through Server Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
        free(message);
        return 0;
    }

    while (!receiveSubscription("api/nmission/response/", message, 512) || !strcmp(message, ""))
        sleep(1);

    uint8_t authenticity = 0;
    if (!checkSignature(message, authenticity) || !authenticity) {
        logEntry("Failed to check signature of new mission received through Server Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
        free(message);
        return 0;
    }

    if (strstr(message, "$Approve 0#") != NULL)
        result = 1;
    else if (strstr(message, "$Approve 1#") != NULL)
        result = 0;
    else {
        logEntry("Failed to parse server response on New Mission request", ENTITY_NAME, LogLevel::LOG_WARNING);
        free(message);
        return 0;
    }

    free(message);
    return 1;
}

/**
 * \~English Security module main loop. Waits for all other components to initialize. Authenticates
 * on the ATM server and receives the mission from it. After a mission and an arm request from the autopilot
 * are received, requests permission to take off from the ATM server. On receive supplies power to motors.
 * Then flight control must be performed.
 * \return Returns 1 on completion with no errors.
 * \~Russian Основной цикл модуля безопасности. Ожидает инициализации всех остальных компонентов. Аутентифицируется
 * на сервере ОРВД и получает от него миссию. После получения миссии и запроса на арминг от автопилота, запрашивает разрешение
 * на взлет у сервера ОРВД. При его получении подает питание на двигатели. Далее должен выполняться контроль полета.
 * \return Возвращает 1 при завершении без ошибок.
 */
int main(void) {
    char logBuffer[256] = {0};
    char signBuffer[257] = {0};
    char publicationBuffer[1024] = {0};
    char subscriptionBuffer[4096] = {0};
    //Before do anything, we need to ensure, that other modules are ready to work
    while (!waitForInit("logger_connection", "Logger")) {
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Logger. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("periphery_controller_connection", "PeripheryController")) {
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Periphery Controller. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("autopilot_connector_connection", "AutopilotConnector")) {
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Autopilot Connector. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("navigation_system_connection", "NavigationSystem")) {
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Navigation System. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("server_connector_connection", "ServerConnector")) {
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Server Connector. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("credential_manager_connection", "CredentialManager")) {
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Credential Manager. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }

    //Get ID from ServerConnector
    while (!getBoardId(boardId)) {
        logEntry("Failed to get board ID from ServerConnector. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }
    snprintf(logBuffer, 256, "Board '%s' is initialized", boardId);
    logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);

    //Enable buzzer to indicate, that all modules has been initialized
    if (!enableBuzzer())
        logEntry("Failed to enable buzzer at Periphery Controller", ENTITY_NAME, LogLevel::LOG_WARNING);

    //Copter need to be registered at ORVD
    char authRequest[512] = {0};
    char authSignature[257] = {0};
    snprintf(authRequest, 512, "/api/auth?id=%s", boardId);
    while (!signMessage(authRequest, authSignature, 257)) {
        snprintf(logBuffer, 256, "Failed to sign auth message at Credential Manager. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }

    char authResponse[1024] = {0};
    snprintf(authRequest, 512, "%s&sig=0x%s", authRequest, authSignature);
    while (!sendRequest(authRequest, authResponse, 1024) || !strcmp(authResponse, "TIMEOUT")) {
        snprintf(logBuffer, 256, "Failed to send auth request through Server Connector. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }

    uint8_t authenticity = 0;
    while (!checkSignature(authResponse, authenticity) || !authenticity) {
        snprintf(logBuffer, 256, "Failed to check signature of auth response received through Server Connector. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    logEntry("Successfully authenticated on the server", ENTITY_NAME, LogLevel::LOG_INFO);

    //Constantly ask server, if mission for the drone is available. Parse it and ensure, that mission is correct
    while (!receiveSubscription("api/fmission_kos/", subscriptionBuffer, 4096) || !strcmp(subscriptionBuffer, ""))
        sleep(1);

    authenticity = 0;
    while (!checkSignature(subscriptionBuffer, authenticity) || !authenticity) {
        snprintf(logBuffer, 256, "Failed to check signature of mission received through Server Connector. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }

    if (loadMission(subscriptionBuffer)) {
        logEntry("Successfully received mission from the server", ENTITY_NAME, LogLevel::LOG_INFO);
        printMission();
    }

    //The drone is ready to arm
    logEntry("Ready to arm", ENTITY_NAME, LogLevel::LOG_INFO);
    while (true) {
        //Wait, until autopilot wants to arm (and fails so, as motors are disabled by default)
        while (!waitForArmRequest()) {
            snprintf(logBuffer, 256, "Failed to receive an arm request from Autopilot Connector. Trying again in %ds", RETRY_DELAY_SEC);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            sleep(RETRY_DELAY_SEC);
        }
        logEntry("Received arm request. Notifying the server", ENTITY_NAME, LogLevel::LOG_INFO);

        //When autopilot asked for arm, we need to receive permission from ORVD
        snprintf(publicationBuffer, 1024, "/api/arm?id=%s", boardId);
        while (!signMessage(publicationBuffer, signBuffer, 257)) {
            snprintf(logBuffer, 256, "Failed to sign arm request at Credential Manager. Trying again in %ds", RETRY_DELAY_SEC);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            sleep(RETRY_DELAY_SEC);
        }

        snprintf(publicationBuffer, 1024, "sig=0x%s", signBuffer);
        while (!publishMessage("api/arm/request", publicationBuffer)) {
            snprintf(logBuffer, 256, "Failed to publish arm request through Server Connector. Trying again in %ds", RETRY_DELAY_SEC);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            sleep(RETRY_DELAY_SEC);
        }

        while (!receiveSubscription("api/arm/response/", subscriptionBuffer, 4096) || !strcmp(subscriptionBuffer, ""))
            sleep(1);

        authenticity = 0;
        while (!checkSignature(subscriptionBuffer, authenticity) || !authenticity) {
            snprintf(logBuffer, 256, "Failed to check signature of arm response received through Server Connector. Trying again in %ds", RETRY_DELAY_SEC);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            sleep(RETRY_DELAY_SEC);
        }

        if (strstr(subscriptionBuffer, "$Arm 0$")) {
            //If arm was permitted, we enable motors
            logEntry("Arm is permitted", ENTITY_NAME, LogLevel::LOG_INFO);
            while (!setKillSwitch(true)) {
                snprintf(logBuffer, 256, "Failed to permit motor usage at Periphery Controller. Trying again in %ds", RETRY_DELAY_SEC);
                logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
                sleep(RETRY_DELAY_SEC);
            }
            if (!permitArm())
                logEntry("Failed to permit arm through Autopilot Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
            //Get time until next session
            sessionDelay = parseDelay(strstr(subscriptionBuffer, "$Delay "));
            //Start ORVD threads
            sessionThread = std::thread(pingSession);
            updateThread = std::thread(serverUpdateCheck);
            std::thread routeGuard(routeGuardThreadFunc);
            routeGuard.detach();
            break;
        }
        else if (strstr(subscriptionBuffer, "$Arm 1$")) {
            logEntry("Arm is forbidden", ENTITY_NAME, LogLevel::LOG_INFO);
            if (!forbidArm())
                logEntry("Failed to forbid arm through Autopilot Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
        }
        else
            logEntry("Failed to parse server response", ENTITY_NAME, LogLevel::LOG_WARNING);
        logEntry("Arm was not allowed. Waiting for another arm request from autopilot", ENTITY_NAME, LogLevel::LOG_WARNING);
    };

    //If we get here, the drone is able to arm and start the mission
    //The flight is need to be controlled from now on

    while (true)
        sleep(1000);

    return EXIT_SUCCESS;
}