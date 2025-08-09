/**
 * \file
 * \~English
 * \brief Implementation of methods for simulated drone peripherals control.
 * \details The file contains implementation of methods, that simulate peripherals
 * control via socket communication with a compatible ArduPilot SITL firmware.
 *
 * \~Russian
 * \brief Реализация методов для работы с симулятором периферии дрона.
 * \details В файле реализованы методы, симулирующие управление периферией дрона
 * через взаимодействие с совместимой SITL-прошивкой ArduPilot через сокет.
 */

#include "../include/periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"
#include "../../shared/include/ipc_messages_navigation_system.h"

#include <kos_net.h>

#include <stdio.h>
#include <math.h>
#include <time.h> // для time()

/** \cond */
#define SIM_PERIPHERY_MESSAGE_HEAD_SIZE 4
#define RFID_TAG_NUM 10

static const uint8_t SimPeripheryMessageHead[SIM_PERIPHERY_MESSAGE_HEAD_SIZE] = { 0x06, 0x66, 0xbe, 0xa7 };
// --- Cargo drop authorization state ---
static bool cargoDropAuthorized = false;          // true, если разрешение выставлено миссией/оператором
static int64_t cargoAuthorizationExpiry = 0;     // unix timestamp (seconds) когда разрешение истекает
static int32_t cargoAuthLat = 0;                 // разрешённые координаты (формат как rfidLats)
static int32_t cargoAuthLng = 0;
static float cargoAuthRadiusSquared = 0.0f;      // радиус в той же шкале, что и scanSquaredDistance (скваж. расстояние)
static const int DEFAULT_CARGO_AUTH_TIMEOUT = 30; // сек. по умолчанию — окно, в котором можно выполнить сброс
/** \endcond */
/** \endcond */

/**
 * \~English Control messages types used to simulate
 * peripherals power supply mod change.
 * \~Russian Типы управляющих сообщений, используемых для симуляции
 * изменения режима подачи питания на периферийные устройства.
 */
enum SimPeripheryCommand : uint8_t {
    /**
     * \~English Auxiliary type. Signals an error.
     * \~Russian Вспомогательный тип. Сигнализирует о возникновении ошибки.
     */
    ERROR = 0x00,
    /**
     * \~English Drone motors are energized. On receive SITL firmware disables software restriction on arming.
     * \~Russian Двигатели запитаны. При получении SITL-прошивка снимает программный запрет на арминг.
     */
    MotorPermit,
    /**
     * \~English Drone motors are de-energized. On receive SITL firmware enables software restriction on arming.
     * \~Russian Двигатели обесточены. При получении SITL-прошивка включает программный запрет на арминг.
     */
    MotorForbid,
    /**
     * \~English Cargo drop motor is energized. On receive SITL firmware disables software restriction on cargo drop.
     * \~Russian Мотор сброса груза запитан. При получении SITL-прошивка снимает программный запрет на сброс груза.
     */
    CargoPermit,
    /**
     * \~English Cargo drop motor is de-energized. On receive SITL firmware enables software restriction on cargo drop.
     * \~Russian Мотор сброса груза обесточен. При получении SITL-прошивка включает программный запрет на сброс груза.
     */
    CargoForbid
};

/**
 * \~English A structure describing a message for SITL firmware
 * to simulate peripherals power supply mod change.
 * transmit current drone position.
 * \~Russian Структура, описывающая сообщение для SITL-прошивки
 * для симуляции изменения режима питания периферийных устройств.
 */
struct SimPeripheryMessage {
    /**
     * \~English Fixed header bytes that indicate the start of a new message.
     * \~Russian Фиксированные заголовочные байты, указывающие на начало нового сообщения.
     */
    uint8_t head[SIM_PERIPHERY_MESSAGE_HEAD_SIZE];
    /**
     * \~English Message type. For the types used, see \ref SimPeripheryCommand.
     * \~Russian Тип сообщения. Используемые типы см. в \ref SimPeripheryCommand.
     */
    SimPeripheryCommand command;
    /**
     * \~English Meaningless bytes. Needed to fill the size of the structure to a multiple of 4 bytes.
     * \~Russian Байты, не имеющие смысловой нагрузки. Нужны для заполнения размера структуры до
     * кратного 4 байтам.
     */
    uint8_t filler[3];

    /**
     * \~English Constructor that sets the correct header bytes and given type command.
     * \param[in] cmd Created message type.
     * \~Russian Конструктор, устанавливающий корректные заголовочные байты и команду поданного типа.
     * \param[in] cmd Тип создаваемого сообщения.
     */
    SimPeripheryMessage(SimPeripheryCommand cmd) {
        for (int i = 0; i < SIM_PERIPHERY_MESSAGE_HEAD_SIZE; i++)
            head[i] = SimPeripheryMessageHead[i];
        command = cmd;
    }
};

/** \cond */
int peripherySocket = NULL;
uint16_t peripheryPort = 5767;

float scanSquaredDistance = 0.25;
float latScale = 0.011131884502145f;
float lngScale = 0.011131884502145f;
int32_t rfidLats[RFID_TAG_NUM] = { 600025652, 600025921, 600026011, 600025652, 600026191, 600026370, 600026280, 600026550, 600026775, 600026640 };
int32_t rfidLngs[RFID_TAG_NUM] = { 278574262, 278574082, 278574801, 278575340, 278575340, 278574891, 278574082, 278574352, 278574082, 278575340 };
char rfidIDs[RFID_TAG_NUM][6] = { "rfid0", "rfid1", "rfid2", "rfid3", "rfid4", "rfid5", "rfid6", "rfid7", "rfid8", "rfid9" };

bool killSwitchEnabled;
/** \endcond */

int initPeripheryController() {
    if (!wait_for_network()) {
        logEntry("Connection to network has failed", ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    float avgLat = 0;
    for (int i = 0; i < RFID_TAG_NUM; i++)
        avgLat += 1.0f * rfidLats[i] / RFID_TAG_NUM;
    float scale = cos(avgLat * 1.0e-7 * M_PI / 180.0f);
    if (scale < 0.01f)
        scale = 0.01f;
    lngScale *= scale;

    return 1;
}

int initGpioPins() {
    peripherySocket = NULL;

    if ((peripherySocket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        logEntry("Failed to create socket", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    struct sockaddr_in address = { 0 };
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(SIMULATOR_IP);
    address.sin_port = htons(peripheryPort);

    if (connect(peripherySocket, (struct sockaddr*)&address, sizeof(address)) != 0) {
        char logBuffer[256] = {0};
        snprintf(logBuffer, 256, "Connection to %s:%d has failed", SIMULATOR_IP, peripheryPort);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    return 1;
}

bool isKillSwitchEnabled() {
    return killSwitchEnabled;
}


// Установить разрешение на сброс груза (вызывать миссией/оператором перед подлётом к точке)
void authorizeCargoDrop(int32_t lat, int32_t lng, float radiusMeters, int timeoutSeconds = DEFAULT_CARGO_AUTH_TIMEOUT) {
    // Конвертируем radiusMeters в ту же метрику, где latScale/lngScale уже учитывают cos(lat)
    // У нас в коде используются предскейлы latScale/lngScale на 1e-7 угловых единиц -> оставим простой подход:
    // Преобразуем метр в *скейлованные* градусы приблизительно: 1 deg ~ 111 km -> meters/111000 -> degrees
    // Но поскольку в проекте используется latScale/lngScale (фактор в readRfid), удобнее:
    // Предположим входные radiusMeters задаются в метрах; переводим в "scaled units" через latScale/lngScale:
    // approximate degrees = meters / 111000.0
    float degreesRadius = radiusMeters / 111000.0f;
    // переводим degree -> "scaled" как в readRfid (они умножают (rfid - lat) * latScale)
    // чтобы получить порог сравнения (latDif^2 + lngDif^2 < threshold), нам нужно:
    // threshold = (degreesRadius / latScale_for_coords)^2 roughly. Но latScale already ~0.01113 which is ~degrees->meters ? 
    // Для консистентности: будем хранить порог в тех же единицах, что и scanSquaredDistance:
    // При проверке мы делаем latDif = (rfidLat - currLat) * latScale; И сравниваем latDif^2 + ...
    // чтобы radius в метрах перевести в тот же порог: radiusDegrees = degreesRadius; latDifThreshold = radiusDegrees * (1.0f/1.0f) ??? 
    // (упрощаем: переводим degreesRadius в "dif" с помощью константы 1.0e-7, т.к. rfidLat хранится как lat*1e7)
    // Конкретно: if currLat and rfidLat are in 1e-7 degrees -> diffRaw = (rfid - curr)*1e-7 (deg)
    // latDif = diffRaw * (M_PI/180)?? Но в readRfid they used latScale directly. Чтобы быть консистентным, можно
    // вычислить threshold numerically at runtime using current coords (use conversion used in initPeripheryController).
    // Для простоты: сохраним radiusMeters и при проверке будем конвертировать корректно.
    // Здесь сохраним параметры:
    cargoDropAuthorized = true;
    cargoAuthLat = lat;
    cargoAuthLng = lng;
    time_t now = time(nullptr);
    cargoAuthorizationExpiry = (int64_t)now + timeoutSeconds;
    // Сохранить radiusMeters в cargoAuthRadiusSquared как квадрат радиуса в метрах (придётся конвертировать при проверке)
    cargoAuthRadiusSquared = radiusMeters * radiusMeters;
}

// Отменить разрешение заранее
void revokeCargoAuthorization() {
    cargoDropAuthorized = false;
    cargoAuthorizationExpiry = 0;
    cargoAuthLat = cargoAuthLng = 0;
    cargoAuthRadiusSquared = 0.0f;
}

// Внутренняя проверка: разрешён ли сейчас сброс по координатам и таймауту
static bool isAtAuthorizedDropPointAndAuthorized(int32_t currLat, int32_t currLng, int currAlt) {
    if (!cargoDropAuthorized) return false;
    time_t now = time(nullptr);
    if (now > cargoAuthorizationExpiry) {
        // истёк таймаут
        revokeCargoAuthorization();
        return false;
    }

    // Конвертация: вычислим расстояние в метрах между current и auth через приближение Haversine упрощённо.
    // Простейший способ — использовать lat/lng в 1e-7 degrees -> перевести в degrees -> radians -> расчёт Haversine.
    auto deg_from_int = [](int32_t v)->double { return v * 1e-7; };

    double lat1 = deg_from_int(currLat) * M_PI / 180.0;
    double lon1 = deg_from_int(currLng) * M_PI / 180.0;
    double lat2 = deg_from_int(cargoAuthLat) * M_PI / 180.0;
    double lon2 = deg_from_int(cargoAuthLng) * M_PI / 180.0;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double a = sin(dlat/2)*sin(dlat/2) + cos(lat1)*cos(lat2)*sin(dlon/2)*sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double earthRadius = 6371000.0; // meters
    double distanceMeters = earthRadius * c;

    if (distanceMeters * distanceMeters <= (double)cargoAuthRadiusSquared) {
        // Дополнительные проверки: можно проверить высоту/скорость здесь (alt/velocity) — в текущем api есть currAlt.
        // Например, проверить, что высота не слишком большая/маленькая; пропустим (можно добавить по необходимости).
        return true;
    }

    return false;
}


int setBuzzer(bool enable) {
    char logBuffer[256] = {0};
    snprintf(logBuffer, 256, "Buzzer is %s", enable ? "enabled" : "disabled");
    logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);

    return 1;
}

int readRfid(char* tag) {
    int32_t lat, lng, alt;
    getCoords(lat, lng, alt);

    float latDif, lngDif;
    float altSquaredDif = (alt * alt) / 10000.0f;
    for (int i = 0; i < RFID_TAG_NUM; i++) {
        latDif = (rfidLats[i] - lat) * latScale;
        lngDif = (rfidLngs[i] - lng) * lngScale;

        if (latDif * latDif + lngDif * lngDif + altSquaredDif  < scanSquaredDistance) {
            strcpy(tag, rfidIDs[i]);
            return 1;
        }
    }

    strcpy(tag, "");
    return 1;
}

int setKillSwitch(bool enable) {
    SimPeripheryMessage message = SimPeripheryMessage(enable ? SimPeripheryCommand::MotorPermit : SimPeripheryCommand::MotorForbid);
    write(peripherySocket, &message, sizeof(SimPeripheryMessage));
    killSwitchEnabled = enable;
    if (!publishMessage("api/events", enable ? "type=kill_switch&event=Kill-switch is enabled" : "type=kill_switch&event=Kill-switch is disabled"))
        logEntry("Failed to publish event message", ENTITY_NAME, LogLevel::LOG_WARNING);

    return 1;
}

int setCargoLock(bool enable) {
    // Если пришла команда на "запрет" — всегда разрешаем запрет (оно безопасно).
    if (!enable) {
        SimPeripheryMessage message = SimPeripheryMessage(SimPeripheryCommand::CargoForbid);
        write(peripherySocket, &message, sizeof(SimPeripheryMessage));
        revokeCargoAuthorization(); // безопасность: после запрета отменим разрешения
        if (!publishMessage("api/events", "type=cargo_lock&event=Cargo lock is disabled"))
            logEntry("Failed to publish event message", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 1;
    }

    // enable == true: хотим включить питание мотора сброса — проверяем авторизацию и позицию
    int32_t currLat, currLng, currAlt;
    getCoords(currLat, currLng, currAlt);

    if (isAtAuthorizedDropPointAndAuthorized(currLat, currLng, currAlt)) {
        // Всё ок — разрешаем
        SimPeripheryMessage message = SimPeripheryMessage(SimPeripheryCommand::CargoPermit);
        write(peripherySocket, &message, sizeof(SimPeripheryMessage));
        // после успешного разрешения — логируем и сразу отзываем авторизацию (предотвращаем повторный случайный сброс)
        revokeCargoAuthorization();
        if (!publishMessage("api/events", "type=cargo_lock&event=Cargo lock is enabled"))
            logEntry("Failed to publish event message", ENTITY_NAME, LogLevel::LOG_WARNING);
    } else {
        // Не разрешено: отправляем forbid (отменяем команду), логируем и публикуем предупреждение
        SimPeripheryMessage message = SimPeripheryMessage(SimPeripheryCommand::CargoForbid);
        write(peripherySocket, &message, sizeof(SimPeripheryMessage));

        char logBuffer[256] = {0};
        snprintf(logBuffer, sizeof(logBuffer), "CargoPermit denied: not within authorized drop point or authorization expired (lat=%d lng=%d)", currLat, currLng);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);

        if (!publishMessage("api/events", "type=cargo_lock&event=Cargo drop attempt denied"))
            logEntry("Failed to publish event message", ENTITY_NAME, LogLevel::LOG_WARNING);
    }

    return 1;
}
