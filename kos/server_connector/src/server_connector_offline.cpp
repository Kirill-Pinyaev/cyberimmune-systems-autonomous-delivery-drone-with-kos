/**
 * \file
 * \~English
 * \brief Implementation of methods for ATM server communication simulation.
 * \details The file contains implementation of methods, that simulate
 * requests to an ATM server send and received responses process.
 *
 * \~Russian
 * \brief Реализация методов для имитации общения с сервером ОРВД.
 * \details В файле реализованы методы, имитирующие отправку запросов на сервер ОРВД
 * и обработку полученных ответов.
 */

#include "../include/server_connector.h"

#include <stdio.h>
#include <string.h>

int flightStatusSend, missionSend, areasSend, armSend, newMissionSend;

int initServerConnector() {
    if (strlen(BOARD_ID))
        setBoardName(BOARD_ID);
    else
        setBoardName("00:00:00:00:00:00");

    flightStatusSend = true;
    missionSend = true;
    areasSend = true;
    armSend= false;
    newMissionSend = false;

    return 1;
}

int requestServer(char* query, char* response, uint32_t responseSize) {
    if (strstr(query, "/api/auth?")) {
        if (responseSize < 10) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(response, "$Success#", 10);
    }
    else {
        if (responseSize < 3) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(response, "$#", 3);
    }

    return 1;
}

int publish(char* topic, char* publication) {
    if (strstr(topic, "api/arm/request"))
        armSend = true;
    else if (strstr(topic, "api/nmission/request"))
        newMissionSend = true;

    return 1;
}

int getSubscription(char* topic, char* message, uint32_t messageSize) {
    if (strstr(topic, "ping/")) {
        if (messageSize < 10) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$Delay 1#", 10);
    }
    else if (strstr(topic, "api/flight_status/") && flightStatusSend) {
        if (messageSize < 11) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$Flight 0#", 11);
        flightStatusSend = false;
    }
    else if (strstr(topic, "api/fmission_kos/") && missionSend) {
        if (messageSize < 901) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$FlightMission H60.0025652_27.8573543_0.0&T2.0&W60.0025652_27.8574082_2.0&W60.0025921_27.8574082_2.0&W60.0026011_27.8574801_2.0&W60.0025876_27.857534_2.0&W60.0025652_27.857534_2.0&W60.0025876_27.857534_2.0&W60.0025876_27.8574262_2.0&W60.0025652_27.8574262_2.0&W60.0025876_27.8574262_2.0&W60.0026011_27.857534_2.0&W60.0026191_27.857534_2.0&W60.002637_27.857534_2.0&W60.002637_27.8574891_2.0&W60.002628_27.8574082_2.0&W60.002655_27.8574352_2.0&W60.0026775_27.8574082_2.0&W60.0026909_27.8574082_2.0&W60.0026909_27.857534_2.0&D3.0&S5.0_1200.0&D1.0&S5.0_1800.0&W60.0026909_27.8573543_2.0&W60.0025652_27.8573543_2.0&L60.0025652_27.8573543_0.0&I60.0025652_27.8574262_0.0&I60.0025921_27.8574082_0.0&I60.0026011_27.8574801_0.0&I60.0025652_27.857534_0.0&I60.0026191_27.857534_0.0&I60.002637_27.8574891_0.0&I60.002628_27.8574082_0.0&I60.002655_27.8574352_0.0&I60.0026775_27.8574082_0.0&I60.002664_27.857534_0.0#", 901);
        missionSend = false;
    }
    else if (strstr(topic, "api/forbidden_zones") && areasSend) {
        if (messageSize < 1024) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$ForbiddenZones 5&outerOne&7&60.0025472_27.8573184&60.0025562_27.8573184&60.0025562_27.8575520&60.0026999_27.8575520&60.0026999_27.8575699&60.0025472_27.8575699&60.0025472_27.8573184&outerTwo&7&60.0025562_27.8573184&60.0025562_27.8573364&60.0026999_27.8573364&60.0026999_27.8575699&60.0027089_27.8575699&60.0027089_27.8573184&60.0025562_27.8573184&innerOne&7&60.0025562_27.8574442&60.0025741_27.8574442&60.0025741_27.8574981&60.0025831_27.8574981&60.0025831_27.8575160&60.0025562_27.8575160&60.0025562_27.8574442&innerTwo&11&60.0025741_27.8573723&60.0025741_27.8573903&60.0026101_27.8573903&60.0026101_27.8575160&60.0026280_27.8575160&60.0026280_27.8574981&60.0026191_27.8574981&60.0026191_27.8573903&60.0026819_27.8573903&60.0026819_27.8573723&60.0025741_27.8573723&innerThree&11&60.0026460_27.8575520&60.0026460_27.8574981&60.0026730_27.8574981&60.0026730_27.8574262&60.0026819_27.8574262&60.0026819_27.8575520&60.0026730_27.8575520&60.0026730_27.8575160&60.0026550_27.8575160&60.0026550_27.8575520&60.0026460_27.8575520#", 1024);
        areasSend = false;
    }
    else if (strstr(topic, "api/arm/response/") && armSend) {
        if (messageSize < 16) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$Arm 0$Delay 1#", 16);
    }
    else if (strstr(topic, "api/nmission/response/") && newMissionSend) {
        if (messageSize < 13) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$Approve 0#", 13);
    }
    else
        strcpy(message, "");

    return 1;
}