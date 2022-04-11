#include "main.h"
#include "gps.h"
#include "sensors.h"
#include "can.h"
#include "json.h"
#include "coms.h"
#include "autosteer.h"

#include <ArduinoJson.h>

ThreadWrap(Serial, SerialXtra);

SteerConfig steerConfig;
SteerSettings steerSettings;
SteerSetpoints steerSetpoints;
Switches switches;
ImuData imuData;
VbusData vbusData;
IsobusData isobusData;

StaticJsonDocument<1024> statusJson;

void heartbeat()
{
    while (1)
    {
        statusJson["steerSetpoints"]["steerAngleSetpoint"]  =   steerSetpoints.requestedSteerAngle;
        statusJson["steerSetpoints"]["guidanceStatus"]      =   steerSetpoints.guidanceStatus;
        statusJson["steerSetpoints"]["actualSteerAngle"]    =   steerSetpoints.actualSteerAngle;
        statusJson["imuData"]["roll"]                       =   imuData.roll;
        statusJson["imuData"]["heading"]                    =   imuData.heading;
        statusJson["imuData"]["pitch"]                      =   imuData.pitch;
        statusJson["isobusData"]["rearHitchPosition"]       =   isobusData.rearHitchPosition;
        statusJson["isobusData"]["rearPtoRpm"]              =   isobusData.rearPtoRpm;
        serializeJsonPretty(statusJson, Serial);
        digitalToggleFast(13);
        threads.delay(1000);
    }
}

void setup()
{
    Serial.println("teensy-aog running!");

    pinMode(13, 1);

    Serial.begin(115200);

    threads.addThread(heartbeat);

    initSD();

    loadSteerConfig();
    loadSteerSettings();

    setupEthernet();
    setupGNSS();
    setupSensors();
    setupCAN();
    setupAutosteer();
}

void loop()
{
    yield();
    threads.yield();
}