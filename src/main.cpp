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
        statusJson["steerSetpoints"]["steerAngleSetpoint"]  = steerSetpoints.requestedSteerAngle;
        statusJson["steerSetpoints"]["guidanceStatus"]      = steerSetpoints.guidanceStatus;
        statusJson["steerSetpoints"]["actualSteerAngle"]    = steerSetpoints.actualSteerAngle;
        statusJson["imuData"]["roll"]                       = imuData.roll;
        statusJson["imuData"]["heading"]                    = imuData.heading;
        statusJson["imuData"]["pitch"]                      = imuData.pitch;
        statusJson["isobusData"]["rearHitchPosition"]       = isobusData.rearHitchPosition;
        statusJson["isobusData"]["rearPtoRpm"]              = isobusData.rearPtoRpm;
        statusJson["transfers"]["NTRIPbps"]                 = ntripbps;
        statusJson["transfers"]["UDPpps"]                   = udppps;
        statusJson["switches"]["workswitch"]                = switches.workSwitch;
        statusJson["switches"]["switchType"]                = (uint8_t)steerConfig.workswitchType;
        statusJson["gnssData"]                              = gnssData.to_json();
        ntripbps = 0;
        udppps = 0;
        if (Serial.dtr())
        {
            serializeJsonPretty(statusJson, Serial);
        }
        digitalToggleFast(13);
        threads.delay(1000);
    }
}

void setup()
{
    delay(1000);
    Serial.println("teensy-aog running!");

    pinMode(13, 1);

    Serial.begin(115200);

    initSD();

    loadSteerConfig();
    loadSteerSettings();
    saveSteerConfig();
    saveSteerSettings();

    setupEthernet();
    setupGNSS();
    setupSensors();
    setupCAN();
    setupAutosteer();

    threads.addThread(heartbeat);
}

void loop()
{
    yield();
    threads.yield();
}