#include "main.h"
#include "can.h"
#include "autosteer.h"
#include "coms.h"
#include "gps.h"
#include "cmps.h"

#include <EEPROM.h>
#define EEP_Ident 5005

Metros metro;

TimingData timingData;
SteerConfig steerConfig;
SteerSettings steerSettings;
SteerSetpoints steerSetpoints;
Switches switches;

uint32_t cycleTimer = 0;


void loadSettings()
{
    //Serial.println("Setting up...");
    uint16_t EEread;
    EEPROM.get(0, EEread);
    if (EEread == EEP_Ident)
    {
        Serial.println("Settings found!");
        EEPROM.get(10, steerSettings);
        EEPROM.get(40, steerConfig);
    }
    else
    {
        Serial.println("Loading default Settings");
        EEPROM.put(0, EEP_Ident);
        EEPROM.put(10, steerSettings);
        EEPROM.put(40, steerConfig);
    }
}

void setup()
{
    pinMode(13, OUTPUT);
    pinMode(9, OUTPUT);

    digitalWrite(9, HIGH);
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(9, LOW);
    digitalWrite(13, LOW);

    Serial.begin(115200);

    loadSettings();

    initCMPS();

    initCAN();
    initGPS();
    initEthernet();

    delay(100);
}

void loop()
{
    timingData.cycleTime = micros() - cycleTimer;
    if (timingData.cycleTime > timingData.maxCycleTime)
    {
        timingData.maxCycleTime = timingData.cycleTime;
    }
    if (timingData.maxCycleTime > 1000000) timingData.maxCycleTime = 0;
    cycleTimer = micros();

    if (metro.resetCycle.check() == 1)
    {
        timingData.maxCycleTime = 0;
    }

    udpWorker();

    gpsWorker();

    cmpsWorker();

    sendCurveCommand();
    checkIsobus();

    printStatus();
}