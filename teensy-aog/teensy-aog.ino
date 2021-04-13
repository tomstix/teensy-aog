/*
 Name:		teensy_aog.ino
 Created:	10.03.2021 11:27:03
 Author:	Tom Stirnkorb
*/

#include "teensy-aog.h"
#include "can.h"
#include "autosteer.h"
#include "coms.h"
#include "gps.h"
#include "cmps.h"
#include <EEPROM.h>
#define EEP_Ident 4310

Metros metro;

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
    pinMode(11, OUTPUT);

    digitalWrite(11, HIGH);
    digitalWrite(13, HIGH);
    delay(2000);
    digitalWrite(11, LOW);
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

    sendCurveCommand();
    checkIsobus();

    printStatus();

    //delayMicroseconds(100);
}
