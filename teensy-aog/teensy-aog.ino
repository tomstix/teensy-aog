/*
 Name:		teensy_aog.ino
 Created:	10.03.2021 11:27:03
 Author:	Tom Stirnkorb
*/

#include "teensy-aog.h"
#include "can.h"
#include "autosteer.h"
#include "serial.h"
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
        SerialUSB2.println("Settings found!");
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
    delay(1000);
    pinMode(13, OUTPUT);

    SerialUSB2.begin(115200);

    loadSettings();

    initCMPS();

    initCAN();
    initGPS();
    initSerial();

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

    serialWorker();

    gpsWorker();

    sendCurveCommand();
    checkIsobus();

    printStatus();

    //delayMicroseconds(100);
}
