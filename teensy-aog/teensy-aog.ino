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
#include <eeprom.h>
#define EEP_Ident 4310

/*TimingData timingData;
SteerSetpoints steerSetpoints;
AOGSetup aogSettings;
SteerSettings steerSettings;
Switches switches;*/


void loadSettings()
{
    //Serial.println("Setting up...");
    uint16_t EEread;
    EEPROM.get(0, EEread);
    if (EEread == EEP_Ident)
    {
        Serial.println("Settings found!");
        EEPROM.get(10, steerSettings);
        EEPROM.get(40, aogSettings);
    }
    else
    {
        Serial.println("Loading default Settings");
        EEPROM.put(0, EEP_Ident);
        EEPROM.put(10, steerSettings);
        EEPROM.put(40, aogSettings);
    }
}

void setup()
{
    delay(700);

    pinMode(26, INPUT_PULLUP);
    pinMode(13, OUTPUT);

    Serial.begin(115200);

    loadSettings();

    if (aogSettings.BNOInstalled)
    {
        initCMPS();
    }

    initCAN();
    initGPS();
    initSerial();

    delay(100);
}

void loop()
{
    serialWorker();

    gpsWorker();

    sendCurveCommand();
    checkIsobus();

    printStatus();
}
