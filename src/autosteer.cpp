#include <ArduinoJson.h>

#include "main.h"
#include "autosteer.h"
#include "coms.h"
#include "cmps.h"
#include "gps.h"

void autosteerWorker()
{
    if (benchmode)
    {
        pinMode(14, INPUT);
        int val = analogRead(14);
        steerSetpoints.actualSteerAngle = ((float)map(val, 0, 1023, -400, 400)) * 0.1;
        steerSetpoints.roll = ((float)map(val, 0, 1023, -300, 300)) * 0.1;
    }

    if ((!steerSetpoints.guidanceStatus | ((millis() - steerSetpoints.lastPacketReceived) > 500)))
    {
        if (steerSetpoints.enabled)
        {
            switches.steerSwitch = 1;
        }
        steerSetpoints.enabled = false; //turn off steering
    }
    else //valid conditions to turn on autosteer
    {
        if (!switches.steerSwitch)
        {                                  //steering has been activated by steerSwtich
            steerSetpoints.enabled = true; //enable steering
            timingData.lastEnable = millis();
        }
    }

    switch (steerConfig.PulseCountMax) //using PulseCount Setting to set workswitch type
    {
    case 0:
        switches.workSwitch = !steerSetpoints.enabled;
        break;
    case 1:
        switches.workSwitch = (isobusData.rearHitchPosition * 100 / 255 > (steerSettings.AckermanFix / 2));
        break;
    case 2:
        switches.workSwitch = (isobusData.rearPtoRpm < ptoTreshold);
        break;
    default:
        break;
    }

    sendDataToAOG();
}

void printStatus()
{
    if (metro.printStatus.check() == 1)
    {
        if (Serial.dtr())
        {
            StaticJsonDocument<512> data;
            JsonObject candata = data.createNestedObject("CAN-Data");
            JsonObject general = data.createNestedObject("General");

            candata["mRPM"] = isobusData.motorRpm;
            candata["WhlSpeed"] = isobusData.speed;
            candata["rHitch"] = isobusData.rearHitchPosition;
            candata["rHitchWork"] = isobusData.rearHitchWorking;
            candata["fHitch"] = isobusData.frontHitchPosition;
            candata["rPTO"] = isobusData.rearPtoRpm;
            candata["fPTO"] = isobusData.frontPtoRpm;
            candata["GMSReset"] = isobusData.requestReset;
            candata["GMSReady"] = isobusData.steeringSystemReadiness;
            candata["GMSCurve"] = isobusData.gmsEstimatedCurvature;
            candata["GMSAngle"] = steerSetpoints.actualSteerAngle;
            candata["VBUSCurve"] = vbusData.estCurve;
            candata["PGNrec"] = isobusData.pgn;
            candata["ISO-RX/s"] = isobusData.rxCounter;
            candata["ISO-F0-RX/s"] = isobusData.rxCounterF0;
            candata["V-RX/s"] = vbusData.rxCounter;
            candata["V-TX/s"] = vbusData.txCounter;
            candata["setcurve"] = vbusData.setCurve;


            general["requestAngle"] = steerSetpoints.requestedSteerAngle;
            general["cycletime"] = timingData.cycleTime;
            general["maxcycle"] = timingData.maxCycleTime;
            general["gps/s"] = timingData.gpsCounter;
            general["gpsBytes/s"] = timingData.gpsByteCounter;
            general["seconds"] = gpsData.seconds;
            general["GPS Speed"] = gpsData.speed*1000;
            general["Roll"] = steerSetpoints.roll;
            general["countsSetting"] = steerConfig.PulseCountMax;
            general["steerswitch"] = switches.steerSwitch;
            general["workswitch"] = switches.workSwitch;

            serializeJsonPretty(data, Serial);
            

            /*Serial.print(vbusData.estCurve);
            Serial.print(',');
            Serial.println(isobusData.gmsEstimatedCurvatureRaw);*/
        }
        isobusData.rxCounter = 0;
        isobusData.rxCounterF0 = 0;
        vbusData.txCounter = 0;
        vbusData.rxCounter = 0;
        timingData.gpsCounter = 0;
        timingData.gpsByteCounter = 0;
    }
}
