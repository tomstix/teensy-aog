// 
// 
// 
#include <ArduinoJson.h>
#include "teensy-aog.h"
#include "autosteer.h"
#include "can.h"
#include "serial.h"
#include "cmps.h"
#include "gps.h"

TimingData timingData;
SteerConfig steerConfig;
SteerSettings steerSettings;
SteerSetpoints steerSetpoints;
Switches switches;

void autosteerWorker()
{
    if (benchmode)
    {
        pinMode(14, INPUT);
        int val = analogRead(14);
        steerSetpoints.actualSteerAngle = ((float)map(val, 0, 1023, -400, 400)) * 0.1;
        steerSetpoints.roll = ((float)map(val, 0, 1023, -300, 300)) * 0.1;
    }

    if ((!steerSetpoints.guidanceStatus | (millis() - steerSetpoints.lastPacketReceived) > 500))
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
        else if (millis() - timingData.lastCutout > 3000) //steering has been activated by AOG - 3s to prevent automatic re-engage
        {
            switches.steerSwitch = 0;
            steerSetpoints.enabled = true;
            timingData.lastEnable = millis();
        }
    }

    switch (steerSettings.steerSensorCounts) //using inclino Setting to set workswitch type
    {
    case 0:
        switches.workSwitch = !steerSetpoints.enabled;
        break;
    case 1:
        switches.workSwitch = (isobusData.rearHitchPosition > 50);
        break;
    case 2:
        switches.workSwitch = (isobusData.rearPtoRpm < 150);
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
        if (SerialUSB2.dtr())
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

            serializeJsonPretty(data, SerialUSB2);
            SerialUSB2.send_now();
        }
        isobusData.rxCounter = 0;
        isobusData.rxCounterF0 = 0;
        vbusData.txCounter = 0;
        vbusData.rxCounter = 0;
        timingData.gpsCounter = 0;
        timingData.gpsByteCounter = 0;
    }
}
