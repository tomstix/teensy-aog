// 
// 
// 
#include <ArduinoJson.h>
#include "teensy-aog.h"
#include "autosteer.h"
#include "can.h"

TimingData timingData;
AOGSetup aogSettings;
SteerSettings steerSettings;
SteerSetpoints steerSetpoints;
Switches switches;

void autosteerWorker()
{

    if ((steerSetpoints.distanceFromLine == 32020) | (steerSetpoints.distanceFromLine == 32000) | (steerSetpoints.speed < aogSettings.minSteerSpeed) | (steerSetpoints.speed > aogSettings.maxSteerSpeed) | ((millis() - steerSetpoints.lastPacketReceived) > 500))
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
            //switches.steerSwitch = 0;
            steerSetpoints.enabled = true;
            timingData.lastEnable = millis();
        }
    }

    steerSetpoints.requestedSteerAngle = (steerSetpoints.requestedSteerAngle * aogSettings.AckermanFix + steerSetpoints.previousAngle * (100 - aogSettings.AckermanFix)) / 100.0;

    switch (aogSettings.InclinometerInstalled) //using inclino Setting to set workswitch type
    {
    case 0:
        switches.workSwitch = !steerSetpoints.enabled;
        break;
    case 1:
        switches.workSwitch = (isobusData.rearHitchPosition > 128);
        break;
    case 2:
        switches.workSwitch = (isobusData.rearPtoRpm < 80);
        break;
    default:
        break;
    }
}

void printStatus()
{
    unsigned long currentMillis = millis();
    if (currentMillis - timingData.lastprintStatus > timingData.printStatus)
    {
        timingData.lastprintStatus = millis();
        if (Serial.dtr())
        {
            StaticJsonDocument<512> data;
            JsonObject candata = data.createNestedObject("CAN-Data");

            candata["mRPM"] = isobusData.motorRpm;
            candata["WhlSpeed"] = isobusData.speed;
            candata["rHitch"] = isobusData.rearHitchPosition;
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
            digitalToggle(13);

            /*
            Serial.println("CAN-Data:");
            Serial.print("Motor RPM:\t"); Serial.println(isobusData.motorRpm);
            Serial.print("Wheelspeed:\t"); Serial.println(isobusData.speed);
            Serial.print("Rear Hitch:\t"); Serial.println(isobusData.rearHitchPosition);
            Serial.print("Front Hitch:\t"); Serial.println(isobusData.frontHitchPosition);
            Serial.print("Rear PTO:\t"); Serial.println(isobusData.rearPtoRpm);
            Serial.print("Front PTO:\t"); Serial.println(isobusData.frontPtoRpm);
            Serial.print("GMS Request Reset:\t"); Serial.println(isobusData.requestReset, BIN);
            Serial.print("GMS Readiness:\t"); Serial.println(isobusData.steeringSystemReadiness, BIN);
            Serial.print("GMS Curve:\t"); Serial.println(isobusData.gmsEstimatedCurvature, HEX);
            Serial.print("VBUS Curve:\t"); Serial.println(vbusData.estCurve, HEX);
            Serial.print("Last received PGN:\t"); Serial.println(isobusData.pgn);
            Serial.print("Iso-Msg Rx (1s):\t"); Serial.println(isobusData.rxCounter);
            Serial.print("Iso-F0-Msg Rx (1s):\t"); Serial.println(isobusData.rxCounterF0);
            Serial.print("V-Msg Rx (1s):\t"); Serial.println(vbusData.rxCounter);
            Serial.print("V-Msg Tx (1s):\t"); Serial.println(vbusData.txCounter);*/

            serializeJsonPretty(data, Serial);
            //Serial.flush();
            Serial.send_now();
        }
        isobusData.rxCounter = 0;
        isobusData.rxCounterF0 = 0;
        vbusData.txCounter = 0;
        vbusData.rxCounter = 0;
    }
}
