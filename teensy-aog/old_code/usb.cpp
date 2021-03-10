/*#include "main.hpp"

#define aogVersion 17

byte gpsSendBuffer[63];

byte usbRxBuffer[64];
int8_t packageLength;
int16_t aogPgn;
uint16_t ffeCounter = 0;
uint16_t ffaCounter = 0;
uint16_t ffbCounter = 0;

byte checksum;

uint16_t heartbeatRate = 1000;
uint16_t lastHeartbeat = 0;

void sendDataToAOG()
{
    int16_t temp = (int16_t)(steerSetpoints.actualSteerAngle * 100);
    byte toSend[64];
    toSend[0] = 10;
    toSend[1] = 0x7F;
    toSend[2] = 0xFD;
    toSend[3] = highByte(temp);
    toSend[4] = lowByte(temp);
    temp = 0;
    if (aogSettings.BNOInstalled)
    {
        temp = (int16_t)(steerSetpoints.heading * 16);
    }
    toSend[5] = highByte(temp);
    toSend[6] = lowByte(temp);
    temp = 0;
    if (aogSettings.InclinometerInstalled)
    {
        temp = (int16_t)(steerSetpoints.roll * 16);
    }
    toSend[7] = highByte(temp);
    toSend[8] = lowByte(temp);
    steerSetpoints.switchByte = 0;
    steerSetpoints.switchByte |= switches.steerSwitch << 1;
    steerSetpoints.switchByte |= switches.workSwitch;

    toSend[9] = steerSetpoints.switchByte;
    toSend[10] = steerSetpoints.pwm;
    for (int i = 11; i < 64; i++)
    {
        toSend[i] = 0;
    }

    if (!(RawHID.send(toSend, 100) > 0))
    {
        Serial.println("Failed to send!");
    }
}

void SendTwoThirty(byte check)
{
    byte toSend[64];
    toSend[0] = 10;
    toSend[1] = 127;
    toSend[2] = 230;
    toSend[3] = check;
    toSend[4] = aogVersion;
    toSend[5] = 0;
    toSend[6] = 0;
    toSend[7] = 0;
    toSend[8] = 0;
    toSend[9] = 0;

    //version to match in AOG
    toSend[10] = 0;

    for (int i = 11; i < 64; i++)
    {
        toSend[i] = 0;
    }

    //off to AOG
    if (!(RawHID.send(toSend, 100) > 0))
    {
        Serial.println("Failed to send!");
    }
}

void usbWorker()
{
    if (RawHID.recv(usbRxBuffer, 5)) //check for received Data
    {
        Serial.println("USB Data received!");
        packageLength = usbRxBuffer[0];

        if (usbRxBuffer[1] == 0x7F) //Data from AOG
        {

            aogPgn = (usbRxBuffer[1] << 8) | usbRxBuffer[2];

            switch (aogPgn)
            {
            case 0x7FFE:
            {
                ffeCounter++;
                steerSetpoints.speed = usbRxBuffer[4] * 0.25;
                steerSetpoints.distanceFromLine = (usbRxBuffer[5] << 8 | usbRxBuffer[6]);
                steerSetpoints.requestedSteerAngle = (float)((int16_t)(usbRxBuffer[7] << 8 | usbRxBuffer[8])) * 0.01;

                if ((steerSetpoints.distanceFromLine == 32020) | (steerSetpoints.distanceFromLine == 32000) | (steerSetpoints.speed < aogSettings.minSteerSpeed) | (steerSetpoints.speed > aogSettings.maxSteerSpeed))
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
                Serial.println(switches.steerSwitch);
                autosteerWorker();
                if (aogSettings.BNOInstalled)
                {
                    cmpsWorker();
                }
                sendDataToAOG();
                break;
            }

            case 0x7FFA:
            {
                ffaCounter++;
                steerSetpoints.sections = (usbRxBuffer[3] << 8) | usbRxBuffer[4];
                steerSetpoints.uTurn = usbRxBuffer[6];
                break;
            }

            case 0x7FFB:
            {
                ffbCounter++;
                byte sett = usbRxBuffer[3];
                if (bitRead(sett, 0))
                    aogSettings.InvertWAS = 1;
                else
                    aogSettings.InvertWAS = 0;
                if (bitRead(sett, 1))
                    aogSettings.InvertRoll = 1;
                else
                    aogSettings.InvertRoll = 0;
                if (bitRead(sett, 2))
                    aogSettings.MotorDriveDirection = 1;
                else
                    aogSettings.MotorDriveDirection = 0;
                if (bitRead(sett, 3))
                    aogSettings.SingleInputWAS = 1;
                else
                    aogSettings.SingleInputWAS = 0;
                if (bitRead(sett, 4))
                    aogSettings.CytronDriver = 1;
                else
                    aogSettings.CytronDriver = 0;
                if (bitRead(sett, 5))
                    aogSettings.SteerSwitch = 1;
                else
                    aogSettings.SteerSwitch = 0;
                if (bitRead(sett, 6))
                    aogSettings.UseMMA_X_Axis = 1;
                else
                    aogSettings.UseMMA_X_Axis = 0;
                if (bitRead(sett, 7))
                    aogSettings.ShaftEncoder = 1;
                else
                    aogSettings.ShaftEncoder = 0;

                //set1
                sett = usbRxBuffer[4];
                if (bitRead(sett, 0))
                    aogSettings.BNOInstalled = 1;
                else
                    aogSettings.BNOInstalled = 0;
                if (bitRead(sett, 1))
                    aogSettings.isRelayActiveHigh = 1;
                else
                    aogSettings.isRelayActiveHigh = 0;

                aogSettings.maxSteerSpeed = usbRxBuffer[5]; //actual speed
                aogSettings.minSteerSpeed = usbRxBuffer[6];

                sett = usbRxBuffer[7];
                aogSettings.InclinometerInstalled = sett & 192;
                aogSettings.InclinometerInstalled = aogSettings.InclinometerInstalled >> 6;
                aogSettings.PulseCountMax = sett & 63;

                aogSettings.AckermanFix = usbRxBuffer[8];

                checksum = 0;
                for (int i = 2; i < 10; i++)
                    checksum += usbRxBuffer[i + 1];

                //send udpData back - version number.
                SendTwoThirty((byte)checksum);

                EEPROM.put(40, aogSettings);

                Serial.print("New settings received! Checksum: ");
                Serial.println(checksum);
                break;
            }

            case 0x7FFC:
            {
                steerSettings.Kp = (float)usbRxBuffer[3];            // read Kp from AgOpenGPS
                steerSettings.lowPWM = (float)usbRxBuffer[4];        // read lowPWM from AgOpenGPS
                steerSettings.Kd = (float)usbRxBuffer[5] * 1.0;      // read Kd from AgOpenGPS
                steerSettings.Ko = (float)usbRxBuffer[6] * 0.1;      // read Ko from AgOpenGPS
                steerSettings.steeringPositionZero = usbRxBuffer[7]; //read steering zero offset

                steerSettings.minPWM = usbRxBuffer[8];  //read the minimum amount of PWM for instant on
                steerSettings.highPWM = usbRxBuffer[9]; //
                steerSettings.steerSensorCounts = usbRxBuffer[10];

                checksum = 0;
                for (int i = 2; i < 10; i++)
                    checksum += usbRxBuffer[i + 1];

                //send udpData back - version number.
                SendTwoThirty((byte)checksum);

                EEPROM.put(10, steerSettings);
                break;
            }

            default:
            {
                break;
            }
            }
        }
    }
}*/