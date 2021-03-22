// 
// 
// 

#include "serial.h"

#include "teensy-aog.h"
#include "autosteer.h"

#include <EEPROM.h>

void sendDataToAOG()
{
    SerialUSB1.print("127,253,");
    SerialUSB1.print((int)(steerSetpoints.actualSteerAngle * 100)); //The actual steering angle in degrees
    SerialUSB1.print(",");
    SerialUSB1.print((int)(steerSetpoints.requestedSteerAngle * 100));
    SerialUSB1.print(",");
    if (aogSettings.BNOInstalled)
    {
        SerialUSB1.print((int)(steerSetpoints.heading * 16));
    }
    else
        SerialUSB1.print("0");
    SerialUSB1.print(",");

    if (aogSettings.BNOInstalled)
    {
        SerialUSB1.print((int)(steerSetpoints.roll * 16));
    }
    else
        SerialUSB1.print("0");
    SerialUSB1.print(",");

    steerSetpoints.switchByte = 0;
    steerSetpoints.switchByte |= switches.steerSwitch << 1;
    steerSetpoints.switchByte |= switches.workSwitch;

    SerialUSB1.print(steerSetpoints.switchByte);
    SerialUSB1.print(",");
    SerialUSB1.print(steerSetpoints.pwm);
    SerialUSB1.println(",0,0");
    SerialUSB1.flush();
}

void SendTwoThirty(byte check)
{
    //Serial Send to agopenGPS **** you must send 10 numbers ****
    SerialUSB1.print("127,230,");
    SerialUSB1.print(check);
    SerialUSB1.print(",");

    //version in AOG ex. v4.1.13 -> 4+1+13=18
    SerialUSB1.print(aogVersion);

    SerialUSB1.println(",0,0,0,0,0,0");

    SerialUSB1.flush();
}

bool aogHeaderFound = false;
int aogPgn;

void serialWorker()
{
    unsigned long currentMillis = millis();
    if (currentMillis - timingData.lastSerialWorker > timingData.serialWorker)
    {
        while (SerialUSB1.available() && !aogHeaderFound)
        {
            if (SerialUSB1.read() == 0x7F)
            {
                aogHeaderFound = true;
                //Serial.println("AOG Header found!");
            }
        }

        if (SerialUSB1.available() > 8)
        {
            aogHeaderFound = false;
            aogPgn = SerialUSB1.read();

            //Serial.println(aogPgn,HEX);

            switch (aogPgn)
            {
            case 0xFE:
            {
                //was section control lo byte
                SerialUSB1.read();
                steerSetpoints.speed = SerialUSB1.read() * 0.25; //actual speed times 4, single byte

                //distance from the guidance line in mm
                steerSetpoints.distanceFromLine = (float)(SerialUSB1.read() << 8 | SerialUSB1.read()); //high,low bytes

                //set point steer angle * 100 is sent
                steerSetpoints.requestedSteerAngle = ((float)(SerialUSB1.read() << 8 | SerialUSB1.read())) * 0.01; //high low bytes

                SerialUSB1.read();
                SerialUSB1.read();
                steerSetpoints.lastPacketReceived = millis();
                autosteerWorker();
                break;
            }
            case 0xFA:
            {
                steerSetpoints.sections = SerialUSB1.read() << 8 | SerialUSB1.read();
                SerialUSB1.read();
                steerSetpoints.uTurn = SerialUSB1.read();

                SerialUSB1.read();
                steerSetpoints.hydLift = SerialUSB1.read();
                SerialUSB1.read();
                SerialUSB1.read();
                break;
            }
            case 0xFB:
            {
                byte checksum = 0;
                byte reed = 0;

                reed = SerialUSB1.read();
                checksum += reed;
                byte sett = reed; //setting0
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
                reed = SerialUSB1.read();
                checksum += reed;
                sett = reed; //setting1
                if (bitRead(sett, 0))
                    aogSettings.BNOInstalled = 1;
                else
                    aogSettings.BNOInstalled = 0;
                if (bitRead(sett, 1))
                    aogSettings.isRelayActiveHigh = 1;
                else
                    aogSettings.isRelayActiveHigh = 0;

                reed = SerialUSB1.read();
                checksum += reed;
                aogSettings.maxSteerSpeed = reed; //actual speed

                reed = SerialUSB1.read();
                checksum += reed;
                aogSettings.minSteerSpeed = reed;

                reed = SerialUSB1.read();
                checksum += reed;
                byte inc = reed;
                aogSettings.InclinometerInstalled = inc & 192;
                aogSettings.InclinometerInstalled = aogSettings.InclinometerInstalled >> 6;
                aogSettings.PulseCountMax = inc & 63;

                reed = SerialUSB1.read();
                checksum += reed;
                aogSettings.AckermanFix = reed;

                reed = SerialUSB1.read();
                checksum += reed;

                //send usbData back - version number etc.
                Serial.print("Settings received! Checksum: ");
                Serial.println(checksum, DEC);
                //SendTwoThirty((byte)checksum);

                EEPROM.put(40, aogSettings);
                break;
            }
            case 0xFC:
            {
                byte checksum = 0;
                byte reed = 0;

                //change the factors as required for your own PID values
                reed = SerialUSB1.read();
                checksum += reed;
                steerSettings.Kp = ((float)reed); // read Kp from AgOpenGPS

                reed = SerialUSB1.read();
                checksum += reed;
                steerSettings.lowPWM = (float)reed; // read lowPWM from AgOpenGPS

                reed = SerialUSB1.read();
                checksum += reed;
                steerSettings.Kd = (float)reed; // read Kd from AgOpenGPS

                reed = SerialUSB1.read();
                checksum += reed;
                steerSettings.Ko = (float)reed; // read  from AgOpenGPS

                reed = SerialUSB1.read();
                checksum += reed;
                steerSettings.steeringPositionZero = reed; //read steering zero offset

                reed = SerialUSB1.read();
                checksum += reed;
                steerSettings.minPWM = reed; //read the minimum amount of PWM for instant on

                reed = SerialUSB1.read();
                checksum += reed;
                steerSettings.highPWM = reed; //

                reed = SerialUSB1.read();
                checksum += reed;
                steerSettings.steerSensorCounts = reed; //sent as 10 times the setting displayed in AOG

                //send usbData back - version number.
                Serial.print("Settings received! Checksum: ");
                Serial.println(checksum, DEC);
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
}

void initSerial()
{
    SerialUSB1.begin(115200);
}
