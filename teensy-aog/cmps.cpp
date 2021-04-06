// 
// 
// 

#include "cmps.h"

#include <Wire.h>

#include "teensy-aog.h"

void initCMPS()
{
    Wire.begin();

    if (Wire.requestFrom(CMPSAddress, 1) > 0)
    {
        byte reading = Wire.read();
        SerialUSB2.print("CMPS Software Version: ");
        SerialUSB2.println(reading);
        steerSetpoints.useCMPS = true;
    }
    else
    {
        SerialUSB2.println("CMPS init failed!");
        steerSetpoints.useCMPS = false;
    }
}

byte readValue[27];

int requestBytes(int address, int readReg, int numBytes)
{
    int value = 0;
    Wire.beginTransmission(address);
    Wire.write(readReg);
    Wire.endTransmission();
    Wire.requestFrom(address, numBytes);
    for (int i = 0; i < numBytes; i++)
    {
        value = value << 8 | Wire.read();
    }
    return value;
}

int8_t requestSingle(int address, int readReg)
{
    Wire.beginTransmission(address);
    Wire.write(readReg);
    Wire.endTransmission();
    Wire.requestFrom(address, 1);
    return Wire.read();
}

int16_t requestTwoSigned(int address, int readReg)
{
    Wire.beginTransmission(address);
    Wire.write(readReg);
    Wire.endTransmission();
    Wire.requestFrom(address, 2);
    int16_t value = (Wire.read() << 8) | Wire.read();
    return value;
}

void cmpsWorker()
{
    steerSetpoints.headingInt = requestBytes(CMPSAddress, 0x02, 2);
    steerSetpoints.rollInt = requestTwoSigned(CMPSAddress, 0x1C);
    steerSetpoints.heading = ((float)steerSetpoints.headingInt) / 10.0;
    steerSetpoints.roll = ((float)steerSetpoints.rollInt) / 10.0;
}

