#include "imu.h"
#include "main.h"

#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

struct euler_t
{
    float yaw;
    float pitch;
    float roll;
} ypr;

void setReports(void)
{
    Serial.println("Setting desired reports");
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR))
    {
        Serial.println("Could not enable game vector");
    }
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *ypr, bool degrees = false)
{

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees)
    {
        ypr->yaw *= RAD_TO_DEG;
        ypr->pitch *= RAD_TO_DEG;
        ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t *rotational_vector, euler_t *ypr, bool degrees = false)
{
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

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

void imuWorker()
{
    if (steerConfig.imuType == SteerConfig::ImuType::BNO085)
    {
        if (bno08x.wasReset())
        {
            Serial.print("sensor was reset ");
            setReports();
        }

        if (!bno08x.getSensorEvent(&sensorValue))
        {
            return;
        }

        switch (sensorValue.sensorId)
        {

        case SH2_GAME_ROTATION_VECTOR:
            quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
            steerSetpoints.heading = 180 - ypr.yaw;
            steerSetpoints.roll = ypr.roll;
            steerSetpoints.headingInt = (uint16_t) (steerSetpoints.heading*10.0);
            steerSetpoints.rollInt = (int16_t) (ypr.roll*10.0);
            break;
        }
    }
    else if (steerConfig.imuType == SteerConfig::ImuType::CMPS14)
    {
        steerSetpoints.headingInt = requestBytes(CMPSAddress, 0x02, 2);
        steerSetpoints.rollInt = requestTwoSigned(CMPSAddress, 0x1C);
        steerSetpoints.heading = ((float)steerSetpoints.headingInt) / 10.0;
        steerSetpoints.roll = ((float)steerSetpoints.rollInt) / 10.0;
    }
}

void initIMU()
{
    if (!bno08x.begin_I2C())
    {
        Serial.println("Failed to find BNO08x!");
        Wire.begin();

        if (Wire.requestFrom(CMPSAddress, 1) > 0)
        {
            uint8_t reading = Wire.read();
            Serial.print("CMPS Software Version: ");
            Serial.println(reading);
            steerConfig.imuType = SteerConfig::ImuType::CMPS14;
        }
        else
        {
            Serial.println("CMPS init failed!");
            steerConfig.imuType = SteerConfig::ImuType::None;
        }
    }
    else
    {
        steerConfig.imuType = SteerConfig::ImuType::BNO085;
        Serial.println("Using BNO08x!");

        for (int n = 0; n < bno08x.prodIds.numEntries; n++)
        {
            Serial.print("Part ");
            Serial.print(bno08x.prodIds.entry[n].swPartNumber);
            Serial.print(": Version :");
            Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
            Serial.print(".");
            Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
            Serial.print(".");
            Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
            Serial.print(" Build ");
            Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
        }

        Wire.setClock(400000);

        setReports();
    }
}