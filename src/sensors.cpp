#include "sensors.h"

#include <Wire.h>
#include <SPI.h>
#include <ADS1115_WE.h>
#include <Adafruit_BNO08x.h>

#define PIN_HIT 41
#define PIN_HIT_ONOFF 40
#define PIN_PTO 39
#define PIN_WHL 37

volatile uint16_t whlCounts = 0;
volatile uint16_t ptoCounts = 0;

Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

ADS1115_WE adc = ADS1115_WE();

// DIN9684 interrupt handlers
void isrPTO()
{
    ptoCounts++;
}
void isrWHL()
{
    whlCounts++;
}

void setBNOReports(void)
{
    Serial.println("Setting desired reports");
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR))
    {
        Serial.println("Could not enable game vector");
    }
}

struct euler_t
{
    float yaw;
    float pitch;
    float roll;
} ypr;

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

void cmpsWorker()
{
    Serial.println("CMPS Worker started!");
    while (1)
    {
        imuData.headingInt = requestBytes(CMPSAddress, 0x02, 2);
        imuData.rollInt = requestTwoSigned(CMPSAddress, 0x1C);
        imuData.pitchInt = requestTwoSigned(CMPSAddress, 0x1A);
        imuData.heading = ((float)imuData.headingInt) / 10.0;
        imuData.roll = ((float)imuData.rollInt) / 10.0;
        imuData.pitch = ((float)imuData.pitchInt) / 10.0;
        threads.delay(10);
    }
}

void bnoWorker()
{
    Serial.println("BNO Worker started!");
    while (1)
    {

        if (bno08x.wasReset())
        {
            setBNOReports();
        }

        if (bno08x.getSensorEvent(&sensorValue))
        {
            // DBG("BNO Value received!");
            switch (sensorValue.sensorId)
            {
            case SH2_GAME_ROTATION_VECTOR:
                quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
                imuData.heading = 180 - ypr.yaw;
                imuData.roll = ypr.roll;
                imuData.pitch = ypr.pitch;
                break;
            }
        }
    }
}

int16_t adsWorker()
{
    int16_t res = adc.getRawResult();
    return res;
}

void dinSignalWorker()
{
    while (1)
    {
        isobusData.rearHitchPosition = analogRead(PIN_HIT) / 10;
        isobusData.rearPtoRpm = ptoCounts / 40 * 60;
        ptoCounts = 0;
        whlCounts = 0;
        threads.delay(1000);
    }
}

uint8_t setupSensors()
{
    Wire.begin();

    Wire.setClock(400000);

    if (!adc.init())
    {
        Serial.println("Failed to init ADS!");
        steerConfig.wasType = SteerConfig::WASType::CAN;
        steerConfig.outputType = SteerConfig::OutputType::FendtCAN;
    }
    else
    {
        Serial.println("Setting up ADS!");
        adc.setVoltageRange_mV(ADS1115_RANGE_4096);
        adc.setConvRate(ADS1115_128_SPS);
        adc.setMeasureMode(ADS1115_CONTINOUS);
        adc.setCompareChannels(ADS1115_COMP_1_GND);
        // adc.setCompareChannels(ADS1115_COMP_0_1);
        steerConfig.wasType = SteerConfig::WASType::ADS1115;
        steerConfig.outputType = SteerConfig::OutputType::PWM;

        pinMode(PIN_HIT, INPUT);
        pinMode(PIN_HIT_ONOFF, INPUT);
        pinMode(PIN_PTO, INPUT);
        pinMode(PIN_WHL, INPUT);
        attachInterrupt(digitalPinToInterrupt(PIN_WHL), isrWHL, RISING);
        attachInterrupt(digitalPinToInterrupt(PIN_PTO), isrPTO, RISING);
        threads.addThread(dinSignalWorker);
    }

    if (!bno08x.begin_I2C())
    {
        Serial.println("Failed to find BNO08x!");

        if (Wire.requestFrom(CMPSAddress, 1) > 0)
        {
            uint8_t reading = Wire.read();
            Serial.print("CMPS Software Version: ");
            Serial.println(reading);
            steerConfig.imuType = SteerConfig::ImuType::CMPS14;
            threads.addThread(cmpsWorker);
            return 0;
        }
        else
        {
            Serial.println("CMPS init failed!");
            steerConfig.imuType = SteerConfig::ImuType::None;
            return 0;
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

        setBNOReports();

        threads.addThread(bnoWorker);
        return 0;
    }
    return -1;
}