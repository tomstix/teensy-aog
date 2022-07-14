#include "gps.h"
#include "coms.h"

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <USBHost_t36.h>
#include <panda.h>

SFE_UBLOX_GNSS GNSS;

CircularBuffer<uint8_t, 512> ntripRingBuf;
Threads::Mutex ntripRingBufLock;

volatile uint16_t ntripbps;

bool gpsFound = false;

GNSSData gnssData;

char nmea[200];

void pvtCallback(UBX_NAV_PVT_data_t *pvt)
{
    gnssData.lastGPSFix = millis();
    makePANDAfromPVT(pvt, nmea, imuData.heading, imuData.roll, imuData.pitch);
    sendNMEA(nmea, strlen(nmea));
    gnssData.lat = (double)pvt->lat / 10000000;
    gnssData.lon = (double)pvt->lon / 10000000;
    gnssData.fixType = pvt->fixType;
    gnssData.numSV = pvt->numSV;
    gnssData.hAcc = pvt->hAcc;
    gnssData.vAcc = pvt->vAcc;
    gnssData.gSpeed = pvt->gSpeed;
}

void gnssThread()
{
    Serial.println("GNSS Thread is running!");
    while (1)
    {
        GNSS.checkUblox();
        GNSS.checkCallbacks();
        ntripRingBufLock.lock();
        uint8_t i;
        while (!ntripRingBuf.isEmpty() && GPS_PORT.availableForWrite())
        {
            i = ntripRingBuf.first();
            GPS_PORT.write(i);
            ntripbps++;
            ntripRingBuf.shift();
        }
        ntripRingBufLock.unlock();
        threads.yield();
    }
}

void setupGNSS()
{
    GPS_PORT.begin(115200);
    Serial.println("Setting up GPS!");
    if (GNSS.begin(GPS_PORT) == true)
    {
        Serial.println("GPS found!");
        GNSS.setUART1Output(COM_TYPE_UBX);
        GNSS.setNavigationFrequency(10);
        GNSS.setAutoPVTcallbackPtr(pvtCallback);
        threads.addThread(gnssThread);
    }
    else
    {
        Serial.println(F("u-blox GNSS not detected. Please check wiring."));
    }
}