#include "gps.h"
#include "coms.h"

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <USBHost_t36.h>
#include <panda.h>

SFE_UBLOX_GNSS GNSS;

CircularBuffer<uint8_t, 512> ntripRingBuf;
Threads::Mutex ntripRingBufLock;

volatile uint16_t ntripbps;

void pvtCallback(UBX_NAV_PVT_data_t *pvt)
{
    char nmea[200];
    makePANDAfromPVT(pvt, nmea, imuData.heading, imuData.roll, imuData.pitch);
    sendNMEA(nmea, strlen(nmea));
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
            //Serial.write(i);
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
    if (GNSS.begin(GPS_PORT) == false)
    {
        Serial.println(F("u-blox GNSS not detected. Please check wiring. Freezing."));
        while (1)
            ;
    }
    Serial.println("GPS found!");

    GNSS.setUART1Output(COM_TYPE_UBX);
    GNSS.setNavigationFrequency(10);
    GNSS.setAutoPVTcallbackPtr(pvtCallback);

    threads.addThread(gnssThread);
}