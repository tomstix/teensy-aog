#include "gps.h"
#include "coms.h"

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <USBHost_t36.h>
#include <panda.h>

USBHost myusb;
USBSerial USB_HOST(myusb);

SFE_UBLOX_GNSS GNSS;

void pvtCallback(UBX_NAV_PVT_data_t *pvt)
{
    char nmea[100];
    makePANDAfromPVT(pvt, nmea, imuData.heading, imuData.roll, imuData.pitch);
    sendNMEA(nmea, strlen(nmea));
}

void gnssThread()
{
    while (1)
    {
        GNSS.checkUblox();
        GNSS.checkCallbacks();
        threads.yield();
    }
}

void setupGNSS()
{
    GPS_PORT.begin(115200);

    Serial.println("Setting up GPS!");
    if (GNSS.begin(GPS_PORT) == false)
    {
        Serial.println("U-Blox GPS not found on Serial. Trying USB.");
        if (GNSS.begin(USB_HOST) == false)
        {
            Serial.println(F("u-blox GNSS not detected. Please check wiring. Freezing."));
            while (1)
                ;
        }
    }
    Serial.println("GPS found!");

    GNSS.setNavigationFrequency(10);
    GNSS.setAutoPVTcallbackPtr(pvtCallback);

    threads.addThread(gnssThread);
}