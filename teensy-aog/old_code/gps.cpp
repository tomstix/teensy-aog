#include "main.hpp"
#include <MicroNMEA.h>

byte nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
byte temp;
uint8_t counter = 0;

bool send = true;

void gpsWorker()
{
    while (GPS.available())
    {
        char c = GPS.read();
        SerialUSB2.write(c);
        if (nmea.process(c))
        {
            if (send)   //Only send Data every second NMEA String (GGA & VTG are received)
            {
                send = false;
                if (aogSettings.BNOInstalled)
                {
                    cmpsWorker();
                }
                sendDataToAOG();
            }
            else
                send = true;
        }
    }
    while (SerialUSB2.available())
    {
        GPS.write(SerialUSB2.read());
    }
}

void initGPS()
{
    GPS.begin(115200);
}