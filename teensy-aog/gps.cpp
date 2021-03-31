#include "gps.h"
#include "cmps.h"
#include "teensy-aog.h"
#include "serial.h"
#include <MicroNMEA.h>

GPSData gpsData;

byte nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
byte temp;
uint8_t counter = 0;

uint8_t lastHundreths = 0;

bool send = true;

void gpsWorker()
{
	while (GPS.available())
	{
		SerialUSB2.write(GPS.read());
	}


	if (SerialUSB2.available() > 0)
	{
		uint8_t c = SerialUSB2.read();
		GPS.write(c);
	}
}

void initGPS()
{
	GPS.begin(115200);
}
