#include "gps.h"
#include "cmps.h"
#include "teensy-aog.h"
#include "coms.h"
#include <MicroNMEA.h>

GPSData gpsData;

byte nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

uint8_t counter = 0;

void gpsWorker()
{
	while (GPS.available())
	{
		timingData.gpsByteCounter++;
		char c = GPS.read();
		if (nmea.process(c)) {
			String id = nmea.getMessageID();
			if (id == "GGA") {
				timingData.gpsCounter++;
				gpsData.speed = nmea.getSpeed();
				gpsData.seconds = nmea.getSecond();
				sendNMEA(nmea.getSentence());
				if (steerSetpoints.useCMPS)
				{
					cmpsWorker();
				}
				digitalToggle(13);
			}
			else if (id == "VTG") {
				sendNMEA(nmea.getSentence());
			}
		}
	}
}

void sendNtrip(char buffer[], size_t size)
{
	GPS.write(buffer, size);
}

void initGPS()
{
	GPS.begin(115200);
}
