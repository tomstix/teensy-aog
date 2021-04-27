#include "gps.h"
#include "cmps.h"
#include "coms.h"
#include "autosteer.h"

#include <MicroNMEA.h>

GPSData gpsData;

uint8_t nmeaBuffer[100];
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
				autosteerWorker();
				digitalToggle(13);
			}
			else if (id == "VTG") {
				sendNMEA(nmea.getSentence());
			}
		}
	}
}

void initGPS()
{
	GPS.begin(115200);
}