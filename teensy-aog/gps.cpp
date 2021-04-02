#include "gps.h"
#include "cmps.h"
#include "teensy-aog.h"
#include "serial.h"
#include <MicroNMEA.h>

GPSData gpsData;

byte nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

uint8_t counter = 0;

bool send = true;

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
					Serial.println(nmea.getSentence());
					if (aogSettings.BNOInstalled)
					{
						cmpsWorker();
					}
					digitalToggle(13);
				}
				else if (id == "VTG") {
					Serial.println(nmea.getSentence());
				}
			}
		}

		int avail = GPS.availableForWrite();
		while (Serial.available() > 0 && counter < avail)
		{
			counter++;
			GPS.write(Serial.read());
		}
		counter = 0;
}

void initGPS()
{
	GPS.begin(115200);
}
