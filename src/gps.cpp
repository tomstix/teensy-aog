#include "main.h"
#include "gps.h"
#include "coms.h"

#include <MicroNMEA.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <USBHost_t36.h>

volatile uint16_t ntripBytesIn = 0;
volatile uint16_t ntripBytesOut = 0;

USBHost myusb;
USBSerial userial(myusb);

SFE_UBLOX_GNSS myGNSS;

uint8_t nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

uint8_t ntripByte;

void gpsWorker(void *arg)
{
	DBG("GPS running!");
	while (1)
	{
		while (GPS.available())
		{
			char c = GPS.read();
			//Serial.write(c);
			if (nmea.process(c))
			{
				String id = nmea.getMessageID();
				if (id == "GGA")
				{
					sendNMEA(nmea.getSentence());
					sendDataToAOG();
				}
				else if (id == "VTG")
				{
					sendNMEA(nmea.getSentence());
				}
			}
		}
		while (GPS.availableForWrite())
		{
			if ((xQueueReceive(gpsQueue, &ntripByte, 0) == pdTRUE))
			{
				GPS.write(ntripByte);
			}
			else
				break;
		}
		taskYIELD();
	}
}

void initGPS()
{
	DBG("Init GPS!");
	myusb.begin();
	GPS.begin(1000000);

	if (myGNSS.begin(GPS) == true)
	{
		DBG("GPS successfully connected!");
	}
	else
	{
		DBG("Could not connect to GPS");
	}
	myGNSS.setNavigationFrequency(10);

	xTaskCreate(gpsWorker, NULL, 4096, NULL, 2, NULL);
}