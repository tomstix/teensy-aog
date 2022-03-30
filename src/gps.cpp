#include "main.h"
#include "gps.h"
#include "coms.h"

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <USBHost_t36.h>

volatile uint16_t ntripBytesIn = 0;
volatile uint16_t ntripBytesOut = 0;

USBHost myusb;
USBSerial userial(myusb);

SFE_UBLOX_GNSS myGNSS;

uint8_t ntripByte;

void gpsWorker(void *arg)
{
	DBG("GPS running!");
	while (1)
	{
		// write NTRIP from Queue to GPS
		while (GPS.availableForWrite())
		{
			if ((xQueueReceive(gpsQueue, &ntripByte, 0) == pdTRUE))
			{
				GPS.write(ntripByte);
			}
			else
				break;
		}

		myGNSS.checkUblox();
		myGNSS.checkCallbacks();

		taskYIELD();
	}
}

void processPVT(UBX_NAV_PVT_data_t *pvt)
{
	char panda[100];

	uint8_t latDegrees = pvt->lat / 10000000;
	float latMinutes = (((float)pvt->lat / 10000000.0) - (float)latDegrees) * 60.0;

	uint8_t lonDegrees = pvt->lon / 10000000;
	float lonMinutes = (((float)pvt->lon / 10000000.0) - (float)lonDegrees) * 60.0;

	char lonLetter = (pvt->lon > 0) ? 'E' : 'W';
	char latLetter = (pvt->lat > 0) ? 'N' : 'S';

	uint8_t fixType = 0;
	if (pvt->flags.bits.gnssFixOK)
	{
		fixType = 1;
	}
	if (pvt->flags.bits.diffSoln)
	{
		fixType = 2;
	}
	if (pvt->flags.bits.carrSoln == 1)
	{
		fixType = 5;
	}
	if (pvt->flags.bits.carrSoln == 2)
	{
		fixType = 4;
	}

	sprintf(panda, "$PANDA,%02u%02u%02u.%02u,%02u%2.7f,%c,%03u%3.7f,%c,%u,%u,%.1f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%u*",
			pvt->hour,
			pvt->min,
			pvt->sec,
			(uint8_t)((pvt->iTOW % 1000) / 10),
			latDegrees,
			latMinutes,
			latLetter,
			lonDegrees,
			lonMinutes,
			lonLetter,
			fixType,
			pvt->numSV,
			(float)pvt->pDOP * 0.01,
			(float)pvt->hMSL / 1000.0,
			0.0,
			(float)pvt->gSpeed * 0.00194384,
			imuData.heading,
			imuData.roll,
			imuData.pitch,
			0);

	int16_t sum = 0, inx;
	char tmp;

	// The checksum calc starts after '$' and ends before '*'
	for (inx = 1; inx < 200; inx++)
	{
		tmp = panda[inx];
		// * Indicates end of data and start of checksum
		if (tmp == '*')
			break;
		sum ^= tmp; // Build checksum
	}

	sprintf(panda + strlen(panda), "%X\r\n", sum);

	sendNMEA(panda);
}

void initGPS()
{
	DBG("Init GPS!");
	myusb.begin();

	GPS.begin(1000000);
	SerialGPS.begin(115200);

	if (myGNSS.begin(GPS) == true)
	{
		DBG("GPS via USB successfully connected!");
		myGNSS.setUSBOutput(COM_TYPE_UBX);
	}
	else if (myGNSS.begin(SerialGPS) == true)
	{
		DBG("GPS via Serial successfully connected!");
		myGNSS.setUART1Output(COM_TYPE_UBX);
	}
	else
	{
		DBG("Could not connect to GPS");
	}

	myGNSS.setNavigationFrequency(10);
	myGNSS.setAutoPVTcallbackPtr(&processPVT);

	xTaskCreate(gpsWorker, NULL, 4096, NULL, 2, NULL);
}