#include "gps.h"
#include "imu.h"
#include "coms.h"
#include "autosteer.h"

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
SFE_UBLOX_GNSS myGNSS;

GPSData gpsData;

#include <MicroNMEA.h>
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

void gpsWorker()
{
	myGNSS.checkUblox();

	if(metro.gps.check() == 1)
	{
		//gpsData.acc = myGNSS.getPositionAccuracy() / 10.0;
	}
}

void initGPS()
{
	GPS.begin(230400);
	if(!myGNSS.begin(GPS))
	{
		Serial.println("GPS not found at definded Port and Baudrate! Freezing.");
		while(1);
	}
	else Serial.println("GPS found!");

	myGNSS.setUART1Output(COM_TYPE_NMEA | COM_TYPE_UBX);
	myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
	myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_ALL);
}

void SFE_UBLOX_GNSS::processNMEA(char incoming)
{
	//timingData.gpsByteCounter++;
	if (nmea.process(incoming)) {
			String id = nmea.getMessageID();
			if (id == "GGA") {
				timingData.gpsCounter++;
				//sendNMEA(nmea.getSentence());
				//autosteerWorker();
				//Serial.println(nmea.getSentence());
				digitalToggle(13);
				imuWorker();
			}
			else if (id == "VTG") {
				//sendNMEA(nmea.getSentence());
				//Serial.println(nmea.getSentence());
			}
		}
}