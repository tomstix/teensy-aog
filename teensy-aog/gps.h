// gps.h

#ifndef _GPS_h
#define _GPS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#endif

void gpsWorker();
void initGPS();
void sendNtrip(char buffer[], size_t size);