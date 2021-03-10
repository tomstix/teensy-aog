// serial.h

#ifndef _SERIAL_h
#define _SERIAL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#endif

void sendDataToAOG();
void SendTwoThirty(byte);
void serialWorker();
void initSerial();