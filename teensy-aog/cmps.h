// cmps.h

#ifndef _CMPS_h
#define _CMPS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#endif

void initCMPS();
void cmpsWorker();
int16_t requestTwoSigned(int, int);
int8_t requestSingle(int, int);
int requestBytes(int, int, int);