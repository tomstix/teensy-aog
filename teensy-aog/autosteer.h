// autosteer.h

#ifndef _AUTOSTEER_h
#define _AUTOSTEER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#endif

void autosteerWorker();
void printStatus();