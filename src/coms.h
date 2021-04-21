#ifndef _COMS_h
#define _COMS_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif


#endif

#include <StreamUtils.h>

void sendDataToAOG();
void sendNMEA(const char* nmeastring);
void udpWorker();
void initEthernet();