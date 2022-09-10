#pragma once

#define GPS_PORT Serial4

#include "main.h"

#include <CircularBuffer.h>

extern CircularBuffer<uint8_t, 512> ntripRingBuf;
extern Threads::Mutex ntripRingBufLock;

void setupGNSS();
void addNTRIP();