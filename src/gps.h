#pragma once

#include "main.h"

void gpsWorker();
void initGPS();
void sendNtrip(char buffer[], size_t size);