#pragma once

#include "main.h"

void sendDataToAOG();
void sendNMEA(const char* nmeastring);
void udpWorker();
void initEthernet();