#pragma once

#include "main.h"

#define aogPort 8888
#define aogSendPort 9999
#define ntripPort 2233
#define myPort 5577
#define nmeaPort 5544

void setupEthernet();
void sendDataToAOG();
void sendNMEA(char *nmeastring, size_t size);