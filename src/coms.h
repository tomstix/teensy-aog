#define aogPort 8888
#define aogSendPort 9999
#define ntripPort 2233
#define myPort 5577
#define nmeaPort 5544

void initEthernet();
void sendDataToAOG();
void sendNMEA(const char *nmeastring);