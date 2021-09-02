#include "main.h"
#include "coms.h"
#include "json.h"
#include "can.h"

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

QueueHandle_t gpsQueue;

SemaphoreHandle_t udpMutex;

uint8_t mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 137, 177);

IPAddress broadcastIP(192, 168, 137, 255);

uint8_t aogRxBuffer[14];
uint8_t ntripBuffer[512];

EthernetUDP aogUDP;
EthernetUDP ntripUDP;
EthernetUDP sendUDP;
EthernetUDP nmeaUDP;

uint8_t pgn;
uint8_t length;
uint8_t sendbuffer[14] = {0x80, 0x81, 0x7f, 0xFD, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC};

uint8_t tempHeader = 0;

bool isHeaderFound;
bool isPGNFound;

void sendDataToAOG()
{
    //send FD Message
    int16_t steerAngle = (int16_t)(steerSetpoints.actualSteerAngle * 100);
    sendbuffer[5] = (uint8_t)steerAngle;
    sendbuffer[6] = steerAngle >> 8;
    if (steerConfig.imuType != SteerConfig::ImuType::None)
    {
        uint16_t headingInt = (uint16_t)(imuData.heading * 10.0);
        uint16_t rollInt = (int16_t)(imuData.roll * 10.0);
        sendbuffer[8] = headingInt >> 8;
        sendbuffer[7] = headingInt;
        sendbuffer[10] = rollInt >> 8;
        sendbuffer[9] = rollInt;
    }
    else
    {
        //heading
        sendbuffer[7] = (uint8_t)9999;
        sendbuffer[8] = 9999 >> 8;

        //roll
        sendbuffer[9] = (uint8_t)8888;
        sendbuffer[10] = 8888 >> 8;
    }
    steerSetpoints.switchByte = 0;
    steerSetpoints.switchByte |= (switches.steerSwitch << 1); //put steerswitch status in bit 1 position
    steerSetpoints.switchByte |= switches.workSwitch;
    sendbuffer[11] = steerSetpoints.switchByte;
    sendbuffer[12] = (uint8_t) abs(steerSetpoints.pidOutput);

    int CK_A = 0;
    for (uint8_t i = 2; i < sizeof(sendbuffer) - 1; i++)
    {
        CK_A = (CK_A + sendbuffer[i]);
    }
    sendbuffer[13] = CK_A;

    //DBG("Sending to AOG!");

    xSemaphoreTake(udpMutex, pdMS_TO_TICKS(5));
    sendUDP.beginPacket(broadcastIP, aogSendPort);
    sendUDP.write(sendbuffer, sizeof(sendbuffer));
    sendUDP.endPacket();
    xSemaphoreGive(udpMutex);
}

void sendNMEA(const char *nmeastring)
{
    xSemaphoreTake(udpMutex, pdMS_TO_TICKS(5));
    sendUDP.beginPacket(broadcastIP, aogSendPort);
    sendUDP.println(nmeastring);
    sendUDP.endPacket();
    xSemaphoreGive(udpMutex);
}

void udpWorker(void *arg)
{
    while (1)
    {
        xSemaphoreTake(udpMutex, pdMS_TO_TICKS(5));
        uint16_t packetsize = aogUDP.parsePacket();
        uint16_t ntripSize = ntripUDP.parsePacket();

        if (packetsize)
        {
            aogUDP.read(aogRxBuffer, sizeof(aogRxBuffer));
            if (aogRxBuffer[0] == 0x80 && aogRxBuffer[1] == 0x81 && aogRxBuffer[2] == 0x7F)
            {
                pgn = aogRxBuffer[3];
                switch (pgn)
                {
                case 0xFE:
                {
                    //DBG("FE received");
                    steerSetpoints.speed = ((float)(aogRxBuffer[5] | aogRxBuffer[5] << 8)) * 0.1;
                    steerSetpoints.guidanceStatus = aogRxBuffer[7];
                    steerSetpoints.requestedSteerAngle = (float)((int16_t)(aogRxBuffer[8] | aogRxBuffer[9] << 8)) * 0.01;
                    steerSetpoints.tram = aogRxBuffer[10];
                    steerSetpoints.sections = aogRxBuffer[11] << 8 | aogRxBuffer[12];

                    steerSetpoints.lastPacketReceived = millis();
                    //sendDataToAOG();
                    break;
                }

                case 0xEF:
                {
                    steerSetpoints.uTurn = aogRxBuffer[5];

                    steerSetpoints.hydLift = aogRxBuffer[7];
                    steerSetpoints.tram = aogRxBuffer[8];

                    if ( steerSetpoints.hydLift != steerSetpoints. hydLiftPrev )
                    {
                        steerSetpoints.hydLiftPrev = steerSetpoints.hydLift;
                        sendGoEnd();
                    }
                    break;
                }

                case 0xFC:
                {
                    //PID values
                    steerSettings.Kp = ((float)aogRxBuffer[5]); // read Kp from AgOpenGPS
                    steerSettings.Kp *= 0.5;

                    steerSettings.highPWM = aogRxBuffer[6];

                    steerSettings.lowPWM = (float)aogRxBuffer[7]; // read lowPWM from AgOpenGPS

                    steerSettings.minPWM = aogRxBuffer[8]; //read the minimum amount of PWM for instant on

                    steerSettings.steerSensorCounts = aogRxBuffer[9] + 100; //sent as setting displayed in AOG

                    steerSettings.wasOffset = (aogRxBuffer[10]); //read was zero offset Hi

                    steerSettings.wasOffset |= (aogRxBuffer[11] << 8); //read was zero offset Lo

                    steerSettings.AckermanFix = (float)(aogRxBuffer[12]) / 100.0;

                    saveSteerSettings();

                    DBG("Steer Settings received!");

                    break;
                }

                case 0xFB: //FB - steerConfig
                {
                    DBG("Steer Config received!");
                    uint8_t sett = aogRxBuffer[5];

                    if (bitRead(sett, 0))
                        steerConfig.InvertWAS = 1;
                    else
                        steerConfig.InvertWAS = 0;
                    if (bitRead(sett, 1))
                        steerConfig.IsRelayActiveHigh = 1;
                    else
                        steerConfig.IsRelayActiveHigh = 0;
                    if (bitRead(sett, 2))
                        steerConfig.MotorDriveDirection = 1;
                    else
                        steerConfig.MotorDriveDirection = 0;
                    if (bitRead(sett, 3))
                        steerConfig.SingleInputWAS = 1;
                    else
                        steerConfig.SingleInputWAS = 0;
                    if (bitRead(sett, 4))
                        steerConfig.CytronDriver = 1;
                    else
                        steerConfig.CytronDriver = 0;
                    if (bitRead(sett, 5))
                        steerConfig.SteerSwitch = 1;
                    else
                        steerConfig.SteerSwitch = 0;
                    if (bitRead(sett, 6))
                        steerConfig.SteerButton = 1;
                    else
                        steerConfig.SteerButton = 0;
                    if (bitRead(sett, 7))
                        steerConfig.ShaftEncoder = 1;
                    else
                        steerConfig.ShaftEncoder = 0;

                    steerConfig.PulseCountMax = aogRxBuffer[6];

                    //Danfoss type hydraulics
                    steerConfig.IsDanfoss = aogRxBuffer[8]; //byte 8

                    sett = aogRxBuffer[12];
                    if (bitRead(sett, 0))
                    {
                        if ((sett & 1) == 1)
                            steerConfig.workswitchType = SteerConfig::WorkswitchType::Hitch;
                        else if ((sett & 2) == 2)
                            steerConfig.workswitchType = SteerConfig::WorkswitchType::PTO;
                        else
                            steerConfig.workswitchType = SteerConfig::WorkswitchType::None;
                    }

                    saveSteerConfig();
                    break;
                }
                default:
                {
                    break;
                }
                }
            }
        }
        if (ntripSize)
        {
            ntripUDP.read(ntripBuffer, sizeof(ntripBuffer));
            //DBG("NTRIP Packet received!");
            for (int i = 0; i < ntripSize; i++)
            {
                if (xQueueSend(gpsQueue, &ntripBuffer[i], 0) != pdTRUE)
                {
                    DBG("GPS Queue full!");
                }
                else{
                    ntripBytesIn++;
                }
            }
        }
        xSemaphoreGive(udpMutex);
        taskYIELD();
    }
}

void initEthernet()
{
    DBG("Starting Ethernet!");
    Ethernet.begin(mac, ip);
    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
        DBG("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
        while (true)
        {
            delay(1); // do nothing, no point running without Ethernet hardware
            digitalWrite(13, HIGH);
        }
    }
    if (Ethernet.linkStatus() == LinkOFF)
    {
        DBG("Ethernet cable is not connected.");
    }
    else
        DBG("Ethernet is running!");

    // start UDP
    aogUDP.begin(aogPort);
    ntripUDP.begin(ntripPort);
    sendUDP.begin(myPort);
    nmeaUDP.begin(nmeaPort);

    gpsQueue = xQueueCreate(1024, sizeof(uint8_t));

    udpMutex = xSemaphoreCreateMutex();

    xTaskCreate(udpWorker, NULL, 4096, NULL, 2, NULL);
}