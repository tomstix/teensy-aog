#include "coms.h"
#include "json.h"
#include "can.h"
#include "gps.h"

#include <QNEthernet.h>
namespace qn = qindesign::network;

uint8_t aogRxBuffer[32];
uint8_t ntripBuffer[512];

qn::EthernetUDP aogUDP;
qn::EthernetUDP ntripUDP;
qn::EthernetUDP sendUDP;
qn::EthernetUDP nmeaUDP;

Threads::Mutex ethLock;

volatile uint16_t udppps = 0;

uint8_t mac[6];
IPAddress ip(192, 168, 1, 177);
IPAddress netmask(255, 255, 255, 0);
IPAddress gateway(192, 168, 1, 1);
IPAddress broadcastIP(192, 168, 1, 255);

uint8_t pgn;
uint8_t length;
uint8_t sendbuffer[14] = {0x80, 0x81, 0x7f, 0xFD, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC};

void sendDataToAOG()
{
    // send FD Message
    int16_t steerAngle = (int16_t)(steerSetpoints.actualSteerAngle * 100);
    sendbuffer[5] = (uint8_t)steerAngle;
    sendbuffer[6] = steerAngle >> 8;
    // heading
    sendbuffer[7] = (uint8_t)9999;
    sendbuffer[8] = 9999 >> 8;

    // roll
    sendbuffer[9] = (uint8_t)8888;
    sendbuffer[10] = 8888 >> 8;

    steerSetpoints.switchByte = 0;
    steerSetpoints.switchByte |= (switches.steerSwitch << 1); // put steerswitch status in bit 1 position
    steerSetpoints.switchByte |= switches.workSwitch;
    sendbuffer[11] = steerSetpoints.switchByte;
    sendbuffer[12] = (uint8_t)abs(steerSetpoints.pidOutput);

    int CK_A = 0;
    for (uint8_t i = 2; i < sizeof(sendbuffer) - 1; i++)
    {
        CK_A = (CK_A + sendbuffer[i]);
    }
    sendbuffer[13] = CK_A;

    sendUDP.send(broadcastIP, aogSendPort, sendbuffer, sizeof(sendbuffer));
}

void sendNMEA(char *nmeastring, size_t size)
{
    ethLock.lock(10);
    sendUDP.send(broadcastIP, aogSendPort, (uint8_t*)nmeastring, size);
    ethLock.unlock();
}

void udpThread()
{
    while (1)
    {
        ethLock.lock(10);
        uint16_t packetsize = aogUDP.parsePacket();
        uint16_t ntripSize = ntripUDP.parsePacket();

        if (packetsize)
        {
            aogUDP.read(aogRxBuffer, sizeof(aogRxBuffer));
            udppps++;
            ethLock.unlock();
            if (aogRxBuffer[0] == 0x80 && aogRxBuffer[1] == 0x81 && aogRxBuffer[2] == 0x7F)
            {
                pgn = aogRxBuffer[3];
                switch (pgn)
                {
                case 0xFE:
                {
                    // Serial.println("FE received");
                    steerSetpoints.speed = ((float)(aogRxBuffer[5] | aogRxBuffer[5] << 8)) * 0.1;
                    steerSetpoints.guidanceStatus = aogRxBuffer[7];
                    steerSetpoints.requestedSteerAngle = (float)((int16_t)(aogRxBuffer[8] | aogRxBuffer[9] << 8)) * 0.01;
                    steerSetpoints.tram = aogRxBuffer[10];
                    steerSetpoints.sections = aogRxBuffer[11] << 8 | aogRxBuffer[12];

                    steerSetpoints.lastPacketReceived = millis();
                    sendDataToAOG();
                    break;
                }

                case 0xEF:
                {
                    steerSetpoints.uTurn = aogRxBuffer[5];

                    steerSetpoints.hydLift = aogRxBuffer[7];
                    steerSetpoints.tram = aogRxBuffer[8];

                    if (steerSetpoints.hydLift != steerSetpoints.hydLiftPrev)
                    {
                        steerSetpoints.hydLiftPrev = steerSetpoints.hydLift;
                        sendGoEnd();
                    }
                    break;
                }

                case 0xFC:
                {
                    // PID values
                    steerSettings.Kp = ((float)aogRxBuffer[5]); // read Kp from AgOpenGPS
                    steerSettings.Kp *= 0.5;

                    steerSettings.highPWM = aogRxBuffer[6];

                    steerSettings.lowPWM = (float)aogRxBuffer[7]; // read lowPWM from AgOpenGPS

                    steerSettings.minPWM = aogRxBuffer[8]; // read the minimum amount of PWM for instant on

                    steerSettings.steerSensorCounts = aogRxBuffer[9] + 200; // sent as setting displayed in AOG

                    steerSettings.wasOffset = (aogRxBuffer[10]); // read was zero offset Hi

                    steerSettings.wasOffset |= (aogRxBuffer[11] << 8); // read was zero offset Lo

                    steerSettings.AckermanFix = (float)(aogRxBuffer[12]) / 100.0;

                    saveSteerSettings();

                    break;
                }

                case 0xFB: // FB - steerConfig
                {
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

                    // Danfoss type hydraulics
                    steerConfig.IsDanfoss = aogRxBuffer[8]; // byte 8

                    if (steerConfig.SteerSwitch)
                    {
                        steerConfig.workswitchType = SteerConfig::WorkswitchType::Hitch;
                    }
                    else if (steerConfig.SteerButton)
                    {
                        steerConfig.workswitchType = SteerConfig::WorkswitchType::PTO;
                    }
                    else
                    {
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
        ethLock.unlock();
        if (ntripSize)
        {
            ethLock.lock();
            ntripUDP.read(ntripBuffer, sizeof(ntripBuffer));
            ethLock.unlock();
            for (int i = 0; i < ntripSize; i++)
            {
                ntripRingBufLock.lock();
                ntripRingBuf.push(ntripBuffer[i]);
                ntripRingBufLock.unlock();
            }
        }
        ethLock.unlock();
        threads.yield();
    }
}

void setupEthernet()
{
    Serial.println("Starting Ethernet!");
    qn::Ethernet.setHostname("teensy-aog");
    qn::Ethernet.begin(ip, netmask, gateway);

    if (!qn::Ethernet.waitForLink(5000))
    {
        Serial.println("Ethernet Link was not detected!");
    }
    else
    {
        Serial.print("Ethernet running at ");
        Serial.print(qn::Ethernet.linkSpeed());
        Serial.println(" Mbps.");
    }

    // start UDP
    aogUDP.begin(aogPort);
    ntripUDP.begin(ntripPort);
    sendUDP.begin(myPort);
    nmeaUDP.begin(nmeaPort);

    threads.addThread(udpThread);
}