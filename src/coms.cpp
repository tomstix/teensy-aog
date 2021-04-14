#include "coms.h"

#include "teensy-aog.h"
#include "autosteer.h"
#include "gps.h"

#include <EEPROM.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <RingBuf.h>

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 137, 177);

IPAddress broadcastIP(192, 168, 137, 255);

unsigned int aogPort = 8888;
unsigned int aogSendPort = 9999;
unsigned int ntripPort = 2233;
unsigned int myPort = 5577;
unsigned int nmeaPort = 5544;

char aogRxBuffer[14];
char ntripBuffer[512];

EthernetUDP udp;
EthernetUDP ntripUdp;
EthernetUDP udpsend;
EthernetUDP nmeaSend;

RingBuf<uint8_t, 800> ntripRingbuffer;

uint8_t pgn;
uint8_t length;
uint8_t sendbuffer[14] = { 0x80,0x81, 0x7f, 0xFD, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };

const byte helloAgIO[] = { 0x80,0x81, 0x7f, 0xC7, 1, 0, 0x47 };

byte tempHeader = 0;

bool isHeaderFound;
bool isPGNFound;

void sendDataToAOG()
{
	int16_t steerAngle = (int16_t)(steerSetpoints.actualSteerAngle * 100);
	sendbuffer[5] = (uint8_t)steerAngle;
	sendbuffer[6] = steerAngle >> 8;
	if (steerSetpoints.useCMPS) {
		sendbuffer[8] = steerSetpoints.headingInt >> 8;
		sendbuffer[7] = steerSetpoints.headingInt;
		sendbuffer[10] = steerSetpoints.rollInt >> 8;
		sendbuffer[9] = steerSetpoints.rollInt;
	}
	else
	{
		//heading         
		sendbuffer[7] = (byte)9999;
		sendbuffer[8] = 9999 >> 8;

		//roll
		sendbuffer[9] = (byte)8888;
		sendbuffer[10] = 8888 >> 8;
	}
	steerSetpoints.switchByte = 0;
	steerSetpoints.switchByte |= (switches.steerSwitch << 1);   //put steerswitch status in bit 1 position
	steerSetpoints.switchByte |= switches.workSwitch;
	sendbuffer[11] = steerSetpoints.switchByte;

	int CK_A = 0;
	for (byte i = 2; i < sizeof(sendbuffer) - 1; i++)
	{
		CK_A = (CK_A + sendbuffer[i]);
	}
	sendbuffer[13] = CK_A;

	nmeaSend.beginPacket(broadcastIP, aogSendPort);
	nmeaSend.write(sendbuffer, sizeof(sendbuffer));
	nmeaSend.endPacket();
}

void sendNMEA(const char* nmeastring)
{
	udpsend.beginPacket(broadcastIP, aogSendPort);
	udpsend.println(nmeastring);
	udpsend.endPacket();
}

void udpWorker()
{
	if (metro.sendHello.check() == 1)
	{
		udp.beginPacket(broadcastIP, aogSendPort);
		udp.write(helloAgIO, sizeof(helloAgIO));
		udp.endPacket();
	}

	uint16_t packetsize = udp.parsePacket();

	if (packetsize)
	{
		digitalToggle(9);
		udp.read(aogRxBuffer, sizeof(aogRxBuffer));
		if (aogRxBuffer[0] == 0x80 && aogRxBuffer[1] == 0x81 && aogRxBuffer[2] == 0x7F)
		{
			pgn = aogRxBuffer[3];
			switch (pgn)
			{
			case 0xFE:
			{
				steerSetpoints.speed = ((float)(aogRxBuffer[5] | aogRxBuffer[5] << 8)) * 0.1;
				steerSetpoints.guidanceStatus = aogRxBuffer[7];
				steerSetpoints.requestedSteerAngle = (float)((int16_t)(aogRxBuffer[8] | aogRxBuffer[9] << 8)) * 0.01;
				steerSetpoints.tram = aogRxBuffer[10];
				steerSetpoints.sections = aogRxBuffer[11] << 8 | aogRxBuffer[12];

				steerSetpoints.lastPacketReceived = millis();
				autosteerWorker();
				break;
			}

			case 0xFC:
			{
				//PID values
				steerSettings.Kp = ((float)aogRxBuffer[5]);   // read Kp from AgOpenGPS
				steerSettings.Kp *= 0.5;

				steerSettings.highPWM = aogRxBuffer[6];

				steerSettings.lowPWM = (float)aogRxBuffer[7];   // read lowPWM from AgOpenGPS

				steerSettings.minPWM = aogRxBuffer[8]; //read the minimum amount of PWM for instant on

				steerSettings.steerSensorCounts = aogRxBuffer[9]; //sent as setting displayed in AOG

				steerSettings.wasOffset = (aogRxBuffer[10]);  //read was zero offset Hi

				steerSettings.wasOffset |= (aogRxBuffer[11] << 8);  //read was zero offset Lo

				steerSettings.AckermanFix = (float)aogRxBuffer[12] * 0.01;

				//store in EEPROM
				EEPROM.put(10, steerSettings);

				Serial.println("Steer Settings received!");

				break;
			}

			case 0xFB: //FB - steerConfig
			{
				Serial.println("Steer Config received!");
				byte sett = aogRxBuffer[5];

				if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
				if (bitRead(sett, 1)) steerConfig.isRelayActiveHigh = 1; else steerConfig.isRelayActiveHigh = 0;
				if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
				if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
				if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
				if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
				if (bitRead(sett, 6)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;

				steerConfig.PulseCountMax = aogRxBuffer[6];

				//Danfoss type hydraulics
				steerConfig.isDanfoss = aogRxBuffer[8]; //byte 8

				EEPROM.put(40, steerConfig);
				break;
			}
			default:
			{
				break;
			}
			}
		}
	}

	int ntripSize = ntripUdp.parsePacket();
	if (ntripSize)
	{
		ntripUdp.read(ntripBuffer, sizeof(ntripBuffer));
		for (int i = 0; i < ntripSize; i++)
		{
			ntripRingbuffer.push(ntripBuffer[i]);
		}
	}

	uint16_t avail = GPS.availableForWrite();
	uint8_t c = 0;
	for (int i = 0; i < avail; i++)
	{
		if (ntripRingbuffer.pop(c)) {
			GPS.write(c);
		}
	}
}

void initEthernet()
{
	Ethernet.begin(mac, ip);
	if (Ethernet.hardwareStatus() == EthernetNoHardware) {
		Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
		while (true) {
			delay(1); // do nothing, no point running without Ethernet hardware
			digitalWrite(13, HIGH);
		}
	}
	if (Ethernet.linkStatus() == LinkOFF) {
		Serial.println("Ethernet cable is not connected.");
	}

	// start UDP
	udp.begin(aogPort);
	ntripUdp.begin(ntripPort);
	udpsend.begin(myPort);
	nmeaSend.begin(nmeaPort);
}