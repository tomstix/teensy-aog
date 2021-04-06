#include "serial.h"

#include "teensy-aog.h"
#include "autosteer.h"
#include "cmps.h"

#include <EEPROM.h>

#define start 0x80
#define second 0x81

uint8_t pgn;
uint8_t length;
uint8_t sendbuffer[14] = { 0x80,0x81, 0x7f, 0xFD, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };

byte tempHeader = 0;

bool isHeaderFound;
bool isPGNFound;

void sendDataToAOG()
{
	int16_t steerAngle = (int16_t)(steerSetpoints.actualSteerAngle * 100);
	sendbuffer[5] = (uint8_t)steerAngle;
	sendbuffer[6] = steerAngle >> 8;
	sendbuffer[8] = steerSetpoints.headingInt >> 8;
	sendbuffer[7] = steerSetpoints.headingInt;
	sendbuffer[10] = steerSetpoints.rollInt >> 8;
	sendbuffer[9] = steerSetpoints.rollInt;
	sendbuffer[11] = steerSetpoints.switchByte;

	int CK_A = 0;
	for (byte i = 2; i < sizeof(sendbuffer) - 1; i++)
	{
		CK_A = (CK_A + sendbuffer[i]);
	}
	sendbuffer[13] = CK_A;

	SerialUSB1.write(sendbuffer, sizeof(sendbuffer));
}

void serialWorker()
{
	if (SerialUSB1.available() > 1 && !isHeaderFound && !isPGNFound)
	{
		byte temp = Serial.read();
		if (tempHeader == start && temp == second)
		{
			isHeaderFound = true;
			tempHeader = 0;
		}
		else
		{
			tempHeader = temp;
			return;
		}
	}

	if (SerialUSB1.available() > 2 && isHeaderFound && !isPGNFound)
	{
		SerialUSB1.read();
		pgn = SerialUSB1.read();
		length = SerialUSB1.read();
		isPGNFound = true;
	}

	if (SerialUSB1.available() > length && isHeaderFound && isPGNFound)
	{
		switch (pgn)
		{
		case 0xFE:
			steerSetpoints.speed = ((float)(Serial.read() | Serial.read() << 8)) * 0.1;
			steerSetpoints.guidanceStatus = Serial.read();
			steerSetpoints.requestedSteerAngle = (float)((int16_t)(SerialUSB1.read() | SerialUSB1.read() << 8)) * 0.01;
			steerSetpoints.tram = SerialUSB1.read();
			steerSetpoints.sections = SerialUSB1.read() << 8 | SerialUSB1.read();
			SerialUSB1.read(); //CRC
			isHeaderFound = isPGNFound = false;
			pgn = length = 0;

			autosteerWorker();
		}
	}
}

void initSerial()
{
	SerialUSB1.begin(115200);
}