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
	if (metro.sendHello.check() == 1)
	{
		SerialUSB1.write(helloAgIO, sizeof(helloAgIO));
	}

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
		{
			steerSetpoints.speed = ((float)(Serial.read() | Serial.read() << 8)) * 0.1;
			steerSetpoints.guidanceStatus = Serial.read();
			steerSetpoints.requestedSteerAngle = (float)((int16_t)(SerialUSB1.read() | SerialUSB1.read() << 8)) * 0.01;
			steerSetpoints.tram = SerialUSB1.read();
			steerSetpoints.sections = SerialUSB1.read() << 8 | SerialUSB1.read();
			SerialUSB1.read(); //CRC
			isHeaderFound = isPGNFound = false;
			pgn = length = 0;

			steerSetpoints.lastPacketReceived = millis();
			break;
		}

		case 0xFC:
		{
			//PID values
			steerSettings.Kp = ((float)Serial.read());   // read Kp from AgOpenGPS
			steerSettings.Kp *= 0.5;

			steerSettings.highPWM = Serial.read();

			steerSettings.lowPWM = (float)Serial.read();   // read lowPWM from AgOpenGPS

			steerSettings.minPWM = Serial.read(); //read the minimum amount of PWM for instant on

			steerSettings.steerSensorCounts = Serial.read(); //sent as setting displayed in AOG

			steerSettings.wasOffset = (Serial.read());  //read was zero offset Hi

			steerSettings.wasOffset |= (Serial.read() << 8);  //read was zero offset Lo

			steerSettings.AckermanFix = (float)Serial.read() * 0.01;

			//crc
			Serial.read();

			//store in EEPROM
			EEPROM.put(10, steerSettings);

			//reset for next pgn sentence
			isHeaderFound = isPGNFound = false;
			pgn = length = 0;
			break;
		}

		case 0xFB: //FB - steerConfig
		{
			byte sett = Serial.read();

			if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
			if (bitRead(sett, 1)) steerConfig.isRelayActiveHigh = 1; else steerConfig.isRelayActiveHigh = 0;
			if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
			if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
			if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
			if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
			if (bitRead(sett, 6)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;

			steerConfig.PulseCountMax = Serial.read();

			//was speed
			Serial.read();

			//Danfoss type hydraulics
			steerConfig.isDanfoss = Serial.read(); //byte 8

			Serial.read(); //byte 9
			Serial.read(); //byte 10

			Serial.read(); //byte 11
			Serial.read(); //byte 12

			//crc byte 13
			Serial.read();

			EEPROM.put(40, steerConfig);

			//reset for next pgn sentence
			isHeaderFound = isPGNFound = false;
			pgn = length = 0;
			break;
		}
		default:
		{
			isHeaderFound = isPGNFound = false;
			pgn = length = 0;
		}
		}
	}
}

void initSerial()
{
	SerialUSB1.begin(115200);
}