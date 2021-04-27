#pragma once

#include <Arduino.h>
#include <Metro.h>

#define GPS Serial3
#define CMPSAddress 0x60

#define vbusScaleLeft 897
#define vbusScaleRight 1039
#define WHEELBASE 2.783
#define benchmode 0

const uint16_t ptoTreshold = 300;

struct Metros
{
	Metro sendCurveCommand = Metro(40);
	Metro checkIsobus = Metro(1);
	Metro printStatus = Metro(1000);
	Metro gps = Metro(10);
	Metro imu = Metro(5);
	Metro sendHello = Metro(200);
	Metro resetCycle = Metro(10000);
	Metro sendAdressClaim = Metro(1000);
};
extern Metros metro;

struct TimingData
{
	uint32_t lastCutout = 0;
	uint32_t lastEnable = 0;

	uint32_t cycleTime;
	uint32_t maxCycleTime = 0;

	uint8_t gpsCounter = 0;
	uint32_t gpsByteCounter = 0;

	uint32_t lastCANRx = 0;
};
extern TimingData timingData;

struct SteerConfig
{
	uint8_t InvertWAS = 0;
	uint8_t IsRelayActiveHigh = 0; //if zero, active low (default)
	uint8_t MotorDriveDirection = 0;
	uint8_t SingleInputWAS = 1;
	uint8_t CytronDriver = 1;
	uint8_t SteerSwitch = 0;
	uint8_t SteerButton = 0;
	uint8_t ShaftEncoder = 0;
	uint8_t PulseCountMax = 1;
	uint8_t IsDanfoss = 0;     //sent as percent
};
extern SteerConfig steerConfig;
//10 bytes

struct SteerSettings
{
	uint8_t Kp = 100;  //proportional gain
	uint8_t highPWM = 60;//max PWM value
	uint8_t lowPWM = 10;  //band of no action
	uint8_t minPWM = 9;
	uint8_t steerSensorCounts = 1;
	int16_t wasOffset = 0;
	uint8_t AckermanFix = 80;     //sent as percent
};
extern SteerSettings steerSettings;
//8 bytes

struct SteerSetpoints
{
	uint16_t sections = 0;
	uint8_t uTurn = 0;
	uint8_t hydLift = 0;
	uint8_t hydLiftPrev = 0;
	uint8_t tram = 0;
	uint8_t tree = 0;
	float speed = 0;

	double requestedSteerAngle = 0;
	int16_t previousAngle = 0;

	bool enabled = false;
	byte guidanceStatus = 0;

	float roll = 0;
	int16_t rollInt = 0;
	float heading = 0;
	int16_t headingInt = 0;
	double actualSteerAngle = 0;
	uint8_t switchByte = 0;
	uint8_t pwm = 0;

	bool useCMPS = true;

	time_t lastPacketReceived = 0;
};
extern SteerSetpoints steerSetpoints;


struct Switches
{
	uint8_t steerSwitch = 1;
	uint8_t workSwitch = 1;
};
extern Switches switches;

struct GPSData
{
	long speed = 0;
	uint8_t seconds = 0;
};
extern GPSData gpsData;

struct VbusData
{
	int16_t setCurve = 0; //Variable for Set Curve to V-Bus
	int16_t estCurve = 0; //Variable for WAS from V-Bus
	double steerAngle = 0;
	bool cutoutCAN = 0;   //Variable for Cutout from V-Bus

	unsigned int rxCounter = 0;
	unsigned int txCounter = 0;
};
extern VbusData vbusData;


struct IsobusData
{
	float speed;
	uint16_t motorRpm;
	uint8_t frontHitchPosition;
	uint8_t rearHitchPosition;
	uint8_t rearHitchWorking = 0;
	uint16_t frontPtoRpm;
	uint16_t rearPtoRpm;

	uint16_t gmsEstimatedCurvatureRaw;
	double gmsEstimatedCurvature;
	double gmsSteerAngle;

	unsigned int rxCounter = 0;
	unsigned int rxCounterF0 = 0;

	uint16_t pgn;
};
extern IsobusData isobusData;