// main.h

#ifndef _MAIN_h
#define _MAIN_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif


#endif

#define GPS Serial3
#define PRINT_DATA 1
#define CMPSAddress 0x60
#define aogVersion 17

#define benchmode 0

#include <Metro.h>

struct Metros
{
	Metro sendCurveCommand = Metro(40);
	Metro checkIsobus = Metro(1);
	Metro printStatus = Metro(1000);
	Metro gps = Metro(10);
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
	uint8_t PulseCountMax = 5;
	uint8_t IsDanfoss = 0;     //sent as percent
};
extern SteerConfig steerConfig;
//9 bytes

struct SteerSettings
{
	uint8_t Kp = 40;  //proportional gain
	uint8_t highPWM = 60;//max PWM value
	uint8_t lowPWM = 10;  //band of no action
	uint8_t minPWM = 9;
	uint8_t steerSensorCounts = 1;
	int16_t wasOffset = 0;
	uint8_t AckermanFix = 1;     //sent as percent
};
extern SteerSettings steerSettings;
//8 bytes

struct SteerSetpoints
{
	uint16_t sections = 0;
	uint8_t uTurn = 0;
	uint8_t hydLift = 0;
	uint8_t tram = 0;
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
	bool cutoutCAN = 0;   //Variable for Cutout from V-Bus
	uint8_t fendtCAN = 20;   //Variable for Fendt disconect from V-Bus (Not Working Yet)

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
	int16_t gmsEstimatedCurvature;
	uint8_t steeringSystemReadiness;
	uint8_t remoteswitchStatus;
	uint8_t requestReset;

	unsigned int rxCounter = 0;
	unsigned int rxCounterF0 = 0;

	uint16_t pgn;
};
extern IsobusData isobusData;