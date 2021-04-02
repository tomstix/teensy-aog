// main.h

#ifndef _MAIN_h
#define _MAIN_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif


#endif

#define GPS Serial5
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
	Metro serial = Metro(10);
	Metro resetCycle = Metro(10000);
};
extern Metros metro;

struct TimingData
{
	/*uint8_t sendCurveCommand = 40;
	uint32_t lastSendCurveCommand = 0;
	uint8_t checkIsobus = 5;
	uint32_t lastCheckIsobus = 0;

	uint8_t serialWorker = 10;
	uint32_t lastSerialWorker = 0;

	uint16_t printStatus = 1000;
	uint32_t lastprintStatus = 0;*/

	uint32_t lastCutout = 0;
	uint32_t lastEnable = 0;

	uint32_t cycleTime;
	uint32_t maxCycleTime = 0;

	uint8_t gpsCounter = 0;
	uint32_t gpsByteCounter = 0;
};
extern TimingData timingData;

struct AOGSetup
{
	byte InvertWAS = 0;
	byte InvertRoll = 0;
	byte MotorDriveDirection = 0;
	byte SingleInputWAS = 1;
	byte CytronDriver = 1;
	byte SteerSwitch = 0;
	byte UseMMA_X_Axis = 0;
	byte ShaftEncoder = 0;

	byte BNOInstalled = 1;
	byte InclinometerInstalled = 1; // set to 0 for none
									// set to 1 if DOGS2 Inclinometer is installed and connected to ADS pin A2
									// set to 2 if MMA8452 installed GY-45 (1C)
									// set to 3 if MMA8452 installed Sparkfun, Adafruit MMA8451 (1D)
	byte maxSteerSpeed = 20;
	byte minSteerSpeed = 0;
	byte PulseCountMax = 5;
	byte AckermanFix = 100;     //sent as percent
	byte isRelayActiveHigh = 0; //if zero, active low (default)
};
extern AOGSetup aogSettings;
//15 bytes

struct SteerSettings
{
	float Ko = 0.0f;     //overall gain
	float Kp = 0.0f;     //proportional gain
	float lowPWM = 0.0f; //band of no action
	float Kd = 0.0f;     //derivative gain
	float steeringPositionZero = 3320.0;
	byte minPWM = 0;
	byte highPWM = 100; //max PWM value
	float steerSensorCounts = 10;
	enum class WorkswitchMode : uint8_t
	{
		None = 0,
		Hitch,
		PTO
	} workswitchMode = WorkswitchMode::None;
};
extern SteerSettings steerSettings;
//28 bytes

struct SteerSetpoints
{
	uint16_t sections = 0;
	uint8_t uTurn = 0;
	uint8_t hydLift = 0;
	float speed = 0;
	int16_t distanceFromLine = 32020;
	double requestedSteerAngle = 0;
	int16_t previousAngle = 0;

	bool enabled = false;

	float roll = 0;
	float heading = 0;
	double actualSteerAngle = 0;
	uint8_t switchByte = 0;
	uint8_t pwm = 0;

	time_t lastPacketReceived = 0;
};
extern SteerSetpoints steerSetpoints;


struct Switches
{
	byte steerSwitch = 1;
	byte workSwitch = 1;
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
	byte fendtCAN = 20;   //Variable for Fendt disconect from V-Bus (Not Working Yet)

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
