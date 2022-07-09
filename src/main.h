#pragma once

#include <Arduino.h>
#include <TeensyThreads.h>

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
	uint8_t PressureSensor = 0;
    uint8_t CurrentSensor = 0;
	uint8_t PulseCountMax = 1;
	uint8_t IsDanfoss = 0; //sent as percent

	enum class ImuType : uint8_t
	{
		None = 0,
		CMPS14 = 1,
		BNO085 = 2
	} imuType = ImuType::None;

	enum class WASType : uint8_t
	{
		None = 0,
		ADS1115 = 1,
		CAN = 2
	} wasType = WASType::ADS1115;

	enum class OutputType : uint8_t
	{
		None = 0,
		PWM = 1,
		PWM2 = 2, 
		FendtCAN = 3
	} outputType = OutputType::None;

	enum class WorkswitchType : uint8_t
	{
		None = 0,
		Hitch = 1,
		PTO = 2
	} workswitchType = WorkswitchType::Hitch;
};
extern SteerConfig steerConfig;

struct SteerSettings
{
	uint8_t Kp = 100;	  //proportional gain
	double Ki = 2.0;
	double Kd = 2.0;
	uint8_t highPWM = 60; //max PWM value
	uint8_t lowPWM = 10;  //band of no action
	uint8_t minPWM = 9;
	uint16_t steerSensorCounts = 100;
	int16_t wasOffset = 0;
	float AckermanFix = 1.0; //sent as percent
};
extern SteerSettings steerSettings;

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

	byte guidanceStatus = 0;

	double actualSteerAngle = 0;
	int16_t wasCountsRaw = 0;
	uint8_t switchByte = 0;
	double pidOutput;
	uint8_t pwm = 0;

	uint16_t csenseRaw = 0;
	uint16_t csense = 0;

	time_t lastPacketReceived = 0;
};
extern SteerSetpoints steerSetpoints;

struct Switches
{
	uint8_t steerSwitch = 1;
	uint8_t workSwitch = 1;
};
extern Switches switches;

struct ImuData
{
    float roll = 0.0;
	float heading = 0.0;
	float pitch = 0.0;
	int16_t rollInt = 0;
	uint16_t headingInt = 0;
	int16_t pitchInt = 0;
};
extern ImuData imuData;

struct VbusData
{
	int16_t setCurve = 0; //Variable for Set Curve to V-Bus
	int16_t estCurve = 0; //Variable for WAS from V-Bus
	double steerAngle = 0;
	bool cutoutCAN = 0; //Variable for Cutout from V-Bus

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

extern volatile uint16_t ntripbps;
extern volatile uint16_t udppps;