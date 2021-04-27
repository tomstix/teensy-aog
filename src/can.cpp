#include <FlexCAN_T4.h>
#include "can.h"


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> vbus;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> isobus;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> kbus;

CAN_message_t rxMsg;

CAN_message_t goEnd;

uint8_t pduFormat;
constexpr uint16_t j1939PgnEEC1 = 61444;
constexpr uint16_t j1939PgnWBSD = 65096;
constexpr uint16_t j1939PgnPHS = 65093;
constexpr uint16_t j1939PgnFHS = 65094;
constexpr uint16_t j1939PgnRPTO = 65091;
constexpr uint16_t j1939PgnFPTO = 65092;
constexpr uint16_t isoPgnGms = 0xAC00;

CAN_message_t addressClaimMsg;
CAN_message_t curveCommandMsg;
CAN_message_t steerStateReq;

VbusData vbusData;
IsobusData isobusData;

uint16_t vbusScale;
int16_t sendAngle = 0;

void addressClaim()
{

	vbus.write(addressClaimMsg); //claim VBUS address 2C
	isobus.write(MB9, addressClaimMsg);
	//Serial.println("Adress claim!");
}

//***Interupt handlers begin
void handleFromF0(const CAN_message_t& msg)
{
	vbusData.rxCounter++;
	timingData.lastCANRx = millis();
	//Serial.println("Message for me from F0!");
	if (msg.len == 3 && msg.buf[2] == 0)
	{
		//Serial.println("Cutout!");
		vbusData.cutoutCAN = 1;
		timingData.lastCutout = millis();
		switches.steerSwitch = 1;
	}
	if (msg.len == 8 && msg.buf[0] == 5 && msg.buf[1] == 10)
	{
		vbusData.estCurve = ((msg.buf[4] << 8) | msg.buf[5]);
		//steerSetpoints.actualSteerAngle = (double)(vbusData.estCurve / (double)vbusScale);
	}
}

void handleIsoFromF0(const CAN_message_t& msg)
{
	isobusData.rxCounterF0++;
	//Serial.println("ISO Message from F0!");
	if ((msg.buf[0]) == 0x0F || (msg.buf[1]) == 0x60)
	{

		if ((msg.buf[2]) == 0x01)
		{
			//Serial.println("\t Steering GO!! ");
			switches.steerSwitch = 0; //enable steerswitch
			vbusData.cutoutCAN = 0;   //reset cutout
			timingData.lastEnable = millis();
		}

		/*else
		{
			Serial.println("\t Steering Failed");
		}*/
	}
}
//***Interupt handlers end

void sendCurveCommand()
{
	if ((millis() - timingData.lastCANRx > 1000) & (metro.sendAdressClaim.check() == 1)) //claim address if no one is talking to you
	{
		addressClaim();
	}
	vbus.events();
	if (metro.sendCurveCommand.check() == 1)
	{
		if (steerSetpoints.enabled)
		{
			if (steerSetpoints.requestedSteerAngle > 0)					//use different scale factors for left/right steering
			{
				vbusScale = vbusScaleRight;
				if (steerSetpoints.requestedSteerAngle > 31.5)			//avoid variable overflow when reaching max steer angle
				{
					steerSetpoints.requestedSteerAngle = 31.5;
				}
			}
			else
			{
				vbusScale = vbusScaleLeft;
				if (steerSetpoints.requestedSteerAngle < -35.0)
				{
					steerSetpoints.requestedSteerAngle = -35.0;
				}
			}
			int16_t angleRate = steerSettings.Kp * 16;						//use Kp Setting to limit steering Speed
			sendAngle = (int16_t)(steerSetpoints.requestedSteerAngle * vbusScale);
			if ((sendAngle - steerSetpoints.previousAngle) > angleRate)		//limit steering speed a little bit
			{
				sendAngle = steerSetpoints.previousAngle + angleRate;
			}
			else if ((sendAngle - steerSetpoints.previousAngle < -angleRate))
			{
				sendAngle = steerSetpoints.previousAngle - angleRate;
			}
			vbusData.setCurve = sendAngle;
			curveCommandMsg.buf[2] = 3;

			steerSetpoints.previousAngle = sendAngle;
		}
		else
		{
			vbusData.setCurve = vbusData.estCurve;
			steerSetpoints.previousAngle = (int16_t)(vbusData.estCurve * vbusScale);
			curveCommandMsg.buf[2] = 2;
		}
		curveCommandMsg.buf[4] = highByte(sendAngle);
		curveCommandMsg.buf[5] = lowByte(sendAngle);

		vbus.write(MB2, steerStateReq);
		vbus.write(MB1, curveCommandMsg);
		vbusData.txCounter += 2;
	}
}


void checkIsobus()
{
	kbus.events();
	isobus.events();
	if (metro.checkIsobus.check() == 1)
	{
		if (isobus.readFIFO(rxMsg))
		{
			isobusData.rxCounter++;

			pduFormat = (rxMsg.id & 0x00FF0000) >> 16;
			if (pduFormat > 0xEF)
			{
				isobusData.pgn = (rxMsg.id & 0x00FFFF00) >> 8;
			}
			else
			{
				isobusData.pgn = (rxMsg.id & 0x00FF0000) >> 8;
			}


			switch (isobusData.pgn)
			{
			case j1939PgnEEC1:
				isobusData.motorRpm = (rxMsg.buf[4] << 8 | rxMsg.buf[3]) / 8;
				break;
			case j1939PgnWBSD:
				isobusData.speed = (rxMsg.buf[1] << 8 | rxMsg.buf[0]) / 1000 * 3.6;
				break;
			case j1939PgnPHS:
				isobusData.rearHitchPosition = rxMsg.buf[0];
				break;
			case j1939PgnFHS:
				isobusData.frontHitchPosition = rxMsg.buf[0];
				break;
			case j1939PgnRPTO:
				isobusData.rearPtoRpm = (rxMsg.buf[1] << 8 | rxMsg.buf[0]) / 8;
				break;
			case j1939PgnFPTO:
				isobusData.frontPtoRpm = (rxMsg.buf[1] << 8 | rxMsg.buf[0]) / 8;
				break;
			case isoPgnGms:
				isobusData.gmsEstimatedCurvatureRaw = (rxMsg.buf[1] << 8 | rxMsg.buf[0]);
				isobusData.gmsEstimatedCurvature = ((double)isobusData.gmsEstimatedCurvatureRaw / 4.0) - 8032;
				steerSetpoints.actualSteerAngle = atan(WHEELBASE * isobusData.gmsEstimatedCurvature / 1000.0) * 180/PI;
				break;
			}
		}
	}
}

void sendGoEnd()
{
	if(steerSetpoints.hydLift == 0) return;

	if(steerSetpoints.hydLift == 1)	//hydraulics down -> GO
	{
		Serial.println("Sending GO!");
		goEnd.buf[1] = 0x20;
		goEnd.buf[4] = 0x80;
		goEnd.buf[5] = 0x01;
		kbus.write(goEnd);		//press
		goEnd.buf[4] = 0x00;
		goEnd.buf[5] = 0x02;
		kbus.write(goEnd);		//release
	}
	else if(steerSetpoints.hydLift == 2) //hydLift up -> END
	{
		Serial.println("Sending END!");
		goEnd.buf[1] = 0x21;
		goEnd.buf[4] = 0x80;
		goEnd.buf[5] = 0x01;
		kbus.write(goEnd);		//press
		goEnd.buf[4] = 0x00;
		goEnd.buf[5] = 0x02;
		kbus.write(goEnd);		//release
	}
}

void initCAN()
{
	//Set up CAN Messages to send
	addressClaimMsg.flags.extended = true;
	addressClaimMsg.id = 0x18EEFF2C;
	addressClaimMsg.len = 8;
	addressClaimMsg.buf[0] = 0x00;
	addressClaimMsg.buf[1] = 0x00;
	addressClaimMsg.buf[2] = 0xC0;
	addressClaimMsg.buf[3] = 0x0C;
	addressClaimMsg.buf[4] = 0x00;
	addressClaimMsg.buf[5] = 0x17;
	addressClaimMsg.buf[6] = 0x02;
	addressClaimMsg.buf[7] = 0x20;

	curveCommandMsg.flags.extended = true;
	curveCommandMsg.id = 0x0CEFF02C;
	curveCommandMsg.len = 6;
	curveCommandMsg.buf[0] = 0x05;
	curveCommandMsg.buf[1] = 0x09;
	curveCommandMsg.buf[2] = 0x00;
	curveCommandMsg.buf[3] = 0x0A;
	curveCommandMsg.buf[4] = 0x00;
	curveCommandMsg.buf[5] = 0x00;

	steerStateReq.flags.extended = true;
	steerStateReq.id = 0x0CEFF02C;
	steerStateReq.len = 2;
	steerStateReq.buf[0] = 0x5;
	steerStateReq.buf[1] = 0x19;

	goEnd.id = 0x613;
	goEnd.flags.extended = false;
	goEnd.len = 8;
	goEnd.buf[0] = 0x15;
	goEnd.buf[1] = 0x20;
	goEnd.buf[2] = 0x06;
	goEnd.buf[3] = 0xCA;
	goEnd.buf[4] = 0x80;
	goEnd.buf[5] = 0x01;
	goEnd.buf[6] = 0x00;
	goEnd.buf[7] = 0x00;
	goEnd.seq = 1;


	vbus.begin();
	vbus.setBaudRate(250000);

	vbus.setMaxMB(3);
	vbus.setMB(MB0, RX, EXT); //set MB0 to catch Messages from F0 for 2C -> call handleFromF0
	vbus.setMB(MB1, TX, EXT); //MB1 used to transmit curve command
	vbus.setMB(MB2, TX, EXT); //MB2 used to transmit cutout request
	vbus.setMBFilter(REJECT_ALL);
	vbus.enableMBInterrupt(MB0);
	vbus.onReceive(MB0, handleFromF0);
	vbus.setMBUserFilter(MB0, 0x2CF0, 0xFFFF);
	//vbus.mailboxStatus();

	isobus.begin();
	isobus.setBaudRate(250000);

	isobus.setMaxMB(10);
	isobus.enableFIFO();
	isobus.setMB(MB8, RX, EXT); //For Messages to 2C from F0
	isobus.setMB(MB9, TX, EXT);
	isobus.setMBFilter(REJECT_ALL);
	isobus.setFIFOFilter(REJECT_ALL);
	isobus.setFIFOUserFilter(0, 0xF00400, 0xFFFF00, EXT);
	isobus.setFIFOUserFilter(1, 0xAC0000, 0xFF0000, EXT);
	isobus.setFIFOUserFilter(2, 0xFE4300, 0xFE4400, 0xFE4500, 0xFE4600, 0xFFFF00, EXT);
	isobus.setFIFOUserFilter(3, 0xFE4700, 0xFE4800, 0xFFFF00, EXT);

	isobus.setMBUserFilter(MB8, 0x2CF0, 0xFFFF);
	isobus.enableMBInterrupt(MB8);
	isobus.onReceive(MB8, handleIsoFromF0);
	//isobus.mailboxStatus();

	kbus.begin();
	kbus.setBaudRate(250000);
	kbus.setMaxMB(2);
	kbus.setMB(MB0,TX,STD);
	kbus.setMBFilter(REJECT_ALL);

	addressClaim();
}