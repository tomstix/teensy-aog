// 
// 
#include <FlexCAN_T4.h>
#include "can.h"

#include "teensy-aog.h"


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> vbus;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> isobus;


CAN_message_t rxMsg;

uint8_t pduFormat;
constexpr uint16_t j1939PgnEEC1 = 61444;
constexpr uint16_t j1939PgnWBSD = 65096;
constexpr uint16_t j1939PgnPHS = 65093;
constexpr uint16_t j1939PgnFHS = 65094;
constexpr uint16_t j1939PgnRPTO = 65091;
constexpr uint16_t j1939PgnFPTO = 65092;

CAN_message_t curveCommandMsg;
CAN_message_t steerStateReq;

VbusData vbusData;
IsobusData isobusData;

//***Interupt handlers begin
void handleFromF0(const CAN_message_t& msg)
{
    vbusData.rxCounter++;
    //Serial.println("Message for me from F0!");
    if (msg.len == 3 && msg.buf[2] == 0)
    {
        Serial.println("Cutout!");
        vbusData.cutoutCAN = 1;
        timingData.lastCutout = millis();
        switches.steerSwitch = 1;
        //Serial.println(switches.steerSwitch);
    }
    if (msg.len == 8 && msg.buf[0] == 5 && msg.buf[1] == 10)
    {
        vbusData.estCurve = ((msg.buf[4] << 8) | msg.buf[5]);
        //vbusData.estCurve = (vbusData.estCurve / 10);
        steerSetpoints.actualSteerAngle = (double)(vbusData.estCurve / 819.0);
        //Serial.print("Steer angle: "); Serial.println(steerSetpoints.actualSteerAngle);
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
            Serial.println("\t Steering GO!! ");
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
    vbus.events();
    if (millis() - timingData.lastSendCurveCommand > timingData.sendCurveCommand)
    {
        timingData.lastSendCurveCommand = millis();
        if (steerSetpoints.enabled)
        {
            vbusData.setCurve = (int16_t)(steerSetpoints.requestedSteerAngle * 819);
            curveCommandMsg.buf[2] = 3;
        }
        else
        {
            vbusData.setCurve = 0;
            curveCommandMsg.buf[2] = 2;
        }
        curveCommandMsg.buf[4] = highByte(vbusData.setCurve);
        curveCommandMsg.buf[5] = lowByte(vbusData.setCurve);

        vbus.write(MB2, steerStateReq);
        vbus.write(MB1, curveCommandMsg);
        vbusData.txCounter += 2;
    }
}

void checkIsobus()
{
    isobus.events();
    if (millis() - timingData.lastCheckIsobus > timingData.checkIsobus)
    {
        if (isobus.readMB(rxMsg))
        {
            isobusData.rxCounter++;
            //Serial.println("Isobus Message!");
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
                isobusData.rearPtoRpm = rxMsg.buf[1] << 8 | rxMsg.buf[0];
                break;
            case j1939PgnFPTO:
                isobusData.frontPtoRpm = rxMsg.buf[1] << 8 | rxMsg.buf[0];
                break;
            case 0xAC00:
                isobusData.gmsEstimatedCurvature = (rxMsg.buf[1] << 8 | rxMsg.buf[0]) - 8032;
                isobusData.requestReset = (rxMsg.buf[2] & 0b11000000) >> 6;
                isobusData.steeringSystemReadiness = (rxMsg.buf[2] & 0b1100 >> 2);
                break;
            }
        }
    }
}

void initCAN()
{
    //Set up CAN Messages to send
    CAN_message_t addressClaimMsg;
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

    vbus.begin();
    vbus.setBaudRate(250000);

    vbus.setMaxMB(3);
    vbus.setMB(MB0, RX, EXT); //set MB0 to catch Messages from F0 for 2C -> call handleFromF0
    vbus.setMB(MB1, TX, EXT); //MB1 used to transmit curve command
    vbus.setMB(MB2, TX, EXT); //MB2 used to transmit cutout request
    vbus.setMBFilter(REJECT_ALL);
    vbus.enableMBInterrupts();
    vbus.onReceive(MB0, handleFromF0);
    vbus.setMBUserFilter(MB0, 0x2CF0, 0xFFFF);
    vbus.mailboxStatus();

    vbus.write(addressClaimMsg); //claim VBUS address 2C

    isobus.begin();
    isobus.setBaudRate(250000);

    isobus.setMaxMB(3);
    isobus.setMB(MB0, RX, EXT);
    isobus.setMB(MB1, RX, EXT);
    isobus.setMB(MB2, TX, EXT);
    isobus.setMBFilter(MB0, REJECT_ALL);
    isobus.setMBFilter(MB1, ACCEPT_ALL);
    isobus.setMBUserFilter(MB0, 0x2CF0, 0xFFFF);
    isobus.enableMBInterrupt(MB0);
    isobus.onReceive(MB0, handleIsoFromF0);
    isobus.mailboxStatus();

    isobus.write(addressClaimMsg);

    steerSetpoints.switchByte = 0;
}

