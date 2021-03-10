// can.h

#ifndef _CAN_h
#define _CAN_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#endif
/*#ifndef _FLEXCAN_T4_H_
#include <kinetis_flexcan.h>
#include <imxrt_flexcan.h>
#include <FlexCAN_T4.h>
#include <circular_buffer.h>
#endif*/

void initCAN();
void checkIsobus();
void sendCurveCommand();
//void handleIsoFromF0(const CAN_message_t&);
//void handleFromF0(const CAN_message_t&);
