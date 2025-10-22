/**
 * @file    can_communication.c
 * @author  Sebastien DI MERCURIO
 * @version V1.0
 * @date    20 Aout 2023
 * @brief   Functions for CAN communication.
 * This file contains the implementation of functions to initialize and manage CAN communication,
 */

#ifndef __CAN_COMMUNICATION_H__
#define __CAN_COMMUNICATION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app.h"

#define CAN_ID_SENSORS			0x200	// Odometry, Motors speed => to Raspberry
#define CAN_ID_ECOMPASS			0x211	// Front Ultrasonic mesures [cm] => to Raspberry
#define CAN_ID_TILT				0x221	// Rear Ultrasonic mesures [cm] => to Raspberry
#define CAN_ID_COMM_CHECKING 	0x410   // Communication checking <= from Raspberry and => to Raspberry

#define COMM_CHECKING_REQUEST 	0x1 	// frame[0] cmd for communication checking (<= from Raspberry)
#define COMM_CHECKING_ACK 		0x1	  	// frame[1] ack for communication checking (=> to Raspberry)

typedef struct {
	AppMessage_typeDef header;
	uint16_t can_id; 	// ID of received CAN frame
	uint8_t length;		// Length of received CAN frame
	uint8_t data[8];	// Data of received CAN frame
} CANReceivedFrame_typeDef;

void CAN_COM_Init(void);
void CAN_COM_Send(uint32_t id, uint8_t* data, uint8_t length) ;
void CAN_COM_ReceiveTask(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_COMMUNICATION_H__ */

