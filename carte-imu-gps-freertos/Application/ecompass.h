/*
 * ecompass.h
 *
 *  Created on: Oct 23, 2025
 *      Author: dimercur
 */

#ifndef ECOMPASS_H_
#define ECOMPASS_H_

#include "app.h"
#include "configuration.h"

#include "sensors.h"

typedef struct {
	float yaw;
	float pitch;
	float roll;
	float heading;
	int32_t heading_valid;
} ECOMPASS_Values_t;

typedef struct {
	AppMessage_typeDef header;
	ECOMPASS_Values_t values;
} ECOMPASS_Attitude_t;

typedef struct {
	AppMessage_typeDef header;
	uint32_t elapsedTimeMs;
	SENSORS_TriaxeValues_t acc;
	SENSORS_TriaxeValues_t gyro;
	SENSORS_TriaxeValues_t mag;
} ECOMPASS_SensorsValues_t;

uint32_t ECOMPASS_Init(float freq) ;
void ECOMPASS_ProcessMessage(const ECOMPASS_SensorsValues_t *msg);
void ECOMPASS_SendMesures(void);
#endif /* ECOMPASS_H_ */
