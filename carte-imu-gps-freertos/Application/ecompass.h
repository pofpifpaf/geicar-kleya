/**
 * @file ecompass.c
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 14 November 2023
 *
 * @brief Ecompass management.
 * This file contains the surrounding functions to manage the ecompass calculations
 */

#ifndef ECOMPASS_H_
#define ECOMPASS_H_

#include "app.h"
#include "configuration.h"

#include "sensors.h"

/**
 * @brief  ECOMPASS values structure
 *
 */
typedef struct {
	float yaw;
	float pitch;
	float roll;
	float heading;
	int32_t heading_valid;
} ECOMPASS_Values_t;

/**
 * @brief  ECOMPASS attitude message structure
 *
 * @note used to send output data from ECOMPASS task
 */
typedef struct {
	AppMessage_typeDef header;
	ECOMPASS_Values_t values;
} ECOMPASS_Attitude_t;

/**
 * @brief  ECOMPASS motion sensors values message structure
 *
 * @note used to send input data to ECOMPASS task
 */
typedef struct {
	AppMessage_typeDef header;
	uint32_t elapsedTimeMs;
	SENSORS_TriaxeValues_t acc;
	SENSORS_TriaxeValues_t gyro;
	SENSORS_TriaxeValues_t mag;
} ECOMPASS_SensorsValues_t;

/**
 * @brief  Initialize ECOMPASS module
 *
 * @param freq Frequency of sensor data update in Hz
 * @return uint32_t 0 on success, error code otherwise
 */
uint32_t ECOMPASS_Init(float freq);

/**
 * @brief  Process ECOMPASS sensor data message
 *
 * @remark : this function is called by ECOMPASS task when a new message is received.
 * @param msg Pointer to ECOMPASS_SensorsValues_t message containing sensor data
 */
void ECOMPASS_ProcessMessage(const ECOMPASS_SensorsValues_t *msg);

/**
 * @brief  Send ECOMPASS mesures request
 *
 * @note This function is used to trigger sending eCompass measures when corresponding timer expires
 */
void ECOMPASS_SendMesures(void);

#endif /* ECOMPASS_H_ */
