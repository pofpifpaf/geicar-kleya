/**
 * @file ultrasound.h
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 20 Aout 2023
 * @brief Header file for ultrasound.c
 * This file contains the declarations for ultrasonic sensor functions.
 */

#ifndef __ULTRASOUND_H__
#define __ULTRASOUND_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app.h"

#include "lsm6dsv16x.h"
#include "lis2duxs12.h"
#include "lis2mdl.h"

#include "lps22df.h"
#include "sht40ad1b.h"
#include "stts22h.h"

typedef struct {
	float x;
	float y;
	float z;
} SENSORS_TriaxeValues_t;

typedef struct {
	AppMessage_typeDef header;
	LSM6DSV16X_AxesRaw_t acc; // Accelerometer data
	LSM6DSV16X_AxesRaw_t gyro; // Gyroscope data
	LIS2MDL_AxesRaw_t mag; // Magnetometer data
} SENSORS_MotionMesures_t;

typedef struct {
	AppMessage_typeDef header;
	uint8_t sensorIndex; // Index of the environmental sensor;
    float temperature; // Temperature data
    float pressure; // Pressure data
    float humidity; // Humidity data
} SENSORS_EnvironementMesures_t;

void SENSORS_Init(void);
void SENSORS_TriggerMeasurements(void);
void SENSORS_SendMesures(void);

SENSORS_TriaxeValues_t SENSORS_GYRO_RawtoDPS(LSM6DSV16X_AxesRaw_t *raw);
SENSORS_TriaxeValues_t SENSORS_ACC_RawtoG(LSM6DSV16X_AxesRaw_t *raw);
SENSORS_TriaxeValues_t SENSORS_MAG_RawtoMilliGauss(LIS2MDL_AxesRaw_t *raw);

#ifdef __cplusplus
}
#endif

#endif /* __ULTRASOUND_H__ */
