/**
 * @file sensors.c
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 14 November 2023
 *
 * @brief Sensors management file.
 * This file contains the implementation of sensor initialization and measurement functions.
 */

#ifndef __SENSORS_H__
#define __SENSORS_H__

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


/**
 * @brief Message structure for motion sensor measurements.
 */
typedef struct {
	AppMessage_typeDef header;
	LSM6DSV16X_AxesRaw_t acc; // Accelerometer data
	LSM6DSV16X_AxesRaw_t gyro; // Gyroscope data
	LIS2MDL_AxesRaw_t mag; // Magnetometer data
} SENSORS_MotionMesures_t;

/**
 * @brief Message structure for environmental sensor measurements.
 */
typedef struct {
	AppMessage_typeDef header;
	uint8_t sensorIndex; // Index of the environmental sensor;
    float temperature; // Temperature data
    float pressure; // Pressure data
    float humidity; // Humidity data
} SENSORS_EnvironementMesures_t;

/* -------------------------------------------------------------------------
 * Prototypes des fonctions publiques
 * ------------------------------------------------------------------------- */

/**
 * @brief  Initialize all sensors.
 * This function initializes the motion and environmental sensors.
 */
void SENSORS_Init(void);

/**
 * @brief  Trigger measurements from all sensors.
 * This function triggers the measurement process for both motion and environmental sensors.
 */
void SENSORS_TriggerMeasurements(void);

/**
 * @brief  Send motion sensor measurements.
 * This function publish an event in application mailbox for sending motion frame.
 */
void SENSORS_SendMotionMesures(void);

/**
 * @brief  Send environmental sensor measurements.
 * This function publish an event in application mailbox for sending environment frame.
 */
void SENSORS_SendEnvMesures(void);

/**
 * @brief  Convert raw gyroscope data to degrees per second (DPS).
 *
 * @param raw Pointer to raw gyroscope data.
 * @return SENSORS_TriaxeValues_t Converted gyroscope data in DPS.
 */
SENSORS_TriaxeValues_t SENSORS_GYRO_RawtoDPS(LSM6DSV16X_AxesRaw_t *raw);

/**
 * @brief  Convert raw accelerometer data to g (gravity).
 *
 * @param raw Pointer to raw accelerometer data.
 * @return SENSORS_TriaxeValues_t Converted accelerometer data in g.
 */
SENSORS_TriaxeValues_t SENSORS_ACC_RawtoG(LSM6DSV16X_AxesRaw_t *raw);

/**
 * @brief  Convert raw magnetometer data to milliGauss.
 *
 * @param raw Pointer to raw magnetometer data.
 * @return SENSORS_TriaxeValues_t Converted magnetometer data in milliGauss.
 */
SENSORS_TriaxeValues_t SENSORS_MAG_RawtoMilliGauss(LIS2MDL_AxesRaw_t *raw);

#ifdef __cplusplus
}
#endif

#endif /* __SENSORS_H__ */
