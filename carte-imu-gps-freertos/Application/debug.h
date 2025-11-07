/**
 * @file debug.h
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 29 aout 2025
 * @brief Debug functions for the car application.
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include "app.h"
#include "sensors.h"
#include "ecompass.h"

void DEBUG_Print(char *str);
void DEBUG_PrintPeriodicInfo(void);
void DEBUG_UpdateMotionData(SENSORS_TriaxeValues_t acc,
		SENSORS_TriaxeValues_t gyro,
		SENSORS_TriaxeValues_t mag);
void DEBUG_UpdateEnvData(SENSORS_EnvironementMesures_t env);
void DEBUG_UpdateECompassData(ECOMPASS_Values_t ecompass);

void DEBUG_Panic(uint8_t *file, uint32_t line);
void runTimeCounterOverflowHandler(void);

#endif /* DEBUG_H_ */
