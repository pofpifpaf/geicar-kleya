/**
 * @file app.c
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 20 Aout 2023
 *
 * @brief Main application file.
 * This file contains the main application logic, including initialization and the main loop.
 * It handles the control of the car's motors, ultrasonic sensors, and communication via CAN.
 */

#ifndef PRINT_MGR_H_
#define PRINT_MGR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "sensors.h"
#include "ecompass.h"

typedef struct {
	uint16_t id;
	SENSORS_TriaxeValues_t acc;
	SENSORS_TriaxeValues_t gyro;
	SENSORS_TriaxeValues_t mag;
} PrintMessage_Motion_typeDef;

typedef struct {
	uint16_t id;
	SENSORS_EnvironementMesures_t env;
} PrintMessage_Env_typeDef;

typedef struct {
	uint16_t id;
	ECOMPASS_Values_t ecompass;
} PrintMessage_ECompass_typeDef;

/**
 * @brief Initialize the application.
 * This function sets up the necessary software components
 */
void PRINT_Init(void);

/**
 * @brief Run the application.
 *
 * This function is the main loop of the application. It handles the main logic,
 * processes inputs, and updates outputs.
 *
 * @remark: this function never returns, it runs indefinitely.
 */
void PRINT_Run(AppMessage_typeDef *msg);

#ifdef __cplusplus
}
#endif

#endif /* PRINT_MGR_H_ */
