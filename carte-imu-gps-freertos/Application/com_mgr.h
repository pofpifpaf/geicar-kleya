/**
 * @file com_mgr.h
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 20 Aout 2023
 *
 * @brief communication handler.
 * This file contains the main application logic, including initialization and the main loop.
 * It handles the control of the car's motors, ultrasonic sensors, and communication via CAN.
 */

#ifndef COM_MGR_H_
#define COM_MGR_H_

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
} COM_Msg_Motion_typeDef;

typedef struct {
	uint16_t id;
	SENSORS_EnvironementMesures_t env;
} COM_Msg_Env_typeDef;

typedef struct {
	uint16_t id;
	ECOMPASS_Values_t ecompass;
} COM_Msg_ECompass_typeDef;

/**
 * @brief Initialize the communication manager.
 * This function sets up the necessary software components
 */
void COM_Init(void);

/**
 * @brief Run the communication manager.
 *
 * This function is the main loop of the communication manager. It handles the main logic,
 * processes inputs, and updates outputs.
 *
 * @remark: this function never returns, it runs indefinitely.
 */
void COM_Run(AppMessage_typeDef *msg);

#ifdef __cplusplus
}
#endif

#endif /* COM_MGR_H_ */
