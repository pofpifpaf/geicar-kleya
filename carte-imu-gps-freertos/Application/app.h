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

#ifndef APP_H_
#define APP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct {
	uint16_t id;
} AppMessage_typeDef;

/**
 * @brief Initialize the application.
 * This function sets up the necessary software components
 */
void APP_Init(void);

/**
 * @brief Run the application.
 *
 * This function is the main loop of the application. It handles the main logic,
 * processes inputs, and updates outputs.
 *
 * @remark: this function never returns, it runs indefinitely.
 */
void APP_Run(AppMessage_typeDef *msg);

void APP_PeriodicCountersUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H_ */
