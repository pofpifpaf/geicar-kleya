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

/**
 * @brief Application message structure.
 * This structure defines the base format of messages used in the application.
 */
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
 * @remark: this function is called by TASKS_AppLoop tasks when a new message is received.
 * @param msg Pointer to the application message to be processed.
 */
void APP_Run(AppMessage_typeDef *msg);

#ifdef __cplusplus
}
#endif

#endif /* APP_H_ */
