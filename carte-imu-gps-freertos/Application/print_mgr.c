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

#include "print_mgr.h"

#include "configuration.h"

#include "main.h"
//#include "can.h"
#include "usart.h"
#include "gpio.h"

#include "tasks.h"

#include "FreeRTOS.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void PRINT_FormatFrame(uint8_t frame_id, uint8_t *data, uint8_t length) {
	//******** Format frame for UART transmission **********
	// TODO:
}

/**
 * @brief Initialize the print manager.
 * This function sets up the necessary software components
 */
void PRINT_Init(void) {
	// nothing to do here for print manager
}

/**
 * @brief Run the print manager.
 *
 * This function is the main loop of the print manager. It handles the main logic,
 * processes inputs, and updates outputs.
 *
 * @remark: this function never returns, it runs indefinitely.
 */
void PRINT_Run(AppMessage_typeDef *msg) {
	switch (msg->id) {
	case PRINT_MOTION_MSG_ID: // motion data were received, to send over uart
		PrintMessage_Motion_typeDef *sensors_print_msg = pvPortMalloc(sizeof(PrintMessage_Motion_typeDef));
		if (sensors_print_msg == NULL) {
			// Memory allocation failed, drop the message and assert
			assert_param(0);
		}

		/********** Send motion data **********/
		/* TODO :
		 * Implement the printing of motion data over UART.
		 * Use the huart2 handle for UART transmission.
		 * Format the output as needed.
		 */

		PRINT_FormatFrame(0x10, (uint8_t*)&sensors_print_msg->acc, sizeof(SENSORS_TriaxeValues_t) * 3);
		break;

	case PRINT_ECOMPASS_MSG_ID: // ecompass data were received, to send over uart
		PrintMessage_ECompass_typeDef *ecompass_print_msg = pvPortMalloc(sizeof(PrintMessage_ECompass_typeDef));
		if (ecompass_print_msg == NULL) {
			// Memory allocation failed, drop the message and assert
			assert_param(0);
		}

		/********** Send ecompass data **********/
		/* TODO :
		 * Implement the printing of ecompass data over UART.
		 * Use the huart2 handle for UART transmission.
		 * Format the output as needed.
		 */

		PRINT_FormatFrame(0x20, (uint8_t*)&ecompass_print_msg->ecompass, sizeof(ECOMPASS_Values_t));

		break;

	case PRINT_ENV_MSG_ID: // environmental data were received, to send over uart
		PrintMessage_Env_typeDef *env_print_msg = pvPortMalloc(sizeof(PrintMessage_Env_typeDef));
		if (env_print_msg == NULL) {
			// Memory allocation failed, drop the message and assert
			assert_param(0);
		}

		/********** Send environmental data **********/
		/* TODO :
		 * Implement the printing of environmental data over UART.
		 * Use the huart2 handle for UART transmission.
		 * Format the output as needed.
		 */

		PRINT_FormatFrame(0x30, (uint8_t*)&env_print_msg->env, sizeof(SENSORS_EnvironementMesures_t));

		break;

	default:
		// unknown message id, do nothing
		break;
	}

	vPortFree((void*)msg);
}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  ch: Character to be printed
 * @retval Character sent
 */
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
