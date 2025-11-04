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
void PRINT_Run(PrintMessage_typeDef *msg) {
	switch (msg->id) {
	case PRINT_MSG_ID: // a message frame was received to send over uart
		printf("%s", msg->s);
		vPortFree((void*)msg->s);
		break;

	default:
		// unknown message id, do nothing
		break;
	}

	vPortFree((void*)msg);
}

PrintMessage_typeDef* PRINT_AllocateMessage(uint32_t length) {
	PrintMessage_typeDef* msg = pvPortMalloc(sizeof(PrintMessage_typeDef));

	if (msg != NULL) {
		msg->s = pvPortMalloc(length);
		if (msg->s != NULL) {
			memset(msg->s, 0, length); // clear the string
			msg->id = PRINT_MSG_ID;
		} else {
			vPortFree((void*) msg);
			msg = NULL;
		}
	}

	return msg;
}

void PRINT_PostMessage(PrintMessage_typeDef *msg) {
	// Send to PRINT task, no wait
	if (xQueueSend(xPrintLoopQueue, &msg, portMAX_DELAY) != pdPASS) {
		// Queue full, drop the message
		vPortFree((void*) msg->s);
		vPortFree((void*) msg);
		assert_param(0);
	}
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
