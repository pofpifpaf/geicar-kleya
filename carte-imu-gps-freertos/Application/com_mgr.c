/**
 * @file com_mgr.c
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 20 Aout 2023
 *
 * @brief communication manager.
 * This file contains the main application logic, including initialization and the main loop.
 * It handles the control of the car's motors, ultrasonic sensors, and communication via CAN.
 */

#include <com_mgr.h>
#include "configuration.h"

#include "main.h"
#include "usart.h"
#include "gpio.h"

#include "tasks.h"

#include "FreeRTOS.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef union {
	float value;
	uint8_t bytes[4];
} FloatToBytes_t;

#define COM_FRAME_MOTION_ID    0x10
#define COM_FRAME_ECOMPASS_ID  0x11
#define COM_FRAME_ENV_ID       0x12

#define COM_FRAME_MOTION_SIZE    (3*3*4) // 3 triaxe values (acc, gyro, mag) of 4 bytes each
#define COM_FRAME_ECOMPASS_SIZE  (4*4+1) // 4 float values (heading, pitch, roll, heading) of 4 bytes each + 1 byte heading status
#define COM_FRAME_ENV_SIZE       (3*4)   // 3 float values (temperature, pressure, humidity) of 4 bytes each

#define COM_FRAME_START_BYTE 	0x01	 // Start of Heading (SOH) ASCII

static void COM_SendFrame(uint8_t frame_id, uint8_t *data, uint16_t length) {
	//******** Format frame for UART transmission **********
	uint8_t *buffer = pvPortMalloc(length+5);

	if (buffer == NULL) {
		// Memory allocation failed, assert
		assert_param(0);
	}

	buffer[0] = COM_FRAME_START_BYTE; // Start byte - Start of heading SOH
	buffer[1] = frame_id; // Frame ID

	buffer[2] = (length >> 8) & 0xFF; // Length MSB
	buffer[3] = length & 0xFF; // Length LSB

	memcpy(&buffer[4], data, length); // Data

	// Simple checksum: sum of all bytes modulo 256
	uint8_t checksum = 0;
	for (uint8_t i = 0; i < length + 3; i++) {
		checksum += buffer[i];
	}
	buffer[length + 4] = checksum; // Checksum

	HAL_UART_Transmit(&huart2, buffer, length + 5, HAL_MAX_DELAY);

	vPortFree(buffer);
}

/**
 * @brief Initialize the communication manager.
 * This function sets up the necessary software components
 */
void COM_Init(void) {
	// nothing to do here for print manager
}

/**
 * @brief Run the communication manager.
 *
 * This function is the main loop of the print manager. It handles the main logic,
 * processes inputs, and updates outputs.
 *
 * @remark: this function never returns, it runs indefinitely.
 */
void COM_Run(AppMessage_typeDef *msg) {
	uint8_t *frame_data;
	FloatToBytes_t float_to_bytes;

	switch (msg->id) {
	case COM_MSG_MOTION_ID: // motion data were received, to send over uart
		COM_Msg_Motion_typeDef *sensors_print_msg = (COM_Msg_Motion_typeDef *)msg;

		/********** Send motion data **********/
		frame_data = pvPortMalloc(COM_FRAME_MOTION_SIZE); // 3 triaxe values (acc, gyro, mag) of 4 bytes each
		if (frame_data == NULL) {
			// Memory allocation failed, drop the message and assert
			assert_param(0);
		}

		float_to_bytes.value = sensors_print_msg->acc.x;
		memcpy(&frame_data[0], float_to_bytes.bytes, 4);
		float_to_bytes.value = sensors_print_msg->acc.y;
		memcpy(&frame_data[4], float_to_bytes.bytes, 4);
		float_to_bytes.value = sensors_print_msg->acc.z;
		memcpy(&frame_data[8], float_to_bytes.bytes, 4);

		float_to_bytes.value = sensors_print_msg->gyro.x;
		memcpy(&frame_data[12], float_to_bytes.bytes, 4);
		float_to_bytes.value = sensors_print_msg->gyro.y;
		memcpy(&frame_data[16], float_to_bytes.bytes, 4);
		float_to_bytes.value = sensors_print_msg->gyro.z;
		memcpy(&frame_data[20], float_to_bytes.bytes, 4);

		float_to_bytes.value = sensors_print_msg->mag.x;
		memcpy(&frame_data[24], float_to_bytes.bytes, 4);
		float_to_bytes.value = sensors_print_msg->mag.y;
		memcpy(&frame_data[28], float_to_bytes.bytes, 4);
		float_to_bytes.value = sensors_print_msg->mag.z;
		memcpy(&frame_data[32], float_to_bytes.bytes, 4);

		COM_SendFrame(COM_FRAME_MOTION_ID, frame_data, COM_FRAME_MOTION_SIZE);

		vPortFree(frame_data);
		break;

	case COM_MSG_ECOMPASS_ID: // ecompass data were received, to send over uart
		COM_Msg_ECompass_typeDef *ecompass_print_msg = (COM_Msg_ECompass_typeDef *)msg;

		/********** Send ecompass data **********/
		frame_data = pvPortMalloc(COM_FRAME_ECOMPASS_SIZE); // 4 float values (heading, pitch, roll, heading) of 4 bytes each + 1 byte heading status
		if (frame_data == NULL) {
			// Memory allocation failed, drop the message and assert
			assert_param(0);
		}

		float_to_bytes.value = ecompass_print_msg->ecompass.yaw;
		memcpy(&frame_data[0], float_to_bytes.bytes, 4);
		float_to_bytes.value = ecompass_print_msg->ecompass.pitch;
		memcpy(&frame_data[4], float_to_bytes.bytes, 4);
		float_to_bytes.value = ecompass_print_msg->ecompass.roll;
		memcpy(&frame_data[8], float_to_bytes.bytes, 4);
		float_to_bytes.value = ecompass_print_msg->ecompass.heading;
		memcpy(&frame_data[12], float_to_bytes.bytes, 4);
		frame_data[16] = (uint8_t)ecompass_print_msg->ecompass.heading_valid;

		COM_SendFrame(COM_FRAME_ECOMPASS_ID, frame_data, COM_FRAME_ECOMPASS_SIZE);

		vPortFree(frame_data);
		break;

	case COM_MSG_ENV_ID: // environmental data were received, to send over uart
		COM_Msg_Env_typeDef *env_print_msg = (COM_Msg_Env_typeDef*)msg;

		/********** Send environmental data **********/
		frame_data = pvPortMalloc(COM_FRAME_ENV_SIZE); // 3 float values (temperature, pressure, humidity) of 4 bytes each
		if (frame_data == NULL) {
			// Memory allocation failed, drop the message and assert
			assert_param(0);
		}

		float_to_bytes.value = env_print_msg->env.temperature;
		memcpy(&frame_data[0], float_to_bytes.bytes, 4);
		float_to_bytes.value = env_print_msg->env.pressure;
		memcpy(&frame_data[4], float_to_bytes.bytes, 4);
		float_to_bytes.value = env_print_msg->env.humidity;
		memcpy(&frame_data[8], float_to_bytes.bytes, 4);

		COM_SendFrame(COM_FRAME_ENV_ID, frame_data, COM_FRAME_ENV_SIZE);

		vPortFree(frame_data);
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
