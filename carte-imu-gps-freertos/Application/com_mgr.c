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

#define COM_FRAME_GET_VERSION_ID    0x10
#define COM_FRAME_VERSION_ID       	0x11

#define COM_FRAME_RESET_HEADING_ID  0x20
#define COM_FRAME_CALIBRATE_MAG_ID  0x21

#define COM_FRAME_MOTION_ID    		0x30
#define COM_FRAME_ECOMPASS_ID  		0x31
#define COM_FRAME_ENV_ID       		0x32

#define COM_FRAME_VERSION_SIZE    (3*1)   // 3 bytes for version major, minor, patch

#define COM_FRAME_MOTION_SIZE    (3*3*4) // 3 triaxe values (acc, gyro, mag) of 4 bytes each
#define COM_FRAME_ECOMPASS_SIZE  (4*4+1) // 4 float values (heading, pitch, roll, heading) of 4 bytes each + 1 byte heading status
#define COM_FRAME_ENV_SIZE       (3*4)   // 3 float values (temperature, pressure, humidity) of 4 bytes each

#define COM_FRAME_START_BYTE 	0x01	 // Start of Heading (SOH) ASCII

#define COM_RX_BUFFER_LEN 256
uint8_t COM_RxBuffer[COM_RX_BUFFER_LEN]; // Buffer de réception DMA


static void COM_SendFrame(uint8_t frame_id, uint8_t *data, uint16_t length);
static size_t COM_AvailableBytes(void);
static void COM_CopyFromCircular(uint8_t *dst, size_t len);

/* ----------------- Configuration / types ----------------- */

typedef struct {
	UART_HandleTypeDef *huart;       // handle UART (uniquement pour info si besoin)
	DMA_HandleTypeDef  *hdma_rx;     // handle DMA rx (utilisé par __HAL_DMA_GET_COUNTER)
	uint8_t            *rx_buffer;   // buffer circular DMA
	size_t              rx_buffer_len;
	size_t              read_pos;    // index (0..rx_buffer_len-1) du prochain octet à lire
} COM_RX_Handle_t;

static COM_RX_Handle_t com = {0};

/**
 * @brief Initialize the communication manager.
 * This function sets up the necessary software components
 */
void COM_Init(UART_HandleTypeDef *huart,
		DMA_HandleTypeDef  *hdma_rx)
{
	if (!huart || !hdma_rx) {
		// Invalid parameters
		assert_param(0);
	}

	com.huart = huart;
	com.hdma_rx = hdma_rx;
	com.rx_buffer = COM_RxBuffer;
	com.rx_buffer_len = COM_RX_BUFFER_LEN;
	com.read_pos = 0;

	if (HAL_UART_Receive_DMA(com.huart, com.rx_buffer, COM_RX_BUFFER_LEN) != HAL_OK) {
		// DMA start error
		assert_param(0);
	}
}

/* ----------------- COM_Read (bloquant) ----------------- */
/* Remplit 'dst' avec 'len' octets. Bloque tant qu'il n'y a pas assez d'octets.
 * Utilisateur appelle depuis une tâche FreeRTOS. */
void COM_Read(uint8_t *dst, size_t len)
{
	if (dst == NULL || len == 0) {
		assert_param(0);
		return;
	}

	size_t total_copied = 0;

	while (total_copied < len) {
		size_t avail = COM_AvailableBytes();
		size_t remaining = len - total_copied;

		if (avail > 0) {
			size_t to_copy = (avail < remaining) ? avail : remaining;

			taskENTER_CRITICAL();
			COM_CopyFromCircular(&dst[total_copied], to_copy);
			taskEXIT_CRITICAL();

			total_copied += to_copy;
		}

		/* Si on n’a pas encore tout lu, on attend 10 ms avant de rechecker */
		if (total_copied < len) {
			vTaskDelay(pdMS_TO_TICKS(10));
		}
	}
}

/**
 * @brief Run the communication manager.
 *
 * This function is the main loop of the print manager. It handles the main logic,
 * processes inputs, and updates outputs.
 *
 * @remark: this function never returns, it runs indefinitely.
 */
void COM_RunTX(AppMessage_typeDef *msg) {
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

	case COM_MSG_GET_VERSION_ID: {
		COM_Msg_RX_typeDef *get_version_msg = (COM_Msg_RX_typeDef *)msg;

		frame_data = pvPortMalloc(COM_FRAME_VERSION_SIZE);
		if (frame_data == NULL) {
			// Memory allocation failed, drop the message and assert
			assert_param(0);
		}

		frame_data[0] = APP_VERSION_MAJOR;
		frame_data[1] = APP_VERSION_MINOR;
		frame_data[2] = APP_VERSION_PATCH;

		COM_SendFrame(COM_FRAME_VERSION_ID, frame_data, COM_FRAME_VERSION_SIZE); // Frame ID 0xFF for version string

		vPortFree(get_version_msg->data);
		vPortFree(frame_data);
	}
	break;
	default:
		// unknown message id, do nothing
		break;
	}

	vPortFree((void*)msg);
}

void COM_RunRX(void) {
	uint8_t header[4]={0};
	uint8_t frame_id = 0;
	uint16_t length = 0;

	uint8_t received_checksum = 0;
	uint8_t computed_checksum = 0;

	while (header[0] != COM_FRAME_START_BYTE) {
		// Read one byte at a time until start byte is found
		COM_Read(&header[0], 1);
	}

	// Get header (frame ID + length)
	COM_Read(&header[1], 3);

	frame_id = header[1];
	length = ((uint16_t) header[2] << 8) | header[3];

	// Process data
	// Read data + checksum

	uint8_t *data = pvPortMalloc(length + 1);
	if (data == NULL) {
		// Memory allocation failed, assert
		assert_param(0);
	}

	COM_Read(data, length + 1);

	// Process data and checksum
	received_checksum = data[length];
	for (uint16_t i = 0; i < 4; i++) {
		computed_checksum += header[i];
	}

	for (uint16_t i = 0; i < length; i++) {
		computed_checksum += data[i];
	}

	if (received_checksum == computed_checksum) {
		// Checksum valid, process frame
		COM_Msg_RX_typeDef *rx_msg = pvPortMalloc(sizeof(COM_Msg_RX_typeDef));
		if (rx_msg == NULL) {
			// Memory allocation failed, drop the message and assert
			vPortFree(data);
			assert_param(0);
		}

		rx_msg->header.id	= 0;

		switch (frame_id) {
		case COM_FRAME_GET_VERSION_ID:
			rx_msg->header.id = COM_MSG_GET_VERSION_ID;
			break;
		case COM_FRAME_RESET_HEADING_ID:
			rx_msg->header.id = COM_MSG_RESET_HEADING_ID;
			break;
		case COM_FRAME_CALIBRATE_MAG_ID:
			rx_msg->header.id = COM_MSG_CALIBRATE_MAG_ID;
			break;
		default:
			// Unknown frame ID, drop the message
			vPortFree(data);
		}

		if (rx_msg->header.id == 0) {
			// Unknown frame ID, drop the message
			vPortFree(rx_msg);
			vPortFree(data);
			return;
		}

		rx_msg->data = data;
		rx_msg->length = length;

		if (frame_id  == COM_FRAME_GET_VERSION_ID) {
			// Send to COM TX task, directly
			if (xQueueSend(xComLoopQueue, &rx_msg, 0) != pdPASS) {
				// Queue full, drop the message
				vPortFree(rx_msg);
				assert_param(0);
			}
		} else {
			// Send to APP task, no wait
			if (xQueueSend(xAppLoopQueue, &rx_msg, 0) != pdPASS) {
				// Queue full, drop the message
				vPortFree(rx_msg);
				assert_param(0);
			}
		}
	} else {
		vPortFree(data);
	}
}


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

/* ----------------- Helper: get DMA write position ----------------- */
/* Utilise la macro HAL pour récupérer le compteur restant (no direct register access). */
static size_t COM_GetDMAWritePos(void)
{
	/* counter = nombre d'octets restants dans le transfert DMA (macro HAL) */
	uint32_t cnt = __HAL_DMA_GET_COUNTER(com.hdma_rx); /* HAL macro - allowed */
	size_t pos = 0;

	if (com.rx_buffer_len == 0) return 0;

	/* si DMA configuré pour rx_buffer_len, la position courante d'écriture = len - cnt */
	pos = (size_t)(com.rx_buffer_len - cnt);
	if (pos >= com.rx_buffer_len) pos %= com.rx_buffer_len; /* sécurité */

	return pos;
}

/* ----------------- Helper: available bytes in circular buffer ----------------- */
static size_t COM_AvailableBytes(void)
{
	size_t write_pos = COM_GetDMAWritePos();
	if (write_pos >= com.read_pos) {
		return write_pos - com.read_pos;
	} else {
		return com.rx_buffer_len - (com.read_pos - write_pos);
	}
}

/* ----------------- Copy from circular buffer (manages wrap) ----------------- */
static void COM_CopyFromCircular(uint8_t *dst, size_t len)
{
	/* assume caller already verified that len <= available */
	size_t first_chunk = com.rx_buffer_len - com.read_pos;
	if (first_chunk > len) first_chunk = len;

	memcpy(dst, &com.rx_buffer[com.read_pos], first_chunk);
	if (len > first_chunk) {
		memcpy(dst + first_chunk, &com.rx_buffer[0], len - first_chunk);
		com.read_pos = len - first_chunk;
	} else {
		com.read_pos = (com.read_pos + first_chunk) % com.rx_buffer_len;
	}
}

/* ----------------- Optionnelle: fonction non-bloquante (pour debug/tests) ------------- */
size_t COM_Available(void)
{
	return COM_AvailableBytes();
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
