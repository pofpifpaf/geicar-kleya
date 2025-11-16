/**
 * @file com_mgr.h
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 14 November 2023
 *
 * @brief communication handler.
 * This file contains the communication manager definitions and function prototypes.
 */

#ifndef COM_MGR_H_
#define COM_MGR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "sensors.h"
#include "ecompass.h"

/**
 * @brief COM_Msg_RX_typeDef message structure.
 *
 * This structure represents a received frame by the communication manager.
 */
typedef struct {
	AppMessage_typeDef header;
	uint8_t *data;
	uint16_t length;
} COM_Msg_RX_typeDef;

/**
 * @brief COM_Msg_Motion_typeDef message structure.
 *
 * This structure represents motion sensor data to be sent over USB by the communication manager.
 */
typedef struct {
	AppMessage_typeDef header;
	SENSORS_TriaxeValues_t acc;
	SENSORS_TriaxeValues_t gyro;
	SENSORS_TriaxeValues_t mag;
} COM_Msg_Motion_typeDef;

/**
 * @brief COM_Msg_Env_typeDef message structure.
 *
 * This structure represents environmental sensor data to be sent over USB by the communication manager.
 */
typedef struct {
	AppMessage_typeDef header;
	SENSORS_EnvironementMesures_t env;
} COM_Msg_Env_typeDef;

/**
 * @brief COM_Msg_ECompass_typeDef message structure.
 *
 * This structure represents ecompass data to be sent over USB by the communication manager.
 */
typedef struct {
	AppMessage_typeDef header;
	ECOMPASS_Values_t ecompass;
} COM_Msg_ECompass_typeDef;

/**
 * @brief Initialize the communication manager.
 * This function sets up the necessary software components
 */
void COM_Init(UART_HandleTypeDef *huart,
	              DMA_HandleTypeDef  *hdma_rx);

/**
 * @brief Run the communication transmitter manager.
 *
 * This function processes messages received in the application queue and sent them over USB.
 * It handles frame formatting and transmission.
 *
 * @remark: this function is called by TASKS_ComTXLoop tasks when a new message is received.
 * @param  msg: Pointer to the message to process.
 */
void COM_RunTX(AppMessage_typeDef *msg);

/**
 * @brief Run the communication receive manager.
 *
 * This function is periodically called to handle incoming data from the UART.
 * It processes incoming data as it arrives and sends valid messages to the appropriate queues.
 *
 * @remark: this function is called periodically by TASKS_ComRXLoop.
 */
void COM_RunRX(void);

/**
 * @brief Read data from the communication interface.
 *
 * This function reads a specified number of bytes from the communication interface into the provided buffer.
 *
 * @param dst Pointer to the destination buffer where the read data will be stored.
 * @param len Number of bytes to read.
 */
void COM_Read(uint8_t *dst, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* COM_MGR_H_ */
