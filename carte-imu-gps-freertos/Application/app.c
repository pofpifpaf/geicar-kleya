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

#include "app.h"

#include "configuration.h"

#include "print_mgr.h"
#include "main.h"
//#include "can.h"
#include "usart.h"
#include "gpio.h"

#include "can_communication.h"
#include "sensors.h"
#include "ecompass.h"
#include "tasks.h"

#include <stdio.h>
#include <stdlib.h>

#include "debug.h"

#if defined (__TESTS__)
#include "tests.h"
#endif

/* Data buffer for CAN messages */
uint8_t data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};

/* Sensors last mesures */
SENSORS_TriaxeValues_t acc_last_values = {0.0f, 0.0f, 0.0f};
SENSORS_TriaxeValues_t gyro_last_values = {0.0f, 0.0f, 0.0f};
SENSORS_TriaxeValues_t mag_last_values = {0.0f, 0.0f, 0.0f};
float pressure_last_value = 0.0f;
float humidity_last_value = 0.0f;
float temperature_last_value = 0.0f;

float pressure_accumulator = 0.0f;
float humidity_accumulator = 0.0f;
float temperature_accumulator = 0.0f;
uint8_t env_sample_count = 0;

/*** Ecompass variables ***/
ECOMPASS_Values_t ecompass_last_values = {0.0f, 0.0f, 0.0f, 0.0f, 0};

#define MAX_BUF_SIZE 256
//static char dataOut[MAX_BUF_SIZE];

/**
 * @brief Initialize the application.
 * This function sets up the necessary software components
 */
void APP_Init(void) {
	// CAN communications
	//CAN_COM_Init();
	SENSORS_Init();
	ECOMPASS_Init((float)SENSORS_MAG_ODR_HZ); // in Hz, at 100 Hz

	// Tasks
	TASKS_Init();

	printf("IMU - GPS.\r\n");
	printf("Application version: %s\r\n\n", APP_VERSION);
	printf("Application started\r\n");
}

/**
 * @brief Run the application.
 *
 * This function is the main loop of the application. It handles the main logic,
 * processes inputs, and updates outputs.
 *
 * @remark: this function never returns, it runs indefinitely.
 */
void APP_Run(AppMessage_typeDef *msg) {
#if defined (__TESTS__)
	TESTS_Run(); // Run tests if defined
#else
	switch (msg->id) {
	case CAN_RECEIVED_FRAME_ID: // a CAN frame was received
		CANReceivedFrame_typeDef *canFrame = (CANReceivedFrame_typeDef*) msg;

		switch (canFrame->can_id) {
		case CAN_ID_COMM_CHECKING:
			if (canFrame->length >= 1 &&
					canFrame->data[0] == COMM_CHECKING_REQUEST) {
				data[0] = COMM_CHECKING_REQUEST;
				data[1] = COMM_CHECKING_ACK;

				CAN_COM_Send(CAN_ID_COMM_CHECKING, data, 2); // Send ack
			}
			break;
		default:
			break;
		}
		break;

		case MOTION_SEND_MEASURES_ID:
			PrintMessage_Motion_typeDef *motion_data_msg = pvPortMalloc(sizeof(PrintMessage_Motion_typeDef));

			if (motion_data_msg != NULL) {
				memset(motion_data_msg, 0, sizeof(PrintMessage_Motion_typeDef)); // ras des valeurs
				motion_data_msg->id = PRINT_MOTION_MSG_ID;
				motion_data_msg->acc = acc_last_values;
				motion_data_msg->gyro = gyro_last_values;
				motion_data_msg->mag = mag_last_values;

				// Send mesures to Print task, no wait
				if (xQueueSend(xPrintLoopQueue, &motion_data_msg, portMAX_DELAY) != pdPASS) {
					// Queue full, drop the message
					vPortFree(motion_data_msg);
					assert_param(0);
				}
			} else {
				// Memory allocation failed, drop the message and assert
				assert_param(0);
			}
			break;

		case ENV_SEND_MEASURES_ID:
			PrintMessage_Env_typeDef *env_data_msg = pvPortMalloc(sizeof(PrintMessage_Env_typeDef));

			if (env_data_msg != NULL) {
				memset(env_data_msg, 0, sizeof(PrintMessage_Env_typeDef)); // ras des valeurs
				env_data_msg->id = PRINT_ECOMPASS_MSG_ID;
				env_data_msg->env.temperature = temperature_last_value;
				env_data_msg->env.pressure = pressure_last_value;
				env_data_msg->env.humidity = humidity_last_value;

				// Send mesures to Print task, no wait
				if (xQueueSend(xPrintLoopQueue, &env_data_msg, portMAX_DELAY) != pdPASS) {
					// Queue full, drop the message
					vPortFree(env_data_msg);
					assert_param(0);
				}
			} else {
				// Memory allocation failed, drop the message and assert
				assert_param(0);
			}
			break;
		case ECOMPASS_SEND_MEASURES_ID:
			PrintMessage_ECompass_typeDef *ecompass_data_msg = pvPortMalloc(sizeof(PrintMessage_ECompass_typeDef));

			if (ecompass_data_msg != NULL) {
				memset(ecompass_data_msg, 0, sizeof(PrintMessage_ECompass_typeDef)); // ras des valeurs
				ecompass_data_msg->id = PRINT_ECOMPASS_MSG_ID;
				ecompass_data_msg->ecompass = ecompass_last_values;

				// Send mesures to Print task, no wait
				if (xQueueSend(xPrintLoopQueue, &ecompass_data_msg, portMAX_DELAY) != pdPASS) {
					// Queue full, drop the message
					vPortFree(ecompass_data_msg);
					assert_param(0);
				}
			} else {
				// Memory allocation failed, drop the message and assert
				assert_param(0);
			}

			break;

		case TILT_SEND_MEASURES_ID:
			CAN_COM_Send(CAN_ID_TILT, data, 6);
			break;

		case ECOMPASS_ATTITUDE_DATA_ID:
			ecompass_last_values = ((ECOMPASS_Attitude_t*)msg)->values;

			/* Update debug data */
			DEBUG_UpdateECompassData(ecompass_last_values);
			break;

		case SENSORS_MOTION_MEASURES_ID:
			// Process sensors measures if needed
			acc_last_values = SENSORS_ACC_RawtoG(&(((SENSORS_MotionMesures_t*)msg)->acc));
			gyro_last_values = SENSORS_GYRO_RawtoDPS(&(((SENSORS_MotionMesures_t*)msg)->gyro));
			mag_last_values = SENSORS_MAG_RawtoMilliGauss(&(((SENSORS_MotionMesures_t*)msg)->mag));

			/* Update debug data */
			DEBUG_UpdateMotionData(acc_last_values, gyro_last_values, mag_last_values);

			ECOMPASS_SensorsValues_t *ecompass_msg = pvPortMalloc(sizeof(ECOMPASS_SensorsValues_t));
			if (ecompass_msg != NULL) {
				memset(ecompass_msg, 0, sizeof(ECOMPASS_SensorsValues_t)); // ras des valeurs
				// sensors_msg->header.id = ECOMPASS_SEND_MEASURES_ID;
				ecompass_msg->elapsedTimeMs = 10; // TODO: add elapsed time if needed
				ecompass_msg->acc = acc_last_values;
				ecompass_msg->gyro = gyro_last_values;
				ecompass_msg->mag = mag_last_values;

				// Send mesures to ECOMPASS task, no wait
				if (xQueueSend(xEcompassLoopQueue, &ecompass_msg, portMAX_DELAY) != pdPASS) {
					// Queue full, drop the message
					vPortFree(ecompass_msg);
					assert_param(0);
				}
			} else {
				// Memory allocation failed, drop the message and assert
				assert_param(0);
			}
			break;

		case SENSORS_ENV_MEASURES_ID:
			pressure_accumulator += ((SENSORS_EnvironementMesures_t*)msg)->pressure;
			temperature_accumulator += ((SENSORS_EnvironementMesures_t*)msg)->temperature;
			humidity_accumulator += ((SENSORS_EnvironementMesures_t*)msg)->humidity;
			env_sample_count++;

			if (env_sample_count >= SENSORS_ENVIRONMENTAL_FILTER) {
				pressure_last_value = pressure_accumulator
						/ (float) env_sample_count;
				humidity_last_value = humidity_accumulator
						/ (float) env_sample_count;
				temperature_last_value = temperature_accumulator
						/ (float) env_sample_count;

				/* Update debug data */
				DEBUG_UpdateEnvData((SENSORS_EnvironementMesures_t){
					.temperature = temperature_last_value,
							.pressure = pressure_last_value,
							.humidity = humidity_last_value});

				// Reset accumulators
				pressure_accumulator = 0.0f;
				humidity_accumulator = 0.0f;
				temperature_accumulator = 0.0f;
				env_sample_count = 0;
			}

		default:
			break;
	}

	vPortFree((void*)msg);
#endif /* __TESTS__ */
}

