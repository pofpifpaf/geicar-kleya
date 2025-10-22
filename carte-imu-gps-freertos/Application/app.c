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

#include "main.h"
//#include "can.h"
#include "usart.h"
#include "gpio.h"

#include "can_communication.h"
#include "sensors.h"
#include "tasks.h"

#include <stdio.h>
#include <stdlib.h>

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

#define MAX_BUF_SIZE 256
static char dataOut[MAX_BUF_SIZE];

//typedef struct displayFloatToInt_s
//{
//	int8_t sign; /* 0 means positive, 1 means negative*/
//	uint32_t  out_int;
//	uint32_t  out_dec;
//} displayFloatToInt_t;

/**
 * @brief  Splits a float into two integer values.
 * @param  in the float value as input
 * @param  out_value the pointer to the output integer structure
 * @param  dec_prec the decimal precision to be used
 * @retval None
 */
//static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec)
//{
//	if (in >= 0.0f)
//	{
//		out_value->sign = 0;
//	}
//	else
//	{
//		out_value->sign = 1;
//		in = -in;
//	}
//
//	in = in + (0.5f / (float)pow(10, (double)dec_prec));
//	out_value->out_int = (int32_t)in;
//	in = in - (float)(out_value->out_int);
//	out_value->out_dec = (int32_t)trunc((double)in * pow(10, (double)dec_prec));
//}

/**
 * @brief Initialize the application.
 * This function sets up the necessary software components
 */
void APP_Init(void) {
	// CAN communications
	//CAN_COM_Init();
	SENSORS_Init();

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

		case SENSORS_SEND_MEASURES_ID:
			//CAN_COM_Send(CAN_ID_SENSORS, data, 6);
			snprintf(dataOut, MAX_BUF_SIZE,
					"\r\nAcc (g):\r\n X: %f\r\n Y: %f\r\n Z: %f\r\n",
					acc_last_values.x, acc_last_values.y, acc_last_values.z);
			printf("%s", dataOut);

			snprintf(dataOut, MAX_BUF_SIZE,
					"\r\nGyro (dps):\r\n X: %f\r\n Y: %f\r\n Z: %f\r\n",
					gyro_last_values.x, gyro_last_values.y, gyro_last_values.z);
			printf("%s", dataOut);

			snprintf(dataOut, MAX_BUF_SIZE,
					"\r\nMag (dps):\r\n X: %f\r\n Y: %f\r\n Z: %f\r\n",
					mag_last_values.x, mag_last_values.y, mag_last_values.z);
			printf("%s", dataOut);

			snprintf(dataOut, MAX_BUF_SIZE,
					"\r\nPressure (hpa): %f \r\nHumidity (%%): %f\r\nTemperature (°C): %f\r\n",
					pressure_last_value, humidity_last_value, temperature_last_value);
			printf("%s", dataOut);

			snprintf(dataOut, MAX_BUF_SIZE,"\r\n======================\r\n");
			printf("%s", dataOut);
			break;

		case ECOMPASS_SEND_MEASURES_ID:
			CAN_COM_Send(CAN_ID_ECOMPASS, data, 6);
			break;

		case TILT_SEND_MEASURES_ID:
			CAN_COM_Send(CAN_ID_TILT, data, 6);
			break;

		case SENSORS_MOTION_MEASURES_ID:
			// Process sensors measures if needed
			acc_last_values = SENSORS_ACC_RawtoG(&(((SENSORS_MotionMesures_t*)msg)->acc));
			gyro_last_values = SENSORS_GYRO_RawtoDPS(&(((SENSORS_MotionMesures_t*)msg)->gyro));
			mag_last_values = SENSORS_MAG_RawtoMilliGauss(&(((SENSORS_MotionMesures_t*)msg)->mag));
			break;

		case SENSORS_ENV_MEASURES_ID:
			pressure_accumulator += ((SENSORS_EnvironementMesures_t*)msg)->pressure;
			humidity_accumulator += ((SENSORS_EnvironementMesures_t*)msg)->humidity;
			temperature_accumulator += ((SENSORS_EnvironementMesures_t*)msg)->temperature;
			env_sample_count++;

			if (env_sample_count >= SENSORS_ENVIRONMENTAL_FILTER) {
				pressure_last_value = pressure_accumulator
						/ (float) env_sample_count;
				humidity_last_value = humidity_accumulator
						/ (float) env_sample_count;
				temperature_last_value = temperature_accumulator
						/ (float) env_sample_count;

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

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  ch: Character to be printed
 * @retval Character sent
 */
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
