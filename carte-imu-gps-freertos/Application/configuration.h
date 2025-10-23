/**
 * @file configuration.h
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 20 Aout 2023
 * @brief Configuration file for the car application.
 * This file contains definitions and configurations for the car's hardware and software components.
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include "main.h"

// Application version
#define APP_VERSION "1.0.0"

// Periodic events, in ms
#define SENSORS_TIMER_PERIOD_MS 	1000		// Period in ms to send motors data (speed and odometers)
#define SENSORS_LOOP_PERIOD_MS		10		// Debug loop period in ms
#define DEBUG_LOOP_PERIOD_MS		1000	// Debug loop period in ms

#define US_MAX_WAIT_TIME_MS 	50 		// Period in ms to update us data

// Tasks, semaphores, queues constants
#define APPLOOP_QUEUE_LENGTH   	10
#define APPLOOP_QUEUE_ITEM_SIZE sizeof(void*)   // Chaque élément est un pointeur

#define ECOMPASSLOOP_QUEUE_LENGTH   	16
#define ECOMPASSLOOP_QUEUE_ITEM_SIZE sizeof(void*)   // Chaque élément est un pointeur

// Tasks stack sizes
#define APPLOOP_TASK_STACK_SIZE 	256   	// en mots de 32 bits
#define DEBUGLOOP_TASK_STACK_SIZE 	256 	// en mots de 32 bits. Besoin de pas mal d'espace pour la fonction sprintf
#define SENSORS_TASK_STACK_SIZE  	128 	// en mots de 32 bits. Pas besoin d'une stack enorme
#define ECOMPASSLOOP_TASK_STACK_SIZE 	512   	// en mots de 32 bits. Pour les calculs de la boussole
#define CAN_COMMUNICATION_TASK_STACK_SIZE 	128 	// en mots de 32 bits. Pas besoin d'une stack enorme

// Tasks priorities
#define CAN_COMMUNICATION_TASK_PRIORITY    	(tskIDLE_PRIORITY + 6) // Highest priority, no blocking function inside
#define SENSORS_TASK_PRIORITY    	(tskIDLE_PRIORITY + 5)
#define ECOMPASSLOOP_TASK_PRIORITY    	(tskIDLE_PRIORITY + 4)
#define APPLOOP_TASK_PRIORITY   	(tskIDLE_PRIORITY + 3)
#define DEBUGLOOP_TASK_PRIORITY 	(tskIDLE_PRIORITY + 1) // Lowest priority

// APP messages IDs
#define SENSORS_SEND_MEASURES_ID	1
#define SENSORS_MOTION_MEASURES_ID	2
#define SENSORS_ENV_MEASURES_ID	    3
#define ECOMPASS_SEND_MEASURES_ID	4
#define TILT_SEND_MEASURES_ID		5
#define CAN_RECEIVED_FRAME_ID		6

// Sensors configuration
#define SENSORS_ACC_ODR_HZ 		104.0f 	// Accelerometer output data rate in Hz
#define SENSORS_GYRO_ODR_HZ 	104.0f 	// Gyroscope output data rate in Hz
#define SENSORS_MAG_ODR_HZ 		100.0f 	// Magnetometer output data rate in Hz

#define SENSORS_ENV_ODR_HZ 		25.0f 	// Environmental sensors output data rate in Hz
#define SENSORS_PRESS_ODR_HZ 	SENSORS_ENV_ODR_HZ 	// Pressure sensor output data rate in Hz
#define SENSORS_HUM_ODR_HZ 		SENSORS_ENV_ODR_HZ  // Humidity sensor output data rate in Hz
#define SENSORS_TEMP_ODR_HZ 	SENSORS_ENV_ODR_HZ 	// Temperature sensor output data rate in Hz

#define SENSORS_ACC_FS_G 		4 		// Accelerometer full scale in g
#define SENSORS_GYRO_FS_DPS 	500 	// Gyroscope full scale in dps
#define SENSORS_MAG_FS_GAUSS 	50 		// Magnetometer full scale in gauss

#define SENSORS_ENVIRONMENTAL_FILTER 	25	// number of value for filter
#define SENSORS_ENVIRONMENTAL_PERIOD    ((int)SENSORS_MAG_ODR_HZ / (int)SENSORS_ENV_ODR_HZ)

#define DEBUG_BUFFER_SIZE 	1024

#endif /* CONFIGURATION_H_ */
