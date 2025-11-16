/**
 * @file debug.c
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 14 November 2023
 *
 * @brief Debug functions.
 * This file contains the declarations for debug functions.
 */

#include "debug.h"

#include "app.h"
#include "configuration.h"

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

// ITM Stimulus Port pour SWO
#define ITM_STIMULUS_PORT_PRINTF 			0
#define ITM_STIMULUS_PORT_PERIODIC_DEBUG 	1

char DEBUG_Buffer[DEBUG_BUFFER_SIZE];
uint16_t runTimeCounterOverflow = 0;

size_t xFreeHeapSize;

SENSORS_TriaxeValues_t DEBUG_acc;
SENSORS_TriaxeValues_t DEBUG_gyro;
SENSORS_TriaxeValues_t DEBUG_mag;
SENSORS_EnvironementMesures_t DEBUG_env;
ECOMPASS_Values_t DEBUG_ecompass;

/**
 * @brief Print a string via ITM.
 *
 * @param port The ITM stimulus port to use.
 * @param str The string to print.
 */
void DEBUG_PrintITM(uint8_t port, char *str);

/**
 * @brief Print a string via ITM on the default port.
 *
 * @param str The string to print.
 */
void DEBUG_PrintITM(uint8_t port, char *str) {
	if (ITM->TCR & ITM_TCR_ITMENA_Msk) { // Vérifie si l'ITM est activé
		while (*str!=0) {
			while (ITM->PORT[port].u32 == 0) {} // Attend que le port soit prêt
			ITM->PORT[port].u8 = (uint8_t)*str;   // Écrit un caractère

			str++;
		}
	}
}

/**
 * @brief Print a string via ITM on the default port.
 *
 * @param str The string to print.
 */
void DEBUG_Print(char *str) {
	DEBUG_PrintITM(ITM_STIMULUS_PORT_PRINTF, str);
}

/**
 * @brief Print periodic debug information via ITM.
 * This function collects from Freertos and prints periodic debug information,
 * including task states and sensor data.
 */
void DEBUG_PrintPeriodicInfo(void) {
	/******************************************************
	 * Collecte et affiche les informations périodiques
	 * Données liées aux taches et à la mémoire dynamique
	 * Canal de debug 1
	 **************************************************/

	vTaskList(DEBUG_Buffer); // Collecte les stats
	DEBUG_PrintITM(ITM_STIMULUS_PORT_PERIODIC_DEBUG,"Task\tState\tPrio\tStack\tNum\n");
	DEBUG_PrintITM(ITM_STIMULUS_PORT_PERIODIC_DEBUG,DEBUG_Buffer);
	DEBUG_PrintITM(ITM_STIMULUS_PORT_PERIODIC_DEBUG,"\n");

	xFreeHeapSize = xPortGetFreeHeapSize();

	// You can print this value for debugging purposes.
	snprintf(DEBUG_Buffer,DEBUG_BUFFER_SIZE-1,"Current Free Heap Size: %u bytes\n", xFreeHeapSize);
	DEBUG_PrintITM(ITM_STIMULUS_PORT_PERIODIC_DEBUG,DEBUG_Buffer);

	// You can also check if the free heap size is getting too low.
	if (xFreeHeapSize < 500) // Example threshold: 500 bytes
	{
		snprintf(DEBUG_Buffer,DEBUG_BUFFER_SIZE-1,"WARNING: Low heap memory!\n");
		DEBUG_PrintITM(ITM_STIMULUS_PORT_PERIODIC_DEBUG,DEBUG_Buffer);
	}

	/******************************************************
	 * Affichage des données des capteurs et ecompass
	 * Canal de debug principal 0
	 **************************************************/

	snprintf(DEBUG_Buffer, DEBUG_BUFFER_SIZE-1,
			"\r\nAcc (g):\r\n X: %f\r\n Y: %f\r\n Z: %f\r\n",
			DEBUG_acc.x, DEBUG_acc.y, DEBUG_acc.z);
	DEBUG_PrintITM(ITM_STIMULUS_PORT_PRINTF, DEBUG_Buffer);

	snprintf(DEBUG_Buffer, DEBUG_BUFFER_SIZE-1,
			"\r\nGyro (dps):\r\n X: %f\r\n Y: %f\r\n Z: %f\r\n",
			DEBUG_gyro.x, DEBUG_gyro.y, DEBUG_gyro.z);
	DEBUG_PrintITM(ITM_STIMULUS_PORT_PRINTF, DEBUG_Buffer);

	snprintf(DEBUG_Buffer, DEBUG_BUFFER_SIZE-1,
			"\r\nMag (dps):\r\n X: %f\r\n Y: %f\r\n Z: %f\r\n",
			DEBUG_mag.x, DEBUG_mag.y, DEBUG_mag.z);
	DEBUG_PrintITM(ITM_STIMULUS_PORT_PRINTF, DEBUG_Buffer);

	snprintf(DEBUG_Buffer, DEBUG_BUFFER_SIZE-1,
			"\r\nPressure (hpa): %f \r\nHumidity (%%): %f\r\nTemperature (°C): %f\r\n",
			DEBUG_env.pressure, DEBUG_env.humidity, DEBUG_env.temperature);
	DEBUG_PrintITM(ITM_STIMULUS_PORT_PRINTF, DEBUG_Buffer);

	snprintf(DEBUG_Buffer, DEBUG_BUFFER_SIZE-1,"\r\n======================\r\n");
	DEBUG_PrintITM(ITM_STIMULUS_PORT_PRINTF, DEBUG_Buffer);

	snprintf(DEBUG_Buffer, DEBUG_BUFFER_SIZE-1,
			"\r\nEcompass Attitude:\r\n Yaw: %f\r\n Pitch: %f\r\n Roll: %f\r\n Heading: %f (Valid: %ld)\r\n",
			DEBUG_ecompass.yaw, DEBUG_ecompass.pitch, DEBUG_ecompass.roll, DEBUG_ecompass.heading,
			DEBUG_ecompass.heading_valid);
	DEBUG_PrintITM(ITM_STIMULUS_PORT_PRINTF, DEBUG_Buffer);
}

/**
 * @brief Update motion sensor debug data.
 * This function updates the debug data for motion sensors (accelerometer, gyroscope, magnetometer).
 *
 * @param acc The accelerometer data.
 * @param gyro The gyroscope data.
 * @param mag The magnetometer data.
 */
void DEBUG_UpdateMotionData(SENSORS_TriaxeValues_t acc,
		SENSORS_TriaxeValues_t gyro,
		SENSORS_TriaxeValues_t mag) {
	DEBUG_acc = acc;
	DEBUG_gyro = gyro;
	DEBUG_mag = mag;
}

/**
 * @brief Update environmental sensor debug data.
 * This function updates the debug data for environmental sensors (temperature, pressure, humidity).
 *
 * @param env The environmental sensor data.
 */
void DEBUG_UpdateEnvData(SENSORS_EnvironementMesures_t env) {
	DEBUG_env = env;
}

/**
 * @brief Update ecompass debug data.
 * This function updates the debug data for the ecompass (yaw, pitch, roll, heading).
 *
 * @param ecompass The ecompass data.
 */
void DEBUG_UpdateECompassData(ECOMPASS_Values_t ecompass) {
	DEBUG_ecompass = ecompass;
}

/**
 * @brief Handle a panic situation.
 * This function is called when a panic situation occurs, printing the file and line number.
 *
 * @param file The file where the panic occurred.
 * @param line The line number where the panic occurred.
 */
void DEBUG_Panic(uint8_t *file, uint32_t line) {
	snprintf(DEBUG_Buffer, DEBUG_BUFFER_SIZE - 1,
			"\r\nPANIC at line %lu in file %s\r\n", line, file);
	printf(DEBUG_Buffer);

	xFreeHeapSize = xPortGetFreeHeapSize();

	// You can print this value for debugging purposes.
	snprintf(DEBUG_Buffer,DEBUG_BUFFER_SIZE-1,"Current Free Heap Size: %u bytes\r\n", xFreeHeapSize);
	printf(DEBUG_Buffer);

	// Optionally, you can enter an infinite loop to halt the system
	while (1) {
		// Blink an LED or take other actions to indicate a panic state
	}
}

// ----- Run time stats configuration for FreeRTOS -----

extern TIM_HandleTypeDef htim7; // Déclarez la structure de handle de CubeMX
extern void MX_TIM7_Init(void); // Déclarez la fonction d'initialisation de CubeMX

/**
 * @brief Configure the timer for run time stats.
 * This function initializes TIM7 to be used as the run time counter for FreeRTOS.
 */
void configureTimerForRunTimeStats(void)
{
	// C'est la fonction générée par CubeMX pour initialiser TIM7
	MX_TIM7_Init();
	runTimeCounterOverflow=0;
	HAL_TIM_Base_Start_IT(&htim7); // Démarrer le timer en mode interruption
	// Démarrer le timer en mode compteur simple (sans interruption)
	//HAL_TIM_Base_Start(&htim7);
}

/**
 * @brief Run time counter overflow handler.
 * This function handles the overflow of the run time counter used by FREERTOS for task statistics.
 */
void runTimeCounterOverflowHandler(void) {
	runTimeCounterOverflow++;
}

/**
 * @brief Get the current value of the run time counter.
 * This function returns the current value of the run time counter used by FreeRTOS.
 *
 * @return unsigned long The current value of the run time counter.
 */
unsigned long getRunTimeCounterValue(void)
{
	return (((long)(runTimeCounterOverflow))<<16) + __HAL_TIM_GET_COUNTER(&htim7);
}
