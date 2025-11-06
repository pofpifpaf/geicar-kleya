/**
 * @file debug.h
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 29 aout 2025
 * @brief Debug functions for the car application.
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

void DEBUG_PrintITM(uint8_t port, char *str) {
	if (ITM->TCR & ITM_TCR_ITMENA_Msk) { // Vérifie si l'ITM est activé
		while (*str!=0) {
			while (ITM->PORT[port].u32 == 0) {} // Attend que le port soit prêt
			ITM->PORT[port].u8 = (uint8_t)*str;   // Écrit un caractère

			str++;
		}
	}
}

void DEBUG_PrintPeriodicInfo(void) {
	/******************************************************
	 * Collecte et affiche les informations périodiques
	 * Données liées aux taches et à la memoire dynamique
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

void DEBUG_UpdateMotionData(SENSORS_TriaxeValues_t acc,
		SENSORS_TriaxeValues_t gyro,
		SENSORS_TriaxeValues_t mag) {
	DEBUG_acc = acc;
	DEBUG_gyro = gyro;
	DEBUG_mag = mag;
}

void DEBUG_UpdateEnvData(SENSORS_EnvironementMesures_t env) {
	DEBUG_env = env;
}

void DEBUG_UpdateECompassData(ECOMPASS_Values_t ecompass) {
	DEBUG_ecompass = ecompass;
}

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

extern TIM_HandleTypeDef htim7; // Déclarez la structure de handle de CubeMX
extern void MX_TIM7_Init(void); // Déclarez la fonction d'initialisation de CubeMX

void configureTimerForRunTimeStats(void)
{
	// C'est la fonction générée par CubeMX pour initialiser TIM7
	MX_TIM7_Init();
	runTimeCounterOverflow=0;
	HAL_TIM_Base_Start_IT(&htim7); // Démarrer le timer en mode interruption
	// Démarrer le timer en mode compteur simple (sans interruption)
	//HAL_TIM_Base_Start(&htim7);
}

void runTimeCounterOverflowHandler(void) {
	runTimeCounterOverflow++;
}

unsigned long getRunTimeCounterValue(void)
{
	return (((long)(runTimeCounterOverflow))<<16) + __HAL_TIM_GET_COUNTER(&htim7);
}
