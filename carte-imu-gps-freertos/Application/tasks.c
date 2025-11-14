/**
 * @file    tasks.c
 * @author  Sebastien DI MERCURIO
 * @version V1.0
 * @date    27 Aout 2025
 * @brief   FreeRTOS tasks, queues, semaphores and timers management.
 * This file contains the implementation of FreeRTOS tasks, queues, semaphores, and timers used in the car application.
 * It defines the tasks for application logic, debugging, ultrasonic sensor management, control loop, and calibration events.
 */

#include <com_mgr.h>
#include "tasks.h"
#include "configuration.h"

#include "app.h"

#include "sensors.h"
#include "ecompass.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include "debug.h"

#if defined (__TESTS__)
#include "tests.h"
#endif /* __TESTS__ */

/* -------------------------------------------------------------------------
 * Déclaration de la tâche TASKS_AppLoop (statique)
 * ------------------------------------------------------------------------- */
void TASKS_AppLoop(void *argument);
/* Buffer pour la pile et le TCB */
static StackType_t xAppLoopTaskStack[ APPLOOP_TASK_STACK_SIZE ];
static StaticTask_t xAppLoopTaskTCB;

/* Handle vers la tâche */
static TaskHandle_t xAppLoopTaskHandle = NULL;

/* -------------------------------------------------------------------------
 * Déclaration de la tâche TASKS_ComTXLoop (statique)
 * ------------------------------------------------------------------------- */
void TASKS_ComTXLoop(void *argument);
/* Buffer pour la pile et le TCB */
static StackType_t xComLoopTXTaskStack[ COMTXLOOP_TASK_STACK_SIZE ];
static StaticTask_t xComTXLoopTaskTCB;

/* Handle vers la tâche */
static TaskHandle_t xComTXLoopTaskHandle = NULL;

/* -------------------------------------------------------------------------
 * Déclaration de la tâche TASKS_ComRXLoop (statique)
 * ------------------------------------------------------------------------- */
void TASKS_ComRXLoop(void *argument);
/* Buffer pour la pile et le TCB */
static StackType_t xComLoopRXTaskStack[ COMTXLOOP_TASK_STACK_SIZE ];
static StaticTask_t xComRXLoopTaskTCB;
/* Handle vers la tâche */
static TaskHandle_t xComRXLoopTaskHandle = NULL;

/* -------------------------------------------------------------------------
 * Déclaration de la tâche TASKS_DebugLoop (statique)
 * ------------------------------------------------------------------------- */
void TASKS_DebugLoop(void *argument);
/* Buffer pour la pile et le TCB */
static StackType_t xDebugLoopTaskStack[ DEBUGLOOP_TASK_STACK_SIZE ];
static StaticTask_t xDebugLoopTaskTCB;
/* Handle vers la tâche */
static TaskHandle_t xDebugLoopTaskHandle = NULL;

/* -------------------------------------------------------------------------
 * Déclaration de la tâche TASKS_UltrasoundLoop (statique)
 * ------------------------------------------------------------------------- */
void TASKS_SensorsLoop(void *argument);
/* Buffer pour la pile et le TCB */
static StackType_t xSensorsLoopTaskStack[ SENSORS_TASK_STACK_SIZE ];
static StaticTask_t xSensorsLoopTaskTCB;
/* Handle vers la tâche */
static TaskHandle_t xSensorsLoopTaskHandle = NULL;

/* -------------------------------------------------------------------------
 * Déclaration de la tâche TASKS_EcompassLoop (statique)
 * ------------------------------------------------------------------------- */
void TASKS_EcompassLoop(void *argument);
/* Buffer pour la pile et le TCB */
static StackType_t xEcompassLoopTaskStack[ ECOMPASSLOOP_TASK_STACK_SIZE ];
static StaticTask_t xEcompassLoopTaskTCB;

/* Handle vers la tâche */
static TaskHandle_t xEcompassLoopTaskHandle = NULL;

/* -------------------------------------------------------------------------
 * Déclaration du buffer pour la queue xAppLoopQueue
 * ------------------------------------------------------------------------- */
static uint8_t ucAppLoopQueueStorageArea[ APPLOOP_QUEUE_LENGTH * APPLOOP_QUEUE_ITEM_SIZE ];
static StaticQueue_t xAppLoopStaticQueue;
QueueHandle_t xAppLoopQueue = NULL;

/* -------------------------------------------------------------------------
 * Déclaration du buffer pour la queue xComLoopQueue
 * ------------------------------------------------------------------------- */
static uint8_t ucComLoopQueueStorageArea[ COMLOOP_QUEUE_LENGTH * COMLOOP_QUEUE_ITEM_SIZE ];
static StaticQueue_t xComLoopStaticQueue;
QueueHandle_t xComLoopQueue = NULL;

/* -------------------------------------------------------------------------
 * Déclaration du buffer pour la queue xEcompassLoopQueue
 * ------------------------------------------------------------------------- */
static uint8_t ucEcompassLoopQueueStorageArea[ ECOMPASSLOOP_QUEUE_LENGTH * ECOMPASSLOOP_QUEUE_ITEM_SIZE ];
static StaticQueue_t xEcompassLoopStaticQueue;
QueueHandle_t xEcompassLoopQueue = NULL;

/* -------------------------------------------------------------------------
 * Déclaration du timer pour l'envoi des données des capteurs de mouvement (statique)
 * ------------------------------------------------------------------------- */
static void MotionTimerCallback(TimerHandle_t xTimer);
static StaticTimer_t xMotionTimerBuffer;
static TimerHandle_t xMotionTimer   = NULL;

/* -------------------------------------------------------------------------
 * Déclaration du timer pour l'envoi des données des capteurs environnementaux (statique)
 * ------------------------------------------------------------------------- */
static void EnvTimerCallback(TimerHandle_t xTimer);
static StaticTimer_t xEnvTimerBuffer;
static TimerHandle_t xEnvTimer   = NULL;

/* -------------------------------------------------------------------------
 * Déclaration du timer pour l'envoi des données de la boussole electronique (statique)
 * ------------------------------------------------------------------------- */
static void EcompassTimerCallback(TimerHandle_t xTimer);
static StaticTimer_t xEcompassTimerBuffer;
static TimerHandle_t xEcompassTimer   = NULL;

/*
 * @brief  Initialize tasks, queues, semaphores and timers.
 * This function creates the necessary FreeRTOS components for the application.
 * It sets up tasks, queues, semaphores, and timers used in the application.
 */
void TASKS_Init(void) {
	/* Création de la file pour l'application (statiquement) */
	xAppLoopQueue = xQueueCreateStatic(
			APPLOOP_QUEUE_LENGTH,          // nombre d’éléments
			APPLOOP_QUEUE_ITEM_SIZE,       // taille d’un élément
			ucAppLoopQueueStorageArea,    // buffer pour les données
			&xAppLoopStaticQueue          // buffer pour la structure de contrôle
	);

	if (xAppLoopQueue == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Création de la tâche AppLoop (statiquement) */
	xAppLoopTaskHandle = xTaskCreateStatic(
			TASKS_AppLoop,          // fonction de la tâche
			"AppLoop",             // nom (debug)
			APPLOOP_TASK_STACK_SIZE,   // taille pile (en mots de 32 bits)
			NULL,                  // paramètre d’entrée
			APPLOOP_TASK_PRIORITY,     // priorité
			xAppLoopTaskStack,         // buffer pile
			&xAppLoopTaskTCB           // buffer TCB
	);

	if (xAppLoopTaskHandle == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Création de la file pour la tache d'envoi sur uart (statiquement) */
	xComLoopQueue = xQueueCreateStatic(
			COMLOOP_QUEUE_LENGTH,          // nombre d’éléments
			COMLOOP_QUEUE_ITEM_SIZE,       // taille d’un élément
			ucComLoopQueueStorageArea,    // buffer pour les données
			&xComLoopStaticQueue          // buffer pour la structure de contrôle
	);

	if (xComLoopQueue == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Création de la tâche ComTXLoop (statiquement) */
	xComTXLoopTaskHandle = xTaskCreateStatic(
			TASKS_ComTXLoop,          // fonction de la tâche
			"ComTXLoop",             // nom (debug)
			COMTXLOOP_TASK_STACK_SIZE,   // taille pile (en mots de 32 bits)
			NULL,                  // paramètre d’entrée
			COMTXLOOP_TASK_PRIORITY,     // priorité
			xComLoopTXTaskStack,         // buffer pile
			&xComTXLoopTaskTCB           // buffer TCB
	);

	if (xComTXLoopTaskHandle == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Création de la tâche ComRXLoop (statiquement) */
	xComRXLoopTaskHandle = xTaskCreateStatic(TASKS_ComRXLoop, // fonction de la tâche
			"ComRXLoop",             // nom (debug)
			COMTXLOOP_TASK_STACK_SIZE,   // taille pile (en mots de 32 bits)
			NULL,                  // paramètre d’entrée
			COMTXLOOP_TASK_PRIORITY,     // priorité
			xComLoopRXTaskStack,         // buffer pile
			&xComRXLoopTaskTCB           // buffer TCB
			);

	if (xComRXLoopTaskHandle == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Création de la tâche DebugLoop (statiquement) */
	xDebugLoopTaskHandle = xTaskCreateStatic(
			TASKS_DebugLoop,          // fonction de la tâche
			"DebugLoop",             // nom (debug)
			DEBUGLOOP_TASK_STACK_SIZE,   // taille pile (en mots de 32 bits)
			NULL,                  // paramètre d’entrée
			DEBUGLOOP_TASK_PRIORITY,     // priorité
			xDebugLoopTaskStack,         // buffer pile
			&xDebugLoopTaskTCB           // buffer TCB
	);

	if (xDebugLoopTaskHandle == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Création de la tâche SensorsLoop (statiquement) */
	xSensorsLoopTaskHandle = xTaskCreateStatic(TASKS_SensorsLoop, // fonction de la tâche
			"SensorsLoop",             // nom (debug)
			SENSORS_TASK_STACK_SIZE,   // taille pile (en mots de 32 bits)
			NULL,                  // paramètre d’entrée
			SENSORS_TASK_PRIORITY,     // priorité
			xSensorsLoopTaskStack,         // buffer pile
			&xSensorsLoopTaskTCB           // buffer TCB
	);

	if (xSensorsLoopTaskHandle == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();

	}

	/* Création de la file pour la boussole electronique (statiquement) */
	xEcompassLoopQueue = xQueueCreateStatic(
			ECOMPASSLOOP_QUEUE_LENGTH,          // nombre d’éléments
			ECOMPASSLOOP_QUEUE_ITEM_SIZE,       // taille d’un élément
			ucEcompassLoopQueueStorageArea,    // buffer pour les données
			&xEcompassLoopStaticQueue          // buffer pour la structure de contrôle
	);

	if (xEcompassLoopQueue == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Création de la tâche EcompassLoop (statiquement) */
	xEcompassLoopTaskHandle = xTaskCreateStatic(
			TASKS_EcompassLoop,          // fonction de la tâche
			"EcompassLoop",             // nom (debug)
			ECOMPASSLOOP_TASK_STACK_SIZE,   // taille pile (en mots de 32 bits)
			NULL,                  // paramètre d’entrée
			ECOMPASSLOOP_TASK_PRIORITY,     // priorité
			xEcompassLoopTaskStack,         // buffer pile
			&xEcompassLoopTaskTCB           // buffer TCB
	);

	if (xEcompassLoopTaskHandle == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Timer capteurs */
	xMotionTimer = xTimerCreateStatic(
			"MotionTimer",                                  // nom
			pdMS_TO_TICKS(MOTION_COM_PERIOD_MS),          // période
			pdTRUE,                                        // auto-reload
			(void*)0,                                      // identifiant (optionnel)
			MotionTimerCallback,                            // callback
			&xMotionTimerBuffer                             // buffer statique
	);
	configASSERT(xMotionTimer != NULL);
	if (xMotionTimer == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Timer environnement */
	xEnvTimer = xTimerCreateStatic(
			"EnvTimer",                            // nom
			pdMS_TO_TICKS(ENV_COM_PERIOD_MS),          // période
			pdTRUE,                                        // auto-reload
			(void*) 0,                                // identifiant (optionnel)
			EnvTimerCallback,                            // callback
			&xEnvTimerBuffer                             // buffer statique
	);
	configASSERT(xEnvTimer != NULL);
	if (xEnvTimer == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Timer ecompass */
	xEcompassTimer = xTimerCreateStatic(
			"EcompassTimer",                                  // nom
			pdMS_TO_TICKS(ECOMPASS_COM_PERIOD_MS),          // période
			pdTRUE,                                        // auto-reload
			(void*)0,                                      // identifiant (optionnel)
			EcompassTimerCallback,                            // callback
			&xEcompassTimerBuffer                             // buffer statique
	);
	configASSERT(xEcompassTimer != NULL);
	if (xEcompassTimer == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	if (xTimerStart(xMotionTimer, 0) != pdPASS) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	if (xTimerStart(xEnvTimer, 0) != pdPASS) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	if (xTimerStart(xEcompassTimer, 0) != pdPASS) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	vQueueAddToRegistry(xEcompassLoopQueue, "EcompassLoopQueue");
	vQueueAddToRegistry(xAppLoopQueue, "AppLoopQueue");
	vQueueAddToRegistry(xComLoopQueue, "PrintLoopQueue");
}

/**
 * @brief  Task function for the main application loop.
 * This function processes messages received in the application queue.
 * It runs indefinitely, handling messages as they arrive.
 *
 * @param  argument: Not used
 */
void TASKS_AppLoop(void *argument ) {
	void *pReceived = NULL;

	for(;;)
	{
#if defined (__TESTS__)
		TESTS_Run(); // Run tests if defined
#else
		/* Attente infinie d’un élément dans la queue AppLoop*/
		if (xQueueReceive(xAppLoopQueue, &pReceived, portMAX_DELAY) == pdPASS)
		{
			if (pReceived != NULL) {

				/* Traitement du message reçu par l'application */
				APP_Run((AppMessage_typeDef*) pReceived);
			}
		}
#endif /* __TESTS__ */
	}
}

/**
 * @brief  Task function for the communication transmit loop.
 * This function processes messages received in the application queue.
 * It runs indefinitely, handling messages as they arrive.
 *
 * @param  argument: Not used
 */
void TASKS_ComTXLoop(void *argument ) {
	void *pReceived = NULL;

	for(;;)
	{
		/* Attente infinie d’un élément dans la queue PrintLoop*/
		if (xQueueReceive(xComLoopQueue, &pReceived, portMAX_DELAY) == pdPASS)
		{
			if (pReceived != NULL) {

				/* Traitement du message reçu par le gestionnaire de communication */
				COM_RunTX((AppMessage_typeDef*) pReceived);
			}
		}
	}
}

/**
 * @brief  Task function for the communication receive loop.
 * This function continuously runs the communication receive logic.
 * It runs indefinitely, processing incoming data as it arrives.
 *
 * @param  argument: Not used
 *
 * @remark periodically called by freertos timer
 */
void TASKS_ComRXLoop(void *argument ) {
	for (;;) {
		COM_RunRX();
	}
}

/**
 * @brief  Task function for the debug loop.
 * This function runs periodically to perform debug tasks.
 * It runs indefinitely, executing its logic at defined intervals.
 *
 * @param  argument: Not used
 */
void TASKS_DebugLoop(void *argument) {
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(DEBUG_LOOP_PERIOD_MS);

	/* Initialise la référence de temps */
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		DEBUG_PrintPeriodicInfo();

		// Time is compensated from others events that can make processing longer
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
}

/**
 * @brief  Task function for sensors loop.
 * This function continuously triggers sensors measurements.
 * Periodically called to initiate sensor data acquisition.
 *
 * @param  argument: Not used
 */
void TASKS_SensorsLoop(void *argument) {
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(SENSORS_LOOP_PERIOD_MS);

	/* Initialise la référence de temps */
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		SENSORS_TriggerMeasurements();

		// Time is compensated from others events that can make processing longer
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
}

/**
 * @brief  Task function for ecompass loop.
 * This function processes messages received in the ecompass queue.
 * It runs indefinitely, handling messages as they arrive.
 *
 * @param  argument: Not used
 */
void TASKS_EcompassLoop(void *argument ) {
	void *pReceived = NULL;

	for(;;)
	{
#if defined (__TESTS__)
		// TODO: add tests for eCompass
#else
		/* Attente infinie d’un élément dans la queue ECompass*/
		if (xQueueReceive(xEcompassLoopQueue, &pReceived, portMAX_DELAY) == pdPASS)
		{
			if (pReceived != NULL) {

				/* Traitement de l’élément reçu */
				// Exemple : cast et utilisation
				// MyStruct_t *msg = (MyStruct_t*) pReceived;
				ECOMPASS_ProcessMessage((ECOMPASS_SensorsValues_t*) pReceived);
			}
		}
#endif /* __TESTS__ */
	}
}

/**
 * @brief  Callback function for the sensors timer.
 * This function is called when the sensors timer expires.
 */
static void MotionTimerCallback(TimerHandle_t xTimer) {
	/* Send sensors measurements to application main loop, for CAN formating */
	SENSORS_SendMotionMesures();
}

/**
 * @brief  Callback function for the environmental sensors timer.
 * This function is called when the environmental sensors timer expires.
 */
static void EnvTimerCallback(TimerHandle_t xTimer) {
	/* Send environmental sensors measurements to application main loop, for CAN formating */
	SENSORS_SendEnvMesures();
}

/**
 * @brief  Callback function for the ecompass timer.
 * This function is called when the ecompass timer expires.
 */
static void EcompassTimerCallback(TimerHandle_t xTimer) {
	/* Send Ecompass measurements to application main loop, for CAN formating */
	ECOMPASS_SendMesures();
}

