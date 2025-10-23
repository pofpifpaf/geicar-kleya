/**
 * @file    tasks.c
 * @author  Sebastien DI MERCURIO
 * @version V1.0
 * @date    27 Aout 2025
 * @brief   FreeRTOS tasks, queues, semaphores and timers management.
 * This file contains the implementation of FreeRTOS tasks, queues, semaphores, and timers used in the car application.
 * It defines the tasks for application logic, debugging, ultrasonic sensor management, control loop, and calibration events.
 */

#include "tasks.h"
#include "configuration.h"

#include "app.h"

#include "sensors.h"
#include "can_communication.h"
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
 * Déclaration de la tâche TASKS_CANCommunicationEvent (statique)
 * ------------------------------------------------------------------------- */
void TASKS_CANCommunicationEvent(void *argument);
/* Buffer pour la pile et le TCB */
static StackType_t xCANCommunicationTaskStack[ CAN_COMMUNICATION_TASK_STACK_SIZE ];
static StaticTask_t xCANCommunicationTaskTCB;
/* Handle vers la tâche */
static TaskHandle_t xCANCommunicationTaskHandle = NULL;

/* -------------------------------------------------------------------------
 * Déclaration du buffer pour la queue xAppLoopQueue
 * ------------------------------------------------------------------------- */
static uint8_t ucAppLoopQueueStorageArea[ APPLOOP_QUEUE_LENGTH * APPLOOP_QUEUE_ITEM_SIZE ];
static StaticQueue_t xAppLoopStaticQueue;
QueueHandle_t xAppLoopQueue = NULL;

/* -------------------------------------------------------------------------
 * Déclaration du buffer pour la queue xEcompassLoopQueue
 * ------------------------------------------------------------------------- */
static uint8_t ucEcompassLoopQueueStorageArea[ ECOMPASSLOOP_QUEUE_LENGTH * ECOMPASSLOOP_QUEUE_ITEM_SIZE ];
static StaticQueue_t xEcompassLoopStaticQueue;
QueueHandle_t xEcompassLoopQueue = NULL;

/* -------------------------------------------------------------------------
 * Déclaration du sémaphore de calibration (statique)
 * ------------------------------------------------------------------------- */
static StaticSemaphore_t xCalibrationSemaphoreBuffer;
SemaphoreHandle_t xCalibrationSemaphore = NULL;

/* -------------------------------------------------------------------------
 * Déclaration du timer pour l'envoi des données moteurs (statique)
 * ------------------------------------------------------------------------- */
static void SensorsTimerCallback(TimerHandle_t xTimer);
static StaticTimer_t xSensorsTimerBuffer;
static TimerHandle_t xSensorsTimer   = NULL;

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

	/* Création de la tâche CANCommunicationEvent (statiquement) */
	xCANCommunicationTaskHandle = xTaskCreateStatic(TASKS_CANCommunicationEvent, // fonction de la tâche
			"CalibrationEvent",             // nom (debug)
			CAN_COMMUNICATION_TASK_STACK_SIZE,   // taille pile (en mots de 32 bits)
			NULL,                  // paramètre d’entrée
			CAN_COMMUNICATION_TASK_PRIORITY,     // priorité
			xCANCommunicationTaskStack,         // buffer pile
			&xCANCommunicationTaskTCB           // buffer TCB
	);

	if (xCANCommunicationTaskHandle == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Création du sémaphore de calibration (statiquement) */
	xCalibrationSemaphore = xSemaphoreCreateBinaryStatic(&xCalibrationSemaphoreBuffer);
	if (xCalibrationSemaphore == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Au démarrage, le sémaphore est "pris" */
	xSemaphoreTake(xCalibrationSemaphore, 0);

	/* Timer moteur */
	xSensorsTimer = xTimerCreateStatic(
			"SensorsTimer",                                  // nom
			pdMS_TO_TICKS(SENSORS_TIMER_PERIOD_MS),          // période
			pdTRUE,                                        // auto-reload
			(void*)0,                                      // identifiant (optionnel)
			SensorsTimerCallback,                            // callback
			&xSensorsTimerBuffer                             // buffer statique
	);
	configASSERT(xSensorsTimer != NULL);
	if (xSensorsTimer == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	if (xTimerStart(xSensorsTimer, 0) != pdPASS) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}
}

/**
 * @brief  Task function for the main application loop.
 * This function processes messages received in the application queue.
 * It runs indefinitely, handling messages as they arrive.
 * @param  argument: Not used
 */
void TASKS_AppLoop(void *argument ) {
	void *pReceived = NULL;

	for(;;)
	{
#if defined (__TESTS__)
		TESTS_Run(); // Run tests if defined
#else
		/* Attente infinie d’un élément dans la queue */
		if (xQueueReceive(xAppLoopQueue, &pReceived, portMAX_DELAY) == pdPASS)
		{
			if (pReceived != NULL) {

				/* Traitement de l’élément reçu */
				// Exemple : cast et utilisation
				// MyStruct_t *msg = (MyStruct_t*) pReceived;
				APP_Run((AppMessage_typeDef*) pReceived);
			}
		}
#endif /* __TESTS__ */
	}
}

/**
 * @brief  Task function for the debug loop.
 * This function runs periodically to perform debug tasks.
 * It runs indefinitely, executing its logic at defined intervals.
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
 * @brief  Task function for the ultrasound measurement loop.
 * This function continuously triggers ultrasonic measurements.
 * It runs indefinitely, starting new measurements as soon as the previous ones are finished.
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
 * @brief  Task function for the main application loop.
 * This function processes messages received in the application queue.
 * It runs indefinitely, handling messages as they arrive.
 * @param  argument: Not used
 */
void TASKS_EcompassLoop(void *argument ) {
	void *pReceived = NULL;

	for(;;)
	{
#if defined (__TESTS__)
		// TODO: add tests for eCompass
#else
		/* Attente infinie d’un élément dans la queue */
		if (xQueueReceive(xAppLoopQueue, &pReceived, portMAX_DELAY) == pdPASS)
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
 * @brief  Task function for handling CAN communication events.
 * This function processes incoming CAN messages.
 * It runs indefinitely, handling CAN communication as messages are received.
 * @param  argument: Not used
 */
void TASKS_CANCommunicationEvent(void *argument) {
	CAN_COM_ReceiveTask();

	for (;;) {
		// This function never returns, it runs indefinitely
		vTaskDelay(pdMS_TO_TICKS(1000)); // Just to avoid compiler warning
	}
}

/**
 * @brief  Callback function for the motor timer.
 * This function is called when the motor timer expires.
 * It performs sending motors measurements.
 */
static void SensorsTimerCallback(TimerHandle_t xTimer) {
	/* Send sensors measurements to application main loop, for CAN formating */
	SENSORS_SendMesures();
}


