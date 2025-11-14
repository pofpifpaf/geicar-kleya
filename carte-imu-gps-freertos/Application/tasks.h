/**
 * @file tasks.h
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 27 Aout 2025
 * @brief Header file for tasks.c
 * This file contains the declarations for FreeRTOS tasks, queues, semaphores, and timers used in the car application.
 */

#ifndef __TASKS_H__
#define __TASKS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Handle vers la queue de la tache TASK_AppLoop */
extern QueueHandle_t xAppLoopQueue;

/* Handle vers la queue de la tache TASK_EcompassLoop */
extern QueueHandle_t xEcompassLoopQueue;

/* Handle vers la queue de la tache TASK_ComTXLoop */
extern QueueHandle_t xComLoopQueue;

/* -------------------------------------------------------------------------
 * Prototypes des fonctions publiques
 * ------------------------------------------------------------------------- */

/*
 * @brief  Initialize tasks, queues, semaphores and timers.
 * This function creates the necessary FreeRTOS components for the application.
 * It sets up tasks, queues, semaphores, and timers used in the application.
 */
void TASKS_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASKS_H__ */
