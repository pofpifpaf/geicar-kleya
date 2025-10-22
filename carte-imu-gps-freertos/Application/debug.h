/**
 * @file debug.h
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 29 aout 2025
 * @brief Debug functions for the car application.
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include "app.h"

//int __io_putchar(int ch);
//int __io_getchar(void);

void DEBUG_PrintITM(uint8_t port, char *str);
void DEBUG_PrintPeriodicInfo(void);

#endif /* DEBUG_H_ */
