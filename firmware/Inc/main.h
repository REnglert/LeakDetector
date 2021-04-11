#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"

/**
 * Sets up the analog-to-digital converter to be used for vibration sensing.
 */
void SetupADC(void);

/**
 * Sets up the onboard Discovery LEDs as push/pull.
 */
void SetupLEDs(void);

/**
 * Sets up UART for serial communication.
 */
void SetupUART(void);

/**
 * Reads a single character from the UART channel.
 */
char UARTRead(void);

/**
 * Sends a full string across the UART channel.
 */
void UARTSendString(char* str);

/**
 * Sends a single character across the UART channel.
 */
void UARTSend(char c);

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
