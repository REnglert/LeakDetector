#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"

/**
 * Sets up the onboard Discovery LEDs as push/pull.
 */
void SetupLEDs(void);

/**
 * Sets up the analog-to-digital converter to be used for vibration sensing.
 */
void SetupADC(void);

void Error_Handler(void);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
