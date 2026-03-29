#ifndef ADC_HANDLER_H
#define ADC_HANDLER_H

#include "stm32f1xx_hal.h"

#define FFT_SIZE 1024

void ADC_Handler_Init(void);
void ADC_Handler_Collect(uint16_t* pBuffer);

#endif