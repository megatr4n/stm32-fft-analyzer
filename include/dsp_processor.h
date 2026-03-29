#ifndef DSP_PROCESSOR_H
#define DSP_PROCESSOR_H

#include "arm_math.h"

// Константи для математики
#define FFT_SIZE 1024

// Клас або набір функцій для обробки
void DSP_Init(void);
void DSP_ApplyWindow(uint16_t* pInputADC, float32_t* pOutputFFT);
float32_t DSP_Analyze(float32_t* pFFTBuffer, uint32_t* pMaxIndex);

#endif