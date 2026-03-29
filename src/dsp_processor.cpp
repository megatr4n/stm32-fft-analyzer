#include <stdint.h>
#include "dsp_processor.h"

static float32_t hann_window[FFT_SIZE];
static arm_cfft_radix4_instance_f32 fft_handler;

void DSP_Init(void) {
    for (uint16_t i = 0; i < FFT_SIZE; i++) {
        hann_window[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (FFT_SIZE - 1)));
    }
    arm_cfft_radix4_init_f32(&fft_handler, FFT_SIZE, 0, 1);
}

void DSP_ApplyWindow(uint16_t* pInputADC, float32_t* pOutputFFT) {
    float32_t mean_value = 0.0f;
    
    for(uint16_t i = 0; i < FFT_SIZE; i++) {
        mean_value += (float32_t)pInputADC[i];
    }
    mean_value /= FFT_SIZE;

    for(uint16_t i = 0; i < FFT_SIZE; i++) {
        pOutputFFT[i * 2] = ((float32_t)pInputADC[i] - mean_value) * hann_window[i];
        pOutputFFT[(i * 2) + 1] = 0.0f;
    }
}

float32_t DSP_Analyze(float32_t* pFFTBuffer, uint32_t* pMaxIndex) {
    float32_t fft_magnitudes[FFT_SIZE];
    float32_t max_mag = 0.0f;

    arm_cfft_radix4_f32(&fft_handler, pFFTBuffer);
    
    arm_cmplx_mag_f32(pFFTBuffer, fft_magnitudes, FFT_SIZE);

    arm_max_f32(&fft_magnitudes[1], (FFT_SIZE / 2) - 1, &max_mag, pMaxIndex);
    *pMaxIndex += 1; 

    return max_mag;
}
