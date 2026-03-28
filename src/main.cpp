#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "arm_math.h" 

ADC_HandleTypeDef hadc1;

#define FFT_SIZE 1024
uint16_t adc_buffer[FFT_SIZE];
float32_t fft_input[FFT_SIZE * 2];   
float32_t fft_magnitudes[FFT_SIZE];  
float32_t hann_window[FFT_SIZE];     
arm_cfft_radix4_instance_f32 fft_handler;

// Налаштування пінів
void GPIO_Init(void) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

// Налаштування АЦП
void ADC_Init(void) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0; 
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

// Створення вікна Ганна
void generate_hann_window(float32_t* pWindow, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        pWindow[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (size - 1)));
    }
}

extern "C" void SysTick_Handler(void) { HAL_IncTick(); }

int main(void) {
    HAL_Init();
    GPIO_Init();
    ADC_Init();
    
    // Тест світлодіода при старті
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // ВКЛ
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // ВИКЛ
    HAL_Delay(500);

    generate_hann_window(hann_window, FFT_SIZE);
    arm_cfft_radix4_init_f32(&fft_handler, FFT_SIZE, 0, 1);

    while (1) {
        // КРОК 1: Збір даних вручну (без таймера, щоб не ламалось)
        for (int i = 0; i < FFT_SIZE; i++) {
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 10);
            adc_buffer[i] = HAL_ADC_GetValue(&hadc1);
            // Невелика затримка, щоб імітувати частоту дискретизації
            for(volatile int j=0; j<100; j++); 
        }

        // КРОК 2: Математика
        float32_t mean_value = 0.0f;
        for(uint16_t i = 0; i < FFT_SIZE; i++) {
            fft_input[i * 2] = (float32_t)adc_buffer[i];
            mean_value += fft_input[i * 2];
        }
        mean_value /= FFT_SIZE;

        for(uint16_t i = 0; i < FFT_SIZE; i++) {
            fft_input[i * 2] = (fft_input[i * 2] - mean_value) * hann_window[i];
            fft_input[(i * 2) + 1] = 0.0f;
        }

        arm_cfft_radix4_f32(&fft_handler, fft_input);
        arm_cmplx_mag_f32(fft_input, fft_magnitudes, FFT_SIZE);

        float32_t max_magnitude = 0.0f;
        uint32_t max_index = 0;
        arm_max_f32(&fft_magnitudes[1], (FFT_SIZE / 2) - 1, &max_magnitude, &max_index);

        // КРОК 3: Візуалізація
        // Якщо покрутиш потенціометр вгору - почне мигати швидко
        // КРОК 3: Візуалізація (з ДУЖЕ довгими паузами для очей)
        if (max_magnitude > 800.0f) {
            // ТРИВОГА: 5 чітких швидких спалахів
            for(int i = 0; i < 5; i++) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // ВКЛ
                HAL_Delay(100);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // ВИКЛ
                HAL_Delay(100);
            }
        } else {
            // СКАНУВАННЯ: Один довгий спалах
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // ВКЛ
            HAL_Delay(500);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // ВИКЛ
            HAL_Delay(500);
        }
    }
}