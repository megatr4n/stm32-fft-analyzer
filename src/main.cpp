#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "arm_math.h" 

UART_HandleTypeDef huart1;
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;

// Налаштування аналізатора
#define FFT_SIZE 1024
#define SAMPLE_RATE 10000.0f 

// Буфери для даних
uint16_t adc_buffer[FFT_SIZE];
volatile uint16_t sample_count = 0;
volatile uint8_t data_ready = 0; 

// Масиви для математики
float32_t fft_input[FFT_SIZE * 2];   // Сюди кладемо сигнал перед FFT
float32_t fft_magnitudes[FFT_SIZE];  // Сюди запишемо результат (спектр)
float32_t hann_window[FFT_SIZE];     // Масив нашого "дзвона"

arm_cfft_radix4_instance_f32 fft_handler;

// 1. Функція, яка створює форму "дзвона" (Hann Window)
void generate_hann_window(float32_t* pWindow, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        // Математична формула Ганна: робить краї 0, а центр 1
        pWindow[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (size - 1)));
    }
}

extern "C" {
    void SysTick_Handler(void) { HAL_IncTick(); }

    // Переривання таймера: спрацьовує 10 000 разів на секунду
    void TIM2_IRQHandler(void) {
        HAL_TIM_IRQHandler(&htim2); 
        if (data_ready == 0) {
            HAL_ADC_Start(&hadc1); 
            if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
                adc_buffer[sample_count] = HAL_ADC_GetValue(&hadc1);
                sample_count++;
                if (sample_count >= FFT_SIZE) {
                    sample_count = 0;
                    data_ready = 1; 
                    HAL_TIM_Base_Stop_IT(&htim2); 
                }
            }
        }
    }
}

void SystemClock_Config(void) {}

static void MX_ADC1_Init(void) {
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

static void MX_TIM2_Init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 8 - 1;      
    htim2.Init.Period = 100 - 1; 
    HAL_TIM_Base_Init(&htim2);
}

int main(void) {
    HAL_Init();
    
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE(); 

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    HAL_UART_Init(&huart1);

    MX_ADC1_Init();
    MX_TIM2_Init();

    // ПІДГОТОВКА: створюємо маску "дзвона" та ініціалізуємо FFT
    generate_hann_window(hann_window, FFT_SIZE);
    arm_cfft_radix4_init_f32(&fft_handler, FFT_SIZE, 0, 1);

    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_TIM_Base_Start_IT(&htim2);

    char msg[128]; 

    while (1) {
        if (data_ready == 1) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); 
            
            // --- КРОК 1: Очищення сигналу ---
            float32_t mean_value = 0.0f;
            for(uint16_t i = 0; i < FFT_SIZE; i++) {
                fft_input[i * 2] = (float32_t)adc_buffer[i];
                mean_value += fft_input[i * 2];
            }
            mean_value /= FFT_SIZE; // Рахуємо середній рівень шуму

            // --- КРОК 2: Накладання "дзвона" (Hann Window) ---
            for(uint16_t i = 0; i < FFT_SIZE; i++) {
                // Віднімаємо середнє і МНОЖИМО на коефіцієнт дзвона
                fft_input[i * 2] = (fft_input[i * 2] - mean_value) * hann_window[i];
                fft_input[(i * 2) + 1] = 0.0f; // Уявна частина для математики завжди 0
            }

            // --- КРОК 3: Аналіз частот (FFT) ---
            arm_cfft_radix4_f32(&fft_handler, fft_input);

            // --- КРОК 4: Отримання амплітуд (сили сигналу) ---
            arm_cmplx_mag_f32(fft_input, fft_magnitudes, FFT_SIZE);

            // --- КРОК 5: Пошук найгучнішої частоти ---
            float32_t max_magnitude = 0.0f;
            uint32_t max_index = 0;
            // Шукаємо пік (ігноруємо нульову частоту)
            arm_max_f32(&fft_magnitudes[1], (FFT_SIZE / 2) - 1, &max_magnitude, &max_index);
            max_index += 1; 

            // Переводимо індекс масиву в реальні Герци
            float32_t peak_frequency = (float32_t)max_index * (SAMPLE_RATE / FFT_SIZE);

            // Виводимо результат
            sprintf(msg, "DETECTED FREQ: %.2f Hz | MAGNITUDE: %.2f\r\n", 
                    peak_frequency, max_magnitude);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

            HAL_Delay(500); 
            data_ready = 0;
            HAL_TIM_Base_Start_IT(&htim2);
        }
    }
}