#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

// Підключаємо наші нові модулі
#include "adc_handler.h"
#include "dsp_processor.h"

UART_HandleTypeDef huart1;
uint16_t adc_raw_data[FFT_SIZE];
float32_t fft_working_buffer[FFT_SIZE * 2];

// Налаштування UART для виводу тексту
void UART1_Init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

// Порожні функції для стабільності
extern "C" void SystemClock_Config(void) {}
extern "C" void SysTick_Handler(void) { HAL_IncTick(); }

int main(void) {
    HAL_Init();
    
    // Ініціалізація світлодіода PC13 (ми її випадково видалили при рефакторингу)
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    UART1_Init();
    ADC_Handler_Init();
    DSP_Init();

    char msg[128];
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // Вимкнути діод спочатку

    while (1) {
        // 2. Збираємо дані через модуль (він сам знає про hadc1)
        ADC_Handler_Collect(adc_raw_data);

        // 3. Готуємо дані (накладаємо вікно Ганна)
        DSP_ApplyWindow(adc_raw_data, fft_working_buffer);

        // 4. Аналізуємо (FFT та пошук піку)
        uint32_t peak_index = 0;
        float32_t max_val = DSP_Analyze(fft_working_buffer, &peak_index);

        // 5. Розрахунок частоти
        float32_t freq = (float32_t)peak_index * (10000.0f / FFT_SIZE);

        // 6. Візуалізація та вивід
        if (max_val > 1500.0f) {
            sprintf(msg, "!!! ALERT !!! Freq: %.2f Hz | Mag: %.2f\r\n", freq, max_val);
            for(int i = 0; i < 6; i++) {
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                HAL_Delay(50);
            }
        } else {
            sprintf(msg, "Scanning... Peak: %.2f Hz | Mag: %.2f\r\n", freq, max_val);
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            HAL_Delay(200);
        }
        
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        HAL_Delay(300);
    }
}