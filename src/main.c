/**
 * SureFeed Microchip Pet Feeder - Pin Scanner Firmware
 * Target: STM32L071VZT6 (LQFP100)
 * 
 * Configures all GPIO pins on Ports A, B, C, D, E as inputs with pull-ups.
 * Continuously polls all pins. When any pin changes state (pressed, released,
 * beam broken, limit switch tripped), it logs the event into a RAM buffer
 * at a known memory address (0x20000100) for inspection via Black Magic Probe!
 */

#include "stm32l0xx_hal.h"
#include <string.h>

#define EVENT_LOG_MAGIC 0xFEED5EED
#define EVENT_LOG_SIZE  64

typedef struct {
    uint32_t timestamp_ms;
    uint8_t  port;      // 'A', 'B', 'C', 'D', 'E'
    uint8_t  pin;       // 0 - 15
    uint8_t  new_state; // 0 (Low / Pressed to GND) or 1 (High / Released)
    uint8_t  reserved;
} __attribute__((packed)) PinEvent;

typedef struct {
    uint32_t magic;
    uint32_t event_count;
    uint32_t head_index;
    PinEvent events[EVENT_LOG_SIZE];
} __attribute__((packed)) EventLog;

/* Place the log buffer at a fixed location in SRAM for easy reading */
__attribute__((section(".data"))) volatile EventLog g_log = {
    .magic = EVENT_LOG_MAGIC,
    .event_count = 0,
    .head_index = 0
};

static uint16_t prev_state_A = 0xFFFF;
static uint16_t prev_state_B = 0xFFFF;
static uint16_t prev_state_C = 0xFFFF;
static uint16_t prev_state_D = 0xFFFF;
static uint16_t prev_state_E = 0xFFFF;

void SystemClock_Config(void);
void GPIO_Init_All_Inputs(void);
void Log_Pin_Change(char port, uint8_t pin, uint8_t new_state);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init_All_Inputs();

    /* Read initial states */
    prev_state_A = (uint16_t)GPIOA->IDR;
    prev_state_B = (uint16_t)GPIOB->IDR;
    prev_state_C = (uint16_t)GPIOC->IDR;
    prev_state_D = (uint16_t)GPIOD->IDR;
    prev_state_E = (uint16_t)GPIOE->IDR;

    while (1)
    {

        /* Check Port A (skip SWD pins PA13 / PA14 to maintain debug connection) */
        uint16_t cur_A = (uint16_t)GPIOA->IDR;
        uint16_t diff_A = cur_A ^ prev_state_A;
        if (diff_A) {
            for (uint8_t p = 0; p < 16; p++) {
                if (p == 13 || p == 14) continue; // Keep SWD alive!
                if (diff_A & (1U << p)) {
                    Log_Pin_Change('A', p, (cur_A >> p) & 1);
                }
            }
            prev_state_A = cur_A;
        }

        /* Check Port B */
        uint16_t cur_B = (uint16_t)GPIOB->IDR;
        uint16_t diff_B = cur_B ^ prev_state_B;
        if (diff_B) {
            for (uint8_t p = 0; p < 16; p++) {
                if (diff_B & (1U << p)) {
                    Log_Pin_Change('B', p, (cur_B >> p) & 1);
                }
            }
            prev_state_B = cur_B;
        }

        /* Check Port C */
        uint16_t cur_C = (uint16_t)GPIOC->IDR;
        uint16_t diff_C = cur_C ^ prev_state_C;
        if (diff_C) {
            for (uint8_t p = 0; p < 16; p++) {
                if (diff_C & (1U << p)) {
                    Log_Pin_Change('C', p, (cur_C >> p) & 1);
                }
            }
            prev_state_C = cur_C;
        }

        /* Check Port D */
        uint16_t cur_D = (uint16_t)GPIOD->IDR;
        uint16_t diff_D = cur_D ^ prev_state_D;
        if (diff_D) {
            for (uint8_t p = 0; p < 16; p++) {
                if (diff_D & (1U << p)) {
                    Log_Pin_Change('D', p, (cur_D >> p) & 1);
                }
            }
            prev_state_D = cur_D;
        }

        /* Check Port E */
        uint16_t cur_E = (uint16_t)GPIOE->IDR;
        uint16_t diff_E = cur_E ^ prev_state_E;
        if (diff_E) {
            for (uint8_t p = 0; p < 16; p++) {
                if (diff_E & (1U << p)) {
                    Log_Pin_Change('E', p, (cur_E >> p) & 1);
                }
            }
            prev_state_E = cur_E;
        }

        HAL_Delay(5); /* 5ms debounce / sampling rate */
    }
}

void Log_Pin_Change(char port, uint8_t pin, uint8_t new_state)
{
    uint32_t idx = g_log.head_index;
    g_log.events[idx].timestamp_ms = HAL_GetTick();
    g_log.events[idx].port = (uint8_t)port;
    g_log.events[idx].pin = pin;
    g_log.events[idx].new_state = new_state;

    g_log.head_index = (idx + 1) % EVENT_LOG_SIZE;
    g_log.event_count++;
}

void GPIO_Init_All_Inputs(void)
{
    /* Enable GPIO Clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    /* Port A (Exclude PA13 & PA14 SWD pins!) */
    GPIO_InitStruct.Pin = GPIO_PIN_All & ~(GPIO_PIN_13 | GPIO_PIN_14);
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Port B */
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Port C */
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Port D */
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Port E */
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}
