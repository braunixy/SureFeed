/**
 * SureFeed Microchip Pet Feeder - Rock-Solid Button & Sensor Controller
 * Target: STM32L071VZT6 (LQFP100)
 * 
 * Hardware Mappings:
 * - Green LED: PC10 (Pin 78) - Active LOW (RESET = ON, SET = OFF)
 * - Red LED:   PA15 (Pin 77) - Active LOW (RESET = ON, SET = OFF)
 * - Both ON (PC10 RESET + PA15 RESET) = Solid Orange / Amber
 * 
 * Inputs:
 * - Open / Close (SW2): Detected via PA1 (Active HIGH) or PA6 (drop > 350) -> Solid RED
 * - Add Pet      (SW1): PA6 with internal pull-up (drop > 150)             -> Solid ORANGE
 * - Learn Mode   (SW3): PA6 with internal pull-up (drop > 45)              -> Solid GREEN
 * - Idle: Both LEDs OFF, Green pulses 40ms heartbeat every 1.2s
 * 
 * Mains Hum Rejection:
 * - PA6 is sampled synchronously across 20ms (full 50Hz AC period)
 *   so 50Hz induced mains ripple integrates to 0.
 */

#include "stm32l0xx_hal.h"
#include <stdlib.h>

#define LOG_MAGIC 0xFEEDC014
#define LOG_SIZE  64

typedef enum {
    BTN_NONE       = 0,
    BTN_OPEN_CLOSE = 1,  /* SW2 on PA1 or PA6 */
    BTN_ADD_PET    = 2,  /* SW1 on PA6 (2.2k ohm) */
    BTN_LEARN      = 3   /* SW3 on PA6 (7.5k ohm) */
} ButtonId;

typedef struct {
    uint32_t timestamp_ms;
    uint32_t type;       /* 1 = Digital Event, 2 = ADC Event, 3 = Button State */
    uint32_t param1;     /* ButtonId or Port */
    uint32_t param2;     /* Drop counts or Pin */
    uint32_t param3;     /* Raw ADC or State */
} LogEvent;

typedef struct {
    uint32_t magic;
    uint32_t event_count;
    uint32_t head_index;
    uint32_t live_adc[8];    /* [1]=PA1, [4]=PA4, [5]=PA5, [6]=PA6 */
    uint32_t live_idr[5];    /* [0]=Port A, [1]=Port B, etc. */
    uint32_t active_button;  /* 0=None, 1=Open/Close, 2=Add Pet, 3=Learn */
    uint32_t adc6_baseline;  /* Tracked idle baseline on PA6 */
    uint32_t adc6_smooth;    /* 20ms filtered PA6 value */
    uint32_t slider_event_pin;
    LogEvent events[LOG_SIZE];
} DiagnosticLog;

volatile DiagnosticLog g_diag = {
    .magic = LOG_MAGIC,
    .event_count = 0,
    .head_index = 0,
    .active_button = 0,
    .adc6_baseline = 2830,
    .adc6_smooth = 2830,
    .slider_event_pin = 0
};

/* Status LEDs: Both are Active LOW (RESET = ON, SET = OFF) */
#define LED_GREEN_ON()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET)
#define LED_GREEN_OFF()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET)

#define LED_RED_ON()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
#define LED_RED_OFF()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)

ADC_HandleTypeDef hadc;

void SystemClock_Config(void);
void GPIO_Init_All(void);
void ADC_Init_Multi(void);
uint16_t ADC_Read_20ms_Filter(uint32_t channel);
uint16_t ADC_Read_Fast(uint32_t channel);
void Log_Event(uint32_t type, uint32_t p1, uint32_t p2, uint32_t p3);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init_All();
    ADC_Init_Multi();

    HAL_Delay(100);

    uint16_t prev_idr_A = (uint16_t)GPIOA->IDR;
    uint16_t prev_idr_B = (uint16_t)GPIOB->IDR;
    uint16_t prev_idr_C = (uint16_t)GPIOC->IDR;
    uint16_t prev_idr_D = (uint16_t)GPIOD->IDR;
    uint16_t prev_idr_E = (uint16_t)GPIOE->IDR;

    uint16_t prev_adc_4   = ADC_Read_Fast(ADC_CHANNEL_4);
    uint16_t prev_adc_5   = ADC_Read_Fast(ADC_CHANNEL_5);
    uint16_t prev_adc_7   = ADC_Read_Fast(ADC_CHANNEL_7);

    ButtonId current_btn  = BTN_NONE;
    ButtonId reported_btn = BTN_NONE;

    uint32_t last_heartbeat = 0;
    uint8_t  heartbeat_on   = 0;

    /* Initialize slider position */
    uint8_t prev_slider_pos = ((GPIOB->IDR & (1 << 4)) == 0 ? 1 : ((GPIOB->IDR & (1 << 5)) == 0 ? 3 : 2));
    uint8_t slider_feedback_pos = 0;
    uint32_t slider_feedback_until = 0;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* --- 1. Read Digital Pin Transitions across all Ports --- */
        uint16_t cur_A = (uint16_t)GPIOA->IDR;
        uint16_t cur_B = (uint16_t)GPIOB->IDR;
        uint16_t cur_C = (uint16_t)GPIOC->IDR;
        uint16_t cur_D = (uint16_t)GPIOD->IDR;
        uint16_t cur_E = (uint16_t)GPIOE->IDR;

        g_diag.live_idr[0] = cur_A;
        g_diag.live_idr[1] = cur_B;
        g_diag.live_idr[2] = cur_C;
        g_diag.live_idr[3] = cur_D;
        g_diag.live_idr[4] = cur_E;

        /* Port A: Exclude Open/Close (Pin 1), Analog (Pin 6), SWD (13,14), and Red LED (15) */
        uint16_t diff_A = (cur_A ^ prev_idr_A) & ~( (1 << 1) | (1 << 6) | (1 << 13) | (1 << 14) | (1 << 15) );
        if (diff_A) {
            for (int p = 0; p < 16; p++) {
                if (diff_A & (1 << p)) {
                    Log_Event(1, 'A', p, (cur_A >> p) & 1);
                }
            }
            prev_idr_A = cur_A;
        }

        uint16_t diff_B = cur_B ^ prev_idr_B;
        if (diff_B) {
            for (int p = 0; p < 16; p++) {
                if (diff_B & (1 << p)) {
                    Log_Event(1, 'B', p, (cur_B >> p) & 1);
                }
            }
            prev_idr_B = cur_B;
        }

        uint16_t diff_C = (cur_C ^ prev_idr_C) & ~( (1 << 4) | (1 << 5) | (1 << 10) );
        if (diff_C) {
            for (int p = 0; p < 16; p++) {
                if (diff_C & (1 << p)) {
                    Log_Event(1, 'C', p, (cur_C >> p) & 1);
                }
            }
            prev_idr_C = cur_C;
        }

        uint16_t diff_D = cur_D ^ prev_idr_D;
        if (diff_D) {
            for (int p = 0; p < 16; p++) {
                if (diff_D & (1 << p)) {
                    Log_Event(1, 'D', p, (cur_D >> p) & 1);
                }
            }
            prev_idr_D = cur_D;
        }

        uint16_t diff_E = cur_E ^ prev_idr_E;
        if (diff_E) {
            for (int p = 0; p < 16; p++) {
                if (diff_E & (1 << p)) {
                    Log_Event(1, 'E', p, (cur_E >> p) & 1);
                }
            }
            prev_idr_E = cur_E;
        }

        /* --- 2. Button Line on PA1 (Pin 23 / ADC_IN1) ---
         * The 3 buttons connect between 3.3V (VDDA / Pin 22) and PA1 via resistors:
         * SW2 (Open / Close): 2.20 kΩ -> Highest voltage (~2.5V - 3.3V)
         * SW1 (Add Pet):     4.67 kΩ -> Middle voltage  (~1.7V - 2.5V)
         * SW3 (Learn Mode):  7.45 kΩ -> Lowest voltage  (~0.7V - 1.6V)
         * Idle: Motherboard pulldown keeps line near 0V
         */
        uint8_t sw_open_pin = ((cur_A & (1 << 1)) != 0);
        uint16_t raw_1 = ADC_Read_20ms_Filter(ADC_CHANNEL_1);
        g_diag.live_adc[1] = raw_1;

        /* Sample candidate analog channels for slider detection */
        uint16_t raw_4 = ADC_Read_Fast(ADC_CHANNEL_4);
        uint16_t raw_5 = ADC_Read_Fast(ADC_CHANNEL_5);
        uint16_t raw_6 = ADC_Read_Fast(ADC_CHANNEL_6);
        uint16_t raw_7 = ADC_Read_Fast(ADC_CHANNEL_7);

        g_diag.live_adc[4]   = raw_4;
        g_diag.live_adc[5]   = raw_5;
        g_diag.live_adc[6]   = raw_6;
        g_diag.live_adc[7]   = raw_7;

        /* Check for slider ADC shifts on PA4, PA5, PA7 */
        if (abs((int)raw_4 - (int)prev_adc_4) > 150) {
            Log_Event(2, 4, raw_4, (uint32_t)abs((int)raw_4 - (int)prev_adc_4));
            prev_adc_4 = raw_4;
        }
        if (abs((int)raw_5 - (int)prev_adc_5) > 150) {
            Log_Event(2, 5, raw_5, (uint32_t)abs((int)raw_5 - (int)prev_adc_5));
            prev_adc_5 = raw_5;
        }
        if (abs((int)raw_7 - (int)prev_adc_7) > 150) {
            Log_Event(2, 7, raw_7, (uint32_t)abs((int)raw_7 - (int)prev_adc_7));
            prev_adc_7 = raw_7;
        }

        /* --- 3. Resolve Active Button on PA1 ---
         * Exact measured resistor divider voltages with onboard ~2.2k pull-down:
         * SW2 (Open / Close): 2.20 kΩ -> ~2050 counts (~1.65V) -> Solid RED
         * SW1 (Add Pet):     4.67 kΩ -> ~1310 - 1600 counts (~1.15V) -> Solid ORANGE
         * SW3 (Learn Mode):  7.45 kΩ -> ~650 - 770 counts (~0.58V)  -> Solid GREEN
         * Idle: Motherboard pulldown keeps line < 300 counts (~0V)
         */
        ButtonId candidate_btn = BTN_NONE;
        if (sw_open_pin || raw_1 > 1800) {
            candidate_btn = BTN_OPEN_CLOSE; /* SW2: ~2050 counts */
        } else if (raw_1 > 1000) {
            candidate_btn = BTN_ADD_PET;    /* SW1: ~1310 - 1600 counts */
        } else if (raw_1 > 400) {
            candidate_btn = BTN_LEARN;      /* SW3: ~650 - 770 counts */
        } else {
            candidate_btn = BTN_NONE;
        }

        current_btn = candidate_btn;
        g_diag.active_button = current_btn;

        /* Log button state changes */
        if (current_btn != reported_btn) {
            Log_Event(3, current_btn, raw_1, sw_open_pin);
            reported_btn = current_btn;
        }

        /* --- 4. Decode 3-Position Slider Switch ---
         * Position 1 (Fast Delay):   PB4 LOW -> Solid GREEN feedback (600ms)
         * Position 2 (Medium Delay): Center  -> Solid ORANGE feedback (600ms)
         * Position 3 (Slow Delay):   PB5 LOW -> Solid RED feedback (600ms)
         */
        uint8_t pb3 = (cur_B >> 3) & 1;
        uint8_t pb4 = (cur_B >> 4) & 1;
        uint8_t pb5 = (cur_B >> 5) & 1;

        uint8_t slider_pos = 2;
        if (pb4 == 0 || pb3 == 0) {
            slider_pos = 1;
        } else if (pb5 == 0) {
            slider_pos = 3;
        } else {
            slider_pos = 2;
        }
        g_diag.slider_event_pin = (uint32_t)slider_pos;

        if (slider_pos != prev_slider_pos) {
            Log_Event(4, slider_pos, (cur_B & 0x38), 0);
            slider_feedback_pos = slider_pos;
            slider_feedback_until = now + 600;
            prev_slider_pos = slider_pos;
        }

        /* --- 5. Real-Time Status LED Indication --- */
        if (current_btn == BTN_OPEN_CLOSE) {
            /* Open / Close -> Solid RED for entire duration of hold (UNTOUCHED) */
            LED_RED_ON();
            LED_GREEN_OFF();
            heartbeat_on = 0;
        } else if (current_btn == BTN_ADD_PET) {
            /* Add Pet -> Solid ORANGE for entire duration of hold */
            LED_RED_ON();
            LED_GREEN_ON();
            heartbeat_on = 0;
        } else if (current_btn == BTN_LEARN) {
            /* Learn Mode -> Solid GREEN for entire duration of hold */
            LED_RED_OFF();
            LED_GREEN_ON();
            heartbeat_on = 0;
        } else if (now < slider_feedback_until) {
            /* Slider switch position feedback:
             * Position 1 -> GREEN
             * Position 2 -> ORANGE (Red + Green)
             * Position 3 -> RED
             */
            if (slider_feedback_pos == 1) {
                LED_RED_OFF();
                LED_GREEN_ON();
            } else if (slider_feedback_pos == 2) {
                LED_RED_ON();
                LED_GREEN_ON();
            } else if (slider_feedback_pos == 3) {
                LED_RED_ON();
                LED_GREEN_OFF();
            }
            heartbeat_on = 0;
        } else {
            /* Idle: Red is completely OFF */
            LED_RED_OFF();

            /* Green Heartbeat: 1160ms OFF, 40ms ON */
            if (now - last_heartbeat >= 1200) {
                last_heartbeat = now;
                heartbeat_on = 1;
                LED_GREEN_ON();
            } else if (heartbeat_on && (now - last_heartbeat >= 40)) {
                heartbeat_on = 0;
                LED_GREEN_OFF();
            } else if (!heartbeat_on) {
                LED_GREEN_OFF();
            }
        }
    }
}

void Log_Event(uint32_t type, uint32_t p1, uint32_t p2, uint32_t p3)
{
    uint32_t idx = g_diag.head_index % LOG_SIZE;
    g_diag.events[idx].timestamp_ms = HAL_GetTick();
    g_diag.events[idx].type = type;
    g_diag.events[idx].param1 = p1;
    g_diag.events[idx].param2 = p2;
    g_diag.events[idx].param3 = p3;

    g_diag.head_index = (idx + 1) % LOG_SIZE;
    g_diag.event_count++;
}

void GPIO_Init_All(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 1. All GPIOs default to INPUT with PULL-UP (safe, high impedance) */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    /* Port A (skip SWD pins PA13 & PA14, and LED pin PA15) */
    GPIO_InitStruct.Pin = GPIO_PIN_All & ~(GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA1 (Pin 23): Shared Button Line - configure NOPULL so external pulldown holds line at 0V */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Port B */
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Port C (skip motor pins PC4, PC5 and LED pin PC10) */
    GPIO_InitStruct.Pin = GPIO_PIN_All & ~(GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_10);
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Port D */
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Port E */
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* 2. Status LEDs: Both are Active LOW (RESET = ON, SET = OFF) */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    LED_GREEN_OFF();

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    LED_RED_OFF();

    /* 3. Motor H-Bridge: PC4 (Q501), PC5 (Q506) held safely LOW */
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
}

void ADC_Init_Multi(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc.Instance = ADC1;
    hadc.Init.OversamplingMode      = DISABLE;
    hadc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc.Init.SamplingTime          = ADC_SAMPLETIME_160CYCLES_5;
    hadc.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc.Init.ContinuousConvMode    = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc.Init.DMAContinuousRequests = DISABLE;
    HAL_ADC_Init(&hadc);

    HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
}

uint16_t ADC_Read_20ms_Filter(uint32_t channel)
{
    hadc.Instance->CHSELR = 0;

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);

    uint32_t sum = 0;
    /* 20 samples spaced 1ms apart = 20ms full 50Hz mains cycle */
    for (int i = 0; i < 20; i++) {
        HAL_ADC_Start(&hadc);
        HAL_ADC_PollForConversion(&hadc, 2);
        sum += HAL_ADC_GetValue(&hadc);
        HAL_ADC_Stop(&hadc);
        HAL_Delay(1);
    }
    return (uint16_t)(sum / 20);
}

uint16_t ADC_Read_Fast(uint32_t channel)
{
    hadc.Instance->CHSELR = 0;

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);

    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, 2);
    uint16_t val = (uint16_t)HAL_ADC_GetValue(&hadc);
    HAL_ADC_Stop(&hadc);
    return val;
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
