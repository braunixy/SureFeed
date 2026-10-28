/**
 * SureFeed Microchip Pet Feeder - Real-Time Motor Current Sensing & Stall Detection
 * Target: STM32L071VZT6 (LQFP100)
 * 
 * Hardware Mappings:
 * - Green LED: PC10 (Pin 78) - Active LOW (RESET = ON, SET = OFF)
 * - Red LED:   PA15 (Pin 77) - Active LOW (RESET = ON, SET = OFF)
 * - Both ON (PC10 RESET + PA15 RESET) = Solid Orange / Amber
 * 
 * Inputs:
 * - Open / Close (SW2): PA1 (ADC_IN1) > 1800 counts
 * - Add Pet      (SW1): PA1 (ADC_IN1) 1000 - 1800 counts -> Drives Motor CLOSE
 * - Learn Mode   (SW3): PA1 (ADC_IN1) 400 - 1000 counts  -> Drives Motor OPEN
 * - Slider Switch: PE4 (Common ground driven LOW) + PB3 (Pos 1), PB4 (Pos 2), PB5 (Pos 3)
 * 
 * Motor H-Bridge:
 * - PE11 (Pin 42): Forward / OPEN
 * - PE12 (Pin 43): Reverse / CLOSE
 * - Shoot-through dead-time strictly enforced in software.
 * 
 * Multi-Channel Current Sense & Stall Detection:
 * - High-speed scanning across all candidate ADC channels (0..15).
 * - Tracks idle baseline, run peak, and stall surge (delta).
 * - Automatic stall cutoff with startup blanking window.
 */

#include "stm32l0xx_hal.h"
#include <stdlib.h>

#define LOG_MAGIC 0xFEEDC015
#define LOG_SIZE  64

typedef enum {
    BTN_NONE       = 0,
    BTN_OPEN_CLOSE = 1,  /* SW2 on PA1 (~2050 counts) */
    BTN_ADD_PET    = 2,  /* SW1 on PA1 (~1350 counts) */
    BTN_LEARN      = 3   /* SW3 on PA1 (~700 counts)  */
} ButtonId;

typedef enum {
    MOTOR_STOP  = 0,
    MOTOR_OPEN  = 1,  /* Forward (PE11 = HIGH) */
    MOTOR_CLOSE = 2   /* Reverse (PE12 = HIGH) */
} MotorDirection;

typedef struct {
    uint32_t timestamp_ms;
    uint32_t type;       /* 1=Digital, 2=ADC shift, 3=Button, 4=Slider, 5=Stall Cutoff */
    uint32_t param1;     /* Channel, ButtonId, or Port */
    uint32_t param2;     /* Surge counts, Pin, or IDR */
    uint32_t param3;     /* Runtime ms or State */
} LogEvent;

typedef struct {
    uint32_t magic;
    uint32_t event_count;
    uint32_t head_index;
    uint32_t live_adc[16];          /* Live ADC channels 0-15 */
    uint32_t live_idr[5];           /* [0]=Port A, [1]=Port B, etc. */
    uint32_t active_button;         /* 0=None, 1=Open/Close, 2=Add Pet, 3=Learn */
    uint32_t slider_event_pin;      /* Slider position: 1, 2, 3 */

    /* Motor & Current Sensing Diagnostics */
    uint32_t motor_state;           /* 0=STOP, 1=OPEN, 2=CLOSE */
    uint16_t motor_idle_adc[16];    /* Idle baseline sampled before motor start */
    uint16_t motor_peak_adc[16];    /* Peak ADC observed during motor run */
    int16_t  motor_diff_adc[16];    /* Max surge (peak - baseline) difference */
    uint32_t stall_channel;         /* Channel that exhibited highest surge */
    int16_t  stall_max_surge;       /* Value of highest surge */
    uint32_t stall_count;           /* Number of stall cutoffs triggered */
    uint32_t motor_stop_reason;     /* 0=released, 1=stall cutoff, 2=timeout */

    /* IR Break-Beam Diagnostics */
    uint32_t ir_rx_pc8;             /* Live state of PC8 */
    uint16_t ir_beam1_raw_off;      /* PB0 (ADC_IN8) reading before LED1 pulse */
    uint16_t ir_beam1_raw_on;       /* PB0 (ADC_IN8) reading during LED1 pulse */
    int16_t  ir_beam1_delta;        /* Pulse response on Beam 1: (on - off) */
    uint16_t ir_beam2_raw_off;      /* PB0 (ADC_IN8) reading before LED2 pulse */
    uint16_t ir_beam2_raw_on;       /* PB0 (ADC_IN8) reading during LED2 pulse */
    int16_t  ir_beam2_delta;        /* Pulse response on Beam 2: (on - off) */
    uint32_t ir_blocked;            /* 1 if beam is broken, 0 if clear */
    uint16_t pulse_curve_b1[8];     /* [0]=off, [1]=50us, [2]=100us, [3]=200us, [4]=300us, [5]=500us, [6]=800us, [7]=1200us */
    uint16_t pulse_curve_b2[8];     /* Beam 2 timing curve */

    LogEvent events[LOG_SIZE];
} DiagnosticLog;

volatile DiagnosticLog g_diag = {
    .magic = LOG_MAGIC,
    .event_count = 0,
    .head_index = 0,
    .active_button = 0,
    .slider_event_pin = 0,
    .stall_channel = 0,
    .stall_max_surge = 0,
    .stall_count = 0,
    .motor_stop_reason = 0,
    .ir_rx_pc8 = 0,
    .ir_beam1_raw_off = 0,
    .ir_beam1_raw_on = 0,
    .ir_beam1_delta = 0,
    .ir_beam2_raw_off = 0,
    .ir_beam2_raw_on = 0,
    .ir_beam2_delta = 0,
    .ir_blocked = 0
};

/* Live tunable stall detection:
 * - g_stall_sag_threshold: rail sag on PC4 in ADC counts (90 counts ≈ 75mV)
 *   When motor stalls, rail drops by ~175 counts.
 */
volatile uint16_t g_stall_sag_threshold = 90;
volatile uint16_t g_stall_threshold = 4096;

/* Status LEDs: Both are Active LOW (RESET = ON, SET = OFF) */
#define LED_GREEN_ON()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET)
#define LED_GREEN_OFF()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET)

#define LED_RED_ON()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
#define LED_RED_OFF()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)

/* IR Emitter LEDs: PC6 (Pin 63 - IR LED 1 via Q807), PC7 (Pin 64 - IR LED 2 via Q806)
 * Driving HIGH turns on the N-channel transistor, sinking cathode to GND -> LED ON.
 */
#define IR_LEDS_ON()     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET)
#define IR_LEDS_OFF()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET)
#define IR_LED1_ON()     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET)
#define IR_LED1_OFF()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET)
#define IR_LED2_ON()     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET)
#define IR_LED2_OFF()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET)

ADC_HandleTypeDef hadc;

void SystemClock_Config(void);
void GPIO_Init_All(void);
void ADC_Init_Multi(void);
uint16_t ADC_Read_20ms_Filter(uint32_t channel);
uint16_t ADC_Read_Fast(uint32_t channel);
void Log_Event(uint32_t type, uint32_t p1, uint32_t p2, uint32_t p3);
void Motor_Set(MotorDirection dir);

void delay_us(uint32_t us)
{
    volatile uint32_t count = us * 8;
    while (count--) {
        __NOP();
    }
}

static inline uint32_t micros_hw(void)
{
    uint32_t ms1, val1, ms2, val2;
    do {
        ms1  = HAL_GetTick();
        val1 = SysTick->VAL;
        ms2  = HAL_GetTick();
        val2 = SysTick->VAL;
    } while (ms1 != ms2 || val1 < val2);
    uint32_t load = SysTick->LOAD + 1;
    return (ms1 * 1000U) + ((load - 1U - val1) * 1000U) / load;
}

static inline void wait_until_us(uint32_t target_us)
{
    while ((int32_t)(micros_hw() - target_us) < 0) {
        __NOP();
    }
}

volatile uint16_t g_scope_ch8[40];
volatile uint16_t g_scope_ch9[40];
volatile uint32_t g_scope_seq = 0;

/* Wait for the start of the 10.5ms quiet half-cycle of 50Hz mains:
 * First wait until we see at least one noisy sample (|ch9 - 2860| > 90 or |d9| > 90),
 * then wait for 4 consecutive quiet samples spaced 500us apart (2.0ms quiet) where
 * both Ch8 and Ch9 are stable within [2770..2950] and step-to-step change < 65.
 */
static void Sync_To_Quiet_HalfCycle(void)
{
    uint32_t t0 = micros_hw();
    uint16_t p8 = ADC_Read_Fast(8);
    uint16_t p9 = ADC_Read_Fast(9);
    int seen_noise = 0;
    int quiet_streak = 0;

    for (int step = 0; step < 80; step++) { /* Max 40ms (2 full mains cycles) */
        wait_until_us(t0 + (uint32_t)(step + 1) * 500U);
        uint16_t c8 = ADC_Read_Fast(8);
        uint16_t c9 = ADC_Read_Fast(9);
        int16_t d8 = (int16_t)c8 - (int16_t)p8;
        int16_t d9 = (int16_t)c9 - (int16_t)p9;
        p8 = c8;
        p9 = c9;

        uint8_t is_noisy = (abs(d8) > 80 || abs(d9) > 80 ||
                            c8 < 2740 || c8 > 2970 ||
                            c9 < 2740 || c9 > 2970) ? 1 : 0;
        if (is_noisy) {
            seen_noise = 1;
            quiet_streak = 0;
        } else {
            quiet_streak++;
            /* If we saw the burst end and have 3 quiet samples (1.5ms into the 10.5ms quiet window),
             * or if the room is completely dark (no burst at all for 12 samples), we are ready! */
            if ((seen_noise && quiet_streak >= 3) || (!seen_noise && quiet_streak >= 12)) {
                return;
            }
        }
    }
}

static void Capture_Single_Channel_Pulse(uint32_t channel, uint16_t led_pin, volatile uint16_t *dst)
{
    /* Configure and lock ADC on single channel with full 160.5-cycle sampling time (~12us/sample) */
    if ((hadc.Instance->CR & ADC_CR_ADEN) == 0) {
        hadc.Instance->ISR = ADC_ISR_ADRDY;
        hadc.Instance->CR |= ADC_CR_ADEN;
        while ((hadc.Instance->ISR & ADC_ISR_ADRDY) == 0) __NOP();
    }
    hadc.Instance->CHSELR = (1UL << channel);

    /* Warmup 2 dummy reads so S&H cap is settled on this channel */
    for (int w = 0; w < 2; w++) {
        hadc.Instance->ISR = ADC_ISR_EOC | ADC_ISR_EOS;
        hadc.Instance->CR |= ADC_CR_ADSTART;
        while ((hadc.Instance->ISR & ADC_ISR_EOC) == 0) __NOP();
        (void)hadc.Instance->DR;
    }

    uint32_t t0 = micros_hw() + 20U;
    for (int i = 0; i < 20; i++) {
        wait_until_us(t0 + (uint32_t)i * 25U); /* Exact 25us sample period (40 kHz) */
        if (i == 4) {
            HAL_GPIO_WritePin(GPIOC, led_pin, GPIO_PIN_SET);   /* LED ON at t = 100us */
        } else if (i == 16) {
            HAL_GPIO_WritePin(GPIOC, led_pin, GPIO_PIN_RESET); /* LED OFF at t = 400us (300us pulse) */
        }
        hadc.Instance->ISR = ADC_ISR_EOC | ADC_ISR_EOS;
        hadc.Instance->CR |= ADC_CR_ADSTART;
        while ((hadc.Instance->ISR & ADC_ISR_EOC) == 0) __NOP();
        dst[i] = (uint16_t)hadc.Instance->DR;
    }
    HAL_GPIO_WritePin(GPIOC, led_pin, GPIO_PIN_RESET);
}

static int16_t Eval_Matched_Pulse(volatile uint16_t *dst)
{
    int32_t v_pre = ((int32_t)dst[2] + (int32_t)dst[3]) / 2;
    int32_t v_on  = (int32_t)dst[4];
    int32_t v_mid = ((int32_t)dst[14] + (int32_t)dst[15]) / 2;
    int32_t v_off = (int32_t)dst[16];

    int32_t drop_on  = v_pre - v_on;  /* Positive when optical turn-on pulls PB0/PB1 low */
    int32_t jump_off = v_off - v_mid; /* Positive when optical turn-off spikes PB0/PB1 high */

    /* Both the falling turn-on edge AND rising turn-off edge must be present */
    if (drop_on < 150 || jump_off < 100) {
        return 0;
    }

    return (int16_t)(drop_on + jump_off);
}

static void IR_Sample_Pulsed(void)
{
    /* PC8 is Active-LOW: Drive LOW (0V) to enable high-side switch Q801 (powers NET 7 VCC_IR) */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    g_diag.ir_rx_pc8 = 0;

    IR_LED1_OFF();
    IR_LED2_OFF();

    /* 1. Quiet Window 1: Capture Beam 1 (Left LED PC6 -> PB0 / ADC_IN8) */
    Sync_To_Quiet_HalfCycle();
    Capture_Single_Channel_Pulse(8, GPIO_PIN_6, &g_scope_ch8[0]);

    HAL_Delay(14);

    /* 2. Quiet Window 2: Capture Beam 2 (Right LED PC7 -> PB1 / ADC_IN9) */
    Sync_To_Quiet_HalfCycle();
    Capture_Single_Channel_Pulse(9, GPIO_PIN_7, &g_scope_ch9[20]);

    g_scope_seq++;

    /* Evaluate Double-Edge Matched Filter (drop_on + jump_off) */
    int16_t b1_ch8 = Eval_Matched_Pulse(&g_scope_ch8[0]);  /* Direct Beam 1 */
    int16_t b2_ch9 = Eval_Matched_Pulse(&g_scope_ch9[20]); /* Direct Beam 2 */

    g_diag.ir_beam1_raw_off = g_scope_ch8[3];
    g_diag.ir_beam1_raw_on  = g_scope_ch8[4];
    g_diag.ir_beam1_delta   = b1_ch8;

    g_diag.ir_beam2_raw_off = g_scope_ch9[23];
    g_diag.ir_beam2_raw_on  = g_scope_ch9[24];
    g_diag.ir_beam2_delta   = b2_ch9;

    g_diag.pulse_curve_b1[0] = (uint16_t)b1_ch8;
    g_diag.pulse_curve_b1[1] = g_scope_ch8[2];
    g_diag.pulse_curve_b1[2] = g_scope_ch8[3];
    g_diag.pulse_curve_b1[3] = g_scope_ch8[4];
    g_diag.pulse_curve_b1[4] = g_scope_ch8[15];
    g_diag.pulse_curve_b1[5] = g_scope_ch8[16];

    g_diag.pulse_curve_b2[0] = (uint16_t)b2_ch9;
    g_diag.pulse_curve_b2[1] = g_scope_ch9[22];
    g_diag.pulse_curve_b2[2] = g_scope_ch9[23];
    g_diag.pulse_curve_b2[3] = g_scope_ch9[24];
    g_diag.pulse_curve_b2[4] = g_scope_ch9[35];
    g_diag.pulse_curve_b2[5] = g_scope_ch9[36];

    /* --- Optical Break-Beam Evaluation ---
     * Unobstructed:
     *   Beam 1 (PC6 -> PB0/Ch8): ~1100 to 1300 counts (drop_on ~780 + jump_off ~450)
     *   Beam 2 (PC7 -> PB1/Ch9): ~1680 to 1780 counts (drop_on ~1130 + jump_off ~600)
     * Obstructed:
     *   When hand/pet blocks either beam, its matched pulse response drops below threshold.
     */
    static uint8_t blocked_hits = 0;
    uint8_t sample_is_blocked = ((b1_ch8 < 650) || (b2_ch9 < 900)) ? 1 : 0;

    if (sample_is_blocked) {
        if (blocked_hits < 4) blocked_hits++;
    } else {
        if (blocked_hits > 0) blocked_hits--;
    }

    g_diag.ir_blocked = (blocked_hits >= 2) ? 1 : 0;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init_All();
    ADC_Init_Multi();

    HAL_Delay(100);

    /* Sample initial baseline on all 16 ADC channels */
    for (uint32_t ch = 0; ch < 16; ch++) {
        uint16_t v = ADC_Read_Fast(ch);
        g_diag.live_adc[ch] = v;
        g_diag.motor_idle_adc[ch] = v;
    }

    uint16_t prev_idr_A = (uint16_t)GPIOA->IDR;
    uint16_t prev_idr_B = (uint16_t)GPIOB->IDR;
    uint16_t prev_idr_C = (uint16_t)GPIOC->IDR;
    uint16_t prev_idr_D = (uint16_t)GPIOD->IDR;
    uint16_t prev_idr_E = (uint16_t)GPIOE->IDR;

    ButtonId current_btn  = BTN_NONE;
    ButtonId reported_btn = BTN_NONE;

    uint32_t last_heartbeat = 0;
    uint8_t  heartbeat_on   = 0;

    /* Initialize slider position */
    uint8_t prev_slider_pos = ((GPIOB->IDR & (1 << 3)) == 0 ? 1 :
                              ((GPIOB->IDR & (1 << 4)) == 0 ? 2 :
                              ((GPIOB->IDR & (1 << 5)) == 0 ? 3 : 2)));
    uint8_t slider_feedback_pos = 0;
    uint32_t slider_feedback_until = 0;

    /* Motor state tracking */
    uint32_t motor_start_time = 0;
    uint8_t  stall_latched = 0;
    uint32_t stall_feedback_until = 0;
    uint16_t motor_peak_pc4 = 0;

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

        /* Port A: Exclude Open/Close (Pin 1), SWD (13,14), and Red LED (15) */
        uint16_t diff_A = (cur_A ^ prev_idr_A) & ~( (1 << 1) | (1 << 13) | (1 << 14) | (1 << 15) );
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

        /* Port C: Exclude LED (Pin 10), IR Emitters (Pins 6, 7), and IR RX Enable (Pin 8) */
        uint16_t diff_C = (cur_C ^ prev_idr_C) & ~( (1 << 6) | (1 << 7) | (1 << 8) | (1 << 10) );
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

        /* Port E: Exclude Slider Ground (Pin 4) and Motor Gates (Pins 11, 12) */
        uint16_t diff_E = (cur_E ^ prev_idr_E) & ~( (1 << 4) | (1 << 11) | (1 << 12) );
        if (diff_E) {
            for (int p = 0; p < 16; p++) {
                if (diff_E & (1 << p)) {
                    Log_Event(1, 'E', p, (cur_E >> p) & 1);
                }
            }
            prev_idr_E = cur_E;
        }

        /* --- 2. Button Line Sampling ---
         * When motor is STOPPED, use 20ms mains-hum integration filter.
         * When motor is RUNNING, use high-speed single conversion for fast control loop.
         */
        uint16_t raw_1;
        if (g_diag.motor_state == MOTOR_STOP) {
            raw_1 = ADC_Read_20ms_Filter(ADC_CHANNEL_1);
        } else {
            raw_1 = ADC_Read_Fast(ADC_CHANNEL_1);
        }
        g_diag.live_adc[1] = raw_1;

        /* Decode Active Button */
        ButtonId candidate_btn = BTN_NONE;
        if (raw_1 > 1800) {
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

        if (current_btn != reported_btn) {
            Log_Event(3, current_btn, raw_1, 0);
            reported_btn = current_btn;
            /* If user releases button, clear the stall lockout latch */
            if (current_btn == BTN_NONE) {
                stall_latched = 0;
            }
        }

        /* --- 3. Decode 3-Position Slider Switch --- */
        uint8_t pb3 = (cur_B >> 3) & 1;
        uint8_t pb4 = (cur_B >> 4) & 1;
        uint8_t pb5 = (cur_B >> 5) & 1;

        uint8_t slider_pos = prev_slider_pos;
        if (pb3 == 0) {
            slider_pos = 1;
        } else if (pb4 == 0) {
            slider_pos = 2;
        } else if (pb5 == 0) {
            slider_pos = 3;
        }
        g_diag.slider_event_pin = (uint32_t)slider_pos;

        if (slider_pos != prev_slider_pos) {
            Log_Event(4, slider_pos, (cur_B & 0x38), 0);
            slider_feedback_pos = slider_pos;
            slider_feedback_until = now + 600;
            prev_slider_pos = slider_pos;
        }

        /* --- 4. Motor Drive & Real-Time Stall Current Detection ---
         * Learn Mode (SW3) -> Drives Motor OPEN (PE11 = HIGH)
         * Add Pet    (SW1) -> Drives Motor CLOSE (PE12 = HIGH)
         * Automatic Stall Cutoff: Stops motor if current surge exceeds threshold.
         */
        MotorDirection requested_dir = MOTOR_STOP;
        if (!stall_latched) {
            if (current_btn == BTN_LEARN) {
                requested_dir = MOTOR_OPEN;
            } else if (current_btn == BTN_ADD_PET) {
                requested_dir = MOTOR_CLOSE;
            }
        }

        if (requested_dir != MOTOR_STOP) {
            if (g_diag.motor_state == MOTOR_STOP) {
                /* Starting motor: sample idle baseline across all 16 channels */
                for (uint32_t ch = 0; ch < 16; ch++) {
                    if (ch == 1) continue;
                    uint16_t v = ADC_Read_Fast(ch);
                    g_diag.live_adc[ch] = v;
                    g_diag.motor_idle_adc[ch] = v;
                    g_diag.motor_peak_adc[ch] = v;
                    g_diag.motor_diff_adc[ch] = 0;
                }
                g_diag.stall_channel = 0;
                g_diag.stall_max_surge = 0;
                motor_peak_pc4 = 0;
                motor_start_time = now;
                Motor_Set(requested_dir);
                g_diag.motor_state = requested_dir;
            }

            uint32_t run_duration = now - motor_start_time;

            /* Safety 4.0s timeout to protect motor */
            if (run_duration >= 4000) {
                Motor_Set(MOTOR_STOP);
                g_diag.motor_state = MOTOR_STOP;
                g_diag.motor_stop_reason = 2; /* 2 = Timeout */
                stall_latched = 1;
            } else {
                /* Fast multi-channel ADC scan during motor run */
                for (uint32_t ch = 0; ch < 16; ch++) {
                    if (ch == 1) continue; /* Skip button line */
                    uint16_t v = ADC_Read_Fast(ch);
                    g_diag.live_adc[ch] = v;

                    if (v > g_diag.motor_peak_adc[ch]) {
                        g_diag.motor_peak_adc[ch] = v;
                    }

                    int16_t diff = (int16_t)v - (int16_t)g_diag.motor_idle_adc[ch];
                    if (diff > g_diag.motor_diff_adc[ch]) {
                        g_diag.motor_diff_adc[ch] = diff;
                    }

                    /* Only evaluate stall surges AFTER the 200ms inrush blanking period */
                    if (run_duration > 200) {
                        if (diff > g_diag.stall_max_surge) {
                            g_diag.stall_max_surge = diff;
                            g_diag.stall_channel = ch;
                        }
                    }
                }

                /* Periodic diagnostic logging every 100ms during run:
                 * param1 = run_duration_ms, param2 = Ch14 (PC4), param3 = Ch15 (PC5) */
                static uint32_t last_run_log_time = 0;
                if (now - last_run_log_time >= 100) {
                    last_run_log_time = now;
                    Log_Event(6, run_duration, g_diag.live_adc[14], g_diag.live_adc[15]);
                }

                /* Stall Cutoff: Detect supply rail sag on PC4 (Channel 14)
                 * While moving freely: PC4 = 3550 ± 20
                 * When hitting mechanical stop: PC4 plunges to 3385 (sag of ~175 counts)
                 */
                uint16_t pc4 = g_diag.live_adc[14];
                if (run_duration > 200) {
                    if (pc4 > motor_peak_pc4) {
                        motor_peak_pc4 = pc4;
                    }

                    if ((motor_peak_pc4 >= 3450) && ((motor_peak_pc4 - pc4) >= g_stall_sag_threshold)) {
                        /* Stall detected! Cut motor power immediately */
                        Motor_Set(MOTOR_STOP);
                        g_diag.motor_state = MOTOR_STOP;
                        g_diag.motor_stop_reason = 1; /* 1 = Stall cutoff */
                        g_diag.stall_count++;
                        stall_latched = 1;
                        stall_feedback_until = now + 500; /* Solid orange visual feedback */

                        Log_Event(5, 14, (uint32_t)(motor_peak_pc4 - pc4), run_duration);
                    }
                }
            }
        } else {
            /* Motor requested STOP */
            if (g_diag.motor_state != MOTOR_STOP) {
                Motor_Set(MOTOR_STOP);
                g_diag.motor_state = MOTOR_STOP;
                g_diag.motor_stop_reason = 0; /* 0 = Normal button release */
            }
        }

        /* --- Dual-Channel Time-Division Multiplexed Break-Beam Detection ---
         * Sampled every 50ms using microsecond pulses to reject 100Hz ambient light ripple.
         */
        static uint32_t last_ir_time = 0;
        if (now - last_ir_time >= 50) {
            IR_Sample_Pulsed();
            now = HAL_GetTick();
            last_ir_time = now;
        }

        /* --- 5. Status LED Indication --- */
        if (now < stall_feedback_until) {
            /* Stall Cutoff Feedback: Solid ORANGE / AMBER */
            LED_RED_ON();
            LED_GREEN_ON();
            heartbeat_on = 0;
        } else if (current_btn == BTN_OPEN_CLOSE) {
            /* Open / Close -> Solid RED */
            LED_RED_ON();
            LED_GREEN_OFF();
            heartbeat_on = 0;
        } else if (current_btn == BTN_ADD_PET) {
            /* Add Pet -> Solid ORANGE */
            LED_RED_ON();
            LED_GREEN_ON();
            heartbeat_on = 0;
        } else if (current_btn == BTN_LEARN) {
            /* Learn Mode -> Solid GREEN */
            LED_RED_OFF();
            LED_GREEN_ON();
            heartbeat_on = 0;
        } else if (now < slider_feedback_until) {
            /* Slider switch feedback */
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
        } else if (g_diag.ir_blocked) {
            /* Beam Broken / Pet Detected in Arch: Solid RED */
            LED_RED_ON();
            LED_GREEN_OFF();
            heartbeat_on = 0;
        } else {
            /* Idle & Beam Clear: Green Heartbeat pulse (40ms ON every 1200ms) */
            LED_RED_OFF();
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

static MotorDirection s_current_motor_dir = MOTOR_STOP;

void Motor_Set(MotorDirection dir)
{
    if (dir == s_current_motor_dir) {
        return;
    }

    if (dir == MOTOR_OPEN) {
        /* Open: Ensure Close (PE12) is OFF first with dead-time before PE11 HIGH */
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);
        for (volatile int i = 0; i < 500; i++) __NOP(); /* Shoot-through dead-time */
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
        s_current_motor_dir = MOTOR_OPEN;
    } else if (dir == MOTOR_CLOSE) {
        /* Close: Ensure Open (PE11) is OFF first with dead-time before PE12 HIGH */
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);
        for (volatile int i = 0; i < 500; i++) __NOP(); /* Shoot-through dead-time */
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
        s_current_motor_dir = MOTOR_CLOSE;
    } else {
        /* Immediate STOP: Both pins LOW */
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);
        s_current_motor_dir = MOTOR_STOP;
    }
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

    /* Port A (skip SWD pins PA13 & PA14, LED PA15, and analog pins PA0-PA7) */
    GPIO_InitStruct.Pin = GPIO_PIN_All & ~(GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | 0x00FF);
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Port A Analog Pins: PA0..PA7 in pure analog mode (no pull-up) */
    GPIO_InitStruct.Pin = 0x00FF; /* PA0 through PA7 */
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Port B Digital Inputs: PB3, PB4, PB5 for Slider Switch */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = GPIO_PIN_All & ~(GPIO_PIN_0 | GPIO_PIN_1);
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Port B Analog Pins: PB0, PB1 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Port C Digital Inputs (skip PC6, PC7, PC8 for IR, PC10 for LED, and PC0-PC5 for analog) */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = GPIO_PIN_All & ~(GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_10 | 0x003F);
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Port C Analog Pins: PC0..PC5 */
    GPIO_InitStruct.Pin = 0x003F; /* PC0 through PC5 */
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Port D: All inputs with internal PULL-UP */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Port E: All inputs with PULL-UP (skip PE4 for slider ground, and PE11 & PE12 for motor) */
    GPIO_InitStruct.Pin = GPIO_PIN_All & ~(GPIO_PIN_4 | GPIO_PIN_11 | GPIO_PIN_12);
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* PE4 (Pin 3): Slider Switch Common Ground Driver -> driven LOW (0V) */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);

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

    /* 3. Motor H-Bridge: PE11 (Pin 42 - Open), PE12 (Pin 43 - Close) */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);

    /* 4. IR Emitter LEDs: PC6 (Pin 63 - IR LED 1 via Q807), PC7 (Pin 64 - IR LED 2 via Q806)
     *    IR Receiver Gate: PC8 (Pin 65 - Q801 Gate / Enable)
     */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    IR_LEDS_OFF(); /* Start with IR LEDs OFF; pulsed during sampling */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); /* Drive PC8 LOW to enable optical receiver */
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

static const uint32_t s_adc_hal_channels[16] = {
    ADC_CHANNEL_0,  ADC_CHANNEL_1,  ADC_CHANNEL_2,  ADC_CHANNEL_3,
    ADC_CHANNEL_4,  ADC_CHANNEL_5,  ADC_CHANNEL_6,  ADC_CHANNEL_7,
    ADC_CHANNEL_8,  ADC_CHANNEL_9,  ADC_CHANNEL_10, ADC_CHANNEL_11,
    ADC_CHANNEL_12, ADC_CHANNEL_13, ADC_CHANNEL_14, ADC_CHANNEL_15
};

static inline uint32_t Get_HAL_Channel(uint32_t ch)
{
    if (ch < 16) {
        return s_adc_hal_channels[ch];
    }
    return ch;
}

uint16_t ADC_Read_20ms_Filter(uint32_t channel)
{
    hadc.Instance->CHSELR = 0;

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = Get_HAL_Channel(channel);
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
    sConfig.Channel = Get_HAL_Channel(channel);
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
