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

typedef enum {
    LID_CLOSED      = 0,
    LID_OPENING     = 1,
    LID_OPEN_AUTO   = 2,
    LID_OPEN_MANUAL = 3,
    LID_CLOSING     = 4
} LidState;

typedef struct {
    uint32_t timestamp_ms;
    uint32_t type;       /* 1=Digital, 2=ADC shift, 3=Button, 4=Slider, 5=Stall Cutoff, 7=RFID */
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

    /* Feeder & Lid State Machine */
    uint32_t lid_state;             /* 0=Closed, 1=Opening, 2=Open Auto, 3=Open Manual, 4=Closing */
    uint32_t target_open_mode;      /* LID_OPEN_AUTO or LID_OPEN_MANUAL */
    uint32_t auto_mode_active;      /* 1 if automatic IR control is active, 0 if manual lock */
    uint32_t rfid_scan_active;      /* 1 if RFID scanner is active */
    uint32_t add_pet_mode;          /* 1 if in Add Pet mode */
    uint32_t learn_mode;            /* 1 if in Learn mode */
    uint32_t motor_speed_pct;       /* 100, 75, or 50 */

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
    uint16_t pulse_curve_b1[8];
    uint16_t pulse_curve_b2[8];

    LogEvent events[LOG_SIZE];
} DiagnosticLog;

volatile DiagnosticLog g_diag = {
    .magic = LOG_MAGIC,
    .event_count = 0,
    .head_index = 0,
    .active_button = 0,
    .slider_event_pin = 2,
    .lid_state = 0,
    .auto_mode_active = 1,
    .rfid_scan_active = 0,
    .add_pet_mode = 0,
    .learn_mode = 0,
    .motor_speed_pct = 75,
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
volatile uint16_t g_stall_sag_threshold = 110;
volatile uint16_t g_stall_threshold = 4096;
volatile uint8_t  g_motor_settled_on = 0;
volatile uint8_t  g_led_systick_blink = 0; /* 0=main loop, 1=Add Pet 1Hz green, 2=RFID scan ~4Hz green */
static   uint16_t s_motor_pc4_filt = 0;
static   uint8_t  s_stall_confirm = 0;

typedef struct {
    uint32_t tx_enable;         /* 1 = 134.2 kHz TIM2_CH1 PWM active on PE9 */
    uint32_t tx_arr;            /* TIM2->ARR (237 -> 134.45 kHz at 32 MHz) */
    uint32_t tx_sweep_count;    /* Number of TX scan sweeps completed */
    uint32_t reset_peaks;       /* Set to 1 via GDB to clear latched peaks */

    /* Pin classification across [0]=A, [1]=B, [2]=C, [3]=D, [4]=E, [5]=H */
    uint16_t pin_pup[6];        /* IDR with internal PULLUP */
    uint16_t pin_pdn[6];        /* IDR with internal PULLDOWN */
    uint16_t idle_toggled[6];   /* Digital transitions observed with TX OFF */
    uint16_t tx_toggled[6];     /* Digital transitions observed with TX ON */
    uint32_t ph_idr_hist[4];    /* GPIOH IDR snapshots at boot */

    /* 16 ADC channels [0..15] = PA0..7, PB0..1, PC0..5 */
    uint16_t adc_idle_min[16];
    uint16_t adc_idle_max[16];
    uint16_t adc_idle_avg[16];

    uint16_t adc_tx_min[16];
    uint16_t adc_tx_max[16];
    uint16_t adc_tx_avg[16];
    uint16_t adc_tx_span[16];   /* Current sweep (max - min) */
    uint16_t adc_peak_span[16]; /* Max latched span while TX ON */
} RfidPinScan;

volatile RfidPinScan g_rfid = {
    .tx_enable = 0,
    .tx_arr = 237,
    .tx_sweep_count = 0,
    .reset_peaks = 0
};

typedef struct {
    uint32_t sweep_count;
    uint32_t active_arr;         /* Current TIM2->ARR: 237 (134.45 kHz), 340 (93.8 kHz), 255 (125 kHz) */
    uint32_t reset_latch;        /* Set to 1 to clear latched detection */
    uint32_t detected_now;       /* 1 if current sweep detected FDX-B intervals */

    /* Live measurements (updated continuously) on PA4, PA5, PA6, PA7 */
    uint16_t edges_15ms[3];      /* Total edges (rising+falling) in 15ms on PA4, PA5, PA6 */
    uint16_t cnt_119us[3];       /* Edge intervals in [90..150 us] (FDX-B half-bit) */
    uint16_t cnt_238us[3];       /* Edge intervals in [200..280 us] (FDX-B full-bit) */
    uint16_t fdxb_score[3];      /* Valid purity-weighted score on PA4, PA5, PA6 */
    uint16_t live_avg[4];        /* PA4, PA5, PA6, PA7 ADC avg */
    uint16_t live_span[4];       /* PA4, PA5, PA6, PA7 ADC span */
    uint16_t pa7_peak_span;      /* Peak span seen on PA7 */
    uint16_t pa7_min_seen;
    uint16_t pa7_max_seen;

    /* Latched best FDX-B detection */
    uint32_t best_score;
    uint32_t best_pin;           /* 4=PA4, 5=PA5, 6=PA6 */
    uint32_t best_arr;
    uint16_t best_pa7_avg;
    uint16_t latched_dt_us[64];  /* Consecutive edge-to-edge pulse widths in us */
    uint16_t latched_wave[64];   /* Slow ADC burst (~11 us/sample = 700 us) on best_pin */
} RfidChipTest;

volatile RfidChipTest g_chip = {
    .active_arr = 237,
    .pa7_min_seen = 4095,
    .pa7_max_seen = 0
};

typedef struct {
    uint8_t hse_active;
    uint8_t pad0;
    uint16_t pa0_low_pa7;
    uint16_t pa0_high_pa7;
    uint16_t steady_arr[10];
    uint16_t steady_pa7[10];
    uint16_t steady_pc4[10];
    uint16_t fast_burst_us;
    uint16_t fast_pa4[48];
    uint16_t fast_pa5[48];
    uint16_t fast_pa6[48];

    /* Manual chip snapshot triggered by BTN_ADD_PET */
    uint32_t snap_count;
    uint32_t snap_arr;
    uint16_t snap_pa7;
    uint16_t snap_edges[3];
    uint16_t snap_119[3];
    uint16_t snap_238[3];
    uint16_t snap_avg[4];
    uint16_t snap_span[4];
    uint16_t snap_pa5[48];
    uint16_t snap_pa6[48];
} RfDiag;

volatile RfDiag g_rfdiag = {0};
volatile uint8_t g_snap_req = 0;

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

static volatile MotorDirection s_target_motor_dir = MOTOR_STOP;
static volatile uint32_t s_target_duty_pct = 100;
static volatile uint32_t s_motor_pwm_counter = 0;

static uint32_t motor_start_time = 0;
static uint16_t motor_peak_pc4 = 0;
static uint32_t stall_feedback_until = 0;
static uint32_t last_run_log_time = 0;

typedef struct {
    uint16_t ph_or;
    uint16_t ph_and;
    uint8_t  hserdy_hi;
    uint8_t  hserdy_lo;
    uint32_t ssr_delta_hi_4ms;
    uint32_t ssr_delta_lo_4ms;
    uint32_t ph0_est_khz_hi;
    uint16_t gpio_nop[6];
    uint16_t gpio_pup[6];
    uint16_t gpio_pdn[6];
    uint16_t wide_arr[45];
    uint16_t wide_pa7[45];
    uint16_t wide_pc4[45];
    uint16_t wide_pa4_span[45];
    uint16_t fast340_pa4[48];
    uint16_t fast350_pa4[48];
    uint16_t fast360_pa4[48];
    uint16_t fast350_us;
    uint16_t sub_arr[17];
    uint16_t sub_pa7[17];
    uint16_t edges_arr[4]; /* 237, 340, 350, 360 */
} RfDiag2;
volatile RfDiag2 g_rfdiag2;

typedef struct {
    uint8_t  pb2_nop_off;
    uint8_t  pb2_pup_50ms;
    uint8_t  pb2_nop_237;
    uint8_t  ph1_pup_50ms;
    uint16_t pb2_lo_pa7;
    uint16_t pb2_hi_pa7;
    uint16_t ph1_lo_pa7;
    uint16_t ph1_hi_pa7;
    uint16_t duty_ccr[11];
    uint16_t duty_pa7[11];
    uint16_t duty_pa4_span[11];
    uint16_t pc0_237;
    uint16_t pc1_237;
    uint16_t pc0_350;
    uint16_t pc1_350;
    uint16_t fast350_pa5[24];
    uint16_t fast350_pa6[24];
    uint16_t pp_hi_hit_port;
    uint16_t pp_hi_hit_pin;
    uint16_t pp_hi_pa7;
    uint16_t pp_hi_base_pa7;
} RfDiag3;
volatile RfDiag3 g_rfdiag3;

typedef struct {
    /* Signal-flow probe: row i = pin tested (0=PA4,1=PA5,2=PA6,3=PC0,4=PC1) with 40k PDN vs PUP;
       cols = resulting ADC avg on {PA4, PA5, PA6, PA7, PC0, PC1} + PB2 bit */
    uint16_t base_adc[6];
    uint16_t pdn_effect[5][6];
    uint16_t pup_effect[5][6];
    uint8_t  pdn_pb2[5];
    uint8_t  pup_pb2[5];
    uint16_t pb2_edges_237;
    uint16_t pb2_edges_340;
    uint16_t pb2_edges_350;
    uint16_t pa7_340;
    uint16_t pa7_350;
} RfDiag4;
volatile RfDiag4 g_rfdiag4;

typedef struct {
    uint32_t cycle_count;
    uint32_t btn_snaps;
    uint32_t hit_count[4];
    uint32_t best_score[4];
    /* Modes: 0={237,119}, 1={237,200}, 2={340,170}, 3={350,175}; cols={PA4,PA5,PA6,PA7,PC1} */
    uint16_t base_avg[4][5];
    uint16_t base_span[4][5];
    uint16_t live_avg[4][5];
    uint16_t live_span[4][5];
    uint16_t live_pb2[4];
    uint16_t hit_avg[4][5];
    uint16_t hit_span[4][5];
    uint16_t hit_pb2[4];
    uint16_t snap_avg[4][5];
    uint16_t snap_span[4][5];
    uint16_t snap_pb2[4];
    uint16_t snap_pa5_350[32];
} RfWatch;
volatile RfWatch g_watch;

typedef struct {
    uint16_t off_base[6];  /* PA4, PA5, PA6, PA7, PC1, PB2_bit */
    uint16_t off_comp_100k[6];
    uint16_t off_inph_100k[6];
    uint16_t tx237_base[6];
    uint16_t tx237_comp_100k[6];
    uint16_t tx237_comp_20k[6];
} RfDiag5;
volatile RfDiag5 g_rfdiag5;

typedef struct {
    uint32_t frames_found;       /* Count of 11-bit header matches */
    uint32_t crc_ok_count;       /* Count of frames with valid CRC & control bits */
    uint16_t country_code;       /* 10-bit ISO country/manufacturer code */
    uint64_t national_id;        /* 38-bit ISO national ID (12 decimal digits) */
    uint64_t raw_id64;           /* Full 64-bit data field (bytes 0..7) */
    uint16_t rx_crc16;           /* Received 16-bit CRC (bytes 8..9) */
    uint16_t calc_crc16;         /* Calculated CRC-CCITT (0x8408 LSB-first) */
    uint16_t ctrl_bits_mask;     /* 13 control bits (0x1FFF when all 13 are 1) */
    uint16_t best_ctrl_count;    /* Best number of valid control bits (0..13) */
    uint8_t  raw_bytes[13];      /* Decoded bytes 0..12 after header */
    uint16_t last_bit_count;     /* Total DBP bits decoded in capture window */
    uint8_t  bit_stream[256];    /* Decoded DBP bits (0 or 1) */
    uint8_t  raw_dt_4us[320];    /* Raw edge intervals (dt_us / 4) */
    char     iso_str[20];        /* Formatted 15-digit ISO ID "CCC-NNNNNNNNNNNN" */
} FdxbDecoder;
volatile FdxbDecoder g_fdxb;

#define MAX_LEARNED_PETS 32U
#define EEPROM_PET_MAGIC 0x53465031UL /* "SFP1" */
#define EEPROM_PET_BASE  0x08080000UL

typedef struct {
    uint32_t count;                          /* Number of learned pets (0..32) */
    uint64_t ids[MAX_LEARNED_PETS];          /* 64-bit FDX-B raw_id64 table */
    char     iso_strs[MAX_LEARNED_PETS][20]; /* Formatted "CCC-NNNNNNNNNNNN" */
    uint64_t last_seen_id;                   /* Most recently decoded chip raw_id64 */
    char     last_seen_iso[20];              /* Most recently decoded chip ISO string */
    uint32_t last_authorized;                /* 1 if last_seen_id matched a learned pet */
    uint32_t auth_open_count;                /* Number of times lid opened for a learned pet */
    uint32_t learn_event_count;              /* Number of chips learned in this session */
    uint32_t training_stage;                 /* 0=Normal, 1..4=Official Training Stages 1..4 */
    uint32_t close_delay_ms;                 /* Slider close delay: I=3500ms, II=6000ms, III=10000ms */
    uint32_t factory_reset_count;            /* Number of factory resets executed */
    uint32_t recheck_count;                  /* 5s anti-hijack RFID rechecks started */
    uint32_t recheck_auth_count;             /* Rechecks confirming an authorized pet */
    uint32_t intruder_close_count;           /* Slow closes triggered by wrong-pet detection */
    uint32_t intruder_closing;               /* 1 while slow (33%) intruder close is active */
    uint32_t vdd_mv;                         /* Regulated MCU VDD via VREFINT (mV, 3300 nom) */
    uint32_t vbat_load_mv;                   /* Estimated loaded battery voltage via PC4 (mV) */
    uint16_t pc4_peak_counts;                /* Last normal-speed motor free-travel PC4 peak */
    uint8_t  low_battery;                    /* 1 when low battery warning is active */
    uint32_t ir_online;                      /* 1 once unobstructed IR beam is detected */
    uint32_t clear_req;                      /* Set to 1 in GDB to wipe learned pets */
} PetRegistry;
volatile PetRegistry g_pets;

static uint8_t s_rfid_powered = 0U;
void RFID_Power_Up(void);
void RFID_Power_Down(void);
static void Pets_Load_From_EEPROM(void);
static void Pets_Save_To_EEPROM(void);
static uint8_t Pets_Is_Authorized(uint64_t id64);
static void Pets_Add(uint64_t id64);
static void Format_ISO_Buf(volatile char *dst, uint64_t id64);
static void Battery_Check_Update(uint16_t loaded_pc4);

typedef struct {
    uint8_t  hserdy;
    uint8_t  sws_hse;
    uint8_t  reverted_pll;
    uint16_t hsi_pa7[3];
    uint16_t hsi_pa4_sp[3];
    uint16_t hsi_pa5_sp[3];
    uint16_t hsi_pa6_sp[3];
    uint16_t hse_pa7[3];
    uint16_t hse_pa4_sp[3];
    uint16_t hse_pa5_sp[3];
    uint16_t hse_pa6_sp[3];
    uint16_t hse_pa5_l1[32];
    uint16_t hse_pa5_l2[32];
} RfDiag10;
volatile RfDiag10 g_rfdiag10;

typedef struct {
    uint16_t near_arr[6];       /* 234, 237, 238, 240, 242, 245 with PB2=HIGH + 100k pump */
    uint16_t near_pa7[6];
    uint16_t near_pa4[6];
    uint16_t near_pa5[6];
    uint16_t near_pa6[6];
    uint16_t hold_0ms_238;
    uint16_t hold_100ms_238;
    uint16_t base_toggles[6];   /* PA..PH digital edge mask at ARR=238, PA7~3869 */
    uint16_t snap_toggles[4][6];
    uint16_t snap_pa7_238[32];
} RfDiag9;
volatile RfDiag9 g_rfdiag9;

void RFID_Pump_Tune_Enable(uint32_t ms);

void SystemClock_Config(void);
void GPIO_Init_All(void);
void ADC_Init_Multi(void);
uint16_t ADC_Read_20ms_Filter(uint32_t channel);
uint16_t ADC_Read_Fast(uint32_t channel);
void Log_Event(uint32_t type, uint32_t p1, uint32_t p2, uint32_t p3);
void Motor_Drive(MotorDirection dir, uint32_t duty_pct);
void Motor_Set(MotorDirection dir);
void Lid_Start_Opening(LidState target_mode);
void Lid_Start_Closing(void);
void Lid_Start_Closing_Duty(uint32_t duty_pct);
void Lid_Stop_Motion(uint32_t stop_reason);
void RFID_Scan_Start(void);
void RFID_Scan_Stop(void);
void RFID_Learn_Start(void);
void RFID_Learn_Stop(void);
void RFID_Carrier_Start(uint32_t arr);
void RFID_Carrier_Stop(void);
void RFID_Classify_Pins(void);
void RFID_Poll_Digital_Toggles(volatile uint16_t out_toggled[6], uint32_t iterations);
void RFID_Scan_All_ADC(volatile uint16_t out_min[16], volatile uint16_t out_max[16],
                       volatile uint16_t out_avg[16], uint16_t samples_per_ch);
void RFID_Chip_Sweep(void);
void RFID_Full_Diag(void);
void RFID_Watch_Cycle(void);

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
    ADC1->SMPR = ADC_SAMPLETIME_160CYCLES_5;
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

static void __attribute__((unused)) IR_Sample_Pulsed(void)
{
    /* Ensure 134.2 kHz RFID TX carrier is quiet before sampling high-Z phototransistors PB0/PB1 */
    if (g_rfid.tx_enable || (TIM2->CR1 & TIM_CR1_CEN)) {
        RFID_Carrier_Stop();
        HAL_Delay(2);
    }

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

    if (!sample_is_blocked) {
        g_pets.ir_online = 1U;
    }

    if (!g_pets.ir_online) {
        /* If IR arch connector is unplugged on workbench (both beams 0 since boot),
         * do not falsely hold ir_blocked=1 and drain the battery. */
        blocked_hits = 0;
        g_diag.ir_blocked = 0;
        return;
    }

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

    __HAL_RCC_TIM21_CLK_ENABLE();
    TIM21->CR1 = 0;
    TIM21->PSC = 31; /* 32 MHz / 32 = 1.000 MHz (1 us tick) */
    TIM21->ARR = 0xFFFFU;
    TIM21->EGR = TIM_EGR_UG;
    TIM21->CR1 = TIM_CR1_CEN;

    /* Load learned pets from Data EEPROM, check initial supply voltage, and start in low-power idle */
    Pets_Load_From_EEPROM();
    RFID_Power_Down();
    Battery_Check_Update(0U);

    uint32_t learn_baseline_crc_ok = 0U;
    uint32_t learn_success_until = 0U;
    uint32_t unauth_feedback_until = 0U;
    uint32_t factory_reset_led_until = 0U;
    uint32_t add_pet_hold_start = 0U;
    uint8_t  factory_reset_fired = 0U;
    uint8_t  factory_reset_cycle_pending = 0U;
    uint32_t motor_partial_close_ms = 0U;
    uint32_t ir_rfid_active_until = 0U;
    uint32_t ir_last_blocked_time = 0U;
    uint32_t recheck_next_at = 0U;
    uint32_t recheck_active_until = 0U;
    uint32_t next_vbat_check_at = HAL_GetTick() + 30000U;
    uint8_t  recheck_saw_auth = 0U;
    uint8_t  recheck_saw_unauth = 0U;
    uint32_t last_stall_sample_ms = 0U;
    uint32_t last_ir_during_rfid_ms = 0U;
    g_pets.close_delay_ms = 6000U;

    uint16_t prev_idr_A = (uint16_t)GPIOA->IDR;
    uint16_t prev_idr_B = (uint16_t)GPIOB->IDR;
    uint16_t prev_idr_C = (uint16_t)GPIOC->IDR;
    uint16_t prev_idr_D = (uint16_t)GPIOD->IDR;
    uint16_t prev_idr_E = (uint16_t)GPIOE->IDR;

    /* Initialize slider position */
    uint8_t prev_slider_pos = ((GPIOB->IDR & (1 << 3)) == 0 ? 1 :
                              ((GPIOB->IDR & (1 << 4)) == 0 ? 2 :
                              ((GPIOB->IDR & (1 << 5)) == 0 ? 3 : 2)));
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

        /* Port E: Exclude Slider Ground (Pin 4), RFID TX (Pin 9), and Motor Gates (Pins 11, 12) */
        uint16_t diff_E = (cur_E ^ prev_idr_E) & ~( (1 << 4) | (1 << 9) | (1 << 11) | (1 << 12) );
        if (diff_E) {
            for (int p = 0; p < 16; p++) {
                if (diff_E & (1 << p)) {
                    Log_Event(1, 'E', p, (cur_E >> p) & 1);
                }
            }
            prev_idr_E = cur_E;
        }

        /* --- 2. Button Line Sampling & Debouncing --- */
        uint16_t raw_1;
        if (g_diag.lid_state == LID_OPENING || g_diag.lid_state == LID_CLOSING || g_rfid.tx_enable) {
            raw_1 = ADC_Read_Fast(ADC_CHANNEL_1);
        } else {
            raw_1 = ADC_Read_20ms_Filter(ADC_CHANNEL_1);
        }
        g_diag.live_adc[1] = raw_1;

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

        /* 30ms stable debounce with rising-edge detection */
        static ButtonId debounced_btn = BTN_NONE;
        static ButtonId last_raw_btn  = BTN_NONE;
        static uint32_t btn_change_time = 0;
        ButtonId btn_pressed_event = BTN_NONE;

        if (candidate_btn != last_raw_btn) {
            last_raw_btn = candidate_btn;
            btn_change_time = now;
        } else if ((now - btn_change_time) >= 30) {
            if (candidate_btn != debounced_btn) {
                if (candidate_btn != BTN_NONE && debounced_btn == BTN_NONE) {
                    btn_pressed_event = candidate_btn; /* Fresh rising edge! */
                }
                debounced_btn = candidate_btn;
                g_diag.active_button = debounced_btn;
                Log_Event(3, debounced_btn, raw_1, 0);
            }
        }

        /* --- 3. Button Action Handling --- */
        if (btn_pressed_event == BTN_OPEN_CLOSE) {
            g_pets.intruder_closing = 0U;
            recheck_active_until = 0U;
            if (g_diag.lid_state == LID_CLOSED) {
                /* User manually opened lid: hold open and DEACTIVATE automatic mode */
                g_diag.auto_mode_active = 0;
                Lid_Start_Opening(LID_OPEN_MANUAL);
            } else if (g_diag.lid_state == LID_OPEN_MANUAL || g_diag.lid_state == LID_OPEN_AUTO) {
                /* User pressed while open: close lid and RE-ACTIVATE automatic mode */
                g_diag.auto_mode_active = 1;
                Lid_Start_Closing();
            } else if (g_diag.lid_state == LID_OPENING) {
                /* Cancel opening: switch to closing */
                g_diag.auto_mode_active = 1;
                Lid_Start_Closing();
            } else if (g_diag.lid_state == LID_CLOSING) {
                /* Safety manual interrupt: re-open */
                g_diag.auto_mode_active = 0;
                Lid_Start_Opening(LID_OPEN_MANUAL);
            }
        } else if (btn_pressed_event == BTN_ADD_PET) {
            add_pet_hold_start = now;
            factory_reset_fired = 0U;
            /* Official Add Pet Mode: open lid automatically & flash green until chip is read or cancelled */
            if (!g_diag.add_pet_mode) {
                g_diag.add_pet_mode = 1U;
                learn_baseline_crc_ok = g_fdxb.crc_ok_count;
                RFID_Learn_Start();
                if (g_diag.lid_state == LID_CLOSED || g_diag.lid_state == LID_CLOSING) {
                    motor_partial_close_ms = 0U;
                    Lid_Start_Opening(LID_OPEN_AUTO);
                }
            } else {
                g_diag.add_pet_mode = 0U;
                RFID_Learn_Stop();
                RFID_Power_Down();
                if (g_diag.lid_state == LID_OPEN_AUTO || g_diag.lid_state == LID_OPENING) {
                    motor_partial_close_ms = (g_pets.training_stage == 2U) ? 250U :
                                             (g_pets.training_stage == 3U) ? 450U :
                                             (g_pets.training_stage == 4U) ? 650U : 0U;
                    if (g_pets.training_stage != 1U) {
                        Lid_Start_Closing();
                    }
                }
            }
            Log_Event(3, BTN_ADD_PET, g_diag.add_pet_mode, g_pets.count);
        } else if (btn_pressed_event == BTN_LEARN) {
            /* Official 4-Stage Training Mode (0 -> 1 -> 2 -> 3 -> 4 -> 0) */
            uint32_t next_stage = (g_pets.training_stage + 1U) % 5U;
            g_pets.training_stage = next_stage;
            g_diag.learn_mode = next_stage;
            Log_Event(3, BTN_LEARN, next_stage, 0);

            if (next_stage == 1U) {
                /* Stage 1: Lid stays 100% open continuously */
                motor_partial_close_ms = 0U;
                if (g_diag.lid_state != LID_OPEN_AUTO && g_diag.lid_state != LID_OPENING) {
                    Lid_Start_Opening(LID_OPEN_AUTO);
                }
            } else if (next_stage >= 2U && next_stage <= 4U) {
                /* Stages 2..4: step lid down to partial parked position (250ms / 450ms / 650ms from full open) */
                if (g_diag.lid_state == LID_OPEN_AUTO) {
                    motor_partial_close_ms = (next_stage == 2U) ? 250U :
                                             (next_stage == 3U) ? 450U : 650U;
                    Lid_Start_Closing();
                } else if (g_diag.lid_state == LID_CLOSED) {
                    motor_partial_close_ms = 200U;
                    Lid_Start_Closing();
                }
            } else {
                /* Stage 0: Exit Training Mode -> close lid completely */
                motor_partial_close_ms = 0U;
                Lid_Start_Closing();
            }
        }

        /* 10-Second Continuous Hold on Add Pet Button -> Official Factory Reset */
        if (debounced_btn == BTN_ADD_PET && !factory_reset_fired && (now - add_pet_hold_start) >= 10000U) {
            factory_reset_fired = 1U;
            g_pets.clear_req = 1U;
            g_diag.add_pet_mode = 0U;
            g_diag.learn_mode = 0U;
            g_pets.training_stage = 0U;
            RFID_Learn_Stop();
            RFID_Power_Down();
            factory_reset_led_until = now + 3000U;
            factory_reset_cycle_pending = 1U;
            motor_partial_close_ms = 0U;
            if (g_diag.lid_state == LID_OPEN_AUTO || g_diag.lid_state == LID_OPEN_MANUAL) {
                factory_reset_cycle_pending = 0U;
                Lid_Start_Closing();
            } else {
                Lid_Start_Opening(LID_OPEN_AUTO);
            }
            Log_Event(7, 99, 0, 0);
        }

        if (g_pets.clear_req) {
            g_pets.clear_req = 0U;
            g_pets.count = 0U;
            g_pets.factory_reset_count++;
            for (uint32_t i = 0U; i < MAX_LEARNED_PETS; i++) {
                g_pets.ids[i] = 0ULL;
                g_pets.iso_strs[i][0] = '\0';
            }
            Pets_Save_To_EEPROM();
        }

        /* --- 4. Decode 3-Position Lid Close Delay Slider (I = 3.5s, II = 6.0s, III = 10.0s) --- */
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

        if (slider_pos == 1) {
            g_pets.close_delay_ms = 3500U;
        } else if (slider_pos == 2) {
            g_pets.close_delay_ms = 6000U;
        } else {
            g_pets.close_delay_ms = 10000U;
        }

        if (slider_pos != prev_slider_pos) {
            Log_Event(4, slider_pos, (cur_B & 0x38), g_pets.close_delay_ms);
            slider_feedback_pos = slider_pos;
            slider_feedback_until = now + 600;
            prev_slider_pos = slider_pos;
        }

        /* --- 5 & 6. IR Break-Beam Gate & Power-Gated RFID Antenna Control --- */
        if (g_diag.lid_state != LID_OPENING && g_diag.lid_state != LID_CLOSING) {
            if (!s_rfid_powered || (now - last_ir_during_rfid_ms >= 200U)) {
                last_ir_during_rfid_ms = now;
                IR_Sample_Pulsed();
            }
        }

        if (g_diag.add_pet_mode) {
            /* Official Add Pet Mode: lid stays open, LED flashes green 1Hz, antenna scans until chip is learned */
            if (now >= factory_reset_led_until && now >= learn_success_until &&
                now >= unauth_feedback_until && now >= stall_feedback_until &&
                now >= slider_feedback_until) {
                g_led_systick_blink = 1U;
            }
            if (g_diag.lid_state != LID_OPENING && g_diag.lid_state != LID_CLOSING) {
                RFID_Chip_Sweep();
            }
            if (g_fdxb.crc_ok_count != learn_baseline_crc_ok && g_fdxb.raw_id64 != 0ULL) {
                Pets_Add(g_fdxb.raw_id64);
                Log_Event(7, 4, (uint32_t)(g_fdxb.raw_id64 & 0xFFFFFFFFULL), g_pets.count);
                g_diag.add_pet_mode = 0U;
                RFID_Learn_Stop();
                RFID_Power_Down();
                ir_last_blocked_time = HAL_GetTick();
                recheck_next_at = ir_last_blocked_time + 5000U;
                recheck_active_until = 0U;
                learn_success_until = ir_last_blocked_time + 2500U;
            }
        } else if (g_diag.auto_mode_active) {
            if (g_diag.ir_blocked) {
                ir_last_blocked_time = now;
                if (g_diag.lid_state == LID_CLOSING && !g_pets.intruder_closing) {
                    /* Normal close safety: re-open if pet breaks IR beam while closing */
                    motor_partial_close_ms = 0U;
                    recheck_active_until = 0U;
                    Lid_Start_Opening(LID_OPEN_AUTO);
                } else if (g_diag.lid_state == LID_CLOSED && g_pets.training_stage != 1U) {
                    ir_rfid_active_until = now + 2000U;
                }
            }

            if (g_diag.lid_state == LID_OPEN_AUTO && g_pets.training_stage != 1U) {
                if (g_diag.ir_blocked) {
                    /* Anti-hijack guard: recheck every 5s while beam stays broken */
                    if (recheck_active_until == 0U && (int32_t)(now - recheck_next_at) >= 0) {
                        recheck_active_until = now + 1500U;
                        recheck_saw_auth = 0U;
                        recheck_saw_unauth = 0U;
                        g_pets.recheck_count++;
                    }
                    if (recheck_active_until != 0U) {
                        if ((int32_t)(recheck_active_until - now) > 0) {
                            uint32_t prev_crc = g_fdxb.crc_ok_count;
                            RFID_Chip_Sweep();
                            if (g_fdxb.crc_ok_count != prev_crc && g_fdxb.raw_id64 != 0ULL) {
                                g_pets.last_seen_id = g_fdxb.raw_id64;
                                Format_ISO_Buf(g_pets.last_seen_iso, g_fdxb.raw_id64);
                                if (Pets_Is_Authorized(g_fdxb.raw_id64)) {
                                    g_pets.last_authorized = 1U;
                                    g_pets.recheck_auth_count++;
                                    recheck_saw_auth = 1U;
                                    recheck_active_until = 0U;
                                    recheck_next_at = HAL_GetTick() + 5000U;
                                    RFID_Power_Down();
                                } else {
                                    g_pets.last_authorized = 0U;
                                    recheck_saw_unauth = 1U;
                                }
                            }
                        } else {
                            recheck_active_until = 0U;
                            RFID_Power_Down();
                            if (recheck_saw_unauth && !recheck_saw_auth) {
                                /* Only wrong pet detected: close gently at ~1/3 speed (33% PWM) */
                                g_pets.intruder_close_count++;
                                g_pets.intruder_closing = 1U;
                                unauth_feedback_until = HAL_GetTick() + 1500U;
                                motor_partial_close_ms = (g_pets.training_stage == 2U) ? 550U :
                                                         (g_pets.training_stage == 3U) ? 1000U :
                                                         (g_pets.training_stage == 4U) ? 1450U : 0U;
                                Lid_Start_Closing_Duty(33U);
                            } else {
                                recheck_next_at = HAL_GetTick() + 5000U;
                            }
                        }
                    }
                } else {
                    recheck_active_until = 0U;
                    recheck_next_at = now + 5000U;
                    if (s_rfid_powered || g_rfid.tx_enable) {
                        RFID_Power_Down();
                    }
                    /* Auto-close lid after slider delay (3.5s / 6.0s / 10.0s) once pet leaves the IR arch */
                    if ((now - ir_last_blocked_time) >= g_pets.close_delay_ms) {
                        g_pets.intruder_closing = 0U;
                        motor_partial_close_ms = (g_pets.training_stage == 2U) ? 250U :
                                                 (g_pets.training_stage == 3U) ? 450U :
                                                 (g_pets.training_stage == 4U) ? 650U : 0U;
                        Lid_Start_Closing();
                    }
                }
            } else {
                uint8_t want_rfid = (g_diag.lid_state == LID_CLOSED &&
                                     g_pets.training_stage != 1U &&
                                     (g_diag.ir_blocked ||
                                      (ir_rfid_active_until != 0U && (int32_t)(ir_rfid_active_until - now) > 0))) ? 1U : 0U;

                if (want_rfid) {
                    if (now >= factory_reset_led_until && now >= learn_success_until &&
                        now >= unauth_feedback_until && now >= stall_feedback_until &&
                        now >= slider_feedback_until && g_pets.training_stage == 0U) {
                        g_led_systick_blink = 2U;
                    }
                    uint32_t prev_crc = g_fdxb.crc_ok_count;
                    RFID_Chip_Sweep();
                    if (g_fdxb.crc_ok_count != prev_crc && g_fdxb.raw_id64 != 0ULL) {
                        g_pets.last_seen_id = g_fdxb.raw_id64;
                        Format_ISO_Buf(g_pets.last_seen_iso, g_fdxb.raw_id64);
                        if (Pets_Is_Authorized(g_fdxb.raw_id64)) {
                            g_pets.last_authorized = 1U;
                            g_pets.auth_open_count++;
                            g_pets.intruder_closing = 0U;
                            ir_rfid_active_until = 0U;
                            ir_last_blocked_time = HAL_GetTick();
                            recheck_active_until = 0U;
                            recheck_next_at = ir_last_blocked_time + 5000U;
                            RFID_Power_Down();
                            motor_partial_close_ms = 0U;
                            Lid_Start_Opening(LID_OPEN_AUTO);
                        } else {
                            g_pets.last_authorized = 0U;
                            unauth_feedback_until = HAL_GetTick() + 1000U;
                        }
                    }
                } else {
                    if (s_rfid_powered || g_rfid.tx_enable) {
                        RFID_Power_Down();
                    }
                }
            }
        } else {
            /* Manual hold mode: keep RFID antenna powered down */
            recheck_active_until = 0U;
            if (s_rfid_powered || g_rfid.tx_enable) {
                RFID_Power_Down();
            }
        }

        /* --- 7. Motor Motion Supervision & Real-Time Stall Cutoff --- */
        if (g_diag.lid_state == LID_OPENING || g_diag.lid_state == LID_CLOSING) {
            now = HAL_GetTick();
            uint32_t run_duration = (now >= motor_start_time) ? (now - motor_start_time) : 0;
            uint8_t  settled_before = g_motor_settled_on;
            uint32_t pe_before = GPIOE->ODR & (GPIO_PIN_11 | GPIO_PIN_12);
            uint16_t s0 = ADC_Read_Fast(14);
            uint16_t s1 = ADC_Read_Fast(14);
            uint16_t s2 = ADC_Read_Fast(14);
            uint32_t pe_after = GPIOE->ODR & (GPIO_PIN_11 | GPIO_PIN_12);
            uint8_t  settled_after = g_motor_settled_on;

            /* Median of 3 ADC reads to reject single-sample switching glitches */
            uint16_t pc4_med;
            if (s0 > s1) { uint16_t t = s0; s0 = s1; s1 = t; }
            if (s1 > s2) { s1 = s2; }
            pc4_med = (s0 > s1) ? s0 : s1;

            uint8_t pc4_valid = (settled_before != 0U && settled_after != 0U &&
                                 pe_before != 0U && pe_after == pe_before &&
                                 pc4_med < 4090U) ? 1U : 0U;
            if (pc4_valid) {
                s_motor_pc4_filt = (s_motor_pc4_filt == 0U)
                                       ? pc4_med
                                       : (uint16_t)((((uint32_t)s_motor_pc4_filt * 3U) + pc4_med) >> 2U);
                g_diag.live_adc[14] = s_motor_pc4_filt;
            }

            /* Periodic diagnostic logging every 100ms during run */
            if (run_duration > 0 && (now - last_run_log_time >= 100)) {
                last_run_log_time = now;
                Log_Event(6, run_duration, g_diag.live_adc[14], g_diag.lid_state);
            }

            uint32_t timeout_ms = (s_target_duty_pct <= 45U) ? 8000U : 4000U;

            /* Training Mode partial close target or safety timeout */
            if (g_diag.lid_state == LID_CLOSING && motor_partial_close_ms > 0U && run_duration >= motor_partial_close_ms) {
                motor_partial_close_ms = 0U;
                g_pets.intruder_closing = 0U;
                s_stall_confirm = 0U;
                Lid_Stop_Motion(0);
                g_diag.lid_state = LID_CLOSED;
            } else if (run_duration >= timeout_ms) {
                uint16_t peak_snap = motor_peak_pc4;
                motor_partial_close_ms = 0U;
                g_pets.intruder_closing = 0U;
                s_stall_confirm = 0U;
                Lid_Stop_Motion(2); /* 2 = Timeout */
                if (s_target_duty_pct >= 70U && peak_snap >= 1200U) {
                    Battery_Check_Update(peak_snap);
                }
                if (g_diag.lid_state == LID_OPENING) {
                    if (factory_reset_cycle_pending) {
                        factory_reset_cycle_pending = 0U;
                        Lid_Start_Closing();
                    } else {
                        g_diag.lid_state = g_diag.target_open_mode;
                        ir_last_blocked_time = HAL_GetTick();
                        recheck_next_at = ir_last_blocked_time + 5000U;
                    }
                } else {
                    g_diag.lid_state = LID_CLOSED;
                }
            } else if (run_duration > 350 && pc4_valid && (now - last_stall_sample_ms >= 4U)) {
                /* After 350ms inrush blanking, monitor smoothed settled-ON PC4 rail sag */
                last_stall_sample_ms = now;
                uint16_t pc4 = s_motor_pc4_filt;
                if (pc4 > motor_peak_pc4) {
                    motor_peak_pc4 = pc4;
                    s_stall_confirm = 0U;
                } else if ((motor_peak_pc4 >= 1200U) && ((motor_peak_pc4 - pc4) >= g_stall_sag_threshold)) {
                    if (++s_stall_confirm >= 4U) {
                        /* Mechanical end-stop or obstruction reached! Cut motor immediately */
                        uint16_t peak_snap = motor_peak_pc4;
                        uint32_t duty_snap = s_target_duty_pct;
                        Log_Event(5, 14, (uint32_t)(motor_peak_pc4 - pc4), run_duration);
                        motor_partial_close_ms = 0U;
                        g_pets.intruder_closing = 0U;
                        s_stall_confirm = 0U;
                        Lid_Stop_Motion(1); /* 1 = Stall cutoff */
                        if (duty_snap >= 70U && peak_snap >= 1200U) {
                            Battery_Check_Update(peak_snap);
                        }

                        if (g_diag.lid_state == LID_OPENING) {
                            if (factory_reset_cycle_pending) {
                                factory_reset_cycle_pending = 0U;
                                Lid_Start_Closing();
                            } else {
                                g_diag.lid_state = g_diag.target_open_mode;
                                ir_last_blocked_time = HAL_GetTick();
                                recheck_next_at = ir_last_blocked_time + 5000U;
                            }
                        } else {
                            g_diag.lid_state = LID_CLOSED;
                        }
                    }
                } else {
                    s_stall_confirm = 0U;
                }
            }
        } else {
            s_stall_confirm = 0U;
            s_motor_pc4_filt = 0U;
            if ((int32_t)(now - next_vbat_check_at) >= 0) {
                next_vbat_check_at = now + 30000U;
                Battery_Check_Update(0U);
            }
        }

        /* --- 8. Status LED Indication (Official SureFeed Manual Behavior) --- */
        if (now < factory_reset_led_until) {
            g_led_systick_blink = 0U;
            /* Factory Reset: Alternating RED / GREEN */
            if ((now / 150U) % 2U) {
                LED_RED_ON();
                LED_GREEN_OFF();
            } else {
                LED_RED_OFF();
                LED_GREEN_ON();
            }
        } else if (now < learn_success_until) {
            g_led_systick_blink = 0U;
            /* Microchip Learned / Read: Solid GREEN */
            LED_RED_OFF();
            LED_GREEN_ON();
        } else if (now < unauth_feedback_until) {
            g_led_systick_blink = 0U;
            /* Unauthorized Chip Feedback: Fast-Flashing RED (1.0s) */
            if ((now / 120U) % 2U) LED_RED_ON(); else LED_RED_OFF();
            LED_GREEN_OFF();
        } else if (now < stall_feedback_until) {
            g_led_systick_blink = 0U;
            /* Stall Cutoff Feedback: Solid ORANGE / AMBER */
            LED_RED_ON();
            LED_GREEN_ON();
        } else if (now < slider_feedback_until) {
            g_led_systick_blink = 0U;
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
        } else if (g_diag.add_pet_mode) {
            /* Official Add Pet / Learn Mode: Flashing GREEN once per second (1 Hz) */
            g_led_systick_blink = 1U;
        } else if (g_diag.lid_state == LID_OPEN_AUTO && g_pets.training_stage != 1U) {
            g_led_systick_blink = 0U;
            /* Authorized Pet Eating / Microchip Read: Solid GREEN */
            LED_RED_OFF();
            LED_GREEN_ON();
        } else if (g_pets.training_stage >= 1U && g_pets.training_stage <= 4U) {
            g_led_systick_blink = 0U;
            /* Official Training Mode: Flashing ORANGE N times (1..4) per 2.4s cycle */
            uint32_t phase = now % 2400U;
            uint32_t pulse_idx = phase / 350U;
            uint32_t pulse_rem = phase % 350U;
            if (pulse_idx < g_pets.training_stage && pulse_rem < 180U) {
                LED_RED_ON();
                LED_GREEN_ON();
            } else {
                LED_RED_OFF();
                LED_GREEN_OFF();
            }
        } else if (g_diag.lid_state == LID_OPEN_MANUAL) {
            g_led_systick_blink = 0U;
            /* Manual Hold Open: Solid RED (automatic mode paused) */
            LED_RED_ON();
            LED_GREEN_OFF();
        } else if (g_diag.rfid_scan_active) {
            /* IR beam broken -> RFID Antenna Scanning: Fast-Flashing GREEN (~4.2 Hz) */
            g_led_systick_blink = 2U;
        } else if (g_pets.low_battery) {
            g_led_systick_blink = 0U;
            /* Official Low Battery Warning: brief RED flash every 4.0s at idle */
            if ((now % 4000U) < 150U) {
                LED_RED_ON();
            } else {
                LED_RED_OFF();
            }
            LED_GREEN_OFF();
        } else {
            g_led_systick_blink = 0U;
            /* Official Idle State (RFID TX & RX OFF): LED completely OFF */
            LED_GREEN_OFF();
            LED_RED_OFF();
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

void Motor_Drive(MotorDirection dir, uint32_t duty_pct)
{
    if (dir == MOTOR_STOP) {
        s_target_motor_dir = MOTOR_STOP;
        g_motor_settled_on = 0U;
        GPIOE->BSRR = ((uint32_t)GPIO_PIN_11 << 16U) | ((uint32_t)GPIO_PIN_12 << 16U);
        return;
    }

    if (dir != s_target_motor_dir) {
        s_target_motor_dir = MOTOR_STOP;
        g_motor_settled_on = 0U;
        GPIOE->BSRR = ((uint32_t)GPIO_PIN_11 << 16U) | ((uint32_t)GPIO_PIN_12 << 16U);
        for (volatile int i = 0; i < 500; i++) __NOP(); /* Shoot-through dead-time */
    }

    s_target_duty_pct = duty_pct;
    s_target_motor_dir = dir;
    g_motor_settled_on = (duty_pct >= 100U) ? 1U : 0U;

    /* Drive active direction pin immediately */
    if (dir == MOTOR_OPEN) {
        GPIOE->BSRR = ((uint32_t)GPIO_PIN_12 << 16U) | GPIO_PIN_11;
    } else if (dir == MOTOR_CLOSE) {
        GPIOE->BSRR = ((uint32_t)GPIO_PIN_11 << 16U) | GPIO_PIN_12;
    }
}

void Motor_Set(MotorDirection dir)
{
    Motor_Drive(dir, 100);
}

void Lid_Start_Opening(LidState target_mode)
{
    g_diag.target_open_mode = target_mode;
    g_diag.lid_state = LID_OPENING;
    motor_start_time = HAL_GetTick();
    motor_peak_pc4 = 0;
    s_motor_pc4_filt = 0;
    s_stall_confirm = 0;
    last_run_log_time = motor_start_time;
    Motor_Drive(MOTOR_OPEN, g_diag.motor_speed_pct);
    g_diag.motor_state = MOTOR_OPEN;
    Log_Event(6, 1, target_mode, g_diag.motor_speed_pct);
}

void Lid_Start_Closing_Duty(uint32_t duty_pct)
{
    g_diag.lid_state = LID_CLOSING;
    motor_start_time = HAL_GetTick();
    motor_peak_pc4 = 0;
    s_motor_pc4_filt = 0;
    s_stall_confirm = 0;
    last_run_log_time = motor_start_time;
    Motor_Drive(MOTOR_CLOSE, duty_pct);
    g_diag.motor_state = MOTOR_CLOSE;
    Log_Event(6, 2, g_pets.intruder_closing, duty_pct);
}

void Lid_Start_Closing(void)
{
    g_pets.intruder_closing = 0U;
    Lid_Start_Closing_Duty(g_diag.motor_speed_pct);
}

void Lid_Stop_Motion(uint32_t stop_reason)
{
    Motor_Drive(MOTOR_STOP, 0);
    g_diag.motor_state = MOTOR_STOP;
    g_diag.motor_stop_reason = stop_reason;
    if (stop_reason == 1) { /* Stall cutoff */
        g_diag.stall_count++;
        stall_feedback_until = HAL_GetTick() + 500;
    }
}

/* RFID Subsystem Stubs (Antenna & 134.2 kHz carrier decoding in progress) */
void RFID_Scan_Start(void)
{
    Log_Event(7, 1, 0, 0); /* 7=RFID, 1=Scan Start */
}

void RFID_Scan_Stop(void)
{
    Log_Event(7, 0, 0, 0); /* 7=RFID, 0=Scan Stop */
}

void RFID_Learn_Start(void)
{
    Log_Event(7, 2, 0, 0); /* 7=RFID, 2=Learn Mode Start */
}

void RFID_Learn_Stop(void)
{
    Log_Event(7, 3, 0, 0); /* 7=RFID, 3=Learn Mode Stop */
}

/* Enable 134.2 kHz carrier on PE9 (Pin 40, TIM2_CH1, AF0) */
void RFID_Carrier_Start(uint32_t arr)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_TIM2;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    if (arr < 10) arr = 237;
    uint32_t start_arr = (arr > 280U) ? 180U : arr;
    TIM2->CR1 = 0;
    TIM2->PSC = 0;
    TIM2->ARR = start_arr;
    TIM2->CCR1 = (start_arr + 1U) / 2U;
    TIM2->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM2->CCER = TIM_CCER_CC1E;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    if (arr > 280U) {
        static const uint16_t ramp_steps[3] = {220, 260, 300};
        for (int i = 0; i < 3; i++) {
            if (ramp_steps[i] < arr) {
                HAL_Delay(4);
                TIM2->ARR = ramp_steps[i];
                TIM2->CCR1 = (ramp_steps[i] + 1U) / 2U;
                TIM2->EGR = TIM_EGR_UG;
            }
        }
        HAL_Delay(4);
        TIM2->ARR = arr;
        TIM2->CCR1 = (arr + 1U) / 2U;
        TIM2->EGR = TIM_EGR_UG;
    }
}

void RFID_Carrier_Stop(void)
{
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->CCER = 0;

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
}

/* Safe non-destructive input pull-up vs pull-down classification across Ports A..E, H */
void RFID_Classify_Pins(void)
{
    GPIO_TypeDef * const ports[6] = { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOH };
    const uint16_t safe_masks[6] = {
        (uint16_t)(~((1U << 1) | (1U << 13) | (1U << 14) | (1U << 15))), /* PA: skip PA1, SWD, Red LED */
        (uint16_t)(~((1U << 0) | (1U << 1) | (1U << 3) | (1U << 4) | (1U << 5))), /* PB: skip IR ADCs, slider */
        (uint16_t)(~((1U << 4) | (1U << 6) | (1U << 7) | (1U << 8) | (1U << 10))), /* PC: skip PC4, IR, Green LED */
        0xFFFFU,                                                          /* PD: all pins */
        (uint16_t)(~((1U << 4) | (1U << 9) | (1U << 11) | (1U << 12))), /* PE: skip PE4, PE9, Motor */
        (uint16_t)((1U << 9) | (1U << 10))                                /* PH: only pull PH9/PH10; leave PH0/PH1 untouched */
    };

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    /* 1. Pull-up scan */
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    for (int i = 0; i < 6; i++) {
        GPIO_InitStruct.Pin = safe_masks[i];
        HAL_GPIO_Init(ports[i], &GPIO_InitStruct);
    }
    delay_us(500);
    for (int i = 0; i < 6; i++) {
        g_rfid.pin_pup[i] = (uint16_t)ports[i]->IDR;
    }

    /* 2. Pull-down scan */
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    for (int i = 0; i < 6; i++) {
        GPIO_InitStruct.Pin = safe_masks[i];
        HAL_GPIO_Init(ports[i], &GPIO_InitStruct);
    }
    delay_us(500);
    for (int i = 0; i < 6; i++) {
        g_rfid.pin_pdn[i] = (uint16_t)ports[i]->IDR;
    }

    /* 3. Snapshot GPIOH quickly in case PH0/PH1 has an external clock */
    for (int k = 0; k < 4; k++) {
        g_rfid.ph_idr_hist[k] = GPIOH->IDR;
        delay_us(10);
    }

    /* 4. Restore digital pins to PULLUP and analog pins back to ANALOG NOPULL */
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    for (int i = 0; i < 6; i++) {
        GPIO_InitStruct.Pin = safe_masks[i];
        HAL_GPIO_Init(ports[i], &GPIO_InitStruct);
    }

    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = 0x00FFU & ~(1U << 1); /* PA0, PA2..PA7 */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = 0x003FU & ~(1U << 4); /* PC0..PC3, PC5 */
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void RFID_Poll_Digital_Toggles(volatile uint16_t out_toggled[6], uint32_t iterations)
{
    uint16_t prev[6] = {
        (uint16_t)GPIOA->IDR,
        (uint16_t)GPIOB->IDR,
        (uint16_t)GPIOC->IDR,
        (uint16_t)GPIOD->IDR,
        (uint16_t)GPIOE->IDR,
        (uint16_t)GPIOH->IDR
    };
    uint16_t tog[6] = {0, 0, 0, 0, 0, 0};

    for (uint32_t it = 0; it < iterations; it++) {
        uint16_t c0 = (uint16_t)GPIOA->IDR; tog[0] |= (uint16_t)(c0 ^ prev[0]); prev[0] = c0;
        uint16_t c1 = (uint16_t)GPIOB->IDR; tog[1] |= (uint16_t)(c1 ^ prev[1]); prev[1] = c1;
        uint16_t c2 = (uint16_t)GPIOC->IDR; tog[2] |= (uint16_t)(c2 ^ prev[2]); prev[2] = c2;
        uint16_t c3 = (uint16_t)GPIOD->IDR; tog[3] |= (uint16_t)(c3 ^ prev[3]); prev[3] = c3;
        uint16_t c4 = (uint16_t)GPIOE->IDR; tog[4] |= (uint16_t)(c4 ^ prev[4]); prev[4] = c4;
        uint16_t c5 = (uint16_t)GPIOH->IDR; tog[5] |= (uint16_t)(c5 ^ prev[5]); prev[5] = c5;
    }

    for (int i = 0; i < 6; i++) {
        out_toggled[i] = tog[i];
    }
}

void RFID_Scan_All_ADC(volatile uint16_t out_min[16], volatile uint16_t out_max[16],
                       volatile uint16_t out_avg[16], uint16_t samples_per_ch)
{
    if (samples_per_ch == 0) samples_per_ch = 1;

    if ((ADC1->CR & ADC_CR_ADEN) == 0U) {
        ADC1->ISR = ADC_ISR_ADRDY;
        ADC1->CR |= ADC_CR_ADEN;
        while ((ADC1->ISR & ADC_ISR_ADRDY) == 0U) {}
    }

    for (uint32_t ch = 0; ch < 16; ch++) {
        ADC1->CHSELR = (1U << ch);
        uint16_t vmin = 4095;
        uint16_t vmax = 0;
        uint32_t sum = 0;

        for (uint16_t s = 0; s < samples_per_ch; s++) {
            ADC1->CR |= ADC_CR_ADSTART;
            while ((ADC1->ISR & ADC_ISR_EOC) == 0U) {}
            uint16_t v = (uint16_t)ADC1->DR;
            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
            sum += v;
        }

        uint16_t avg = (uint16_t)(sum / samples_per_ch);
        out_min[ch] = vmin;
        out_max[ch] = vmax;
        out_avg[ch] = avg;
        g_diag.live_adc[ch] = avg;
    }

    HAL_ADC_Stop(&hadc);
}

static void ADC_Capture_Burst(uint32_t ch, volatile uint16_t *buf, uint16_t count)
{
    ADC1->CHSELR = (1U << ch);
    uint16_t tn = (uint16_t)TIM21->CNT;
    for (uint16_t i = 0; i < count; i++) {
        ADC1->CR |= ADC_CR_ADSTART;
        while ((ADC1->ISR & ADC_ISR_EOC) == 0U) {}
        buf[i] = (uint16_t)ADC1->DR;
        while ((uint16_t)((uint16_t)TIM21->CNT - tn) < 20U) {}
        tn = (uint16_t)(tn + 20U);
    }
}

static void HSE_Enter_32MHz(void)
{
    RCC->CR |= RCC_CR_HSEBYP | RCC_CR_HSEON;
    uint16_t th = (uint16_t)TIM21->CNT;
    while (((RCC->CR & RCC_CR_HSERDY) == 0U) && ((uint16_t)((uint16_t)TIM21->CNT - th) < 15000U)) {}
    if (RCC->CR & RCC_CR_HSERDY) {
        __disable_irq();
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSE;
        for (volatile uint32_t w = 0; w < 2000U; w++) {
            if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSE) break;
        }
        __enable_irq();
    }
}

static void HSE_Exit_To_HSI_PLL(void)
{
    __disable_irq();
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    for (volatile uint32_t w = 0; w < 2000U; w++) {
        if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) break;
    }
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP);
    __enable_irq();
}

static void ADC_Sample_Stats(uint32_t ch, uint16_t count, volatile uint16_t *out_avg, volatile uint16_t *out_span)
{
    ADC1->CHSELR = (1U << ch);
    uint16_t vmin = 4095, vmax = 0;
    uint32_t sum = 0;
    for (uint16_t i = 0; i < count; i++) {
        ADC1->CR |= ADC_CR_ADSTART;
        while ((ADC1->ISR & ADC_ISR_EOC) == 0U) {}
        uint16_t v = (uint16_t)ADC1->DR;
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
        sum += v;
    }
    *out_avg = (uint16_t)(sum / count);
    *out_span = (uint16_t)(vmax - vmin);
}

void RFID_Power_Up(void)
{
    if (!s_rfid_powered) {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);

        RFID_Carrier_Start(237U);
        HAL_Delay(6);
        s_rfid_powered = 1U;
    }
    g_rfid.tx_enable = 1U;
    g_diag.rfid_scan_active = 1U;
}

void RFID_Power_Down(void)
{
    RFID_Carrier_Stop();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIOB->BSRR = ((uint32_t)(GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14) << 16U);

    s_rfid_powered = 0U;
    g_rfid.tx_enable = 0U;
    g_diag.rfid_scan_active = 0U;
    g_chip.detected_now = 0U;
}

void RFID_Chip_Sweep(void)
{
    RFID_Power_Up();
    HSE_Enter_32MHz();
    if ((ADC1->CR & ADC_CR_ADEN) == 0U) {
        ADC1->ISR = ADC_ISR_ADRDY;
        ADC1->CR |= ADC_CR_ADEN;
        while ((ADC1->ISR & ADC_ISR_ADRDY) == 0U) {}
    }
    if (g_snap_req) {
        g_snap_req = 0;
        g_rfdiag.snap_count++;
    }
    RFID_Watch_Cycle();
    g_chip.sweep_count++;
    if (0) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    uint16_t cur_dt[3][64];
    uint16_t cur_purity[3] = {0, 0, 0};

    /* 1. Measure 15ms edge intervals on PA4, PA5, PA6 at 1us TIM21 resolution */
    for (int idx = 0; idx < 3; idx++) {
        uint16_t pin_mask = (uint16_t)(1U << (4 + idx));
        GPIO_InitStruct.Pin = pin_mask;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        uint16_t edges = 0;
        uint16_t c119 = 0;
        uint16_t c238 = 0;
        uint16_t dt_idx = 0;
        uint16_t prev = (uint16_t)(GPIOA->IDR & pin_mask);
        uint16_t t_start = (uint16_t)TIM21->CNT;
        uint16_t last_edge = t_start;

        while ((uint16_t)((uint16_t)TIM21->CNT - t_start) < 15000U) {
            uint16_t cur = (uint16_t)(GPIOA->IDR & pin_mask);
            if (cur != prev) {
                uint16_t now_us = (uint16_t)TIM21->CNT;
                uint16_t dt = (uint16_t)(now_us - last_edge);
                /* Deglitch <60 us carrier-bleed chatter on transitions */
                if (edges == 0U || dt >= 60U) {
                    last_edge = now_us;
                    if (edges > 0U) {
                        if (dt >= 90U && dt <= 150U) {
                            c119++;
                        } else if (dt >= 200U && dt <= 280U) {
                            c238++;
                        }
                        if (dt_idx < 64U) {
                            cur_dt[idx][dt_idx++] = dt;
                        }
                    }
                    edges++;
                    prev = cur;
                }
            }
        }
        while (dt_idx < 64U) {
            cur_dt[idx][dt_idx++] = 0;
        }

        uint16_t valid = (uint16_t)(c119 + c238);
        /* Require at least half of observed edges to fall in FDX-B bit timings */
        uint16_t purity = (edges >= 10U && (uint32_t)valid * 2U >= (uint32_t)edges) ? valid : 0U;

        g_chip.edges_15ms[idx] = edges;
        g_chip.cnt_119us[idx] = c119;
        g_chip.cnt_238us[idx] = c238;
        g_chip.fdxb_score[idx] = purity;
        cur_purity[idx] = purity;

        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }

    /* 2. Sample live ADC stats on PA4, PA5, PA6, PA7 and run analog midpoint slicer on PA5, PA6 */
    if ((ADC1->CR & ADC_CR_ADEN) == 0U) {
        ADC1->ISR = ADC_ISR_ADRDY;
        ADC1->CR |= ADC_CR_ADEN;
        while ((ADC1->ISR & ADC_ISR_ADRDY) == 0U) {}
    }

    for (int idx = 0; idx < 4; idx++) {
        ADC_Sample_Stats((uint32_t)(4 + idx), 64, &g_chip.live_avg[idx], &g_chip.live_span[idx]);
        g_diag.live_adc[4 + idx] = g_chip.live_avg[idx];
    }

    /* Fast (~2.1 us/sample) analog adaptive Schmitt slicer on PA5 (ch 5) and PA6 (ch 6) */
    ADC1->SMPR = ADC_SAMPLETIME_12CYCLES_5;
    for (int idx = 1; idx <= 2; idx++) {
        uint16_t avg = g_chip.live_avg[idx];
        uint16_t spn = g_chip.live_span[idx];
        if (spn >= 200U) {
            uint16_t hyst = spn / 6U;
            if (hyst < 40U) hyst = 40U;
            uint16_t hi_th = avg + hyst;
            uint16_t lo_th = (avg > hyst) ? (avg - hyst) : 0U;

            ADC1->CHSELR = (1U << (4 + idx));
            uint16_t a_edges = 0, a_c119 = 0, a_c238 = 0, a_dt_idx = 0;
            uint8_t state = 2; /* unknown */
            uint16_t t_start = (uint16_t)TIM21->CNT;
            uint16_t last_edge = t_start;
            while ((uint16_t)((uint16_t)TIM21->CNT - t_start) < 12000U) {
                ADC1->CR |= ADC_CR_ADSTART;
                while ((ADC1->ISR & ADC_ISR_EOC) == 0U) {}
                uint16_t v = (uint16_t)ADC1->DR;
                uint8_t nstate = state;
                if (state == 2U) {
                    nstate = (v >= avg) ? 1U : 0U;
                    state = nstate;
                    continue;
                }
                if (state == 0U && v >= hi_th) nstate = 1U;
                else if (state == 1U && v <= lo_th) nstate = 0U;

                if (nstate != state) {
                    uint16_t now_us = (uint16_t)TIM21->CNT;
                    uint16_t dt = (uint16_t)(now_us - last_edge);
                    if (a_edges == 0U || dt >= 60U) {
                        last_edge = now_us;
                        if (a_edges > 0U) {
                            if (dt >= 90U && dt <= 150U) a_c119++;
                            else if (dt >= 200U && dt <= 280U) a_c238++;
                            if (a_dt_idx < 64U) cur_dt[idx][a_dt_idx++] = dt;
                        }
                        a_edges++;
                        state = nstate;
                    }
                }
            }
            uint16_t a_valid = (uint16_t)(a_c119 + a_c238);
            uint16_t a_purity = (a_edges >= 10U && (uint32_t)a_valid * 2U >= (uint32_t)a_edges) ? a_valid : 0U;
            if (a_purity > cur_purity[idx]) {
                while (a_dt_idx < 64U) cur_dt[idx][a_dt_idx++] = 0;
                g_chip.edges_15ms[idx] = a_edges;
                g_chip.cnt_119us[idx] = a_c119;
                g_chip.cnt_238us[idx] = a_c238;
                g_chip.fdxb_score[idx] = a_purity;
                cur_purity[idx] = a_purity;
            }
        }
    }
    ADC1->SMPR = ADC_SAMPLETIME_160CYCLES_5;

    if (g_chip.live_span[3] > g_chip.pa7_peak_span) {
        g_chip.pa7_peak_span = g_chip.live_span[3];
    }
    if (g_chip.live_avg[3] < g_chip.pa7_min_seen) g_chip.pa7_min_seen = g_chip.live_avg[3];
    if (g_chip.live_avg[3] > g_chip.pa7_max_seen) g_chip.pa7_max_seen = g_chip.live_avg[3];

    /* 3. Update live detection flag and latch strongest FDX-B capture */
    int top_idx = 0;
    if (cur_purity[1] > cur_purity[top_idx]) top_idx = 1;
    if (cur_purity[2] > cur_purity[top_idx]) top_idx = 2;

    g_chip.detected_now = (cur_purity[top_idx] >= 50U) ? 1U : 0U;

    if (cur_purity[top_idx] >= 50U && cur_purity[top_idx] >= g_chip.best_score) {
        g_chip.best_score = cur_purity[top_idx];
        g_chip.best_pin = (uint32_t)(4 + top_idx);
        g_chip.best_arr = g_chip.active_arr;
        g_chip.best_pa7_avg = g_chip.live_avg[3];
        for (int k = 0; k < 64; k++) {
            g_chip.latched_dt_us[k] = cur_dt[top_idx][k];
        }
        ADC1->SMPR = ADC_SAMPLETIME_12CYCLES_5;
        ADC_Capture_Burst((uint32_t)(4 + top_idx), g_chip.latched_wave, 64);
        ADC1->SMPR = ADC_SAMPLETIME_160CYCLES_5;
    }

    if (g_snap_req) {
        g_snap_req = 0;
        g_rfdiag.snap_count++;
        g_rfdiag.snap_arr = g_chip.active_arr;
        g_rfdiag.snap_pa7 = g_chip.live_avg[3];
        for (int i = 0; i < 3; i++) {
            g_rfdiag.snap_edges[i] = g_chip.edges_15ms[i];
            g_rfdiag.snap_119[i] = g_chip.cnt_119us[i];
            g_rfdiag.snap_238[i] = g_chip.cnt_238us[i];
        }
        for (int i = 0; i < 4; i++) {
            g_rfdiag.snap_avg[i] = g_chip.live_avg[i];
            g_rfdiag.snap_span[i] = g_chip.live_span[i];
        }
        ADC1->SMPR = ADC_SAMPLETIME_12CYCLES_5;
        ADC_Capture_Burst(5, g_rfdiag.snap_pa5, 48);
        ADC_Capture_Burst(6, g_rfdiag.snap_pa6, 48);
        ADC1->SMPR = ADC_SAMPLETIME_160CYCLES_5;
    }

    HAL_ADC_Stop(&hadc);
    RFID_Watch_Cycle();
    g_chip.sweep_count++;
    }
}

static uint8_t RFID_Servo_PA7(uint16_t target)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    uint8_t did_pump = 0U;
    uint16_t cur = 0, sp = 0;
    for (int step = 0; step < 25; step++) {
        ADC_Sample_Stats(7, 8, &cur, &sp);
        if (cur + 55U >= target && cur <= target + 55U) break;
        did_pump = 1U;
        uint16_t tn = (uint16_t)TIM21->CNT;
        if (cur < target) {
            uint32_t pulses = (target - cur > 250U) ? 400U : 80U;
            for (uint32_t k = 0; k < pulses; k++) {
                GPIOB->BSRR = (k & 1U) ? ((uint32_t)GPIO_PIN_14 << 16) : GPIO_PIN_14;
                while ((uint16_t)((uint16_t)TIM21->CNT - tn) < 10U) {}
                tn = (uint16_t)(tn + 10U);
            }
        } else {
            uint32_t pulses = (cur - target > 250U) ? 200U : 40U;
            for (uint32_t k = 0; k < pulses; k++) {
                GPIOB->BSRR = (k & 1U) ? ((uint32_t)GPIO_PIN_13 << 16) : GPIO_PIN_13;
                while ((uint16_t)((uint16_t)TIM21->CNT - tn) < 15U) {}
                tn = (uint16_t)(tn + 15U);
            }
        }
        GPIOB->BSRR = ((uint32_t)GPIO_PIN_13 << 16) | ((uint32_t)GPIO_PIN_14 << 16);
        uint16_t tw = (uint16_t)TIM21->CNT;
        while ((uint16_t)((uint16_t)TIM21->CNT - tw) < 1500U) {}
    }
    return did_pump;
}

void RFID_Pump_Tune_Enable(uint32_t ms)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    uint32_t n_half = ms * 200U; /* 100 kHz complementary */
    uint16_t tn = (uint16_t)TIM21->CNT;
    for (uint32_t k = 0; k < n_half; k++) {
        GPIOB->BSRR = (k & 1U) ? (((uint32_t)GPIO_PIN_13 << 16) | GPIO_PIN_14)
                               : (GPIO_PIN_13 | ((uint32_t)GPIO_PIN_14 << 16));
        while ((uint16_t)((uint16_t)TIM21->CNT - tn) < 5U) {}
        tn = (uint16_t)(tn + 5U);
    }
    GPIOB->BSRR = ((uint32_t)GPIO_PIN_13 << 16) | ((uint32_t)GPIO_PIN_14 << 16);
}

static uint16_t FDXB_Calc_CRC16(const uint8_t *data8)
{
    uint16_t crc = 0x0000U;
    for (int i = 0; i < 8; i++) {
        crc ^= (uint16_t)data8[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1U) crc = (uint16_t)((crc >> 1U) ^ 0x8408U);
            else          crc = (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

static void Format_ISO_Buf(volatile char *dst, uint64_t id64)
{
    uint16_t country = (uint16_t)((id64 >> 38U) & 0x3FFU);
    uint64_t nat_id  = id64 & 0x3FFFFFFFFFULL;
    dst[0] = (char)('0' + ((country / 100U) % 10U));
    dst[1] = (char)('0' + ((country / 10U) % 10U));
    dst[2] = (char)('0' + (country % 10U));
    dst[3] = '-';
    uint64_t tmp = nat_id;
    for (int i = 15; i >= 4; i--) {
        dst[i] = (char)('0' + (uint32_t)(tmp % 10ULL));
        tmp /= 10ULL;
    }
    dst[16] = '\0';
}

static void FDXB_Format_ISO(uint16_t country, uint64_t nat_id)
{
    (void)country;
    (void)nat_id;
    Format_ISO_Buf(g_fdxb.iso_str, g_fdxb.raw_id64);
}

static void Pets_Save_To_EEPROM(void)
{
    HAL_FLASHEx_DATAEEPROM_Unlock();
    uint32_t cnt = g_pets.count;
    if (cnt > MAX_LEARNED_PETS) cnt = MAX_LEARNED_PETS;
    (void)HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_PET_BASE + 0U, EEPROM_PET_MAGIC);
    (void)HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_PET_BASE + 4U, cnt);
    for (uint32_t i = 0U; i < cnt; i++) {
        uint64_t id = g_pets.ids[i];
        uint32_t lo = (uint32_t)(id & 0xFFFFFFFFULL);
        uint32_t hi = (uint32_t)(id >> 32U);
        (void)HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_PET_BASE + 8U + i * 8U, lo);
        (void)HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_PET_BASE + 12U + i * 8U, hi);
    }
    HAL_FLASHEx_DATAEEPROM_Lock();
}

static void Pets_Load_From_EEPROM(void)
{
    uint32_t magic = *(volatile uint32_t *)(EEPROM_PET_BASE + 0U);
    uint32_t cnt   = *(volatile uint32_t *)(EEPROM_PET_BASE + 4U);
    if (magic == EEPROM_PET_MAGIC && cnt <= MAX_LEARNED_PETS) {
        g_pets.count = cnt;
        for (uint32_t i = 0U; i < cnt; i++) {
            uint32_t lo = *(volatile uint32_t *)(EEPROM_PET_BASE + 8U + i * 8U);
            uint32_t hi = *(volatile uint32_t *)(EEPROM_PET_BASE + 12U + i * 8U);
            uint64_t id = ((uint64_t)hi << 32U) | (uint64_t)lo;
            g_pets.ids[i] = id;
            Format_ISO_Buf(g_pets.iso_strs[i], id);
        }
    } else {
        g_pets.count = 0U;
    }
}

static uint8_t Pets_Is_Authorized(uint64_t id64)
{
    if (id64 == 0ULL) return 0U;
    uint32_t cnt = g_pets.count;
    for (uint32_t i = 0U; i < cnt && i < MAX_LEARNED_PETS; i++) {
        if (g_pets.ids[i] == id64) return 1U;
    }
    return 0U;
}

static void Pets_Add(uint64_t id64)
{
    if (id64 == 0ULL) return;
    g_pets.last_seen_id = id64;
    Format_ISO_Buf(g_pets.last_seen_iso, id64);
    g_pets.learn_event_count++;
    if (!Pets_Is_Authorized(id64) && g_pets.count < MAX_LEARNED_PETS) {
        uint32_t idx = g_pets.count;
        g_pets.ids[idx] = id64;
        Format_ISO_Buf(g_pets.iso_strs[idx], id64);
        g_pets.count = idx + 1U;
        Pets_Save_To_EEPROM();
    }
    g_pets.last_authorized = 1U;
}

static void Battery_Check_Update(uint16_t loaded_pc4)
{
    static uint8_t s_low_streak = 0U;

    if ((ADC1->CR & ADC_CR_ADEN) == 0U) {
        ADC1->ISR = ADC_ISR_ADRDY;
        ADC1->CR |= ADC_CR_ADEN;
        while ((ADC1->ISR & ADC_ISR_ADRDY) == 0U) {}
    }

    ADC1_COMMON->CCR |= ADC_CCR_VREFEN;
    uint16_t tw = (uint16_t)TIM21->CNT;
    while ((uint16_t)((uint16_t)TIM21->CNT - tw) < 30U) {}

    ADC1->SMPR = ADC_SAMPLETIME_160CYCLES_5;
    ADC1->CHSELR = ADC_CHSELR_CHSEL17;

    /* Discard 1st conversion, then average 8 VREFINT readings */
    uint32_t sum = 0U;
    for (int i = 0; i < 9; i++) {
        ADC1->CR |= ADC_CR_ADSTART;
        while ((ADC1->ISR & ADC_ISR_EOC) == 0U) {}
        uint16_t v = (uint16_t)ADC1->DR;
        if (i > 0) sum += v;
    }

    ADC1->CHSELR = 0U;
    ADC1_COMMON->CCR &= ~ADC_CCR_VREFEN;
    HAL_ADC_Stop(&hadc);

    uint32_t vref_avg = sum >> 3U;
    uint16_t cal = *(const volatile uint16_t *)0x1FF80078U; /* VREFINT_CAL @ 3.000V */
    if (vref_avg > 0U && cal > 0U) {
        g_pets.vdd_mv = ((uint32_t)cal * 3000U) / vref_avg;
    }
    if (g_pets.vdd_mv == 0U) {
        g_pets.vdd_mv = 3300U;
    }

    if (loaded_pc4 >= 1200U) {
        g_pets.pc4_peak_counts = loaded_pc4;
        uint32_t pc4_mv = ((uint32_t)loaded_pc4 * g_pets.vdd_mv) / 4095U;
        g_pets.vbat_load_mv = (pc4_mv * 2100U) / 1000U;
    }

    uint8_t is_low = (g_pets.vdd_mv < 3150U ||
                      (g_pets.vbat_load_mv > 0U && g_pets.vbat_load_mv < 4500U)) ? 1U : 0U;
    uint8_t is_severe = (g_pets.vdd_mv < 3000U ||
                         (g_pets.vbat_load_mv > 0U && g_pets.vbat_load_mv < 4000U)) ? 1U : 0U;
    uint8_t is_healthy = (g_pets.vdd_mv >= 3220U &&
                          (g_pets.vbat_load_mv == 0U || g_pets.vbat_load_mv >= 4800U)) ? 1U : 0U;

    if (is_severe) {
        s_low_streak = 2U;
        g_pets.low_battery = 1U;
    } else if (is_low) {
        if (s_low_streak < 4U) s_low_streak++;
        if (s_low_streak >= 2U) g_pets.low_battery = 1U;
    } else if (is_healthy) {
        s_low_streak = 0U;
        g_pets.low_battery = 0U;
    }
}

static void FDXB_Capture_And_Decode(uint32_t ch)
{
    ADC1->SMPR = ADC_SAMPLETIME_12CYCLES_5;
    ADC1->CHSELR = (1U << ch);

    /* 1. 3 ms min/max pre-scan to center slicer threshold between actual peaks */
    uint16_t vmin = 4095U, vmax = 0U;
    uint16_t t_pre = (uint16_t)TIM21->CNT;
    while ((uint16_t)((uint16_t)TIM21->CNT - t_pre) < 3000U) {
        ADC1->CR |= ADC_CR_ADSTART;
        while ((ADC1->ISR & ADC_ISR_EOC) == 0U) {}
        uint16_t v = (uint16_t)ADC1->DR;
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
    }
    uint16_t span = (vmax > vmin) ? (uint16_t)(vmax - vmin) : 0U;
    if (span < 350U) {
        ADC1->SMPR = ADC_SAMPLETIME_160CYCLES_5;
        return;
    }
    uint16_t mid = (uint16_t)((vmin + vmax) / 2U);
    uint16_t hyst = (uint16_t)(span / 6U);
    if (hyst < 65U) hyst = 65U;
    if (hyst > 220U) hyst = 220U;
    uint16_t th_hi = (uint16_t)(mid + hyst);
    uint16_t th_lo = (uint16_t)(mid - hyst);

    /* 2. Capture up to 320 edge intervals over 65 ms (~2.1 full 128-bit FDX-B frames) */
    uint8_t  local_dt4[320];
    uint16_t dt_cnt = 0U;
    uint8_t  st = 2U;
    uint16_t v_hist[4] = {mid, mid, mid, mid};
    uint8_t  h_idx = 0U;
    uint16_t t0 = (uint16_t)TIM21->CNT, last_e = t0;

    while (((uint16_t)((uint16_t)TIM21->CNT - t0) < 65000U) && (dt_cnt < 320U)) {
        ADC1->CR |= ADC_CR_ADSTART;
        while ((ADC1->ISR & ADC_ISR_EOC) == 0U) {}
        v_hist[h_idx] = (uint16_t)ADC1->DR;
        h_idx = (uint8_t)((h_idx + 1U) & 3U);
        uint16_t v = (uint16_t)((v_hist[0] + v_hist[1] + v_hist[2] + v_hist[3]) >> 2U);

        uint8_t nst = st;
        if (v >= th_hi) nst = 1U;
        else if (v <= th_lo) nst = 0U;

        if (nst != 2U && nst != st) {
            uint16_t now = (uint16_t)TIM21->CNT;
            uint16_t dt = (uint16_t)(now - last_e);
            if (st == 2U || dt >= 65U) {
                if (st != 2U) {
                    uint16_t d4 = dt >> 2U;
                    local_dt4[dt_cnt++] = (d4 > 255U) ? 255U : (uint8_t)d4;
                }
                last_e = now;
                st = nst;
            }
        }
    }
    ADC1->SMPR = ADC_SAMPLETIME_160CYCLES_5;

    /* 3. Demodulate Differential Bi-Phase (DBP) intervals into bits */
    uint8_t  bits[256];
    uint16_t nbits = 0U;
    uint8_t  half_pending = 0U;
    for (uint16_t i = 0U; i < dt_cnt && nbits < 256U; i++) {
        uint16_t dt_us = (uint16_t)local_dt4[i] << 2U;
        if (dt_us >= 70U && dt_us <= 176U) {
            if (half_pending) {
                bits[nbits++] = 0U;
                half_pending = 0U;
            } else {
                half_pending = 1U;
            }
        } else if (dt_us >= 177U && dt_us <= 320U) {
            half_pending = 0U;
            bits[nbits++] = 1U;
        } else {
            half_pending = 0U;
        }
    }

    if (nbits >= 64U) {
        g_fdxb.last_bit_count = nbits;
        for (uint16_t i = 0U; i < 320U; i++) {
            g_fdxb.raw_dt_4us[i] = (i < dt_cnt) ? local_dt4[i] : 0U;
        }
        for (uint16_t i = 0U; i < 256U; i++) {
            g_fdxb.bit_stream[i] = (i < nbits) ? bits[i] : 0U;
        }
    }

    /* 4. Search bit_stream for 11-bit FDX-B header 00000000001 + 13x9-bit blocks (128 bits total) */
    for (uint16_t pos = 0U; pos + 128U <= nbits; pos++) {
        uint8_t hdr_ok = 1U;
        for (uint16_t h = 0U; h < 10U; h++) {
            if (bits[pos + h] != 0U) { hdr_ok = 0U; break; }
        }
        if (!hdr_ok || bits[pos + 10U] != 1U) continue;

        g_fdxb.frames_found++;
        uint8_t  bdata[13];
        uint16_t cmask = 0U, ccnt = 0U;
        for (uint16_t blk = 0U; blk < 13U; blk++) {
            uint16_t base = (uint16_t)(pos + 11U + blk * 9U);
            uint8_t val = 0U;
            for (uint16_t b = 0U; b < 8U; b++) {
                if (bits[base + b]) val |= (uint8_t)(1U << b);
            }
            bdata[blk] = val;
            if (bits[base + 8U]) {
                cmask |= (uint16_t)(1U << blk);
                ccnt++;
            }
        }

        uint16_t rx_crc = (uint16_t)bdata[8] | ((uint16_t)bdata[9] << 8U);
        uint16_t calc_crc = FDXB_Calc_CRC16(bdata);
        uint8_t  crc_match = (cmask == 0x1FFFU && rx_crc == calc_crc) ? 1U : 0U;

        if (crc_match || (g_fdxb.crc_ok_count == 0U && ccnt >= g_fdxb.best_ctrl_count)) {
            g_fdxb.best_ctrl_count = ccnt;
            g_fdxb.ctrl_bits_mask = cmask;
            g_fdxb.rx_crc16 = rx_crc;
            g_fdxb.calc_crc16 = calc_crc;
            for (int k = 0; k < 13; k++) g_fdxb.raw_bytes[k] = bdata[k];

            uint64_t id64 = 0ULL;
            for (int k = 0; k < 8; k++) {
                id64 |= ((uint64_t)bdata[k]) << (8 * k);
            }
            g_fdxb.raw_id64 = id64;
            g_fdxb.national_id = id64 & 0x3FFFFFFFFFULL;
            g_fdxb.country_code = (uint16_t)((id64 >> 38U) & 0x3FFU);
            FDXB_Format_ISO(g_fdxb.country_code, g_fdxb.national_id);
            if (crc_match) g_fdxb.crc_ok_count++;
        }
    }
}

void RFID_Watch_Cycle(void)
{
    static uint32_t last_snap_seen = 0;
    uint32_t snap_req = (g_rfdiag.snap_count != last_snap_seen) ? 1U : 0U;
    if (snap_req) {
        last_snap_seen = g_rfdiag.snap_count;
        g_watch.btn_snaps++;
    }

    HAL_ADC_Start(&hadc);
    static const uint32_t chs[5] = {4, 5, 6, 7, 11};
    static const uint16_t mode_arrs[2] = {237, 238};
    static const uint16_t mode_tgts[2] = {1380, 1220};
    uint32_t crc_cycle_start = g_fdxb.crc_ok_count;
    for (int m = 0; m < 2; m++) {
        RFID_Carrier_Start(mode_arrs[m]);
        uint8_t pumped = RFID_Servo_PA7(mode_tgts[m]);
        HAL_Delay(pumped ? 8U : 2U);

        if (snap_req) {
            for (int c = 0; c < 5; c++) {
                ADC_Sample_Stats(chs[c], 128, &g_watch.live_avg[m][c], &g_watch.live_span[m][c]);
            }
        } else {
            ADC_Sample_Stats(5, 64, &g_watch.live_avg[m][1], &g_watch.live_span[m][1]);
        }

        /* Analog FDX-B interval counter on PA5 over 10 ms under synchronous HSE */
        ADC1->SMPR = ADC_SAMPLETIME_12CYCLES_5;
        ADC1->CHSELR = (1U << 5);
        uint16_t mid = g_watch.live_avg[m][1];
        uint16_t th_hi = (mid < 3980U) ? (uint16_t)(mid + 70U) : 4050U;
        uint16_t th_lo = (mid > 115U)  ? (uint16_t)(mid - 70U) : 45U;
        uint8_t  st = 2U;
        uint16_t fdxb_valid = 0U, edges = 0U;
        uint16_t t0 = (uint16_t)TIM21->CNT, last_e = t0;
        while ((uint16_t)((uint16_t)TIM21->CNT - t0) < 10000U) {
            ADC1->CR |= ADC_CR_ADSTART;
            while ((ADC1->ISR & ADC_ISR_EOC) == 0U) {}
            uint16_t v = (uint16_t)ADC1->DR;
            uint8_t nst = st;
            if (v >= th_hi) nst = 1U;
            else if (v <= th_lo) nst = 0U;
            if (nst != 2U && nst != st) {
                uint16_t now = (uint16_t)TIM21->CNT;
                uint16_t dt = (uint16_t)(now - last_e);
                if (st == 2U || dt >= 65U) {
                    if (st != 2U && ((dt >= 90U && dt <= 155U) || (dt >= 195U && dt <= 285U))) {
                        fdxb_valid++;
                    }
                    last_e = now;
                    st = nst;
                    edges++;
                }
            }
        }
        ADC1->SMPR = ADC_SAMPLETIME_160CYCLES_5;
        g_watch.live_pb2[m] = fdxb_valid;

        if (fdxb_valid >= 10U || snap_req) {
            uint32_t crc_before = g_fdxb.crc_ok_count;
            FDXB_Capture_And_Decode(5U);
            if (!snap_req && g_fdxb.crc_ok_count != crc_before) {
                g_chip.detected_now = 1U;
                break;
            }
        }

        if (snap_req) {
            for (int c = 0; c < 5; c++) {
                g_watch.snap_avg[m][c] = g_watch.live_avg[m][c];
                g_watch.snap_span[m][c] = g_watch.live_span[m][c];
            }
            g_watch.snap_pb2[m] = fdxb_valid;
            ADC1->SMPR = ADC_SAMPLETIME_12CYCLES_5;
            if (m == 0) {
                ADC_Capture_Burst(5, g_watch.snap_pa5_350, 32);
                ADC_Capture_Burst(4, g_rfdiag.snap_pa5, 48);
            } else if (m == 1) {
                ADC_Capture_Burst(5, g_rfdiag9.snap_pa7_238, 32);
                ADC_Capture_Burst(6, g_rfdiag.snap_pa6, 48);
            }
            ADC1->SMPR = ADC_SAMPLETIME_160CYCLES_5;
        }
    }

    if (g_fdxb.crc_ok_count != crc_cycle_start) {
        g_chip.detected_now = 1U;
    } else if (g_watch.cycle_count == 2U) {
        for (int m = 0; m < 2; m++) {
            for (int c = 0; c < 5; c++) {
                g_watch.base_avg[m][c] = g_watch.live_avg[m][c];
                g_watch.base_span[m][c] = g_watch.live_span[m][c];
            }
        }
    } else if (g_watch.cycle_count > 2U) {
        uint8_t any_hit = 0U;
        for (int m = 0; m < 2; m++) {
            int32_t d_sp4 = (int32_t)g_watch.live_span[m][0] - (int32_t)g_watch.base_span[m][0];
            int32_t d_sp5 = (int32_t)g_watch.live_span[m][1] - (int32_t)g_watch.base_span[m][1];
            if (d_sp4 < 0) d_sp4 = 0;
            if (d_sp5 < 0) d_sp5 = 0;
            uint32_t sc = (uint32_t)(d_sp4 + d_sp5) + (uint32_t)g_watch.live_pb2[m] * 200U;
            if (d_sp4 >= 350 || d_sp5 >= 350 || g_watch.live_pb2[m] >= 8U) {
                g_watch.hit_count[m]++;
                any_hit = 1U;
                if (sc >= g_watch.best_score[m]) {
                    g_watch.best_score[m] = sc;
                    for (int c = 0; c < 5; c++) {
                        g_watch.hit_avg[m][c] = g_watch.live_avg[m][c];
                        g_watch.hit_span[m][c] = g_watch.live_span[m][c];
                    }
                    g_watch.hit_pb2[m] = g_watch.live_pb2[m];
                }
            }
        }
        g_chip.detected_now = any_hit;
    }

    RFID_Carrier_Stop();
    HAL_ADC_Stop(&hadc);
    HSE_Exit_To_HSI_PLL();
    g_watch.cycle_count++;
}

void RFID_Full_Diag(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if ((ADC1->CR & ADC_CR_ADEN) == 0U) {
        ADC1->ISR = ADC_ISR_ADRDY;
        ADC1->CR |= ADC_CR_ADEN;
        while ((ADC1->ISR & ADC_ISR_ADRDY) == 0U) {}
    }

    /* Drive PB11 HIGH so TL974I RX stages are cleanly biased (PC0 = VDD/2); leave PH10 INPUT_PULLUP */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(15);

    /* Quick PA0 check (not tested earlier) at ARR=237 */
    RFID_Carrier_Start(237);
    HAL_Delay(15);
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    uint16_t dummy_sp;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_Delay(5);
    ADC_Sample_Stats(7, 16, &g_rfdiag.pa0_low_pa7, &dummy_sp);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_Delay(5);
    ADC_Sample_Stats(7, 16, &g_rfdiag.pa0_high_pa7, &dummy_sp);
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 1. Steady-state sweep (15ms settle per step) with PH10 INPUT_PULLUP */
    static const uint16_t st_arrs[10] = {180, 200, 220, 237, 255, 280, 300, 320, 340, 360};
    RFID_Carrier_Start(st_arrs[0]);
    for (int k = 0; k < 10; k++) {
        uint16_t sp;
        g_rfdiag.steady_arr[k] = st_arrs[k];
        TIM2->ARR = st_arrs[k];
        TIM2->CCR1 = (st_arrs[k] + 1U) / 2U;
        TIM2->EGR = TIM_EGR_UG;
        HAL_Delay(15);
        ADC_Sample_Stats(7, 16, &g_rfdiag.steady_pa7[k], &sp);
        ADC_Sample_Stats(14, 8, &g_rfdiag.steady_pc4[k], &sp);
    }

    /* 2. High-speed (~1.8 us/sample) burst on PA4, PA5, PA6 at ARR=237 */
    RFID_Carrier_Start(237);
    HAL_Delay(15);
    ADC1->SMPR = ADC_SAMPLETIME_12CYCLES_5;
    uint16_t t0 = (uint16_t)TIM21->CNT;
    ADC_Capture_Burst(4, g_rfdiag.fast_pa4, 48);
    g_rfdiag.fast_burst_us = (uint16_t)((uint16_t)TIM21->CNT - t0);
    ADC_Capture_Burst(5, g_rfdiag.fast_pa5, 48);
    ADC_Capture_Burst(6, g_rfdiag.fast_pa6, 48);
    ADC1->SMPR = ADC_SAMPLETIME_160CYCLES_5;

    /* 3. Passive GPIO pull-up/pull-down map across Ports A..E, H (skip PA13/14 SWD) */
    RFID_Carrier_Stop();
    GPIO_TypeDef *const ports[6] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOH};
    const uint16_t safe_masks[6] = {
        (uint16_t)~(GPIO_PIN_13 | GPIO_PIN_14),
        0xFFFFU, 0xFFFFU, 0xFFFFU, 0xFFFFU,
        (uint16_t)(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_9 | GPIO_PIN_10)
    };
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    for (int p = 0; p < 6; p++) {
        GPIO_InitStruct.Pin = safe_masks[p];
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(ports[p], &GPIO_InitStruct);
        HAL_Delay(1);
        g_rfdiag2.gpio_nop[p] = (uint16_t)(ports[p]->IDR & safe_masks[p]);

        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(ports[p], &GPIO_InitStruct);
        HAL_Delay(1);
        g_rfdiag2.gpio_pup[p] = (uint16_t)(ports[p]->IDR & safe_masks[p]);

        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(ports[p], &GPIO_InitStruct);
        HAL_Delay(1);
        g_rfdiag2.gpio_pdn[p] = (uint16_t)(ports[p]->IDR & safe_masks[p]);
    }
    GPIO_Init_All();
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(15);

    /* 4. Safely measure PH0/PH1 & HSE_BYPASS frequency via RTC while SYSCLK stays on HSI */
    uint16_t ph_or = 0, ph_and = 0xFFFFU;
    for (int i = 0; i < 256; i++) {
        uint16_t v = (uint16_t)(GPIOH->IDR & (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_9 | GPIO_PIN_10));
        ph_or |= v;
        ph_and &= v;
    }
    g_rfdiag2.ph_or = ph_or;
    g_rfdiag2.ph_and = ph_and;

    /* Enable HSE bypass (divide by 16 for RTC -> /2 async = HSE/32 on SSR) */
    __HAL_RCC_PWR_CLK_ENABLE();
    PWR->CR |= PWR_CR_DBP;
    RCC->CR = (RCC->CR & ~RCC_CR_RTCPRE) | RCC_CR_RTCPRE_0 | RCC_CR_RTCPRE_1; /* /16 */
    RCC->CR |= RCC_CR_HSEBYP | RCC_CR_HSEON;
    uint16_t t_hse = (uint16_t)TIM21->CNT;
    while (((RCC->CR & RCC_CR_HSERDY) == 0U) && ((uint16_t)((uint16_t)TIM21->CNT - t_hse) < 20000U)) {}
    g_rfdiag2.hserdy_hi = (RCC->CR & RCC_CR_HSERDY) ? 1U : 0U;

    if (g_rfdiag2.hserdy_hi) {
        RCC->CSR |= RCC_CSR_RTCRST;
        RCC->CSR &= ~RCC_CSR_RTCRST;
        RCC->CSR = (RCC->CSR & ~RCC_CSR_RTCSEL) | RCC_CSR_RTCSEL_HSE | RCC_CSR_RTCEN;
        RTC->WPR = 0xCAU;
        RTC->WPR = 0x53U;
        RTC->ISR |= RTC_ISR_INIT;
        uint16_t ti = (uint16_t)TIM21->CNT;
        while (((RTC->ISR & RTC_ISR_INITF) == 0U) && ((uint16_t)((uint16_t)TIM21->CNT - ti) < 10000U)) {}
        RTC->PRER = 0x7FFFU;
        RTC->PRER = (1U << RTC_PRER_PREDIV_A_Pos) | 0x7FFFU; /* /2 async -> HSE/32 */
        RTC->CR |= RTC_CR_BYPSHAD;
        RTC->ISR &= ~RTC_ISR_INIT;
        HAL_Delay(2);

        uint32_t s0 = RTC->SSR & 0xFFFFU;
        uint16_t tw = (uint16_t)TIM21->CNT;
        while ((uint16_t)((uint16_t)TIM21->CNT - tw) < 4000U) {}
        uint32_t s1 = RTC->SSR & 0xFFFFU;
        uint32_t d_hi = (s0 >= s1) ? (s0 - s1) : (s0 + 0x8000U - s1);
        g_rfdiag2.ssr_delta_hi_4ms = d_hi;
        g_rfdiag2.ph0_est_khz_hi = (d_hi * 32U) / 4U;

        /* Test PH10 LOW effect on PH0 */
        GPIO_InitStruct.Pin = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_Delay(5);
        s0 = RTC->SSR & 0xFFFFU;
        tw = (uint16_t)TIM21->CNT;
        while ((uint16_t)((uint16_t)TIM21->CNT - tw) < 4000U) {}
        s1 = RTC->SSR & 0xFFFFU;
        g_rfdiag2.ssr_delta_lo_4ms = (s0 >= s1) ? (s0 - s1) : (s0 + 0x8000U - s1);
        g_rfdiag2.hserdy_lo = (g_rfdiag2.ssr_delta_lo_4ms > 0U) ? 1U : 0U;

        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
        RCC->CSR &= ~RCC_CSR_RTCEN;
    }
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP);

    /* 5. Wide ARR sweep 60..500 (533 kHz down to 64 kHz) */
    RFID_Carrier_Start(60);
    for (int k = 0; k < 45; k++) {
        uint16_t arr = (uint16_t)(60 + k * 10);
        uint16_t avg_pa4, sp_pa4, sp_tmp;
        g_rfdiag2.wide_arr[k] = arr;
        TIM2->ARR = arr;
        TIM2->CCR1 = (arr + 1U) / 2U;
        TIM2->EGR = TIM_EGR_UG;
        HAL_Delay(15);
        ADC_Sample_Stats(7, 16, &g_rfdiag2.wide_pa7[k], &sp_tmp);
        ADC_Sample_Stats(14, 8, &g_rfdiag2.wide_pc4[k], &sp_tmp);
        ADC_Sample_Stats(4, 32, &avg_pa4, &sp_pa4);
        g_rfdiag2.wide_pa4_span[k] = sp_pa4;
    }

    /* 6. Extended sub-64kHz sweep ARR=500..900 */
    for (int k = 0; k < 17; k++) {
        uint16_t arr = (uint16_t)(500 + k * 25);
        uint16_t sp_tmp;
        g_rfdiag2.sub_arr[k] = arr;
        TIM2->ARR = arr;
        TIM2->CCR1 = (arr + 1U) / 2U;
        TIM2->EGR = TIM_EGR_UG;
        HAL_Delay(15);
        ADC_Sample_Stats(7, 16, &g_rfdiag2.sub_pa7[k], &sp_tmp);
    }

    /* 7. High-speed PA4 bursts at ARR=340, 350, 360 + digital edge counts */
    ADC1->SMPR = ADC_SAMPLETIME_12CYCLES_5;
    static const uint16_t test_arrs[4] = {237, 340, 350, 360};
    for (int j = 0; j < 4; j++) {
        RFID_Carrier_Start(test_arrs[j]);
        HAL_Delay(20);
        if (j == 1) {
            ADC_Capture_Burst(4, g_rfdiag2.fast340_pa4, 48);
        } else if (j == 2) {
            uint16_t t0f = (uint16_t)TIM21->CNT;
            ADC_Capture_Burst(4, g_rfdiag2.fast350_pa4, 48);
            g_rfdiag2.fast350_us = (uint16_t)((uint16_t)TIM21->CNT - t0f);
        } else if (j == 3) {
            ADC_Capture_Burst(4, g_rfdiag2.fast360_pa4, 48);
        }
    }
    ADC1->SMPR = ADC_SAMPLETIME_160CYCLES_5;

    for (int j = 0; j < 4; j++) {
        RFID_Carrier_Start(test_arrs[j]);
        HAL_Delay(20);
        GPIO_InitStruct.Pin = GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        uint16_t ecnt = 0, prev = (uint16_t)(GPIOA->IDR & GPIO_PIN_4);
        uint16_t ts = (uint16_t)TIM21->CNT;
        while ((uint16_t)((uint16_t)TIM21->CNT - ts) < 15000U) {
            uint16_t cur = (uint16_t)(GPIOA->IDR & GPIO_PIN_4);
            if (cur != prev) { ecnt++; prev = cur; }
        }
        g_rfdiag2.edges_arr[j] = ecnt;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    RFID_Carrier_Start(237);

    /* 8. Check PB2 and PH1 carefully (50ms pull-up settle, safe push-pull only if pull-up reaches 1) */
    RFID_Carrier_Stop();
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_Delay(10);
    g_rfdiag3.pb2_nop_off = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);

    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_Delay(50);
    g_rfdiag3.pb2_pup_50ms = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    HAL_Delay(50);
    g_rfdiag3.ph1_pup_50ms = (uint8_t)HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_1);

    RFID_Carrier_Start(237);
    HAL_Delay(15);
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_Delay(5);
    g_rfdiag3.pb2_nop_237 = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);

    uint16_t sp_d3;
    if (g_rfdiag3.pb2_pup_50ms) {
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pin = GPIO_PIN_2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_Delay(15);
        ADC_Sample_Stats(7, 16, &g_rfdiag3.pb2_lo_pa7, &sp_d3);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_Delay(15);
        ADC_Sample_Stats(7, 16, &g_rfdiag3.pb2_hi_pa7, &sp_d3);
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    if (g_rfdiag3.ph1_pup_50ms) {
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pin = GPIO_PIN_1;
        HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_Delay(15);
        ADC_Sample_Stats(7, 16, &g_rfdiag3.ph1_lo_pa7, &sp_d3);
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_Delay(15);
        ADC_Sample_Stats(7, 16, &g_rfdiag3.ph1_hi_pa7, &sp_d3);
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    }

    /* 9. TIM2 duty cycle sweep at ARR=237 */
    static const uint16_t ccrs[11] = {20, 40, 60, 80, 100, 119, 140, 160, 180, 200, 220};
    for (int i = 0; i < 11; i++) {
        uint16_t avg4;
        g_rfdiag3.duty_ccr[i] = ccrs[i];
        TIM2->CCR1 = ccrs[i];
        HAL_Delay(15);
        ADC_Sample_Stats(7, 16, &g_rfdiag3.duty_pa7[i], &sp_d3);
        ADC_Sample_Stats(4, 32, &avg4, &g_rfdiag3.duty_pa4_span[i]);
    }
    TIM2->CCR1 = 119;

    /* 10. PC0/PC1 bias + PA5/PA6 at ARR=350 vs ARR=237 */
    HAL_Delay(15);
    ADC_Sample_Stats(10, 16, &g_rfdiag3.pc0_237, &sp_d3);
    ADC_Sample_Stats(11, 16, &g_rfdiag3.pc1_237, &sp_d3);
    RFID_Carrier_Start(350);
    HAL_Delay(20);
    ADC_Sample_Stats(10, 16, &g_rfdiag3.pc0_350, &sp_d3);
    ADC_Sample_Stats(11, 16, &g_rfdiag3.pc1_350, &sp_d3);
    ADC1->SMPR = ADC_SAMPLETIME_12CYCLES_5;
    ADC_Capture_Burst(5, g_rfdiag3.fast350_pa5, 24);
    ADC_Capture_Burst(6, g_rfdiag3.fast350_pa6, 24);
    ADC1->SMPR = ADC_SAMPLETIME_160CYCLES_5;

    /* 11. Push-pull HIGH test on all safe high-Z pins at ARR=237 */
    RFID_Carrier_Start(237);
    HAL_Delay(15);
    ADC_Sample_Stats(7, 16, &g_rfdiag3.pp_hi_base_pa7, &sp_d3);
    const uint16_t hi_skip[6] = {
        (uint16_t)(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15),
        (uint16_t)(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_11),
        (uint16_t)(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_10),
        0U,
        (uint16_t)(GPIO_PIN_4 | GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_12),
        (uint16_t)(GPIO_PIN_0 | GPIO_PIN_1)
    };
    for (int p = 0; p < 6; p++) {
        uint16_t mask = (uint16_t)(g_rfdiag2.gpio_pup[p] & ~g_rfdiag2.gpio_pdn[p] & ~hi_skip[p]);
        for (uint16_t b = 0; b < 16; b++) {
            uint16_t bit = (uint16_t)(1U << b);
            if ((mask & bit) == 0U) continue;
            GPIO_InitStruct.Pin = bit;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(ports[p], &GPIO_InitStruct);
            HAL_GPIO_WritePin(ports[p], bit, GPIO_PIN_SET);
            HAL_Delay(5);
            uint16_t p7 = 0;
            ADC_Sample_Stats(7, 8, &p7, &sp_d3);
            if ((int)p7 - (int)g_rfdiag3.pp_hi_base_pa7 > 60 || (int)g_rfdiag3.pp_hi_base_pa7 - (int)p7 > 60) {
                g_rfdiag3.pp_hi_hit_port = (uint16_t)p;
                g_rfdiag3.pp_hi_hit_pin = b;
                g_rfdiag3.pp_hi_pa7 = p7;
            }
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct.Pull = GPIO_PULLUP;
            HAL_GPIO_Init(ports[p], &GPIO_InitStruct);
        }
    }

    /* 12. Signal-flow map with safe 40k internal PULLDOWN / PULLUP (TX OFF) */
    RFID_Carrier_Stop();
    HAL_Delay(20);
    static const uint32_t probe_ch[6] = {4, 5, 6, 7, 10, 11};
    static GPIO_TypeDef *const probe_port[5] = {GPIOA, GPIOA, GPIOA, GPIOC, GPIOC};
    static const uint16_t probe_pin[5] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_0, GPIO_PIN_1};
    for (int c = 0; c < 6; c++) {
        ADC_Sample_Stats(probe_ch[c], 8, &g_rfdiag4.base_adc[c], &sp_d3);
    }
    for (int i = 0; i < 5; i++) {
        GPIO_InitStruct.Pin = probe_pin[i];
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(probe_port[i], &GPIO_InitStruct);
        HAL_Delay(10);
        g_rfdiag4.pdn_pb2[i] = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
        for (int c = 0; c < 6; c++) {
            if ((i < 3 && c == i) || (i >= 3 && c == i + 1)) {
                g_rfdiag4.pdn_effect[i][c] = (uint16_t)(HAL_GPIO_ReadPin(probe_port[i], probe_pin[i]) ? 4095 : 0);
            } else {
                ADC_Sample_Stats(probe_ch[c], 8, &g_rfdiag4.pdn_effect[i][c], &sp_d3);
            }
        }
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(probe_port[i], &GPIO_InitStruct);
        HAL_Delay(10);
        g_rfdiag4.pup_pb2[i] = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
        for (int c = 0; c < 6; c++) {
            if ((i < 3 && c == i) || (i >= 3 && c == i + 1)) {
                g_rfdiag4.pup_effect[i][c] = (uint16_t)(HAL_GPIO_ReadPin(probe_port[i], probe_pin[i]) ? 4095 : 0);
            } else {
                ADC_Sample_Stats(probe_ch[c], 8, &g_rfdiag4.pup_effect[i][c], &sp_d3);
            }
        }
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(probe_port[i], &GPIO_InitStruct);
    }

    /* 13. Check PB2 edges at ARR=237, 340, 350 */
    static const uint16_t pb2_arrs[3] = {237, 340, 350};
    for (int j = 0; j < 3; j++) {
        RFID_Carrier_Start(pb2_arrs[j]);
        HAL_Delay(25);
        if (j == 1) ADC_Sample_Stats(7, 8, &g_rfdiag4.pa7_340, &sp_d3);
        if (j == 2) ADC_Sample_Stats(7, 8, &g_rfdiag4.pa7_350, &sp_d3);
        uint16_t cnt = 0, prev = (uint16_t)(GPIOB->IDR & GPIO_PIN_2);
        uint16_t ts = (uint16_t)TIM21->CNT;
        while ((uint16_t)((uint16_t)TIM21->CNT - ts) < 20000U) {
            uint16_t cur = (uint16_t)(GPIOB->IDR & GPIO_PIN_2);
            if (cur != prev) { cnt++; prev = cur; }
        }
        if (j == 0) g_rfdiag4.pb2_edges_237 = cnt;
        else if (j == 1) g_rfdiag4.pb2_edges_340 = cnt;
        else g_rfdiag4.pb2_edges_350 = cnt;
    }

    /* 14. Test AC-coupled PB13 (C603) + PB14 (C602) charge-pump/gate drive */
    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    static const uint32_t d5_chs[5] = {4, 5, 6, 7, 11};
    uint16_t d5_sp;
    RFID_Carrier_Stop();
    HAL_Delay(40);
    for (int c = 0; c < 5; c++) ADC_Sample_Stats(d5_chs[c], 16, &g_rfdiag5.off_base[c], &d5_sp);
    g_rfdiag5.off_base[5] = (uint16_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);

    for (int mode = 0; mode < 4; mode++) {
        if (mode == 2) {
            RFID_Carrier_Start(237);
            HAL_Delay(30);
            for (int c = 0; c < 5; c++) ADC_Sample_Stats(d5_chs[c], 16, &g_rfdiag5.tx237_base[c], &d5_sp);
            g_rfdiag5.tx237_base[5] = (uint16_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
        }
        uint16_t half_us = (mode == 3) ? 25U : 5U;
        uint32_t bsrr_a = (mode == 1) ? (GPIO_PIN_13 | GPIO_PIN_14) : (GPIO_PIN_13 | ((uint32_t)GPIO_PIN_14 << 16));
        uint32_t bsrr_b = (mode == 1) ? (((uint32_t)GPIO_PIN_13 << 16) | ((uint32_t)GPIO_PIN_14 << 16))
                                      : (((uint32_t)GPIO_PIN_13 << 16) | GPIO_PIN_14);
        uint32_t n_half = (mode == 3) ? 1000U : 5000U; /* 25 ms total */
        uint16_t t_next = (uint16_t)TIM21->CNT;
        for (uint32_t k = 0; k < n_half; k++) {
            GPIOB->BSRR = (k & 1U) ? bsrr_b : bsrr_a;
            while ((uint16_t)((uint16_t)TIM21->CNT - t_next) < half_us) {}
            t_next = (uint16_t)(t_next + half_us);
        }
        volatile uint16_t *dst = (mode == 0) ? g_rfdiag5.off_comp_100k :
                                 (mode == 1) ? g_rfdiag5.off_inph_100k :
                                 (mode == 2) ? g_rfdiag5.tx237_comp_100k :
                                               g_rfdiag5.tx237_comp_20k;
        for (int c = 0; c < 5; c++) {
            ADC1->CHSELR = (1U << d5_chs[c]);
            uint32_t sum = 0;
            for (int s = 0; s < 16; s++) {
                GPIOB->BSRR = (s & 1U) ? bsrr_b : bsrr_a;
                ADC1->CR |= ADC_CR_ADSTART;
                while ((ADC1->ISR & ADC_ISR_EOC) == 0U) {}
                sum += (uint16_t)ADC1->DR;
            }
            dst[c] = (uint16_t)(sum / 16U);
        }
        dst[5] = (uint16_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
    }

    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 15. Compare HSI vs synchronous 32MHz HSE_BYPASS across 3 Q603 tune levels at ARR=238 */
    RFID_Carrier_Start(238);
    for (int pass = 0; pass < 2; pass++) {
        if (pass == 1) {
            HSE_Enter_32MHz();
            g_rfdiag10.hserdy = (RCC->CR & RCC_CR_HSERDY) ? 1U : 0U;
            g_rfdiag10.sws_hse = ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSE) ? 1U : 0U;
        }

        for (int lvl = 0; lvl < 3; lvl++) {
            GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

            if (lvl == 0) {
                /* Discharge Q603 via PB13 only, PB2=INPUT */
                GPIO_InitStruct.Pin = GPIO_PIN_2;
                GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
                uint16_t tn = (uint16_t)TIM21->CNT;
                for (uint32_t k = 0; k < 1500U; k++) {
                    GPIOB->BSRR = (k & 1U) ? ((uint32_t)GPIO_PIN_13 << 16) : GPIO_PIN_13;
                    while ((uint16_t)((uint16_t)TIM21->CNT - tn) < 20U) {}
                    tn = (uint16_t)(tn + 20U);
                }
            } else if (lvl == 1) {
                /* Charge Q603 via PB14/PB13 comp, PB2=INPUT (PA7 ~ 1750) */
                GPIO_InitStruct.Pin = GPIO_PIN_2;
                GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
                uint16_t tn = (uint16_t)TIM21->CNT;
                for (uint32_t k = 0; k < 4000U; k++) {
                    GPIOB->BSRR = (k & 1U) ? (((uint32_t)GPIO_PIN_13 << 16) | GPIO_PIN_14)
                                           : (GPIO_PIN_13 | ((uint32_t)GPIO_PIN_14 << 16));
                    while ((uint16_t)((uint16_t)TIM21->CNT - tn) < 5U) {}
                    tn = (uint16_t)(tn + 5U);
                }
            } else {
                /* Full tune: PB2=HIGH + 100kHz comp (PA7 ~ 3700) */
                RFID_Pump_Tune_Enable(25U);
            }
            GPIOB->BSRR = ((uint32_t)GPIO_PIN_13 << 16) | ((uint32_t)GPIO_PIN_14 << 16);

            uint16_t tw = (uint16_t)TIM21->CNT;
            while ((uint16_t)((uint16_t)TIM21->CNT - tw) < 10000U) {}

            uint16_t avg_tmp;
            if (pass == 0) {
                ADC_Sample_Stats(7, 32, &g_rfdiag10.hsi_pa7[lvl], &d5_sp);
                ADC_Sample_Stats(4, 32, &avg_tmp, &g_rfdiag10.hsi_pa4_sp[lvl]);
                ADC_Sample_Stats(5, 32, &avg_tmp, &g_rfdiag10.hsi_pa5_sp[lvl]);
                ADC_Sample_Stats(6, 32, &avg_tmp, &g_rfdiag10.hsi_pa6_sp[lvl]);
            } else {
                ADC_Sample_Stats(7, 32, &g_rfdiag10.hse_pa7[lvl], &d5_sp);
                ADC_Sample_Stats(4, 32, &avg_tmp, &g_rfdiag10.hse_pa4_sp[lvl]);
                ADC_Sample_Stats(5, 32, &avg_tmp, &g_rfdiag10.hse_pa5_sp[lvl]);
                ADC_Sample_Stats(6, 32, &avg_tmp, &g_rfdiag10.hse_pa6_sp[lvl]);
                if (lvl == 1) {
                    ADC1->SMPR = ADC_SAMPLETIME_12CYCLES_5;
                    ADC_Capture_Burst(5, g_rfdiag10.hse_pa5_l1, 32);
                    ADC1->SMPR = ADC_SAMPLETIME_160CYCLES_5;
                } else if (lvl == 2) {
                    ADC1->SMPR = ADC_SAMPLETIME_12CYCLES_5;
                    ADC_Capture_Burst(5, g_rfdiag10.hse_pa5_l2, 32);
                    ADC1->SMPR = ADC_SAMPLETIME_160CYCLES_5;
                }
            }
        }

        if (pass == 1) {
            HSE_Exit_To_HSI_PLL();
            g_rfdiag10.reverted_pll = ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) ? 1U : 0U;
        }
    }

    RFID_Carrier_Start(237);
    HAL_ADC_Stop(&hadc);
}

void GPIO_Init_All(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

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

    /* Port H: Leave PH0 untouched when HSE is active; PH9/PH10 with PULLUP */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = g_rfdiag.hse_active ? GPIO_PIN_1 : (GPIO_PIN_0 | GPIO_PIN_1);
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

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

    /* Safe initial boot on HSI PLL 32 MHz */
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
    g_rfdiag.hse_active = 0;
}

void SysTick_Handler(void)
{
    HAL_IncTick();

    if (g_led_systick_blink == 1U) {
        uint32_t t = HAL_GetTick();
        if ((t % 1000U) < 500U) LED_GREEN_ON(); else LED_GREEN_OFF();
        LED_RED_OFF();
    } else if (g_led_systick_blink == 2U) {
        uint32_t t = HAL_GetTick();
        if ((t % 240U) < 120U) LED_GREEN_ON(); else LED_GREEN_OFF();
        LED_RED_OFF();
    }

    s_motor_pwm_counter++;

    if (s_target_motor_dir == MOTOR_STOP) {
        g_motor_settled_on = 0U;
        GPIOE->BSRR = ((uint32_t)GPIO_PIN_11 << 16U) | ((uint32_t)GPIO_PIN_12 << 16U);
        return;
    }

    if (s_target_duty_pct >= 100) {
        g_motor_settled_on = 1U;
        if (s_target_motor_dir == MOTOR_OPEN) {
            GPIOE->BSRR = ((uint32_t)GPIO_PIN_12 << 16U) | GPIO_PIN_11;
        } else if (s_target_motor_dir == MOTOR_CLOSE) {
            GPIOE->BSRR = ((uint32_t)GPIO_PIN_11 << 16U) | GPIO_PIN_12;
        }
        return;
    }

    uint8_t on_now;
    if (s_target_duty_pct <= 40U) {
        /* ~1/3 speed (33.3% duty @ 83 Hz): 4 ms ON, 8 ms OFF; ms 1..3 are settled for ADC */
        uint32_t step = s_motor_pwm_counter % 12U;
        on_now = (step < 4U) ? 1U : 0U;
        g_motor_settled_on = (step >= 1U && step < 4U) ? 1U : 0U;
    } else {
        /* 75% duty (6/8 ms @ 125 Hz) or 50% duty (4/8 ms @ 125 Hz); skip step 0 turn-on transient */
        uint32_t step = s_motor_pwm_counter % 8U;
        uint32_t on_steps = (s_target_duty_pct >= 70U) ? 6U : 4U;
        on_now = (step < on_steps) ? 1U : 0U;
        g_motor_settled_on = (on_now && step >= 1U) ? 1U : 0U;
    }

    if (on_now) {
        if (s_target_motor_dir == MOTOR_OPEN) {
            GPIOE->BSRR = ((uint32_t)GPIO_PIN_12 << 16U) | GPIO_PIN_11;
        } else if (s_target_motor_dir == MOTOR_CLOSE) {
            GPIOE->BSRR = ((uint32_t)GPIO_PIN_11 << 16U) | GPIO_PIN_12;
        }
    } else {
        /* Freewheel phase: both gates LOW */
        GPIOE->BSRR = ((uint32_t)GPIO_PIN_11 << 16U) | ((uint32_t)GPIO_PIN_12 << 16U);
    }
}
