# SureFeed Microchip Pet Feeder - Reverse Engineering & Hardware Specification

**Target Hardware:** SureFeed Microchip Pet Feeder (`01523-FG_04` / `MPF2 SureFlap (c) 2019`)  
**Daughterboard:** `01149-FG-01` (`MPF Connect daughter`)  
**Target Microcontroller:** STMicroelectronics **STM32L071VZT6**  
- **Architecture:** ARM Cortex-M0+ @ 32 MHz  
- **Package:** LQFP100 (100-pin, 14×14 mm)  
- **Flash:** 192 KB  
- **SRAM:** 20 KB  
- **EEPROM:** 6 KB  

---

## 1. Pinout Mapping Table

| STM32 Pin # | GPIO / Function | Circuit Net / Component | Function Description |
| :---: | :---: | :---: | :--- |
| **78** | `PC10` | Green LED Cathode | **Status LED - Green** (Active LOW: LOW = ON, HIGH = OFF) |
| **77** | `PA15` | Red LED Cathode | **Status LED - Red** (Active LOW: LOW = ON, HIGH = OFF, Green + Red = **Orange / Amber**) |
| **23** | `PA1` (`ADC_IN1`) | Daughterboard Button Bus | **Shared Analog Button Line** (All 3 buttons: SW1, SW2, SW3 with internal/external ~2.2k pull-down) |
| **89, 90, 91** | `PB3`, `PB4`, `PB5` | Slider Switch Throws (Inputs) | **3-Position Delay Switch Sense**: PB3 = Pos 1 (Fast), PB4 = Pos 2 (Medium), PB5 = Pos 3 (Slow) |
| **3** | `PE4` | Slider Switch Wiper (Drive LOW) | **Active Ground Return Rail** (Driven LOW / 0V by MCU) |
| **42** | `PE11` | Gate Driver / H-Bridge A | **Motor Drive A** (Lid Open Direction) |
| **43** | `PE12` | Gate Driver / H-Bridge B | **Motor Drive B** (Lid Close Direction) |
| **33** | `PC4` (`ADC_IN14`) | H-Bridge Supply Rail Sense | **Motor Current / Supply Sag Monitor** (Detects stall cutoff via rail voltage sag) |
| **76** | `PA14` | SWCLK / Debug Header | **SWD Clock** (Serial Wire Debug) |
| **72** | `PA13` | SWDIO / Debug Header | **SWD Data I/O** (Serial Wire Debug) |
| **14** | `NRST` | Reset Pad / Pushbutton | **Target Hardware Reset** (Active Low) |
| **74** | `VSS` / GND | Ground Plane | System Digital & Power Ground |
| **75** | `VDD` | 3.3V Power Rail | System VDD (Regulated from 4× C-cell batteries) |
| **22** | `VDDA` | 3.3V Analog Power Rail | Analog Reference & Daughterboard Button Power Source |

---

## 2. Debug & SWD Programming Header

The feeder uses an ST SWD interface. The factory boards have test pads / headers near the STM32:

* **SWCLK (STM32 Pin 76 / `PA14`):** Connect to Debugger SWCLK
* **SWDIO (STM32 Pin 72 / `PA13`):** Connect to Debugger SWDIO
* **NRST  (STM32 Pin 14 / `NRST`):** Connect to Debugger Reset (with 50ms active-low pulse capability)
* **GND   (STM32 Pin 74 / `VSS`):**  Common Ground
* **VDD   (STM32 Pin 75 / `VDD`):**  Target 3.3V reference

### Factory Silicon Unlocking & Option Bytes
Factory SurePetcare units are protected with ST Read-Out Protection (RDP Level 1) and a hardware-enforced Independent Watchdog (`IWDG`):
1. **RDP Level 1:** Option byte `0x1FF80000` is originally `0x00BB`. To unlock, change byte to `0x00AA` (Level 0). This triggers a full chip mass-erase and unlocks flash programming.
2. **Hardware Watchdog:** Factory option byte `0x1FF80004` has `WDG_SW = 0` (hardware watchdog enabled). On empty flash or custom firmware without watchdog refresh, the MCU resets endlessly every ~20ms.  
   * **Fix:** Reprogram option byte `0x1FF80004` to `0x8070` (sets `WDG_SW = 1`, making watchdog purely software-controlled).

---

## 3. Subsystem Architecture

### 3.1 Status LED
The status LED is a dual-die bi-color LED located on the button daughterboard:
* **Green:** Driven by **`PC10`** (**Active LOW**: Drive **LOW / 0V** to turn ON, **HIGH / 3.3V** to turn OFF).
* **Red:** Driven by **`PA15`** (**Active LOW**: Drive **LOW / 0V** to turn ON, **HIGH / 3.3V** to turn OFF).
* **Orange / Amber:** Achieved by driving **`PC10` LOW** and **`PA15` LOW** simultaneously.

### 3.2 User Buttons & Slider Switch
The user controls on the daughterboard connect via a 5-pin ribbon cable:
* **Pinout of Daughterboard Connector (Conn1):**
  1. `3.3V` (`VDDA` / Pin 22)
  2. `GND`
  3. `LED Red` (`PA15`)
  4. `LED Green` (`PC10`)
  5. **Shared Button Data Bus** $\rightarrow$ **`PA1` / `ADC_IN1`** (Pin 23)
* **Button Voltage Divider Network on `PA1`:**
  * The motherboard has an on-board pull-down resistor ($\sim 2.2\text{ k}\Omega$) pulling `PA1` to `0V` at idle.
  * Each button connects `3.3V` (`VDDA`) to `PA1` through a dedicated series resistor:
    * **`SW2` (Open / Close):** **$2.20\text{ k}\Omega$** $\rightarrow$ **$\sim 1.65\text{V}$ ($\sim 2050\text{ ADC counts}$)** $\rightarrow$ **Solid RED**
    * **`SW1` (Add Pet):** **$4.67\text{ k}\Omega$** $\rightarrow$ **$\sim 1.15\text{V}$ ($\sim 1310 - 1600\text{ ADC counts}$)** $\rightarrow$ **Solid ORANGE**
    * **`SW3` (Learn Mode):** **$7.45\text{ k}\Omega$** $\rightarrow$ **$\sim 0.58\text{V}$ ($\sim 650 - 770\text{ ADC counts}$)** $\rightarrow$ **Solid GREEN**
    * **Idle (Released):** **$0\text{V}$ ($< 300\text{ ADC counts}$)** $\rightarrow$ Heartbeat Green pulse, Red OFF.
* **Slider Switch (Lid Delay):**
  * 3-position slider switch located on the main board / chassis (Delay 1, 2, 3).
  * **Wiper / Common Pin:** Connected to **`PE4`** (Pin 3), driven LOW (`0V`) by the MCU as an active ground rail.
  * **Position 1 (Fast Delay):** Shorts `PE4` to **`PB3`** $\rightarrow$ `PB3` reads `0` (LOW) $\rightarrow$ **Solid GREEN**
  * **Position 2 (Medium Delay):** Shorts `PE4` to **`PB4`** $\rightarrow$ `PB4` reads `0` (LOW) $\rightarrow$ **Solid ORANGE**
  * **Position 3 (Slow Delay):** Shorts `PE4` to **`PB5`** $\rightarrow$ `PB5` reads `0` (LOW) $\rightarrow$ **Solid RED**

### 3.3 Motor Drive & Lid Mechanism (100% Reverse Engineered & Verified)
* **H-Bridge Control:** Discrete N-MOSFET / P-MOSFET bridge:
  * **`PE11`** (Pin 42) drives Forward / Open.
  * **`PE12`** (Pin 43) drives Reverse / Close.
  * **Safety:** Both pins must NEVER be high simultaneously (shoot-through dead-time strictly enforced in software).
* **Limitless Mechanical Design & Stall-Current Detection:**
  * The feeder has **NO mechanical limit switches, optical interrupters, or rotary encoders** on the lid or gearbox.
  * It employs dynamic **Supply Rail Voltage-Sag Stall Detection** via **`PC4` (`ADC_IN14`, Pin 33)**:
    1. **Idle State:** Motor OFF $\rightarrow$ `PC4` reads $0\text{V}$ ($15\text{ ADC counts}$).
    2. **Startup Blanking:** During the first $200\text{ ms}$, inrush motor starting current is blanked out.
    3. **Free Travel:** While the lid is moving freely, the motor draws $\sim 100\text{ mA}$, holding `PC4` at $\sim 2.86\text{V}$ ($3550 \pm 20\text{ ADC counts}$).
    4. **Mechanical End-Stop (Stall):** When the lid reaches fully open or fully closed, the motor stalls and current jumps to $\sim 1.0\text{ A}$. The battery/inductor internal resistance causes the supply rail to immediately plunge to $\sim 2.72\text{V}$ ($3385\text{ counts}$ — a sharp $\sim 175\text{ count}$ sag).
    5. **Automated Cutoff:** Calibrated with a `90 count` threshold ($\sim 75\text{ mV}$ sag). Cutoff triggers within milliseconds of contacting the end-stop, instantly cutting H-bridge drive and lighting solid Amber LED feedback.

---

## 4. Work in Progress (Upcoming Subsystems)
* **RFID Transceiver:** 134.2 kHz carrier generator (PWM timer) and envelope receiver/comparator for ISO 11784/11785 FDX-B pet microchip decoding.
* **IR Pet Barrier Gate:** Dual infrared emitter and phototransistor beam array in the archway to detect pet head presence and prevent lid closure while feeding.
* **Battery Voltage Monitor:** ADC resistor divider monitoring 4× C-cell battery health (nominal 6.0V down to 4.0V cutoff).

