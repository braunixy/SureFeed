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
| **89, 90, 91** | `PB3`, `PB4`, `PB5` | Slider Switch (Lid Delay) | **3-Position Delay Switch**: Position 1 (Fast), 2 (Medium), 3 (Slow) |
| **33** | `PC4` | Gate Driver Q501 | **Motor Drive A** (Lid Open/Close H-Bridge) |
| **34** | `PC5` | Gate Driver Q506 | **Motor Drive B** (Lid Open/Close H-Bridge) |
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
  * Wired to **`PB3`**, **`PB4`**, and **`PB5`** (Port B digital inputs).

### 3.3 Motor Drive & Lid Mechanism
* **H-Bridge Control:** Discrete N-MOSFET / P-MOSFET bridge:
  * **`PC4`** (Pin 33) drives Q501 (Forward / Open).
  * **`PC5`** (Pin 34) drives Q506 (Reverse / Close).
  * **Safety:** Both pins must NEVER be high simultaneously (dead-time must be respected to prevent shoot-through).
* **Limit Switch Mystery Solved:**
  * The feeder has **NO mechanical limit switches, optical interrupters, or rotary encoders** on the lid or gearbox.
  * The feeder uses **Motor Stall-Current Detection**:
    1. The MCU energizes the motor in the desired direction.
    2. The lid travels until it physically contacts the mechanical end-stop (fully open or fully closed).
    3. The motor stalls, causing a sharp current surge across an on-board shunt resistor.
    4. The MCU analog comparator / ADC detects the current spike and immediately cuts power to the motor.

---

## 4. Work in Progress (Upcoming Subsystems)
* **RFID Transceiver:** 134.2 kHz carrier generator (PWM timer) and envelope receiver/comparator for ISO 11784/11785 FDX-B pet microchip decoding.
* **IR Pet Barrier Gate:** Dual infrared emitter and phototransistor beam array in the archway to detect pet head presence and prevent lid closure while feeding.
* **Battery Voltage Monitor:** ADC resistor divider monitoring 4× C-cell battery health (nominal 6.0V down to 4.0V cutoff).

