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
| **63** | `PC6` (`TIM22_CH1` / `TIM3_CH1`) | Gate Driver `Q807` via `R825` ($220\ \Omega$) | **IR Pet Arch Emitter LED 1** (Active HIGH: drives Q807 to sink cathode) |
| **64** | `PC7` (`TIM22_CH2` / `TIM3_CH2`) | Gate Driver `Q806` via `R821` ($220\ \Omega$) | **IR Pet Arch Emitter LED 2** (Active HIGH: drives Q806 to sink cathode) |
| **65** | `PC8` | Gate Driver `Q801` | **IR Receiver Gate / Enable** (Active LOW: 0V enables receiver, 3.3V clamps to GND) |
| **35** | `PB0` (`ADC_IN8`) | Demodulator Output via `R808` / `IC301` | **IR Receiver Analog Signal** (Pulsed transimpedance optical voltage drop) |
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

### 3.4 IR Pet Arch Emitter (100% Reverse Engineered & Verified)
* **Emitter Circuit Topology:** Common-Anode Capacitor-Discharge Switched 940nm Dual IR LEDs:
  * **Common Anode Rail (`C812` Reservoir):** Middle red wire supplied from the switched receiver/emitter rail (`D801.2` / `C814.1`) via $1.0\text{ k}\Omega$ charging resistor `R820` (`01B`) into reservoir capacitor `C812` (with $4.7\text{ k}\Omega$ parallel bleed resistors `R826` / `R822`).
  * **IR LED 1 (Cathode):** Left wire connected to collector of `Q807` (`6CW` / BC817-40 NPN).
    * Driven by **`PC6`** (Pin 63, `TIM22_CH1` / `TIM3_CH1`) via base resistor `R825` ($220\ \Omega$, `221`) and base-emitter pull-down `R824` (`470`).
    * Active HIGH: driving `PC6 = HIGH` switches `Q807` ON, discharging `C812` through IR LED 1 into `PB0` (`ADC_IN8`) primary receiver.
  * **IR LED 2 (Cathode):** Right wire connected to collector of `Q806` (`6CW` / BC817-40 NPN).
    * Driven by **`PC7`** (Pin 64, `TIM22_CH2` / `TIM3_CH2`) via base resistor `R821` ($220\ \Omega$, `221`) and base-emitter pull-down `R823` (`470`).
    * Active HIGH: driving `PC7 = HIGH` switches `Q806` ON, discharging `C812` through IR LED 2 into `PB1` (`ADC_IN9`) primary receiver.
  * **Wavelength & Forward Voltage:** 940nm near-infrared with measured $V_F = 1.1\text{V}$.

### 3.5 IR Pet Arch Receiver — Dual 2-Stage Differentiated Pulse Amplifiers (100% Reverse Engineered & Verified)
* **Power Regulation & High-Side Switch (`IC301` & `Q801`):**
  * **`IC301` (`AABK LJ21` SOT-23-3):** Dedicated ultra-low quiescent LDO voltage regulator fed from the $+5.98\text{V}$ battery rail (`Pin 2`, `C302`), `Pin 1` = `VSS`, outputting regulated voltage on `Pin 3` (`C301`) through series filter resistor `R801` ($220\ \Omega$) to reservoir capacitor `C801`.
  * **`Q801` (`13W` / `3W` SOT-23-3, controlled by `PC8` / Pin 65):** Active-LOW high-side power switch. Driving `PC8 = LOW` ($0\text{V}$) turns `Q801` ON, energizing switched supply rail **`NET 7` ($V_{\text{CC\_IR}}$)** on `Q801.3` (decoupled by `C816`, `C844`, `C815`).
  * **`D801` (`JG` / BAV99 Dual Series Diode):** Drops `NET 7` (`Pin 1`) to provide the DC bias node `NET 17` (`Pin 3`) feeding the top of both $22\text{ k}\Omega$ photodiode pull-up resistors `R811` and `R802`, and couples via `Pin 2` (`C814.1`) to the emitter charging rail `NET 9` feeding `R820` ($1.0\text{ k}\Omega$) and reservoir cap `C812`.
* **Dual Symmetric 2-Stage AC Differentiating Amplifiers ($484\times$ High-Frequency AC Gain):**
  * **Channel A (`PAD_L` $\rightarrow$ `Q804` $\rightarrow$ `Q805` $\rightarrow$ `PB1` / Pin 36 / `ADC_IN9`):**
    * Sensor pull-up: `R811` = $22\text{ k}\Omega$ to `D801.3` (`L802` ferrite bead, `C818` EMI filter cap to `VSS`), AC-coupled through pale-grey Class-I C0G/NP0 capacitor `C807` into `Q804` (`1GW` / BC847C) base (biased by `R812` = $220\text{k}$ [$560\text{k}$ in-circuit] to `NET 7` and `R813` = $1\text{M}$ [$530\text{k}$ in-circuit] to `VSS`).
    * **Stage 1 (`Q804`):** Collector load `R814` ($33\text{ k}\Omega$) $\parallel$ `C808` to `NET 7`; Emitter DC resistor `R815` ($22\text{ k}\Omega$) to $1.66\text{V}$ bias rail (`NET 20`) $\parallel$ AC gain branch `R816` ($1.5\text{ k}\Omega$) + `C838`.
    * **Stage 2 (`Q805`):** Base driven directly by `Q804.3`; Collector load `R817` ($33\text{ k}\Omega$) $\parallel$ `C810` to `NET 7` outputting to **`PB1` (`ADC_IN9`)**; Emitter DC resistor `R818` ($33\text{ k}\Omega$) to `NET 20` $\parallel$ AC gain branch `R819` ($1.5\text{ k}\Omega$) + `C811` to `VSS`.
  * **Channel B (`PAD_R` $\rightarrow$ `Q802` $\rightarrow$ `Q803` $\rightarrow$ `PB0` / Pin 35 / `ADC_IN8`):**
    * Exact mirror twin: `PAD_R` ($22\text{ k}\Omega$ pull-up `R802`, EMI cap `C817`) $\xrightarrow{\text{C802}}$ `Q802` (`R803`/`R804` base bias, `R805` = $33\text{ k}\Omega \parallel \text{C803}$ collector, `R806` = $22\text{ k}\Omega$, `R807` = $1.5\text{ k}\Omega + \text{C804}$ emitter) $\rightarrow$ `Q803` (`R808` = $33\text{ k}\Omega \parallel \text{C805}$ collector, `R809` = $33\text{ k}\Omega$, `R827` = $1.5\text{ k}\Omega + \text{C806}$ emitter) outputting to **`PB0` (`ADC_IN8`)**.
* **Critical Design Insights for Firmware Engineers:**
  1. **Fast Edge Differentiation (`0–15 µs` Window):**
     * The input coupling capacitors (`C807`, `C802`) and collector filter capacitors (`C808`, `C803`, `C805`) are low-capacitance Class-I C0G/NP0 dielectric ceramic capacitors ($\sim 220\text{ pF to }1\text{ nF}$).
     * Consequently, the 2-stage amplifier acts as an **edge differentiator with a $\sim 30\ \mu\text{s}$ time constant**. It does **NOT** maintain a DC deflection during continuous illumination.
     * The peak optical response occurs strictly within the **first $0\text{--}15\ \mu\text{s}$** following an emitter transition! By $t = +75\ \mu\text{s}$, the output has completely returned to its $2860\text{-count}$ ($2.30\text{V}$) quiescent baseline.
     * Attempting to sample at $600\ \mu\text{s}$ or milliseconds after pulse onset measures zero optical pulse and captures only ambient room lighting noise.
  2. **Double-Edge Matched Filter Architecture:**
     * **Turn-ON Edge ($t = 0\text{..}11\ \mu\text{s}$):** Sinking photodiode current creates a negative voltage pulse at the output (`PB0` plunges by $\sim 810\text{ counts}$; `PB1` plunges by $\sim 1,120\text{ counts}$).
     * **Turn-OFF Edge ($t = 300\text{..}311\ \mu\text{s}$):** Emitter turn-off produces an equal and opposite positive recoil spike (`PB0` jumps by $\sim 485\text{ counts}$; `PB1` jumps by $\sim 610\text{ counts}$).
     * Firmware evaluates both edges: $\Delta_{\text{matched}} = (v_{\text{pre}} - v_{\text{on\_edge}}) + (v_{\text{off\_edge}} - v_{\text{mid}})$. Requiring both edges to be valid ($>100\text{ counts}$) creates an ambient noise barrier that renders the detector immune to 50 Hz/100 Hz room light flicker and shadow artifacts.
  3. **Mains Quiet Half-Cycle Synchronization:**
     * In rooms with 50 Hz / 60 Hz half-wave rectified LED lighting, ambient illumination produces high-frequency switching bursts during one half of each AC cycle ($9.5\text{ ms}$) and complete dark silence during the other ($10.5\text{ ms}$).
     * Syncing the $300\ \mu\text{s}$ pulse capture to start within the quiet half-cycle guarantees clean, jitter-free optical measurements with $>500\text{ counts}$ of headroom against false triggers.
  4. **Empirical Operating Values & Thresholds:**
     * **Beam 1 (`PC6` $\rightarrow$ `PB0` / `ADC_IN8`):** Clear $\approx 1,100\text{ to }1,300\text{ counts}$; Blocked threshold = **`650 counts`**.
     * **Beam 2 (`PC7` $\rightarrow$ `PB1` / `ADC_IN9`):** Clear $\approx 1,680\text{ to }1,780\text{ counts}$; Blocked threshold = **`900 counts`**.

---

## 4. Work in Progress (Upcoming Subsystems)
* **Single-Button Interactive Lid Control:** Integrate break-beam pet safety into lid opening/closing logic (automatic lid open on pet approach, safety stall cutoff, re-open if pet enters while closing).
* **RFID Transceiver:** 134.2 kHz carrier generator (PWM timer) and envelope receiver/comparator for ISO 11784/11785 FDX-B pet microchip decoding.
* **Battery Voltage Monitor:** ADC resistor divider monitoring 4× C-cell battery health (nominal 6.0V down to 4.0V cutoff).

