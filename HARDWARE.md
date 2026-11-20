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

### 3.6 RFID Transceiver — 134.2 kHz Synchronous TX Driver, Closed-Loop 600-Series Tank Tuner & 700-Series RX Demodulator (100% Reverse Engineered & Verified)

#### 3.6.1 32.000 MHz Master Crystal & Synchronous Re-Clocking Stage (`Y101`, `IC102`, `IC103`)
* **`Y101` (`32.000 MHz` SMD Crystal/Oscillator):** Master RF timing reference.
* **`IC103` (`VHE`, Single Gate Buffer, SOT-23-5):**
  * **Enable (`Pin 1`):** Controlled by **`PH10`** (STM32 Pin 11, pulled HIGH via `R108` $10\text{ k}\Omega$ / internal pull-up). Driving `PH10 = LOW` shuts down the $32.000\text{ MHz}$ clock output (`hserdy_lo = 0`); keeping `PH10 = HIGH` (`INPUT_PULLUP`) enables the buffer.
  * **Clock Output:** Drives **`PH0-OSC_IN`** (STM32 Pin 12) via `R10` ($32.000\text{ MHz}$ external clock input for `HSE_BYPASS`).
* **`IC102` (`C79J` = `SN74LVC1G79` Single Positive-Edge-Triggered D Flip-Flop, SOT-23-5):**
  * **Data Input (`Pin 1`, `D`):** Driven by **`PE9`** (STM32 Pin 40, `TIM2_CH1` PWM carrier output, `AF0`).
  * **Clock Input (`Pin 2`, `CLK`):** Driven directly from `Y101.1` via `R107` / `R105`.
  * **Supply (`Pin 5`, `VCC`):** $+3.3\text{V}$ (`U1.50`, decoupled by `C105`/`C115`); `Pin 3` (`GND`) = `VSS`.
  * **Re-Clocked Carrier Output (`Pin 4`, `Q`):** Feeds the dual gate-driver buffers `IC601` and `IC602` via `D603.3` and $1\text{ k}\Omega$ series resistors `R609` and `R610`.
* **CRITICAL Firmware Requirement — Synchronous `32.000 MHz HSE_BYPASS` During RFID Scans:**
  * If `TIM2` (`PE9`) runs off the STM32's internal `HSI` RC oscillator ($\sim 31.984\text{ MHz}$) while `IC102` re-clocks `PE9` against the $32.000\text{ MHz}$ `Y101` crystal, the $\sim 16\text{ kHz}$ frequency difference causes `IC102` to cycle-slip by $31.25\text{ ns}$ at a **$\sim 12.8\text{ kHz}$ beat frequency**.
  * That $12.8\text{ kHz}$ phase/PWM slip modulates the high-$Q$ antenna tank right inside the RX filter passband, saturating `IC701` (`PA4` and `PA5` rail-to-rail at $\sim 4050\text{ ADC counts}$) and completely blinding the receiver.
  * **Solution:** During every RFID scan window, firmware enables `RCC_CR_HSEBYP | RCC_CR_HSEON` and switches `SYSCLK` directly to **`RCC_CFGR_SW_HSE` ($32.000\text{ MHz}$ direct, NO PLL — never feed $32\text{ MHz}$ into `PLLMUL_4`, which would exceed the $96\text{ MHz}$ max VCO and lock the core)**, then reverts to `RCC_CFGR_SW_PLL` (`HSI * 4 / 2`) after the scan. Under synchronous `HSE_BYPASS`, the $12.8\text{ kHz}$ beat disappears completely (`PA4` span drops from $4036 \rightarrow 84\text{ counts}$; `PA5` span drops from $3252 \rightarrow 91\text{ counts}$).

#### 3.6.2 134.2 kHz Coil Excitation Driver (`IC601`, `IC602`, `Q606`, `D603/605/606`, `C619`)
* **`IC601` & `IC602` (Parallel SOT-23-5 Push-Pull Gate Buffers):**
  * Inputs (`Pins 1 & 2`) fed from `IC102.4` via `R609`/`C617`/`D603.1` (`IC601`) and `R610`/`C618`/`D603.2` (`IC602`).
  * Supply (`Pin 5`) fed from the $+6\text{V}$ battery rail via `R615` ($100\ \Omega$, decoupled by `C622`/`C623`).
  * Outputs (`IC601.4` $\rightarrow$ `R611` $\rightarrow$ `Q606.3`; `IC602.4` $\rightarrow$ `R612` $\rightarrow$ `Q606.1`).
* **`Q606` (SOT-23-6 Complementary / Dual MOSFET Stage):**
  * Supply/return path via `Q606.2` $\rightarrow$ `D606.1/2` $\rightarrow$ `D605.1(K)` $\rightarrow$ `D605.2(A)` $\rightarrow$ `C625` / `R616` ($0.47\ \Omega$) / `C626` ($220\ \mu\text{F}\ 16\text{V}$ bulk reservoir).
  * Output (`Q606.4 & 6`) drives antenna pad **`TPA1`** (`A601`) and parallel snubber capacitor `C619` (to `GND`).
* **Carrier Frequency (`TIM2_CH1` on `PE9` at $32.000\text{ MHz}$):**
  * `ARR = 238, CCR1 = 119` $\rightarrow$ $32.000\text{ MHz} / 239 = \mathbf{133.89\text{ kHz}}$
  * `ARR = 237, CCR1 = 119` $\rightarrow$ $32.000\text{ MHz} / 238 = \mathbf{134.45\text{ kHz}}$
  * Nominal ISO 11784/11785 FDX-B frequency = $\mathbf{134.2\text{ kHz}}$.

#### 3.6.3 600-Series High-Voltage Tank Resonance Tuner & Closed-Loop AGC (`Q603`, `Q601/602/604/607`, `D601/602/604`, `PB13`, `PB14`, `PB2`, `PA7`)
* **Unswitched vs. Tuned Resonance:**
  * With `Q603` (`FQD6N40C`, $400\text{V}$ DPAK N-MOSFET) **OFF**, the antenna tank (`TPA0` $\leftrightarrow$ `C609/C610/C612/C613/C607/C605`) self-resonates at **$\sim 93.8\text{ kHz}$ (`ARR = 340`)**, producing minimal amplitude at $134.2\text{ kHz}$ (`PA7` $\approx 460\text{ counts}$).
  * Turning `Q603` **ON** (and biasing `Q604`/`Q607`) switches the tuning capacitor network onto the $134.2\text{ kHz}$ band, raising `PA7` at `ARR = 238` from $460\text{ counts}$ all the way up to $3869\text{--}4095\text{ counts}$.
* **AC-Coupled Gate Charge Pump & Discharge Control:**
  * **`PB14` (STM32 Pin 53) — Charge-UP Pump:** AC-coupled through series capacitor **`C602`** $\rightarrow$ `R602` / `D601` / `Q601` $\rightarrow$ gate storage capacitor **`C604.2`** $\rightarrow$ `R604` $\rightarrow$ **`Q603.1` (Gate)**. Pulsing `PB14` ($20\text{--}100\text{ kHz}$) steps `Q603` gate voltage **UP**, increasing tank resonance amplitude at $134.2\text{ kHz}$.
  * **`PB13` (STM32 Pin 52) — Discharge-DOWN Pump:** AC-coupled through series capacitor **`C603`** $\rightarrow$ `R603` / `D602` (referenced to `GND`) / `Q602` $\rightarrow$ **`C604.2`** (`Q603.1` Gate). Pulsing `PB13` alone discharges `C604.2`, stepping `Q603` gate voltage **DOWN** (`PA7` at `ARR = 238` returns to $459\text{ counts}$ and `ARR = 340` returns to $2733\text{ counts}$).
  * **`PB2` (STM32 Pin 37) — High-Range Bias Switch (`Q607`):** Drives `Q607.1` (`6CW` NPN base) via `R605`, coupling `Q607.3` (`R618`/`R617`/`C601.1`/`Q604.1`) to `Q607.2` (`C611.1`/`R606.2`/`C606.1`/`C604.1`). Driving `PB2 = HIGH` shifts the tank to maximum amplitude (`PA7` $= 3585\text{--}4095$ at `ARR = 238..255`).
* **Tank Envelope Detector (`D604` $\rightarrow$ `PA7` / Pin 32 / `ADC_IN7`):**
  * `D604.3` connects to `Q603.3` (`TAB`/Drain) and tank capacitor bank `C607.2`/`C609.2`/`C610.2`/`C612.2`/`C613.2`.
  * `D604.2` rectifies the tank envelope onto `C620` (to `GND`), feeds voltage divider `R613` / `R614` + filter cap `C621` into **`PA7` (`ADC_IN7`)**, and AC-couples via **`C706`** into the `700`-series RX amplifier (`R704` $\rightarrow$ `D701.3` $\rightarrow$ `IC701.3`).
* **Closed-Loop Servo Setpoint (`RFID_Servo_PA7`):**
  * Under synchronous $32.000\text{ MHz}$ `HSE_BYPASS`, over-pumping `PA7` above $\sim 1600\text{ counts}$ overdrives the `D701.3`/`IC701` front-end into self-oscillation.
  * Regulating `PA7` to **`1100–1400 counts`** (via `PB14` UP / `PB13` DOWN bursts with `PB2 = INPUT`) keeps the $134.2\text{ kHz}$ field strong while `IC701` stays completely quiet at idle (`PA4` span $\approx 80\text{--}110\text{ counts}$, `PA5` span $\approx 90\text{--}160\text{ counts}$, `0` false edges) and yields a massive $2000\text{--}3500\text{ count}$ swing when an FDX-B microchip enters the arch.

#### 3.6.4 700-Series Quad Op-Amp RX Chain (`IC701`, `IC702`, `IC703`, `IC704`, `Q702`–`Q704`, `D701`, `D102`)
* **RX Power & Mid-Rail `V_REF` Switch (`PB11` / Pin 48):**
  * Driving **`PB11 = HIGH`** turns on `Q702` (via `R701`), which switches on high-side transistor `Q703` (`R702`/`R703`/`R706`/`R707`), powering the RX rail and generating the half-supply bias reference **`V_REF`** (`Q703.2` $\rightarrow$ `R708`/`C731` $\rightarrow$ `+` inputs `IC701.5, 10, 12` and `IC702.3, 5, 10, 12`) as well as voltage divider `R111`/`R112` (`C121`) monitored on **`PC1` (`ADC_IN11` $\approx 2840\text{ counts}$ when ON, $0$ when OFF)**.
* **`IC701` (`TL974I` Rail-to-Rail Very-Low-Noise Quad Op-Amp, TSSOP-14):**
  * **Stage 1 (`Pins 1, 2, 3` — Input Buffer):** Tank envelope from `D604.2` couples through `C706` $\rightarrow$ `R704` $\rightarrow$ `D701.3` / `IC701.3` (`1IN+`, biased by `R705` to `V_REF`). `IC701.1` (`OUT1`) ties to `IC701.2` (`1IN-`) as a unity-gain follower.
  * **Stage 2 (`Pins 5, 6, 7` — Primary FDX-B Baseband Amplifier $\rightarrow$ `PA5`):** `IC701.1` couples via `C712` $\rightarrow$ `R718` $\rightarrow$ `IC701.6` (`2IN-`), with bandpass feedback `R719` $\parallel$ `C713` to `IC701.7` (`OUT2`). Output feeds `R720` $\rightarrow$ `C714.1` $\rightarrow$ **`D102.2` $\rightarrow$ `PA5` (STM32 Pin 30, `ADC_IN5`)**.
  * **Stages 4 & 3 (`Pins 12–14` & `8–10` — Cascaded Bandpass Filter $\rightarrow$ `PA4`):** `IC701.1` feeds `R711` $\rightarrow$ `C707`/`C708`/`R897`/`R713` into `IC701.13/14`, then via `R714`/`R715`/`C710`/`C709`/`R716` into `IC701.9/8` (`OUT3`), driving `R717` $\rightarrow$ `C711.1` $\rightarrow$ **`D102.1` $\rightarrow$ `PA4` (STM32 Pin 29, `ADC_IN4`)**.
  * **`D102` (Dual Series Clamp Diode to $+3.3\text{V}$):** `D102.3` ties to `VDD_3V3` (`U1.75`), clamping both `PA4` (`D102.1`) and `PA5` (`D102.2`) to $+3.3\text{V}$ to protect the STM32 ADC inputs from the $+5.8\text{V}$-supplied op-amps.
* **`IC702` (`TL974I` Second Quad Op-Amp), `Q704`, `IC703` & `IC704` ($\rightarrow$ `PA6` / Pin 31 / `ADC_IN6`):**
  * `IC701.1` also feeds `R722`/`R723`/`C717` through three stages of `IC702` (`OUT1` `Pin 1` $\rightarrow$ `OUT2` `Pin 7` $\rightarrow$ `OUT3` `Pin 8` $\rightarrow$ `R730`/`C722` $\rightarrow$ `Q704.1`) and envelope/comparator stages `IC703`/`IC704` (`R736` $\rightarrow$ `C727`/`C728`/`R737`/`C729`) and `IC702.14` (`OUT4`, `R738`/`R739`/`C730`), observed on **`PA6` (`ADC_IN6`)**.

#### 3.6.5 ISO 11784 / 11785 FDX-B 128-Bit Frame Protocol & Verified SureFlap Tags
* **Modulation & Bit Timing:**
  * Subcarrier bit rate: $134,200\text{ Hz} / 32 = \mathbf{4,193.75\text{ bps}}$ (**$238.45\ \mu\text{s}$ full-bit**, **$119.23\ \mu\text{s}$ half-bit**).
  * Differential Bi-Phase (DBP / Bi-Phase Mark): transition at every bit boundary; **`0` bit** = mid-bit transition (two $\sim 119\ \mu\text{s}$ half-bit intervals); **`1` bit** = no mid-bit transition (one $\sim 238\ \mu\text{s}$ full-bit interval, which also uniquely locks bit-cell boundary phase).
* **128-Bit Frame Layout:**
  * **11-Bit Sync Header:** `00000000001` ($10$ zeros + $1$, unique across the entire frame because every subsequent byte has a trailing `'1'` control bit).
  * **13 Blocks $\times$ 9 Bits ($117\text{ bits}$):** Each block has $8$ data bits (LSB first) followed by $1$ control bit (must be `'1'`, yielding `ctrl_bits_mask = 0x1FFF`).
    * **Bytes `0..7` ($64\text{ bits}$, LSB first):**
      * Bits `0..37` ($38\text{ bits}$): **National ID** ($12\text{-digit}$ decimal, `id64 & 0x3FFFFFFFFFULL`).
      * Bits `38..47` ($10\text{ bits}$): **Country / Manufacturer Code** ($3\text{-digit}$ decimal, `(id64 >> 38) & 0x3FFU`).
      * Bit `63` ($1\text{ bit}$): **Animal Application Flag** (`1` = animal tag, `0x80` in `byte[7]`).
    * **Bytes `8..9` ($16\text{ bits}$, LSB first):** **16-Bit CRC-CCITT** (reflected polynomial `0x8408`, init `0x0000` over `byte[0..7]`).
    * **Bytes `10..12` ($24\text{ bits}$):** Extension / trailer bytes.
* **Verified Hardware Tag Reads (`100%` CRC-16 Match):**
  * **SureFlap Tag 1:** `051-073300800864` (`raw_id64 = 0x80000cd111117560`, `CRC = 0x0e19 == 0x0e19`, $33/33$ frames valid)
  * **SureFlap Tag 2:** `051-073300800887` (`raw_id64 = 0x80000cd111117577`, `CRC = 0xc96f == 0xc96f`)

