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

## 1. Master STM32L071VZT6 (LQFP100) Pinout Mapping Table

| STM32 Pin # | GPIO / Function | Subsystem | Circuit Net / Component | Function Description |
| :---: | :---: | :---: | :--- | :--- |
| **3** | `PE4` | UI Slider | Slider Switch Wiper (Drive LOW) | **Active Ground Return Rail** for 3-Position Lid Delay Slider (`0V` output) |
| **11** | `PH10` | RFID Clock | `IC103.1` (`VHE` Enable) & `R108` ($10\text{ k}\Omega$) | **32.000 MHz Master Crystal Buffer Enable** (`HIGH` / Pull-Up = ON, `LOW` = OFF) |
| **12** | `PH0-OSC_IN` | RFID Clock | `IC103` Output via `R10` | **32.000 MHz External Clock Input (`HSE_BYPASS`)** from `Y101` crystal via `IC103` |
| **14** | `NRST` | System / SWD | `C113.1` & Reset Pad | **Target Hardware Reset** (Active LOW) |
| **16** | `PC1` (`ADC_IN11`) | RFID RX | `R111` / `R112` Divider & `C121` from `Q703.3` | **RFID RX Switched Power Rail Monitor** ($\approx 2840\text{ counts}$ when `PB11 = HIGH`, $0$ when OFF) |
| **21** | `VREF+` | Analog Power | `C855.1` Decoupling to `VSS` | **ADC Positive Reference Voltage** ($+3.3\text{V}$) |
| **22** | `VDDA` | Analog Power | `C856.1` & Daughterboard `Conn1.1` | **3.3V Analog Supply & Button Divider Source** |
| **23** | `PA1` (`ADC_IN1`) | UI Buttons | Daughterboard `Conn1.5` Button Bus | **Shared Analog Button Line** (`SW1` Add Pet, `SW2` Open/Close, `SW3` Training Mode) |
| **29** | `PA4` (`ADC_IN4`) | RFID RX | `IC701.8` (`OUT3`) $\rightarrow$ `R717` $\rightarrow$ `D102.1` | **RFID RX Cascaded Bandpass Output** ($+3.3\text{V}$ clamped via `D102.3`) |
| **30** | `PA5` (`ADC_IN5`) | RFID RX | `IC701.7` (`OUT2`) $\rightarrow$ `R720` $\rightarrow$ `C714.1` $\rightarrow$ `D102.2` | **RFID RX Primary FDX-B Baseband Demodulator Input** ($4,193.75\text{ bps}$ DBP waveform) |
| **31** | `PA6` (`ADC_IN6`) | RFID RX | `IC702.14` (`OUT4`) / `IC704.4` Filter Stage | **RFID RX Secondary Envelope / Filter Stage Output** |
| **32** | `PA7` (`ADC_IN7`) | RFID Tuner | `D604.2` $\rightarrow$ `R613` / `R614` Divider & `C621` | **RFID Tank Envelope Amplitude Feedback** (Closed-loop AGC input for `PB13`/`PB14` servo) |
| **33** | `PC4` (`ADC_IN14`) | Motor Drive | H-Bridge Supply Rail Sense | **Motor Current / Supply Sag Monitor** (End-stop stall cutoff detection) |
| **35** | `PB0` (`ADC_IN8`) | IR Gate RX | `Q803.3` Collector via `R808` $\parallel$ `C805` | **IR Break-Beam Channel B Analog Output** (Right Receiver `PAD_R`, paired with `PC6` LED 1) |
| **36** | `PB1` (`ADC_IN9`) | IR Gate RX | `Q805.3` Collector via `R817` $\parallel$ `C810` | **IR Break-Beam Channel A Analog Output** (Left Receiver `PAD_L`, paired with `PC7` LED 2) |
| **37** | `PB2` | RFID Tuner | `R605.1` $\rightarrow$ `Q607.1` (`6CW` NPN Base) | **RFID Tank High-Range Bias Switch** (Couples `Q607.3` $\leftrightarrow$ `Q607.2` / `C604.1`) |
| **40** | `PE9` (`TIM2_CH1`) | RFID TX | `IC102.1` (`SN74LVC1G79` Data `D` Input) | **134.2 kHz RFID TX Carrier Output** (`AF0`, re-clocked to $32.000\text{ MHz}$ `Y101` by `IC102`) |
| **42** | `PE11` | Motor Drive | H-Bridge Gate A | **Motor Drive A** (Lid Open Direction) |
| **43** | `PE12` | Motor Drive | H-Bridge Gate B | **Motor Drive B** (Lid Close Direction) |
| **48** | `PB11` | RFID RX | `R701.1` $\rightarrow$ `Q702.1` $\rightarrow$ `Q703` High-Side Switch | **RFID RX Op-Amp Power & `V_REF` Enable** (`HIGH` = RX Chain Powered ON, `LOW` = $0\ \mu\text{A}$ OFF) |
| **49, 74** | `VSS` / GND | Power | System Ground Plane (`IC102.3`, `PAD_M`, etc.) | **System Digital & Analog Ground** |
| **50, 75** | `VDD` / `VDDIO2` | Power | $+3.3\text{V}$ Regulated Rail (`IC102.5`, `D102.3`) | **System $+3.3\text{V}$ Logic Supply** |
| **52** | `PB13` | RFID Tuner | `C603.1` $\rightarrow$ `R603` / `D602` / `Q602` $\rightarrow$ `C604.2` | **RFID Tank Gate Charge-Pump DOWN (Discharge)** (AC-coupled; pulses step `Q603` gate DOWN) |
| **53** | `PB14` | RFID Tuner | `C602.1` $\rightarrow$ `R602` / `D601` / `Q601` $\rightarrow$ `C604.2` | **RFID Tank Gate Charge-Pump UP (Charge)** (AC-coupled; pulses step `Q603` gate UP) |
| **63** | `PC6` | IR Gate TX | `R825` ($220\ \Omega$) $\rightarrow$ `Q807` (`6CW` NPN Base) | **IR Pet Arch Emitter LED 1** (`HIGH` = sinks LED 1 cathode from `C812` reservoir) |
| **64** | `PC7` | IR Gate TX | `R821` ($220\ \Omega$) $\rightarrow$ `Q806` (`6CW` NPN Base) | **IR Pet Arch Emitter LED 2** (`HIGH` = sinks LED 2 cathode from `C812` reservoir) |
| **65** | `PC8` | IR Gate RX | High-Side Switch `Q801` Base | **IR Receiver Power Enable** (Active LOW: `0V` turns `Q801` ON to power `NET 7` $V_{\text{CC\_IR}}$) |
| **72** | `PA13` | Debug / SWD | SWDIO Pad | **Serial Wire Debug Data I/O** |
| **76** | `PA14` | Debug / SWD | SWCLK Pad | **Serial Wire Debug Clock** |
| **77** | `PA15` | UI LED | Daughterboard `Conn1.3` Red LED Cathode | **Status LED - Red** (Active LOW: `LOW` = ON, `HIGH` = OFF) |
| **78** | `PC10` | UI LED | Daughterboard `Conn1.4` Green LED Cathode | **Status LED - Green** (Active LOW: `LOW` = ON, `HIGH` = OFF; Green + Red = **Orange / Amber**) |
| **89, 90, 91** | `PB3`, `PB4`, `PB5` | UI Slider | Slider Switch Throws (`I`, `II`, `III`) | **3-Position Lid Close Delay Sense** (`PB3` = Pos `I` $3.5\text{s}$, `PB4` = Pos `II` $6.0\text{s}$, `PB5` = Pos `III` $10.0\text{s}$) |

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

---

## 4. Official SureFeed User Manual Parity & Workflow Specification

| Feature / Workflow | Official SureFeed Manual | Firmware Implementation ([`src/main.c`](src/main.c)) | Parity Status |
| :--- | :--- | :--- | :--- |
| **Max Learned Pets** | Up to **32** pet microchip IDs stored in permanent non-volatile memory. | Up to **32** pet IDs (`MAX_LEARNED_PETS = 32`) persisted in STM32L0 Data EEPROM (`0x08080000`). | **Exact Match** |
| **Normal Daily Operation (IR + RFID)** | Proximity/IR sensor in the hoop detects a pet approaching $\rightarrow$ activates the RFID reader $\rightarrow$ opens lid if microchip is authorized $\rightarrow$ keeps lid open while pet remains in hoop $\rightarrow$ closes lid after pet walks away. | Pulsed IR break-beam (`IR_Sample_Pulsed`) detects pet in arch (`ir_blocked = 1`) $\rightarrow$ powers up RFID antenna (`RFID_Power_Up`) $\rightarrow$ opens lid (`LID_OPEN_AUTO`) if `Pets_Is_Authorized` $\rightarrow$ powers off antenna while open $\rightarrow$ closes after IR clears for slider delay (`3.5s / 6.0s / 10.0s`). | **Exact Match** |
| **"Add Pet" (`SW1`, `BTN_ADD_PET`) Sequence** | 1. Press **Add Pet** $\rightarrow$ **lid automatically opens** and LED **flashes green** ($1\text{ Hz}$). Stays in Add Pet mode until a chip is read (or pressed again to cancel).<br>2. Pet approaches to eat $\rightarrow$ chip is read $\rightarrow$ LED turns **solid green** while pet eats.<br>3. When pet steps away (IR clears), **lid closes** and LED turns **off**. | 1. Press **Add Pet** $\rightarrow$ lid automatically opens (`Lid_Start_Opening(LID_OPEN_AUTO)`), LED **flashes green** ($1\text{ Hz}$), and RFID antenna scans continuously.<br>2. When chip is read (`crc_ok_count` increments) $\rightarrow$ ID saved to `g_pets` + Data EEPROM, antenna turns off, LED turns **solid green**.<br>3. When pet leaves (`ir_blocked = 0` for slider close delay), lid closes and LED turns **off**. | **Exact Match** |
| **Factory Reset (Erase All Pets)** | Hold **Add Pet** button for **$\sim 10\text{ seconds}$** $\rightarrow$ LED alternates **red / green** and lid cycles open & closed once $\rightarrow$ all learned pets wiped from memory. | Holding `BTN_ADD_PET` continuously for $\ge 10.0\text{ s}$ (`10000 ms`) wipes `g_pets` + Data EEPROM, alternates **red / green** LED ($150\text{ ms}$) for $3.0\text{ s}$, and cycles the lid open and closed once (`factory_reset_cycle_pending = 1`). | **Exact Match** |
| **3-Position Slider (`I / II / III`, `PB3..PB5`)** | **Lid Close Delay Slider:** controls how long the lid waits to close *after* the pet steps away (`I` = Short delay, `II` = Standard delay, `III` = Long delay). | Mapped to `g_pets.close_delay_ms`:<br>• **Position `I` (`PB3 = 0`):** **`3.5 s`** (`3500 ms`)<br>• **Position `II` (`PB4 = 0`):** **`6.0 s`** (`6000 ms`)<br>• **Position `III` (`PB5 = 0`):** **`10.0 s`** (`10000 ms`) | **Exact Match** |
| **"Training Mode" Button (`SW3`, `BTN_LEARN`)** | Cycles through **4 gradual training stages** (flashing **orange** $1\times, 2\times, 3\times, 4\times$) where the lid stays partially open / closes only partially so timid pets get used to lid movement; pressing once more exits Training Mode and closes the lid fully. | Pressing `BTN_LEARN` cycles `g_pets.training_stage` (`0 -> 1 -> 2 -> 3 -> 4 -> 0`), flashing **orange** $N\times$ ($1..4$ pulses per $2.4\text{ s}$ cycle):<br>• **Stage 1:** Lid stays $100\%$ open continuously.<br>• **Stage 2:** Closes partially ($250\text{ ms}$ from open, $\sim 75\%$ open); opens to $100\%$ for authorized pet.<br>• **Stage 3:** Closes to $\sim 50\%$ open ($450\text{ ms}$); opens to $100\%$ for authorized pet.<br>• **Stage 4:** Closes to $\sim 25\%$ open ($650\text{ ms}$); opens to $100\%$ for authorized pet.<br>• **Exit (`Stage 0`):** Closes lid $100\%$ to end-stop and resumes normal mode. | **Exact Match** |
| **Open / Close Button (`SW2`, `BTN_OPEN_CLOSE`)** | Opens the lid manually for refilling/cleaning bowls; pressing again closes the lid and resumes normal automatic operation. | Opens lid manually (`LID_OPEN_MANUAL`, pausing auto mode); pressing again closes lid and resumes `auto_mode_active = 1`. | **Exact Match** |
| **Idle & Active LEDs** | • **Idle:** LED completely **OFF**.<br>• **Add Pet Mode:** **Flashing Green** ($1\text{ Hz}$).<br>• **Eating / Read:** **Solid Green**.<br>• **Training Mode:** **Flashing Orange** ($1\times\text{--}4\times$).<br>• **Factory Reset:** **Alternating Red / Green**. | • **Idle:** LED completely **OFF**.<br>• **Add Pet Mode:** **Flashing Green** ($1\text{ Hz}$).<br>• **Eating / Read:** **Solid Green**.<br>• **Training Mode:** **Flashing Orange** ($1\times\text{--}4\times$).<br>• **Factory Reset:** **Alternating Red / Green**. | **Exact Match** |

---

## 5. Complete Reverse-Engineered PCB Component BOM & Verified Netlist Reference

### 5.1 100-Series Master Clock, Re-Clocking & ADC Clamp Components
| RefDes | Marking / Value | Package / Type | Verified Pin Connections & Function |
| :--- | :--- | :--- | :--- |
| **`Y101`** | `32.000 MHz` | 4-Pad SMD Crystal | `Pin 1`: `R107.1` / `IC103` input; `Pin 3`: `C110.1`; `Pins 2, 4`: `VSS` |
| **`IC103`** | `VHE` | SOT-23-5 Gate Buffer | `Pin 1 (EN)`: `U1.11` (`PH10`) & `R108.1`; `Pin 3 (GND)`: `VSS` (`R108.2`, `C110.2`, `R105.2`); `Output`: via `R10` to `U1.12` (`PH0-OSC_IN`) |
| **`IC102`** | `C79J` (`SN74LVC1G79`) | SOT-23-5 D Flip-Flop | `Pin 1 (D)`: `U1.40` (`PE9`); `Pin 2 (CLK)`: `R107.2` / `R105.1` ($32.000\text{ MHz}$); `Pin 3 (GND)`: `U1.49` (`VSS`, `C105.2`); `Pin 4 (Q)`: `D603.3`, `R609.1`, `R610.1`; `Pin 5 (VCC)`: `U1.50` (`+3.3V`, `C105.1`, `C115.1`) |
| **`D102`** | Dual Series Diode | SOT-23-3 | `Pin 1`: `U1.29` (`PA4`), `R717.2`, `C711.1`; `Pin 2`: `U1.30` (`PA5`), `C714.1`, `R720.2`; `Pin 3`: `U1.75` (`VDD_3V3` $+3.3\text{V}$ clamp rail) |
| **`R111`, `R112`, `C121`** | Divider | 0603 SMD | `R111.2`: `Q703.3` (`NET_R706_1`); `R111.1` / `R112.1` / `C121.1`: `U1.16` (`PC1`); `R112.2` / `C121.2`: `VSS` |

### 5.2 600-Series RFID TX Driver & Closed-Loop High-Voltage Tank Tuner Netlist
| RefDes | Marking / Value | Package / Type | Verified Pin Connections & Function |
| :--- | :--- | :--- | :--- |
| **`IC601`** | Gate Driver A | SOT-23-5 | `Pins 1, 2 (IN)`: `R609.2`, `C617.1`, `D603.1`; `Pin 3 (GND)`: `C617.2`, `C618.2`, `VSS`; `Pin 4 (OUT)`: `R611.1` ($\rightarrow$ `Q606.3`); `Pin 5 (VCC)`: `R615.1` ($+6\text{V}$ via $100\ \Omega$), `C622.2`, `C623.1` |
| **`IC602`** | Gate Driver B | SOT-23-5 | `Pins 1, 2 (IN)`: `R610.2`, `C618.1`, `D603.2`; `Pin 3 (GND)`: `C618.2`, `VSS`; `Pin 4 (OUT)`: `R612.1` ($\rightarrow$ `Q606.1`); `Pin 5 (VCC)`: `R615.1`, `C623.1` |
| **`D603`** | Dual Diode | SOT-23-3 | `Pin 1`: `C617.1` / `R609.2`; `Pin 2`: `C618.1` / `R610.2`; `Pin 3`: `IC102.4` (`Q`), `R609.1`, `R610.1` |
| **`Q606`** | Dual/Comp FET | SOT-23-6 | `Pin 1`: `R612.2`; `Pin 2`: `D606.1/2` & `D605.1(K)`; `Pin 3`: `R611.2`; `Pins 4, 6`: **`TPA1`** (`A601`) & `C619.2` |
| **`D606`, `D605`** | Supply Diodes | SOT-23-3 / SOD-323 | `D606.1/2` & `D605.1(K)`: `Q606.2`; `D606.3`: `C724.1`; `D605.2(A)`: `C625.2`, `R616.2` ($0.47\ \Omega$), `C626.2` ($220\ \mu\text{F}\ 16\text{V}$) |
| **`TPA1`, `TPA0`** | Antenna Pads | Solder Pads | **`TPA1`**: `Q606.4/6`, `C619.2` (`C619.1` = `VSS`); **`TPA0`**: Antenna Coil return ($<1\ \Omega$ DC through coil) |
| **`Q603`** | `FQD6N40C` | DPAK-3 (400V N-FET) | `Pin 1 (Gate)`: `R604.1` ($\leftarrow$ `C604.2`, `Q601.3`, `Q602.3`); `Pin 2 (Source)`: `C607.1`, `C605.1`, `Q604.2`; `Pin 3 (TAB/Drain)`: `C607.2`, `C609.2`, `C610.2`, `C612.2`, `C8613.2`, `D604.3` |
| **`D604`** | `KAE` (Dual Diode) | SOT-23-3 | `Pin 3 (COM)`: `Q603.3 (TAB)` & tank capacitor bank `C607.2`/`C609.2`/`C610.2`/`C612.2`/`C8613.2`; `Pin 1`: `R601.1` ($\rightarrow$ `C601.2`/`Q601.2`); `Pin 2 (Rectified Envelope)`: `C620.2` (to `GND_8`), `R613.1` ($\rightarrow$ `PA7`), `C706.2` ($\rightarrow$ `IC701.3`) |
| **`R613`, `R614`, `C621`** | `PA7` Divider | 0402 / 0603 SMD | `R613.1`: `D604.2`; `R613.2` / `R614.2` / `C621.2`: `U1.32` (`PA7`); `R614.1` / `C621.1`: `GND_8` (`VSS`) |
| **`C602`, `R602`, `D601`, `Q601`** | `PB14` UP Pump | 0603 / SOT-23-3 | `C602.1`: `U1.53` (`PB14`); `C602.2`: `R602.2`, `Q601.1`, `D601.3`; `Q601.2`: `D601.1`, `R602.1`, `C601.2`, `R601.2`; `Q601.3`: `C604.2`, `R604.2`, `Q602.3` |
| **`C603`, `R603`, `D602`, `Q602`** | `PB13` DOWN Pump | 0603 / SOT-23-3 | `C603.1`: `U1.52` (`PB13`); `C603.2`: `R603.2`, `D602.1`, `Q602.1`; `D602.3`: `GND_8` (`VSS`); `Q602.2`: `C601.1`, `R617.1`; `Q602.3`: `Q601.3`, `C604.2`, `R604.2`; `R603.1`: `C604.1` |
| **`Q607`, `R605`, `R617`, `R618`** | `PB2` Bias Switch | SOT-23-3 (`6CW`) | `R605.1`: `U1.37` (`PB2`); `R605.2`: `Q607.1 (Base)`; `Q607.2`: `C611.1`, `R606.2`, `C606.1`, `C604.1`; `Q607.3`: `R618.1`, `R617.2` (`R617.1` = `C601.1`); `R618.2`: `C605.2`, `C606.2`, `Q604.1` |
| **`Q604`, `R606`, `C605`–`C613`** | Tank Switch Network | SOT-23-3 / 1206 / 0805 | `Q604.1`: `C605.2`, `C606.2`, `R618.2`; `Q604.2`: `C605.1`, `C607.1`, `Q603.2`; `Q604.3`: `R606.1`, `C611.2`, `C609.1`, `C610.1` |

### 5.3 700-Series RFID RX Amplifier, Bandpass Filter & Slicer Netlist
| RefDes | Marking / Value | Package / Type | Verified Pin Connections & Function |
| :--- | :--- | :--- | :--- |
| **`Q702`, `Q703`, `R701`–`R708`** | `PB11` Power & `V_REF` | SOT-23-3 / 0402 | `R701.1`: `U1.48` (`PB11`); `R701.2`: `Q702.1`; `Q702.3`: `R702.2`, `Q703.1`; `Q703.3`: `R703.1`, `R706.1`, `C702.2`, `R111.2`, `R721.2`; `Q703.2`: `R702.1`, `R708.2`, `C731.1`, **`V_REF`** (`IC701.5,10,12` & `IC702.3,5,10,12`) |
| **`D701`, `C706`, `R704`, `R705`** | RX Front-End Clamp | SOT-23-3 / 0603 | `C706.2`: `D604.2` / `C620.2`; `C706.1`: `R704.2`; `R704.1`: `D701.3`, `IC701.3`, `R705.1` (`R705.2` = `V_REF`); `D701.1`: `C619.1` (`VSS`), `R897.1`; `D701.2`: `IC701.4` / `IC702.4` (`VCC`) |
| **`IC701`** | `TL974I` (Quad Op-Amp 1) | TSSOP-14 | • **Op-Amp 1 (Follower):** `Pin 3 (1IN+)` = `D701.3`; `Pin 1 (OUT1)` = `Pin 2 (1IN-)`, `C712.2`, `R711.2`, `R722.2`<br>• **Op-Amp 2 ($\rightarrow$ `PA5`):** `Pin 5 (2IN+)` = `V_REF`; `Pin 6 (2IN-)` = `R718.1` ($\leftarrow$ `C712.1`), `C713.1`, `R719.2`; `Pin 7 (OUT2)` = `C713.2`, `R719.1`, `R720.1` ($\rightarrow$ `C714.1` / `D102.2` / **`PA5`**)<br>• **Op-Amp 4:** `Pin 12 (4IN+)` = `V_REF`; `Pin 13 (4IN-)` = `C708.2`, `R713.2`; `Pin 14 (OUT4)` = `C707.2`, `R713.1`, `R714.2`<br>• **Op-Amp 3 ($\rightarrow$ `PA4`):** `Pin 10 (3IN+)` = `V_REF`; `Pin 9 (3IN-)` = `C710.1`, `R716.2`; `Pin 8 (OUT3)` = `R716.1`, `C709.1`, `R717.1` ($\rightarrow$ `C711.1` / `D102.1` / **`PA4`**)<br>• **Power:** `Pin 4 (VCC)` = `D701.2`, `C705.1`, `C701.2`; `Pin 11 (GND)` = `VSS` |
| **`IC702`** | `TL974I` (Quad Op-Amp 2) | TSSOP-14 | • **Op-Amp 1:** `Pin 3 (1IN+)` = `V_REF`; `Pin 2 (1IN-)` = `C717.1`, `R724.1`; `Pin 1 (OUT1)` = `R724.2`, `C718.1`, `R725.1`<br>• **Op-Amp 2:** `Pin 5 (2IN+)` = `V_REF`; `Pin 6 (2IN-)` = `R727.1`, `C719.1`; `Pin 7 (OUT2)` = `R727.2`, `C720.1`, `R728.2`<br>• **Op-Amp 3:** `Pin 10 (3IN+)` = `V_REF`; `Pin 9 (3IN-)` = `R730.2`, `C721.2`; `Pin 8 (OUT3)` = `R730.1`, `C722.2`, `Q704.1`<br>• **Op-Amp 4 ($\rightarrow$ `PA6`):** `Pin 12 (4IN+)` = `V_REF`; `Pin 13 (4IN-)` = `R738.2`, `C730.2`; `Pin 14 (OUT4)` = `R739.2`, `C730.1` |
| **`Q704`, `IC703`, `IC704`** | Slicer / Filter Stages | SOT-23-3 / SOT-23-5 | `Q704.1`: `C722.2`, `R730.1`, `IC702.8`; `Q704.2`: `R731.1`, `C723.2`; `Q704.3`: `R732.2`, `C724.2`; `IC703.1`: `R734.2`; `IC703.3`: `R734.1`, `C726.1`; `IC703.5`: `C725.1`, `R733.1`, `R735.2`; `IC703.4`: `R735.1`, `IC704.1`; `IC704.4`: `R736.2` ($\rightarrow$ `C727.1` / `C728.1` $\rightarrow$ `R737` $\rightarrow$ `C729.2`); `IC704.5`: `C716.2`, `R721.1` |



