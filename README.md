# OpenSureFeed — Open-Source Firmware & Hardware Docs for the SureFeed Microchip Pet Feeder

Custom, fully open-source STM32 firmware and complete PCB reverse-engineering documentation for the **SureFeed Microchip Pet Feeder** (`01523-FG_04` / `MPF2 SureFlap (c) 2019`, `STM32L071VZT6`).

> [!WARNING]
> Unlocking a stock feeder via SWD requires disabling STM32 Read-Out Protection (RDP Level 1 $\rightarrow$ Level 0), which **permanently mass-erases the factory SureFlap firmware**. You cannot back up or restore the stock binary afterward. Proceed at your own risk.

---

## Features

- **134.2 kHz ISO 11784 / 11785 FDX-B Reader:**
  - Runs the MCU synchronously off the onboard `32.000 MHz` crystal (`HSE_BYPASS` via `IC103`/`PH0`) during RFID windows so `IC102` re-clocking stays phase-locked with zero `12.8 kHz` beat interference.
  - Closed-loop tank amplitude tuning (`PB14` charge-UP / `PB13` discharge-DOWN servo against `PA7` envelope feedback).
  - Full 128-bit FDX-B differential bi-phase decoder with 13-bit framing check and 16-bit CRC-CCITT (`0x8408`) verification.
- **Persistent Multi-Pet Memory:** Learns and stores up to **32** microchip IDs in STM32L0 internal Data EEPROM (`0x08080000`).
- **Power-Gated Arch Scanning:** Keeps both the RFID TX carrier and `700`-series `TL974I` op-amp RX chain completely powered down (`PB11 = LOW`) until the pulsed 940 nm IR break-beam detects a pet in the hoop.
- **5-Second Anti-Hijack Guard & Gentle Slow Close *(Custom Enhancement)*:**
  - While the lid is open for an authorized pet and the IR beam stays broken, firmware re-wakes RFID every **`5.0 s`** for up to **`1.5 s`** to verify the pet currently in the arch is still authorized.
  - If **only** an unauthorized microchip is detected across the window, the lid closes gently at **`1/3` speed (`33.3%` PWM duty, `1 ms ON / 2 ms OFF` @ `333 Hz`)** without IR auto-reopen bounce so the intruding pet can back out safely without getting pinched or trapped.
  - Uses ON-phase-synchronized `PC4` rail-sag monitoring to cut power immediately if a physical obstacle or mechanical end-stop is hit.
- **Full Stock Control & Mode Parity:**
  - **Add Pet (`SW1`):** Opens lid, flashes green (`1 Hz`), learns tag, turns solid green, then closes once the pet steps away.
  - **Factory Reset (`SW1` 10 s hold):** Alternates red/green LED, cycles the lid open & closed, and erases all learned pets from Data EEPROM.
  - **4-Stage Training Mode (`SW3`):** Cycles gradually reduced close travel with `1×–4×` orange LED blinks.
  - **Manual Open / Close (`SW2`):** Manually opens (pausing auto mode) or closes the lid.
  - **Close Delay Slider (`I / II / III`):** `I = 3.5 s`, `II = 6.0 s`, `III = 10.0 s`.

---

## Hardware & Protocol Reference

See **[`HARDWARE.md`](HARDWARE.md)** for the complete reverse-engineering reference, including:
- Full `STM32L071VZT6` LQFP100 pinout table
- SWD test pad connections & factory RDP / watchdog option bytes
- Motor H-bridge & supply rail-sag stall detection measurements
- Pulsed IR break-beam schematic & matched-filter edge timing
- Complete `100` / `600` / `700`-series RFID TX, capacitive charge-pump tuner, and `TL974I` RX schematics, BOM, and verified netlist tables
- Feature-by-feature comparison against the official SureFeed user manual

---

## Quick Install (Pre-Built Release Binaries)

You do **not** need to clone or build the project if you only want to flash the ready-made firmware. Download the pre-built files directly from the [`release/`](release/) folder:

| File | Format | Use With |
| :--- | :--- | :--- |
| [`release/surefeed-firmware.bin`](release/surefeed-firmware.bin) | Raw binary (`0x08000000`) | ST-Link (`st-flash`), STM32CubeProgrammer |
| [`release/surefeed-firmware.hex`](release/surefeed-firmware.hex) | Intel HEX | STM32CubeProgrammer, J-Link |
| [`release/surefeed-firmware.elf`](release/surefeed-firmware.elf) | ELF image | Black Magic Probe (`gdb`), OpenOCD |
| [`release/SHA256SUMS.txt`](release/SHA256SUMS.txt) | SHA-256 checksums | Integrity verification |

### Step 1: Connect an SWD Programmer
Wire an SWD debug probe (ST-Link v2/v3, Black Magic Probe, or J-Link) to the STM32 SWD pads on the mainboard (details in [`HARDWARE.md`](HARDWARE.md#2-debug--swd-programming-header)):
- **SWDIO:** `PA13` (Pin 72)
- **SWCLK:** `PA14` (Pin 76)
- **NRST:** `NRST` (Pin 14)
- **GND:** `VSS` (Pin 74)
- **VDD:** `VDD` (Pin 75, `+3.3V` target voltage reference)

Power the feeder from its batteries (`~6V`) or bench supply.

### Step 2: One-Time Stock Silicon Unlock (New Boards Only)
Factory boards ship with **RDP Level 1** and a **hardware-enforced Independent Watchdog (`WDG_SW = 0`)** that resets the chip every `~20 ms` unless cleared:
1. Set option byte `0x1FF80000` (RDP) from `0x00BB` $\rightarrow$ **`0x00AA`** (Level 0 — triggers mass erase).
2. Set option byte `0x1FF80004` (`USER`) so `WDG_SW = 1` (**`0x8070`**).

*(Easiest via **STM32CubeProgrammer** GUI under **Option Bytes**: set `RDP = AA`, check `WDG_SW`, and click **Apply**.)*

### Step 3: Flash the Release Firmware

- **Using `st-flash` (ST-Link):**
  ```bash
  st-flash --reset write release/surefeed-firmware.bin 0x08000000
  ```

- **Using OpenOCD (ST-Link / CMSIS-DAP):**
  ```bash
  openocd -f interface/stlink.cfg -f target/stm32l0.cfg -c "program release/surefeed-firmware.elf verify reset exit"
  ```

- **Using STM32CubeProgrammer:**
  Open `release/surefeed-firmware.hex` (or `release/surefeed-firmware.bin` at address `0x08000000`), check **Verify programming** and **Run after programming**, and click **Start Programming**.

- **Using Black Magic Probe (`arm-none-eabi-gdb`):**
  ```bash
  arm-none-eabi-gdb -batch release/surefeed-firmware.elf \
    -ex "target extended-remote <BMP_PORT_OR_IP:2345>" \
    -ex "monitor swdp_scan" \
    -ex "attach 1" \
    -ex "load" \
    -ex "compare-sections" \
    -ex "kill"
  ```

---

## Building From Source (PlatformIO)

1. Install [PlatformIO Core](https://platformio.org/install/cli) (or the PlatformIO IDE extension in VS Code).
2. Clone the repository and build:
   ```bash
   git clone https://github.com/braunixy/SureFeed.git
   cd SureFeed

   # Build optimized release firmware (.pio/build/release/firmware.bin & .elf)
   pio run -e release

   # Or build debug firmware with symbols (-g3 -Og)
   pio run -e surefeed
   ```
3. Upload to the feeder:
   - **Via ST-Link (`release` environment):**
     ```bash
     pio run -e release -t upload
     ```
   - **Via Black Magic Probe:** Update `upload_port` / `debug_port` in [`platformio.ini`](platformio.ini) to your probe's serial port or Wi-Fi IP (`host:port`), then run:
     ```bash
     pio run -e surefeed -t upload
     ```

---

## Controls & LED Reference

| Input | Action |
| :--- | :--- |
| **Add Pet (`SW1`) — Short Press** | Opens lid & enters Add Pet mode (green `1 Hz` blink) until a microchip is read (or pressed again to cancel). |
| **Add Pet (`SW1`) — Hold `10 s`** | Factory reset: wipes all learned microchips, alternates red/green LED, and cycles lid open/closed. |
| **Open / Close (`SW2`)** | Manually opens the lid (pauses auto mode) or closes it (resumes auto mode). |
| **Training (`SW3`)** | Cycles through Training Stages `1 → 2 → 3 → 4 → Exit (0)` (`1×–4×` orange blinks). |
| **Bottom Slider (`I / II / III`)** | Sets auto-close delay after pet leaves IR arch: `I = 3.5 s`, `II = 6.0 s`, `III = 10.0 s`. |
| **Status LED** | **Off:** Idle · **Green 1 Hz:** Add Pet · **Solid Green:** Pet recognized / eating · **Orange `1×–4×`:** Training stage · **Fast Red:** Wrong pet detected / closing at `1/3` speed · **Alt Red/Green:** Factory reset |
