# Project Notes — Raspberry Pi ↔ STM32 (Blue Pill) USB CDC

## Hardware
- Board: STM32F103C8T6 (“Blue Pill”)
- LED: External on PB6 (active‑high). Wiring:
  - PB6 → resistor (220–1kΩ) → LED anode → LED cathode → GND
- USB Pull‑up: R10 on D+ (PA12) should be ~1.5 kΩ (“152”). A 10 kΩ (“103”) often causes flaky enumeration (host errors -32/-71).
- Power: Bus‑powered over USB. VBUS sensing disabled in firmware.

## Firmware Overview
- Tooling: STM32CubeIDE, HAL drivers.
- Clock:
  - Current: HSE 8 MHz → PLL ×6 → SYSCLK 48 MHz; USB ← 48 MHz (valid)
  - Alternative (common): 72 MHz SYSCLK with USB = PLL/1.5 (48 MHz)
- USB CDC:
  - Non‑blocking TX queue with robust completion handling
    - Supports both `CDC_TransmitCplt_FS` and `USBD_CDC_TransmitCplt`
    - Auto‑recovers if `TxState` goes idle but no callback fires
  - `CDC_Receive_FS` copies data into `g_usb_rx_buffer` and raises `g_new_usb_data`
- Command Parser (main loop):
  - Commands over USB: `A`, `B`, `C`
    - A → Blink PB6 for 5 s at 100 ms interval
    - B → Blink PB6 for 10 s at 500 ms interval
    - C → Blink PB6 for 20 s at 1000 ms interval
  - Device replies: `ACK X OK\r\n` (X ∈ {A,B,C})
  - Non‑blocking blink executor driven by `HAL_GetTick()`
- GPIO:
  - PB6 configured as push‑pull output (LOW = OFF, HIGH = ON)

## Known Pitfalls
- Enumeration failures (`device descriptor read/64, error -32`, `device not accepting address, error -71`):
  - Use a short, known‑good USB 2.0 cable and a stable port
  - A powered USB 2.0 hub can improve host compatibility
  - Verify D+ pull‑up is ~1.5 kΩ (R10)
- If PB6 doesn’t blink at startup:
  - Ensure `__HAL_RCC_GPIOB_CLK_ENABLE()` before `HAL_GPIO_Init`
  - Confirm no peripheral reconfigures PB6 (e.g., I2C1 on PB6/PB7)
  - A quick pre‑USB toggle test helps verify wiring

## Build & Flash
- Build in CubeIDE (Debug/Release).
- Flash with ST‑LINK.
- Press RESET once after flashing for clean USB enumeration.

## Raspberry Pi Testing
- Typical device node: `/dev/ttyACM0` (VID:PID 0483:5740).
- Example:
  ```bash
  python3 pi_controller.py
  # Enter A, B, or C; expect "ACK X OK"
  ```
- Live kernel log:
  ```bash
  sudo dmesg -w
  ```
