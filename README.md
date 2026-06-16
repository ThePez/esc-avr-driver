# ESC AVR Driver

Sensorless brushless DC (BLDC) motor ESC firmware for the ATmega328PB. Implements 6-step trapezoidal commutation with back-EMF zero crossing detection and a standard RC PWM throttle input. Currently verified in LED testing mode only — hardware PCB design is in progress for full motor testing.

## Features

- Sensorless BLDC commutation via analog comparator back-EMF zero crossing detection
- Standard RC PWM throttle input (1000–2000 µs, 50 Hz)
- Open-loop startup ramp, then automatic handoff to closed-loop back-EMF commutation
- Standard ESC arming sequence — throttle must be at or below 5% before the motor will run
- Failsafe — all outputs shut down within 100 ms of losing the PWM input signal
- 250 ms watchdog timer reset on every iteration of every loop
- DRV8305 SPI driver library included (untested — not yet integrated into the control loop, pending PCB hardware)
- `TESTING` build mode — drives LEDs instead of motor phases for bench verification without hardware

## Hardware

PCB design is in progress. The target components are listed below; none have been tested on real hardware yet.

| Component | Part |
|---|---|
| MCU | ATmega328PB @ 16 MHz |
| Gate driver IC | Texas Instruments DRV8305 (driver written, not yet tested) |
| Half-bridge driver | Infineon IR2104 |
| Power MOSFETs | Infineon IPP015N04NF2S (N-channel, 40 V, 1.5 mΩ) |

## Pin Mapping

| Pin | Function |
|---|---|
| PD0 (OC3A) | Phase A high side PWM output |
| PD1 (OC4A) | Phase C high side PWM output |
| PD2 (OC3B) | Phase B high side PWM output |
| PC3 | Phase A low side GPIO |
| PC4 | Phase B low side GPIO |
| PC5 | Phase C low side GPIO |
| PC0 (ADC0) | Phase A back-EMF sense |
| PC1 (ADC1) | Phase B back-EMF sense |
| PC2 (ADC2) | Phase C back-EMF sense |
| PD6 (AIN0) | Motor neutral / star point (comparator positive input) |
| PB0 (ICP1) | RC PWM throttle input |
| PB2 | DRV8305 chip-select (active low) |
| PB3 (MOSI) | SPI data out / OC2A LED PWM in TESTING mode |
| PB4 (MISO) | SPI data in |
| PB5 (SCK) | SPI clock |

For the full pin mapping spreadsheet see `Pin Mappings.xlsx`.

## Project Structure

```
src/
  main.c              — Application entry point, commutation state machine, ISRs
lib/
  drv8305/
    drv8305.h/.c      — DRV8305 SPI driver (init, read, write)
  timers/
    timers.h/.c       — All timer peripherals: system tick, RC input capture, gate PWM
  serialio/
    serialio.h/.c     — Interrupt-driven UART with ring buffers, printf/scanf redirect
scripts/
  copy_compile_commands.py  — PlatformIO post-build script to copy compile_commands.json
platformio.ini        — Build and upload configuration
```

## Building and Flashing

The project uses [PlatformIO](https://platformio.org/).

**Build:**
```sh
pio run
```

**Flash** (Arduino bootloader via `/dev/ttyUSB0`):
```sh
pio run --target upload
```

**Build flags** (set in `platformio.ini`):
- `-std=gnu11`
- `-Wall`
- `printf_min` — lightweight printf that omits float formatting to save flash

### TESTING Mode

Uncomment `#define TESTING 1` at the top of [src/main.c](src/main.c) to build in testing mode. In this mode:

- PC0–PC5 are configured as LED outputs instead of gate drive GPIOs
- The commutation sequence is visible as a rotating LED pattern
- The startup interval is much longer (800 ms down to 8 ms) for visual verification
- Serial output is enabled — connect a terminal at **19200 8N1** to observe state
- OC2A (PB3) provides a PWM output whose brightness tracks throttle input

Remove `#define TESTING 1` for real motor operation.

## Architecture

### Control Flow

```
Power on
  │
  ├─ Peripheral init (timers, SPI, serial, GPIO)
  │
  ├─ Arming loop — wait for valid PWM signal with throttle ≤ 5%
  │
  ├─ Open-loop startup ramp
  │    Steps through the 6 commutation phases at a fixed interval that
  │    decreases from STARTUP_INTERVAL_START_MS to STARTUP_INTERVAL_END_MS.
  │
  ├─ comparatorInit() — analog comparator enabled for back-EMF detection
  │
  └─ Main loop
       ├─ Failsafe check — shutdown() if PWM signal lost > 100 ms
       ├─ Throttle update — set_gate_duty(getThrottle()) on each fresh pulse
       └─ Commutation advance — run_phase() on each back-EMF zero crossing (emf flag)
```

### Commutation

Six-step trapezoidal commutation. One high side is driven with PWM, one low side is pulled low, and the third phase floats for back-EMF sensing. The high side pins are connected to or disconnected from their timer output by setting/clearing the corresponding DDR bit.

| Step | High | Low | Floating (back-EMF) |
|---|---|---|---|
| 0 | A | B | C |
| 1 | A | C | B |
| 2 | B | C | A |
| 3 | B | A | C |
| 4 | C | A | B |
| 5 | C | B | A |

### Back-EMF Detection

The analog comparator compares the floating phase voltage (via the ADC mux on the negative input, controlled by `ACME`) against the motor neutral point on AIN0 (PD6). The interrupt fires on every output toggle (both rising and falling zero crossings). The ISR sets `emf = 1`; the main loop calls `run_phase()` to advance the step and clears the flag.

### Timers

| Timer | Mode | Clock | Purpose |
|---|---|---|---|
| Timer0 | CTC | 16 MHz / 64 = 250 kHz | 1 ms system tick (`getSysTick()`) |
| Timer1 | Input capture | 16 MHz / 8 = 2 MHz (0.5 µs/tick) | RC PWM measurement on ICP1 (PB0) |
| Timer2 | Fast PWM | 16 MHz / 1 = 16 MHz | LED brightness in TESTING mode (OC2A/PB3) |
| Timer3 | Phase correct PWM, TOP=400 | 16 MHz / 1 = 16 MHz | Gate PWM 20 kHz, phases A (OC3A) and B (OC3B) |
| Timer4 | Phase correct PWM, TOP=400 | 16 MHz / 1 = 16 MHz | Gate PWM 20 kHz, phase C (OC4A) |

Phase correct PWM is used for gate drive to produce a symmetrical waveform and reduce motor winding current ripple versus fast PWM. Timer3 and Timer4 are started in the same instruction to minimise phase offset between the three channels.

### RC PWM Input

Timer1 input capture measures both edges of the incoming RC PWM signal on PB0. Rising edge timestamps record the period; falling edge timestamps record the pulse width. `getThrottle()` maps the 1000–2000 µs pulse range to 0–1000 per-mille. `pwmSignalLost()` returns true if no valid pulse has been received within 100 ms (approximately 5 missed frames at 50 Hz).

### DRV8305 SPI Interface

The SPI driver ([lib/drv8305/](lib/drv8305/)) is written and compiles, but has not yet been tested on hardware and is not integrated into the motor control loop. `drv8305SpiInit()` is called at startup to configure the SPI peripheral, but no register reads or writes are performed beyond that.

The SPI interface operates at SPI mode 1 (CPOL=0, CPHA=1), MSB first, fosc/8 = 2 MHz, with chip-select on PB2 (active low). Frames are 16 bits wide: bit 15 is the read/write flag, bits 14:11 are the 4-bit register address, and bits 10:0 are the 11-bit data payload. A read requires two transactions — the first sends the address, the second clocks out the response.

#### Planned integration and simplifications

The DRV8305 is a fully integrated three-phase gate driver. Integrating it properly would allow several simplifications over the current discrete approach:

**Replace the three IR2104 half-bridge drivers with one IC.** Currently each phase needs its own IR2104 plus a bootstrap capacitor for the high-side drive. The DRV8305 integrates all six gate drivers (3 high-side, 3 low-side) with internal bootstrap diodes, reducing component count and PCB area significantly.

**Reduce the number of AVR PWM channels needed.** The current design uses three hardware PWM outputs (Timer3 OC3A/OC3B and Timer4 OC4A) — one per high-side switch. With the DRV8305, all three INH inputs could be tied to a single PWM signal, and commutation would be controlled solely by toggling the three INL (low-side enable) GPIO lines. Only one timer output would be needed for speed control, freeing Timer4 entirely and eliminating the DDR bit trick currently used to connect and disconnect the OC3x/OC4x pins from their phases.

**Automatic dead-time insertion.** The DRV8305 enforces a programmable dead-time between a high-side and low-side gate signal for the same phase, preventing shoot-through without any software timing constraints. The current discrete approach relies on the IR2104's fixed internal dead-time.

**Built-in protection.** The DRV8305 provides overcurrent protection via internal current sense amplifiers, overtemperature shutdown, and undervoltage lockout — features that would otherwise require additional discrete sensing and comparator circuitry.

**SPI-configurable gate drive current.** Gate drive source and sink currents are programmable via SPI registers, allowing the switching speed (and therefore switching losses vs. EMI trade-off) to be tuned in software rather than by changing gate resistor values.

### Serial I/O (TESTING mode)

Interrupt-driven UART at 19200 baud using separate 100-byte output and 32-byte input ring buffers. `stdout`/`stdin` are redirected to the UART stream so `printf_P` / `scanf` work transparently. Newlines are expanded to `\r\n` on output and `\r` is converted to `\n` on input.

## License

Apache-2.0 — see source file headers.
