# Project 2 — bxCAN Two-Node Communication

Bare-metal CAN bus driver for the STM32F103C8T6, register-level C, no HAL, no CMSIS, no stdlib. Two Blue Pill nodes communicating at 500 kbps through SN65HVD230 transceivers over a common bus.

Project 2 of 3 in a portfolio targeting automotive software engineering roles.

## Architecture

```
┌──────────────┐   UART    ┌──────────┐   UART    ┌──────────────┐
│  Terminal A  │ ◀──────▶  │  Node A  │  ◀─────▶  │  Terminal B  │
└──────────────┘           │  STM32F1 │           └──────────────┘
                           └────┬─────┘
                                │ PA11 / PA12
                                ▼
                         ┌─────────────┐
                         │ SN65HVD230  │
                         └─┬─────────┬─┘
                         CANH       CANL
                           │         │
                       ════╪═════════╪════  CAN bus, 500 kbps
                           │         │
                         CANH       CANL
                         ┌─┴─────────┴─┐
                         │ SN65HVD230  │
                         └──────┬──────┘
                                │ PA11 / PA12
                                ▼
                           ┌──────────┐
                           │  Node B  │
                           │  STM32F1 │
                           └──────────┘
```

Both nodes run the same firmware compiled with a `NODE_A` or `NODE_B` flag. Both can transmit and receive; node A has an additional button-triggered TX on PA0.

## Hardware

| Component | Purpose |
|---|---|
| 2× STM32F103C8T6 Blue Pill | MCU nodes |
| 2× SN65HVD230 breakout | 3.3V CAN transceiver, Rs tied to GND (high-speed mode) |
| 4× 220Ω resistors | two pairs in parallel → 110Ω termination at each bus end |
| 1× CP2102 USB-UART adapter | host serial console at 115200-8-N-1 |
| Push-button on PA0 | TX trigger (node A) |
| LED on PC13 (built-in) | 1 Hz heartbeat |

### Wiring (per node)

```
Blue Pill            SN65HVD230
─────────            ──────────
PA12 (CAN_TX)   ───▶ TXD
PA11 (CAN_RX)  ◀──── RXD
3.3V            ───▶ VCC
GND             ───▶ GND, Rs

                     CANH ──┐
                     CANL ──┤  to bus
                            │
                  110Ω across CANH↔CANL at each endpoint
```

Common GND between both nodes is required. PA11 needs an explicit pull-up via `BSRR` before CAN init — input-float behaviour on this pin causes spurious bus activity otherwise.

## Build & flash

```
make NODE=A          # builds for node A
make NODE=B          # builds for node B
make flash NODE=A    # OpenOCD flash via ST-Link
```

Compile flags: `-std=c89 -Wpedantic -Werror -DDEBUG -fno-exceptions -fno-rtti -nostdlib -ffreestanding`.

The `NODE` flag enables conditional sections (button-triggered TX on A, supplementary RX print on B). The RX/TX paths are otherwise identical between nodes.

## UART command interface

Connect at 115200-8-N-1 to PA9 (TX) / PA10 (RX). Type a command and press Enter.

```
TX <hex_id> <dlc> [hex_byte ...]
```

Examples:

```
TX 0x100 2 CA FE        transmit ID 0x100 with two data bytes
TX 0x123 0              transmit zero-length frame
TX 0x7FF 8 11 22 33 44 55 66 77 88
```

Constraints: `id ≤ 0x7FF` (standard 11-bit), `dlc ≤ 8`, hex case-insensitive, `0x` prefix optional. Invalid commands print `ERR: bad command` and are dropped.

## Bit timing

Target: 500 kbps with a sample point near the automotive recommendation of 87.5%.

```
HSE          8 MHz
PLL ×9       72 MHz SYSCLK
APB1 ÷2      36 MHz   ← CAN peripheral clock

prescaler (BRP)   4   →  tq = 4 / 36 MHz = 111.11 ns
TS1               15
TS2               2
SJW               1

bit time  = (1 + TS1 + TS2) × tq
          = 18 × 111.11 ns
          = 2.000 µs   →   500 kbps   ✓

sample pt = (1 + TS1) / (1 + TS1 + TS2)
          = 16 / 18
          = 88.9%
```

`BTR = 0x001E0003`. The `CAN_BTR_*` macros take the logical value (e.g. `CAN_BTR_TS1(15)`) and subtract 1 for the field encoding, per RM0008 §24.9.2.

## Filter configuration

Two filter banks are configured during `can1_init`:

**Bank 0 — list mode, 32-bit scale.** Two exact-match slots: accepts `0x100` or `0x200`.

```
FR1 = 0x100 << 21 = 0x20000000
FR2 = 0x200 << 21 = 0x40000000
```

**Bank 1 — mask mode, 32-bit scale.** Pattern + mask: top 3 STID bits must equal `0b001`, lower 8 bits are wildcards.

```
FR1 = 0x100 << 21 = 0x20000000   ID pattern
FR2 = 0x700 << 21 = 0xE0000000   mask: 1 = must-match, 0 = wildcard

→ accepts 0x100 — 0x1FF
```

The `<< 21` mirrors the position of the 11-bit STID inside `RIxR`, which is the layout the filter logic compares against (RM0008 §24.7.4). Both banks route to FIFO 0; the RX ISR drains one frame per interrupt and re-fires while `FMP0 ≠ 0`.

### Match table

| TX ID | Bank 0 list | Bank 1 mask | Received |
|---|---|---|---|
| `0x100` | match | match | ✓ |
| `0x150` | — | match | ✓ |
| `0x200` | match | — | ✓ |
| `0x250` | — | — | ✗ |
| `0x300` | — | — | ✗ |

## Error handling

`CAN_SCE_IRQHandler` is enabled for bus warning (`EWGIE`), error passive (`EPVIE`), bus-off (`BOFIE`), and the master error flag (`ERRIE`). The handler snapshots `ESR` once and prints:

```
ERR lec=<code> tec=<count> rec=<count> [BOFF] [EPVF]
```

| Field | Meaning |
|---|---|
| `lec` | last error code, `ESR[6:4]` (RM0008 §24.9.7) |
| `tec` | transmit error counter, `ESR[23:16]` |
| `rec` | receive error counter, `ESR[31:24]` |
| `BOFF` | TEC > 255, bus-off state |
| `EPVF` | error-passive (TEC or REC > 127) |

`NART` (no automatic retransmission) is enabled. A failed TX surfaces immediately rather than retrying silently, which keeps the error counters meaningful in a controlled bench environment. A production driver would tune retry behaviour per use case.

`ABOM` (automatic bus-off management) is disabled. Bus-off recovery requires software to re-enter and leave initialization mode. The bus-off event is logged via `BOFF`; recovery policy is intentionally left to the application layer.

FIFO overrun (`FOVR0`, `RF0R` bit 4) is checked at the top of the RX ISR and logged as `FIFO0 OVR`.

## Clock security

`RCC_CR.CSSON` is enabled. On HSE crystal failure the clock security system automatically switches the system clock to HSI and fires `NMI_Handler`, which logs `HSE Failure` and clears `CSSF`. This is a minimal fault response — a safety-critical system would enter a defined safe state and re-initialize timing-dependent peripherals.

## Design decisions

**Why bxCAN, not an external MCP2515?** The Blue Pill has bxCAN on-chip. An external SPI controller would add latency, an IC, and would not exercise the part of the STM32 register interface that matters for this portfolio. Working register-level with bxCAN is the point.

**Why NART?** Automatic retries hide failure modes in a deterministic bench. With NART, every TX either succeeds or fails visibly. Production policy differs.

**Why a ring buffer for UART output?** Multiple ISRs writing directly to `USART_DR` while another is mid-string produces character-interleaved garbage at the byte level. The ring buffer decouples ISR writes from the actual UART drain: ISRs `rb_put` and return immediately; the main loop drains. All ISR-originated text goes through the buffer.

**Why BSRR, not ODR, for GPIO set/clear?** `GPIOx->ODR |= mask` is a three-instruction read-modify-write. If an ISR writes a different pin on the same port during that window, the change is clobbered. `BSRR` is a single atomic write with no read step. Toggles still use `ODR ^=` because no atomic toggle exists — currently only `TIM2_IRQHandler` touches GPIOC, so that race is uncontended.

**Why distinct types for `can_id_t` and `can_dlc_t`?** Single-field wrapper structs (`{ uint32_t raw; }`) prevent implicit conversion between IDs, DLCs, and data bytes — all integers, all meaning different things. The compiler catches argument-order mistakes at the type level. Raw integers are used at API boundaries where the caller passes literal constants and the wrapper would only add noise.

**Why filter in hardware, not software?** The hardware filter discards rejected frames before they reach the FIFO. On a busy real-world bus, the CPU never wakes for unrelated traffic. In this demo it sets the precedent for the safety-bench follow-on.

**Why no dynamic allocation, no exceptions, no RTTI?** MISRA C / ISO 26262 environments mandate it. The portfolio is targeting that space, so the constraints apply from day one rather than being retrofitted.

## File layout

```
main.c            entry point, ISRs, init, application code
MCU_STM32.h       peripheral structs, register defines, types
startup.c         vector table, reset handler, .data / .bss init
linker.ld         memory map: 64K flash @ 0x08000000, 20K RAM @ 0x20000000
Makefile          build rules, NODE flag handling
```

## Future work

- DataStar SSE dashboard for live bus visualization and TX injection from a web UI
- Multi-ECU safety bench: this driver becomes the sensor-node TX path in a three-node topology with an Infineon AURIX TC277 (brake ECU) and an STM32F767ZI (gateway)
- Runtime-configurable filters via UART command (currently hard-coded in `can1_init`)
- DMA-driven UART drain to replace the polling drain in `main()`

## References

- RM0008 Rev 21, *STM32F101/F103 Reference Manual* — §24 (bxCAN), §24.7.4 (filter banks), §24.9 (register map)
- SN65HVD230 datasheet, Texas Instruments
- ISO 11898-1, CAN specification

---
