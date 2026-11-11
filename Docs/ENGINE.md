# Engine manual: `Instamat-ir-controller_engine.h`

This document explains in detail how the IR engine works: how a program and temperature become an IR signal, how a received signal is turned back into a program and temperature, and how to extend it.

The engine is a single header-only C++ file in the namespace `radiator`. It has **no dependency on ESPHome or Home Assistant** (only the standard library; one optional `ESP_LOGW` call), so it can also be compiled and tested on a PC.

---

## 1. Overview

```
            ┌──────────────────────── SENDING ────────────────────────┐
 program,   │                                                         │   raw timings
 temp, time ─► build() ─► mode_to_bits() ─► add_frame() x 10 ─────────────► (std::vector<int32_t>)
            │                                   └─ crc8_maxim()       │   → transmit_raw
            └─────────────────────────────────────────────────────────┘

            ┌─────────────────────── RECEIVING ───────────────────────┐
 raw timings│                                                         │   program, temp,
 (1 frame) ─► decode_frame() ─► rx_add() ─► interpret() ─────────────────► clock, schedule
            │   (5 bytes + CRC check)  (collects 10 frames)           │   (struct Decoded)
            └─────────────────────────────────────────────────────────┘
```

One button press of the remote (and one transmission from this project) is a burst of **10 frames**. The engine builds all 10 frames at once for sending, and collects the 10 frames one by one when receiving.

### Function reference

| Function | Direction | Purpose |
|---|---|---|
| `crc8_maxim(d, n)` | both | CRC-8/MAXIM over `n` bytes |
| `mode_to_bits(mode, bits, f10_b3)` | send | Program name → mode bits (frame F02) and byte 3 of frame F10. Returns `false` for an unknown name |
| `add_frame(out, f, last)` | send | Appends one frame (header, 40 bits, stop pulse, gap) to the timing list |
| `build(mode, temp, wday, h, m, s)` | send | Builds the complete press (10 frames) as raw timings |
| `decode_frame(x, d)` | receive | One received frame → 5 bytes. Returns `false` on a timing error or CRC error |
| `id_index(id)` | receive | Frame id → position 0..9 within a press |
| `rx_add(d, now_ms, out)` | receive | Adds a frame to the current press. Returns `true` when the press is complete |
| `interpret(press)` | receive | Complete press → `Decoded` (program, temperature, clock, schedule) |
| `round_half(v)` | receive | Rounds to the nearest 0.5 |
| `describe(r)` | receive | Human-readable text such as `wo 14:35 Comfort 22.0 C` |
| `schedule_any(s)` | receive | `true` if any hour is set in the weekly schedule |
| `describe_schedule(s)` | debug | Schedule as text (not used by the YAML, handy for debugging) |

---

## 2. The frame format

Every frame is **5 bytes (40 bits)**:

```
 byte 0   byte 1   byte 2   byte 3   byte 4
┌───────┬────────┬────────┬────────┬───────┐
│  id   │   b1   │   b2   │   b3   │  CRC  │
└───────┴────────┴────────┴────────┴───────┘
                                      └─ CRC-8/MAXIM over bytes 0..3
```

A press always consists of these 10 frames, in this order:

| # | `id` | Name | b1 / b2 / b3 |
|---|---|---|---|
| 1 | `00` | Clock | `(weekday << 5) \| hour`, minutes, seconds |
| 2 | `01` | Program + temperature | see section 2.1 |
| 3–9 | `08`…`0E` | Weekly schedule (Mon…Sun) | 24-bit mask, one bit per hour |
| 10 | `02` | Trailer | `40`, `06`, `06` (or `02`) |

Weekday numbering inside the engine is **0 = Monday … 6 = Sunday**. (ESPHome counts 1 = Sunday … 7 = Saturday; the YAML converts this before calling `build()`.)

### 2.1 Frame 2: program and temperature

```
b1 = (modeBits << 2) | (val20 >> 8)        modeBits: 6 bits, val20: top 2 of 10 bits
b2 =  val20 & 0xFF                          val20: low 8 bits
b3 =  val10                                 temperature in tenths of a degree (8 bits)
```

- `val20 = round(temperature × 20)` is a 10-bit value (15 °C = 300, 24 °C = 480).
- `val10 = round(temperature × 10)` (15 °C = 150, 24 °C = 240).
- The engine **sends the temperature in both fields**. When receiving, it reads `val20` for most programs, and `val10` for *Clock program* (see section 5.4).

| Program | `modeBits` | Frame 10, byte 3 |
|---|---|---|
| Off | `0x04` | `06` |
| Comfort | `0x01` | `06` |
| Night | `0x02` | `06` |
| Fill-Pilot | `0x10` | `06` |
| Clock program | `0x2F` | `06` |
| Frost protection | `0x08` | `02` |
| Boost | `0x00` | `02` |

Boost is the only program with `modeBits = 0x00`. It is recognised by frame 10 ending in `02` instead of `06`. Frost protection also ends in `02`, but is already unique by its mode bits.

### 2.2 Frames 3–9: weekly schedule

One frame per day (ids `08` = Monday … `0E` = Sunday). The three data bytes form a 24-bit mask:

```
mask = (b1 << 16) | (b2 << 8) | b3        bit h = 1  →  heating on during hour h
```

The radiator **takes over the schedule from the IR signal**. The encoder therefore always sends all-zero schedule frames: a schedule set on the physical remote is cleared the next time the ESP transmits.

### 2.3 Worked example

`build("Comfort", 22.0, wday = 2 (Wednesday), 14:35:10)`:

| Frame | Calculation | Bytes (hex) |
|---|---|---|
| 1 Clock | `b1 = (2 << 5) \| 14 = 0x4E`, `b2 = 35 = 0x23`, `b3 = 10 = 0x0A` | `00 4E 23 0A` + CRC `2F` |
| 2 Program | `val20 = 440 = 0x1B8`, `val10 = 220 = 0xDC`; `b1 = (0x01 << 2) \| 0x01 = 0x05`, `b2 = 0xB8`, `b3 = 0xDC` | `01 05 B8 DC` + CRC `3A` |
| 3 Monday | empty | `08 00 00 00` + CRC `1C` |
| 4–9 | empty | `09`…`0E`, `00 00 00` |
| 10 Trailer | Comfort → byte 3 = `06` | `02 40 06 06` + CRC `41` |

---

## 3. The physical layer

The engine produces and expects **raw timings in microseconds**, in the ESPHome convention: a **positive** number is a pulse (carrier on), a **negative** number is a pause (carrier off). The carrier itself (38 kHz) is generated by ESPHome (`carrier_frequency: 38kHz`).

### 3.1 One frame

```
 header           bit 1 (MSB)       bit 2 ...        bit 40          stop    gap
┌─────────┬──────┐┌─────┬────────┐┌─────┬────────┐ ... ┌─────┬──────┐┌─────┐┌───────────┐
│ 9525 µs │-4069 ││ 580 │ -1638  ││ 580 │  -598  │     │ 580 │ ...  ││ 580 ││ -60930 µs │
│  pulse  │pause ││pulse│ long=1 ││pulse│ short=0│     │     │      ││     ││ (or -1000)│
└─────────┴──────┘└─────┴────────┘└─────┴────────┘     └─────┴──────┘└─────┘└───────────┘
```

| Element | Value |
|---|---|
| Header pulse / pause | 9525 µs / −4069 µs |
| Bit pulse | 580 µs |
| Bit `1` pause | −1638 µs |
| Bit `0` pause | −598 µs |
| Stop pulse | 580 µs |
| Gap after a frame | −60930 µs (after the last frame: −1000 µs) |
| Bit order | MSB first, byte 0 first |

A frame is **84 values** in the list (2 header + 80 bit values + stop pulse + gap). A complete press is **840 values** and lasts about **1.2 seconds**. It is stored as a `std::vector<int32_t>` of about 3.4 kB, which is created on every transmission and freed afterwards.

### 3.2 CRC-8/MAXIM

`crc8_maxim()` is the standard CRC-8/MAXIM (also known as Dallas/1-Wire): initial value 0, reflected polynomial `0x8C`, processed bit by bit. It is calculated over bytes 0..3 of each frame and stored in byte 4. The receiver rejects frames whose CRC does not match.

---

## 4. Sending in detail: `build()`

```cpp
std::vector<int32_t> build(const std::string &mode, float temp,
                           int wday, int hour, int minute, int second);
```

Steps:

1. **Look up the program** with `mode_to_bits()`. For an unknown name, `build()` logs a warning and returns an **empty list**, so nothing is transmitted. The program name must therefore match exactly (case-sensitive) with what the YAML passes in.
2. **Clamp the temperature** to 15.0 … 24.0 °C (the radiator's range).
3. **Calculate** `val20` and `val10` (rounded, see 2.1).
4. **Build the 10 frames** in order: clock, program/temperature, seven empty schedule frames, trailer. `add_frame()` adds the CRC, the header, the 40 bits, the stop pulse and the gap.
5. The last frame gets a short gap (−1000 µs) instead of the long one.

The clock frame carries the current time, so the radiator's clock is corrected with every transmission. This is what makes *Clock program* work correctly. Hours are masked to 5 bits and the weekday to 3 bits.

---

## 5. Receiving in detail

### 5.1 `decode_frame()`: one frame to 5 bytes

The ESPHome receiver is configured with `idle: 20ms`. Because the gap between frames (60.9 ms) is longer than that, **every frame arrives as a separate reception** (`on_raw`).

`decode_frame()` accepts a reception only if:

- it has **83 or 84 values** (84 if the trailing gap was captured too),
- the header pulse is between 8000 and 11000 µs,
- the header pause is between 3000 and 5000 µs,
- every bit pause is between 300 and 2500 µs,
- and the **CRC matches**.

A bit pause longer than **1100 µs** is read as `1`, shorter as `0`. That threshold sits between the nominal 598 µs and 1638 µs. Only the pauses are evaluated; the length of the bit pulses is not checked.

### 5.2 `rx_add()`: collecting a press

`rx_add()` keeps its state in one static variable (`rx`) that holds the 10 frames of the current press and a bitmask `have` of which frames have arrived.

```
frame arrives ──► id_index(id) ──► -1 ? ──► ignore (unknown id)
                       │
                       ▼ idx 0..9
        idx == 0  OR  > 500 ms since the previous frame?
                 yes ─► start a new press (have = 0)
                       │
                  store the frame, set bit `idx` in `have`
                       │
                  idx == 9 (trailer)?
                 no  ─► return false (wait for more)
                 yes ─► interpret(rx), have = 0, return out.valid
```

Frame order and index: `00`→0, `01`→1, `08`…`0E`→2…8, `02`→9. A press is **complete when the trailer (`02`) arrives**. Frames lost in between do not block it, but `interpret()` needs at least frame 2 (program/temperature).

The 500 ms limit makes sure that a half-received press is discarded when the next one begins. A normal frame spacing is about 120–160 ms.

### 5.3 `interpret()`: the meaning of a press

`interpret()` returns a `Decoded` struct:

| Field | Meaning |
|---|---|
| `valid` | `true` if frame 2 was present |
| `mode` | Program name, or `"onbekend"` (unknown) |
| `temp`, `temp_valid` | Temperature rounded to 0.5 °C, and whether it may be used |
| `has_clock`, `wday`, `hour`, `minute`, `second` | The remote's clock (if frame 1 was received) |
| `has_schedule`, `schedule[7]` | The 24-bit masks per day (only if **all seven** schedule frames arrived) |

**Program detection** reads `modeBits = b1 >> 2` of frame 2:

| `modeBits` | Result | Temperature source |
|---|---|---|
| `0x04` | Off | `val20 / 20` |
| `0x01` | Comfort | `val20 / 20` |
| `0x02` | Night | `val20 / 20` |
| `0x10` | Fill-Pilot | `val20 / 20` |
| `0x2F` | Clock program | **`b3 / 10`** (byte 3) |
| `0x08` | Frost protection | not adopted (`temp_valid = false`) |
| `0x00` | Boost, **only if** frame 10 is `02 40 06 02` | not adopted (`temp_valid = false`) |
| anything else | `"onbekend"` | not adopted |

### 5.4 Temperature rules

- The temperature is rounded to 0.5 °C and only accepted when it is within **15 … 24 °C**; otherwise `temp_valid` stays `false`.
- For *Frost protection* and *Boost* the temperature is never adopted, because the radiator uses its own value in those programs.
- For *Clock program* the captures showed the radiator following byte 3 while the `val20` field kept a different value, which is why this program reads `b3 / 10`.

Note that `interpret()` sets `valid = true` as soon as frame 2 is present, **even for an unknown program**. The caller must check for `mode == "onbekend"` (the YAML does this).

---

## 6. How the YAML uses the engine

| YAML location | Engine call |
|---|---|
| `script: radiator_send` | `radiator::build(rad_mode, rad_temp, wday, h, m, s)` → `transmit_raw` |
| `remote_receiver: on_raw` | `decode_frame()` → `rx_add()` → fields of `Decoded` |
| `on_raw`, text sensor | `describe(r)` |
| `on_raw`, schedule sensor | `schedule_any(r.schedule)` |

The program names (`"Off"`, `"Comfort"`, `"Night"`, `"Fill-Pilot"`, `"Frost protection"`, `"Clock program"`, `"Boost"`) exist in several places and **must be identical everywhere**:

1. `mode_to_bits()` (engine, sending)
2. the `switch` in `interpret()` (engine, receiving)
3. the `select` options (YAML)
4. the climate `custom_presets` (YAML)
5. the string comparisons in the receiver lambda and number/select actions (YAML)

A mismatch does not cause an error message in Home Assistant: a program that is unknown to `mode_to_bits()` simply sends nothing (a warning appears in the log), and a program that the decoder returns under a different name ends up as a value that the select and climate do not know.

---

## 7. Extending the engine

### Adding a new program

1. Capture the press of the original remote (`dump: raw` in the receiver) and decode the bytes of frame 2 and frame 10.
2. Work out `modeBits` and the byte 3 of frame 10.
3. Add the program in `mode_to_bits()` (sending).
4. Add a `case` in the `switch` of `interpret()` (receiving), and decide whether the temperature is taken from `val20`, from byte 3, or not at all.
5. In the YAML, add the exact same name to the `select` options and the climate `custom_presets`, and give it a default temperature if needed.

### Changing the temperature range

The clamp in `build()` (`15.0f` / `24.0f`), the range check in `interpret()`, and `min_temperature` / `max_temperature` / `min_value` / `max_value` in the YAML must all be changed together. Keep in mind that `val10` is stored in 8 bits (maximum 25.5 °C).

### Possible improvements

- Keep **one table** with name, mode bits, frame-10 byte and temperature source, and let both `mode_to_bits()` and `interpret()` use it. A new program then needs only one line in the engine.
- Define the program names once as constants and use them in the YAML via substitutions.
- Keep a display name separate from the internal name, so renaming a preset in Home Assistant cannot break the protocol.

---

## 8. Testing the engine on a PC

Because the engine has no ESPHome dependency, the complete encode → decode loop can be checked on a PC. This compiles with `g++ -std=c++17`:

```cpp
#include "Instamat-ir-controller_engine.h"
#include <cstdio>
using namespace radiator;

int main() {
  auto v = build("Comfort", 22.0f, 2, 14, 35, 10);     // Wednesday 14:35:10
  Decoded r;
  bool done = false;
  for (int f = 0; f < 10; f++) {
    std::vector<int32_t> frame(v.begin() + f * 84, v.begin() + f * 84 + 83);
    uint8_t d[5];
    if (!decode_frame(frame, d)) { printf("frame %d failed\n", f + 1); return 1; }
    done = rx_add(d, 1000 + f * 140, r);               // fake timestamps, 140 ms apart
  }
  printf("complete=%d  %s\n", done, describe(r).c_str());   // wo 14:35 Comfort 22.0 C
}
```

With the current engine, a round trip of **all 7 programs × 19 temperatures (15.0 … 24.0 in 0.5 steps)** gives back the same program, the same temperature (except Frost protection and Boost, where the temperature is intentionally not adopted), the same clock, and an empty schedule. Also verified:

- an unknown program name returns an empty list (nothing is sent),
- a temperature above 24 °C is clamped to 24.0 °C,
- a frame with a flipped bit is rejected by the CRC check,
- a press whose frames are more than 500 ms apart is discarded.

## 9. Good to know

- **Static state:** `rx` is a single global press buffer. This is fine for one receiver in ESPHome's single-threaded loop, but the engine is not reentrant.
- **Reverse engineered:** the protocol was derived from captures of one radiator model. In particular, that both the `val20` field and byte 3 may carry the temperature in every program is an assumption that has not been verified for all programs.
- **Eco button:** the remote's eco button (which only lowers the temperature by 3 degrees) is not part of the engine.
