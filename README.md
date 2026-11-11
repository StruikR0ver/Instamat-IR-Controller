# Instamat IR Controller: ESPHome for an IR-controlled electric radiator

Control an electric radiator that is operated with an IR remote from Home Assistant, using an ESP8266-based IR blaster (e.g. the Athom IR Remote).

The project:

- **Sends** the radiator's reverse-engineered IR protocol (clock, program, temperature, weekly schedule), so Home Assistant can control the radiator as a normal `climate` entity.
- **Receives** the signal of the original remote, so Home Assistant stays in sync when someone uses the physical remote.
- Supports the programs of the remote: **Comfort, Night, Fill-Pilot, Frost protection, Clock program, Boost**, plus **Off**.

| File | Purpose |
|---|---|
| `Instamat-ir-controller.yaml` | ESPHome configuration: hardware, Home Assistant entities, state handling, scripts |
| `Instamat-ir-controller_engine.h` | C++ header: the IR protocol engine (encoder and decoder) |

---

## Architecture

The project is split into two layers, so the protocol can change without touching the Home Assistant logic, and the other way around.

**The engine (`..._engine.h`)** knows the radiator's IR protocol and nothing else. It has no knowledge of Home Assistant, selects or climate entities.

- *Encoder:* `build()` turns a program, temperature and the current time into the 10 IR frames (with CRC) as raw timings. Helpers: `mode_to_bits()`, `add_frame()`, `crc8_maxim()`.
- *Decoder:* `decode_frame()` validates one received frame and returns its 5 bytes, `rx_add()` collects the frames of one button press, and `interpret()` turns a complete press into program, temperature, clock and schedule.
- *Helpers:* `describe()` and `schedule_any()` produce text and flags for the sensors.

**The YAML** wraps the engine with:

- hardware setup (transmitter, receiver, LED, button)
- the internal state (`rad_mode`, `rad_temp`, `rad_prev_mode`, `rad_last_on`)
- the Home Assistant entities (climate, sensors, button)
- the logic: default temperature per program, boost timer, state synchronisation

To support a different radiator, in principle only the engine needs to be replaced.

---

## Hardware

- ESP8266 (`esp12e`) based IR blaster, e.g. Athom IR Remote
- IR transmitter on `GPIO4`, IR receiver on `GPIO5` (inverted, pull-up)
- Status LED on `GPIO13`, physical button on `GPIO0` (exposed as a binary sensor, no function)

All pins can be changed with the substitutions at the top of the YAML.

## Requirements

- ESPHome (the `climate.template` platform with custom presets is required)
- Home Assistant with the ESPHome integration
- A `secrets.yaml` with `wifi_ssid`, `wifi_password` and `ap_password`
- **An external room temperature sensor in Home Assistant** (see below)

### Why an external temperature sensor is needed

The `Radiator` climate entity is a template climate: it has no temperature measurement of its own. IR is also one-way, so the radiator cannot report its temperature back to the ESP. To show a *current temperature* on the climate card, the entity needs a sensor.

Set it with the `temp_sensor` substitution:

```yaml
substitutions:
  temp_sensor: "sensor.temperature_bathroom"   # change to your own room sensor
```

In this project the sensor is used to show the current room temperature in Home Assistant and to make the climate card complete. The ESP does not run its own control loop with it: the radiator regulates itself against the setpoint that is sent to it.

---

## Features and limitations

### Programs

| Program | Remote symbol | Default temperature (substitution) |
|---|---|---|
| Comfort | Sun | 22 °C (`comfort_function`) |
| Night | Half moon | 18 °C (`night_function`) |
| Fill-Pilot | Fill-Pilot symbol | 20 °C (`fil_Pilot_function`) |
| Clock program | Clock | 21 °C (`chrono_function`) |
| Frost protection | Snowflake | uses the radiator's own temperature |
| Boost | Boost button | runs for max. 2 hours |

The temperature can be adjusted in 0.5 °C steps between 15 and 24 °C.

### Not supported

- **Eco button:** the eco button of the remote is not implemented. It only lowers the temperature by 3 degrees, so the same result can be achieved by lowering the setpoint.
- **Weekly schedule:** the radiator takes over its schedule from the IR signal. This project always sends an **empty** schedule, so a schedule that was set on the physical remote is cleared the next time Home Assistant sends a command. Schedules cannot be set from Home Assistant.

### Other notes

- The protocol was reverse engineered from one radiator model and may differ for other models.
- Assumed, not verified in every mode: both the "20x" field and byte 3 may carry the target temperature.
- Entity names are in Dutch (e.g. "Radiator aan"); rename them in the YAML if you prefer.

---

## The IR protocol (as reverse engineered)

One button press of the remote is a burst of **10 frames**. Each frame is **40 bits = 5 bytes**: `id, b1, b2, b3, CRC`. The CRC is **CRC-8/MAXIM** over the first four bytes.

| Frame | ID | Content |
|---|---|---|
| F01 | `00` | Clock: `(weekday << 5) + hour`, minutes, seconds (Monday = 0 … Sunday = 6) |
| F02 | `01` | Program and temperature (see below) |
| F03–F09 | `08`–`0E` | Weekly schedule, one frame per day |
| F10 | `02` | Trailer: `02 40 06 06` (or `02 40 06 02` for Frost protection and Boost) |

### Frame F02: program and temperature

```
byte 0: 0x01
byte 1: (modeBits << 2) | (val20 >> 8)
byte 2: val20 & 0xFF
byte 3: temperature in tenths of a degree
```

- `val20` = temperature × 20 (a 10-bit value spread over bytes 1 and 2)
- Byte 3 holds the temperature × 10 in 8 bits

| Program | `modeBits` | F10 byte 3 | Notes |
|---|---|---|---|
| Off | `0x04` | `06` | |
| Comfort | `0x01` | `06` | |
| Night | `0x02` | `06` | |
| Fill-Pilot | `0x10` | `06` | |
| Clock program | `0x2F` | `06` | Temperature is taken from byte 3 |
| Frost protection | `0x08` | `02` | Radiator uses its own temperature |
| Boost | `0x00` | `02` | Temperature is not adopted when received |

### Weekly schedule (F03–F09)

One bit per hour: hour *h* corresponds to bit *h* of `(b1 << 16) | (b2 << 8) | b3`. This project always sends all-zero schedule frames.

### Physical encoding

- Carrier: 38 kHz, 50% duty cycle
- Header: 9525 µs pulse + 4069 µs space
- Bit: 580 µs pulse, followed by a space of 1638 µs (= `1`) or 598 µs (= `0`), MSB first
- Stop: 580 µs pulse
- Gap between frames: ~60.9 ms; after the last frame 1 ms

---

## How it works

### Sending (Home Assistant → radiator)

1. The user changes the `Radiator` climate entity (mode, preset or target temperature).
2. The change is stored in the internal state (`rad_mode`, `rad_temp`, plus `rad_prev_mode` and `rad_last_on` for bookkeeping).
3. The script `radiator_send` reads the time from Home Assistant and calls `radiator::build(...)`, which produces the 10 frames as raw timings.
4. `remote_transmitter.transmit_raw` emits them at 38 kHz.
5. `rad_climate_sync` publishes the new state back to the climate entity.

The clock frame sets the radiator's internal clock on every transmission, which the *Clock program* relies on.

### Receiving (original remote → Home Assistant)

1. `remote_receiver` delivers each frame as a separate reception (`idle: 20ms`).
2. `radiator::decode_frame` validates the timings and the CRC.
3. `radiator::rx_add` collects the frames of one press. A new press starts when frame `00` arrives or after more than 500 ms of silence. When the last frame (`02`) arrives, `interpret()` decodes the press.
4. The result updates the internal state, the climate entity and the "last remote signal" text sensor.
5. If any schedule hour is set, the binary sensor *Radiator weekschema ingesteld* turns on. It is reset after every transmission from the ESP, because the ESP always sends an empty schedule.

### Boost

Boost is limited to 2 hours. `radiator_boost_timer` waits 2 hours, then restores the program that was active before the boost (`rad_prev_mode`) and resends it. Selecting another program stops the timer. A boost started with the physical remote starts the same timer.

### HVAC modes in Home Assistant

- **OFF**: radiator off.
- **HEAT**: if the radiator is off, it switches to the last used program (`rad_last_on`); if it is already on, the current program is kept.
- The programs are exposed as **presets** (Comfort, Night, Fill-Pilot, Frost protection, Clock program, Boost).

Changing the target temperature while in *Frost protection* or *Boost* only updates Home Assistant; nothing is sent.

---

## Entities exposed to Home Assistant

| Entity | Type | Description |
|---|---|---|
| Radiator | `climate` | Main control: OFF/HEAT, presets, target temperature 15–24 °C |
| Radiator aan | binary sensor | Radiator power state |
| Radiator weekschema ingesteld | binary sensor | Whether a weekly schedule was set on the remote |
| Radiator laatste remote-signaal | text sensor | Description of the last remote press (e.g. `ma 07:30 Comfort 22.0 C`) |
| Radiator opnieuw sturen | button | Resend the current state |
| Button | binary sensor | The physical button on the device (no function) |

The select (`Radiator programma`) and number (`Radiator temperatuur`) are `internal: true`: they act as the internal state machine and are replaced in Home Assistant by the climate entity.

---

## Installation

1. Put `Instamat-ir-controller.yaml` and `Instamat-ir-controller_engine.h` in the same ESPHome config directory. The file name in `includes:` must match the header file name exactly.
2. Add `wifi_ssid`, `wifi_password` and `ap_password` to `secrets.yaml`.
3. Set `temp_sensor` to your own room temperature sensor, and adjust pins and default temperatures if needed.
4. Flash the device and add it to Home Assistant.

## Debugging

- `remote_receiver` runs with `dump: raw` and the logger at `DEBUG`. The `RAD` log lines (`decode_frame mislukt`, `Frame compleet ontvangen`, `Mode=...`) show what is received.
- To add a program: capture a press from the original remote, compare it with the tables above, and extend `mode_to_bits()` (encoder) and `interpret()` (decoder). Use exactly the same program name as in the select and the climate presets in the YAML.
