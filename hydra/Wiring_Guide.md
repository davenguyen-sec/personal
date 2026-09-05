# Hydroponics System — Wiring Guide

## Purpose

This guide covers the wiring for the two-controller hydroponics system:

- **Arduino Mega 2560** — sensors, LCD, pump controllers, and automatic dosing
- **ESP8266 / NodeMCU v2** — receives telemetry from the Mega and publishes it over MQTT / Home Assistant

The pin assignments and I2C addresses below match the current controller and MQTT bridge code.

---

## 1. Arduino Mega — Master Connection Table

| Device | Device Pin / Signal | Arduino Mega Connection |
|---|---|---|
| DFRobot pH interface | Analog output `A` | **A5** |
| DFRobot pH interface | VCC `+` | **5V** |
| DFRobot pH interface | GND `-` | **GND** |
| DS18B20 temperature probe | DATA | **D4** |
| DS18B20 temperature probe | VCC | **5V** |
| DS18B20 temperature probe | GND | **GND** |
| I2C bus | SDA | **D20 / SDA** |
| I2C bus | SCL | **D21 / SCL** |
| LCD | SDA | **D20 / SDA** |
| LCD | SCL | **D21 / SCL** |
| Atlas EZO-EC | SDA | **D20 / SDA** |
| Atlas EZO-EC | SCL | **D21 / SCL** |
| pH Down pump controller | SDA / SCL | Shared I2C bus |
| Nutrient A pump controller | SDA / SCL | Shared I2C bus |
| Nutrient B pump controller | SDA / SCL | Shared I2C bus |
| EC Down / water pump controller | SDA / SCL | Shared I2C bus |
| ESP8266 telemetry | Mega TX1 | **D18 / TX1** |
| Common ground | GND | All controllers/modules must share GND |

---

## 2. DFRobot pH Sensor

The controller reads pH through an analogue DFRobot pH signal-conditioning board.

### Wiring

| pH Board | Arduino Mega |
|---|---|
| `A` — analogue output | **A5** |
| `+` — VCC | **5V** |
| `-` — GND | **GND** |
| BNC connector | pH probe |

The DFRobot V2 pH interface accepts approximately 3.3–5.5 V power and produces an analogue signal suitable for the Mega input.

**Important:** Keep the pH signal-conditioning board and BNC connector dry.

---

## 3. DS18B20 Water Temperature Probe

The controller code uses:

**DATA = Arduino Mega D4**

Typical three-wire waterproof DS18B20 wiring:

| DS18B20 | Arduino Mega |
|---|---|
| VCC / usually red | **5V** |
| DATA / usually yellow or white | **D4** |
| GND / usually black | **GND** |

Install a **4.7 kΩ pull-up resistor between DATA and 5V** unless your DS18B20 adapter/module already contains the pull-up resistor.

Do not rely only on wire colours if your probe came from an unknown supplier; confirm its pinout first.

---

## 4. I2C Bus

The following devices share the same I2C bus:

- 20x4 LCD
- Atlas EZO-EC circuit
- pH Down pump controller
- Nutrient A pump controller
- Nutrient B pump controller
- EC Down / water pump controller

On the Arduino Mega 2560:

- **SDA = D20**
- **SCL = D21**

Wire every I2C device in parallel:

```text
Mega D20 / SDA ──┬── LCD SDA
                 ├── EZO-EC SDA
                 ├── pH Down controller SDA
                 ├── Nutrient A controller SDA
                 ├── Nutrient B controller SDA
                 └── EC Down controller SDA

Mega D21 / SCL ──┬── LCD SCL
                 ├── EZO-EC SCL
                 ├── pH Down controller SCL
                 ├── Nutrient A controller SCL
                 ├── Nutrient B controller SCL
                 └── EC Down controller SCL

Mega GND ─────────── Common ground for all modules
```

### I2C Addresses

| Device | Address |
|---|---|
| LCD | `0x27` |
| pH Down pump | `0x32` |
| Nutrient A pump | `0x33` |
| Nutrient B pump | `0x34` |
| EC Down / water pump | `0x35` |
| Atlas EZO-EC | `0x64` |

Every device on the shared bus must have a unique address.

---

## 5. 20x4 I2C LCD

Typical four-pin I2C backpack wiring:

| LCD Backpack | Arduino Mega |
|---|---|
| VCC | **5V** |
| GND | **GND** |
| SDA | **D20 / SDA** |
| SCL | **D21 / SCL** |

The controller expects the LCD at address:

**`0x27`**

---

## 6. Atlas Scientific EZO-EC

The controller communicates with the Atlas EZO-EC in **I2C mode** at:

**`0x64`**

### Wiring

| EZO-EC | Arduino Mega |
|---|---|
| VCC | **5V** |
| GND | **GND** |
| SDA | **D20 / SDA** |
| SCL | **D21 / SCL** |
| Probe connection | EC/conductivity probe via the appropriate EZO carrier / connector |

The EZO-EC supports approximately 3.3–5.5 V operation.

The conductivity probe connects to the EZO circuit, **not directly to an Arduino analogue input**.

Make sure the EZO circuit is actually configured for **I2C mode** and address `0x64`.

---

## 7. Peristaltic Pump Controllers

The Mega talks to four separate pump controllers over the same I2C bus.

### Pump Mapping

| Address | Pump | Liquid |
|---|---|---|
| `0x32` | pH Down | pH-down solution |
| `0x33` | Nutrient A | Nutrient Part A |
| `0x34` | Nutrient B | Nutrient Part B |
| `0x35` | EC Down | Plain water / dilution |

For each pump controller:

```text
Controller SDA → Mega D20 / SDA
Controller SCL → Mega D21 / SCL
Controller GND → Common GND
Controller power → According to that controller's hardware specification
Pump motor → Pump controller motor output
```

### Important Pump Power Note

The sketches define the **I2C control wiring and addresses**, but they do not define the motor-driver hardware or pump supply voltage.

Do **not** power peristaltic pump motors directly from an Arduino Mega I/O pin or assume the Mega's 5V rail is suitable for the motors.

Use the supply required by your pump controller and pump hardware, while maintaining a common ground where required by the controller design.

---

## 8. Arduino Mega → ESP8266 / NodeMCU

The telemetry link is one-way:

**Mega Serial1 TX → ESP8266 RX0**

### Connections

| Arduino Mega | ESP8266 / NodeMCU |
|---|---|
| **D18 / TX1** | **RX0** through a level shifter / voltage divider |
| **GND** | **GND** |

Serial speed:

**115200 baud**

### Very Important: Logic Voltage

The Mega TX1 signal is approximately **5 V logic**.

The ESP8266 RX input is a **3.3 V logic input**.

Therefore:

**Do not connect Mega TX1 directly to ESP8266 RX0.**

Use a suitable logic-level shifter or resistor voltage divider between Mega TX1 and NodeMCU RX0.

A typical divider arrangement is:

```text
Mega TX1 ---- resistor ----+---- ESP8266 RX0
                           |
                         resistor
                           |
                          GND
```

The resistor values must reduce the Mega's 5 V TX signal to approximately 3.3 V.

The current MQTT bridge only receives data from the Mega, so an ESP8266 TX → Mega RX connection is not required by the current code.

### ESP8266 Power

The simplest option is to power/program the NodeMCU using its USB connector.

Make sure the ESP8266 and Mega grounds are connected together.

If programming the NodeMCU becomes unreliable while Mega TX1 is connected to RX0, temporarily disconnect the Mega TX1 → ESP8266 RX0 wire while uploading firmware.

---

## 9. Overall Wiring Diagram

```text
                         ┌──────────────────────┐
                         │   Arduino Mega 2560 │
                         │                      │
pH board analogue ──────►│ A5                   │
DS18B20 DATA ───────────►│ D4                   │
                         │                      │
                         │ D20 SDA ─────────────┼──── Shared I2C SDA
                         │ D21 SCL ─────────────┼──── Shared I2C SCL
                         │                      │
                         │ D18 TX1 ── LEVEL ────┼────► ESP8266 RX0
                         │            SHIFT     │
                         └──────────────────────┘
                                  │
                                  │ I2C
          ┌───────────────────────┼─────────────────────────────┐
          │                       │                             │
       LCD 0x27                EZO-EC 0x64                  Pump controllers
          │                       │                         0x32 pH Down
          │                       │                         0x33 Nutrient A
          │                    EC probe                     0x34 Nutrient B
          │                                                 0x35 EC Down
          │
          └──────────────────── Common GND ─────────────────────┐
                                                                │
                                                         ESP8266 / NodeMCU
                                                                │
                                                              Wi-Fi
                                                                │
                                                              MQTT
                                                                │
                                                        Home Assistant
```

---

## 10. Power and Ground Checklist

Before switching the system on:

- Mega and ESP8266 share a common ground.
- All I2C devices share SDA and SCL correctly.
- No two I2C devices use the same address.
- ESP8266 RX0 is protected from the Mega's 5 V TX signal.
- Pump motors are powered through their correct motor/controller supply.
- Sensor electronics are kept dry.
- pH and EC probe connectors are secure.
- DS18B20 has its required pull-up resistor if one is not built into its module.
- Nutrient A and Nutrient B tubing are clearly labelled.
- pH Down is connected to `0x32`.
- Nutrient A is connected to `0x33`.
- Nutrient B is connected to `0x34`.
- Water / EC Down is connected to `0x35`.

---

## 11. Connections Defined by the Current Software

### Arduino Mega controller

```text
pH analogue input       A5
DS18B20 data            D4
I2C SDA                 D20
I2C SCL                 D21
ESP telemetry           TX1 / D18
LCD                     0x27
pH Down pump            0x32
Nutrient A pump         0x33
Nutrient B pump         0x34
EC Down pump            0x35
Atlas EZO-EC            0x64
```

### ESP8266 MQTT bridge

```text
Serial RX0              receives Mega Serial1 TX
Serial baud             115200
Network output          Wi-Fi → MQTT → Home Assistant
```

---

## Important

This guide can identify the signal connections from the software, but the exact **pump controller power input, motor terminals, pump supply voltage, and connector pinout** are not defined in the two sketches.

For those connections, use the specification for the exact pump-controller hardware installed in the system.
