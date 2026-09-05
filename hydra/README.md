# Hydroponics Controller — Quick README

## System Overview

This hydroponics system uses two microcontrollers:

1. **Arduino Mega — Main Controller**
   - Reads pH, EC, and water temperature.
   - Controls Nutrient A, Nutrient B, pH Down, and EC Down/water pumps.
   - Runs the automatic dosing logic.
   - Displays system information on the 20x4 LCD.
   - Sends telemetry to the ESP8266 over `Serial1`.

2. **ESP8266 / NodeMCU — MQTT Bridge**
   - Receives telemetry from the Arduino Mega.
   - Connects to Wi-Fi and the MQTT broker.
   - Publishes system values under the `hydro/*` MQTT topics.
   - Provides Home Assistant MQTT auto-discovery.

## Files

- `hydra_main.ino` — upload to the **Arduino Mega**
- `hydro_mqtt_bridge.ino` — upload to the **ESP8266 / NodeMCU**

## Pump Mapping

| I2C Address | Pump / Liquid |
|---|---|
| `0x32` | pH Down |
| `0x33` | Nutrient A |
| `0x34` | Nutrient B |
| `0x35` | EC Down / Water |

There is no automatic pH-Up pump in the current controller.

## Default Control Settings

- pH target: **5.8**
- EC target: **1000 µS/cm**
- EC deadband: **±50 µS/cm**
- Nutrient dose: **8 mL total**
  - 4 mL Nutrient A
  - wait 2 minutes
  - 4 mL Nutrient B
- EC-up lockout: **10 minutes**
- pH-down dose: **1 mL**
- pH-down lockout: **30 minutes**
- Automatic EC-down/water dosing is currently disabled.

## Startup

1. Fill the reservoir and make sure the water is circulating.
2. Make sure all probes are installed correctly.
3. Power the Arduino Mega.
4. The LCD will initialize and sensor readings will begin after startup.
5. The controller reads the sensors approximately every 10 seconds.
6. Automatic EC-up dosing is blocked for about the first 10 minutes after startup.
7. If EC is below the target range and the latest EC reading is valid, Nutrient A is dosed first, followed by Nutrient B about 2 minutes later.
8. pH control only runs after EC is in range and has settled.

## Important Safety Behaviour

- A failed or invalid EC reading prevents automatic dosing.
- A pump dose is only counted when its I2C command is acknowledged.
- Nutrient B is only scheduled after Nutrient A is successfully commanded.
- Hourly dose limits and mixing lockouts help prevent repeated dosing.

For initial setup, test and label every pump using plain water before connecting nutrient or pH solution.

## Serial Connections

### Mega USB Serial

Use **9600 baud** for controller debugging.

Typical output:

`Snapshot | pHraw: ... | pHc: ... | EC: ... | ECvalid: Y/N | T: ... C`

`ECvalid: Y` means the latest EC measurement is valid for automatic control.

### Mega → ESP8266

- Mega `TX1` → ESP8266 `RX0`
- Serial speed: **115200 baud**
- Common ground required.
- The Mega TX signal is 5 V and the ESP8266 RX input is 3.3 V, so use a suitable voltage divider or level shifter.

## MQTT / Home Assistant

Before uploading the ESP8266 bridge, configure:

- Wi-Fi SSID
- Wi-Fi password
- MQTT broker address
- MQTT username
- MQTT password

The bridge publishes retained MQTT values under the `hydro/*` namespace and publishes Home Assistant discovery messages automatically.

Typical values include pH, filtered pH, EC, water temperature, setpoints, dosing totals, last dosing action, and controller uptime.

## First-Time Use

Before leaving the system unattended:

1. Confirm pH, EC, and temperature readings are sensible.
2. Confirm `ECvalid: Y` appears in Serial Monitor.
3. Confirm the four physical pumps match the I2C mapping above.
4. Confirm Nutrient A and Nutrient B tubing are not swapped.
5. Watch at least one complete automatic A → B nutrient dosing cycle.
6. Confirm MQTT/Home Assistant values match the Mega readings.

## Calibration

Calibrate the pH and EC probes before relying on automatic dosing.

Pump output should also be calibrated so a command such as `D,4.00` actually delivers approximately 4 mL.

---

**Normal data flow:**

Sensors → Arduino Mega → dosing control → Serial1 → ESP8266 → MQTT → Home Assistant
