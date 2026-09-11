# Ender 3 V2 — Custom mriscoc Professional Firmware

> **Tailored build:** Ender 3 V2 · Creality V4.2.2 · STM32F103RET6 · DWIN display · Sprite Extruder Pro · CR Touch · UBL · T13 · MPC

This README documents the exact process used to build a custom **mriscoc Professional Firmware** image for this specific Ender 3 V2.

---

## 1. Hardware profile

| Component | This printer |
|---|---|
| Printer | **Creality Ender 3 V2** |
| Mainboard | **Creality V4.2.2** |
| MCU | **STM32F103RET6** |
| Flash size / build target | **512 KB / STM32F103RE** |
| Display | **DWIN** |
| Extruder / hot end | **Sprite Extruder Pro** |
| Probe | **CR Touch** |
| Bed levelling | **UBL (Unified Bed Levelling)** |
| Thermistor configuration | **T13 / Marlin #13 (3950)** |
| Hot-end control | **MPC** |
| Input Shaping | Not enabled initially |
| Linear Advance | Not enabled initially |

### Target configuration name

```text
Ender3V2-422-BLTUBL-T13-MPC
```

This is the configuration to reproduce.

---

## 2. Important safety note

> [!WARNING]
> **Do not flash or use this T13 firmware with the original Ender 3 V2 hot end.**
>
> This build is intended for the **Sprite Extruder Pro / compatible all-metal hot end and thermistor arrangement**. Install the Sprite Pro and CR Touch before flashing this firmware.

Also:

- Disconnect mains power before working inside the electronics enclosure.
- Double-check heater and thermistor wiring before the first power-on.
- After flashing, verify temperature behaviour at conservative temperatures before attempting high-temperature printing.
- Do not immediately command 300 °C simply because the firmware permits it.

---

## 3. Software required

Install:

- **Python 3**
- **Visual Studio Code**
- VS Code extension: **PlatformIO IDE**
- VS Code extension: **Auto Build Marlin**

Repositories:

- mriscoc Professional Firmware  
  `https://github.com/mriscoc/Ender3V2S1`

- mriscoc Special Configurations  
  `https://github.com/mriscoc/Special_Configurations`

> [!TIP]
> Keep the extracted firmware source in a short Windows path, for example:
>
> ```text
> C:\mriscoc\
> ```
>
> This helps avoid Windows path-length problems during compilation.

---

## 4. Generate the custom configuration

Download the **main branch/repository ZIP** of `Special_Configurations`.

Do **not** use the old release-page source-code placeholder ZIP.

Extract the whole repository. It should contain files/folders such as:

```text
Configurator.pyw
CreateConfigs.py
_printers\
_boards\
_leveling\
_displays\
_thermistor\
_features\
images\
```

Run:

```cmd
python Configurator.pyw
```

The **Professional Firmware Configurator** window should open.

### Select these exact options

| Section | Selection |
|---|---|
| Printer | `Ender3V2` |
| Board | `422` |
| Leveling | `BLT` |
| UBL | ✅ Checked |
| Display | `DWIN` |
| Thermistor | `T13` |
| MPC | ✅ Checked |

Leave the other optional features unticked initially, including:

```text
IS
LA
LockSteppers
HomeOffs
NP
Repeat&Depth
```

### Expected generated configuration

Click **Auto**.

The configuration name should become:

```text
Ender3V2-422-BLTUBL-T13-MPC
```

The generated command shown at the bottom should be equivalent to:

```python
CreateConfigs.Generate(
    'Ender3V2-422-BLTUBL-T13-MPC',
    ['Ender3V2', '422', 'BLT', 'UBL', 'T13', 'MPC']
)
```

Click **Generate**.

The configurator should create a folder containing approximately:

```text
Configuration.h
Configuration_adv.h
Version.h
platformio.ini
log.txt
```

---

## 5. Download the current mriscoc source

Download the current source from:

```text
https://github.com/mriscoc/Ender3V2S1
```

Extract it.

The **project root** is the folder containing:

```text
Marlin\
buildroot\
ini\
platformio.ini
README.md
```

Do **not** open only the `Marlin` subfolder in VS Code.

---

## 6. Copy the generated configuration files

From:

```text
Ender3V2-422-BLTUBL-T13-MPC\
```

copy:

```text
Configuration.h
Configuration_adv.h
Version.h
```

into:

```text
<mriscoc project>\Marlin\
```

Replace the existing files.

Then copy:

```text
platformio.ini
```

into:

```text
<mriscoc project>\
```

Replace the existing `platformio.ini`.

Do **not** copy:

```text
log.txt
```

### Resulting layout

```text
Ender3V2S1\
│
├── platformio.ini              <-- generated file
│
├── Marlin\
│   ├── Configuration.h         <-- generated file
│   ├── Configuration_adv.h     <-- generated file
│   ├── Version.h               <-- generated file
│   └── ...
│
├── buildroot\
├── ini\
└── ...
```

---

## 7. Open the project in VS Code

In VS Code:

```text
File -> Open Folder
```

Open the **project root** — the directory containing `platformio.ini`.

Install/enable:

- PlatformIO IDE
- Auto Build Marlin

Wait for PlatformIO to finish initialising.

Then open **Auto Build Marlin**.

It should detect something similar to:

```text
Machine Name: Ender3V2-422-BLTUBL-T13-MPC
Board: CREALITY V4
Architecture: STM32F1
```

---

## 8. Select the correct build target

This printer has:

```text
STM32F103RET6
```

Therefore use:

```text
STM32F103RE_creality (512K)
```

### Correct

```text
STM32F103RE_creality (512K)  -> Build
```

### Do not use

```text
STM32F103RE_creality_xfer
STM32F103RC_creality
STM32F103RC_creality_xfer
STM32F103RE_creality_maple
```

The `RC` builds are for an STM32F103RCT6 MCU, not this printer.

The `_xfer` targets are not the normal SD-card firmware build.

---

## 9. Compile

Click:

```text
Build
```

beside:

```text
STM32F103RE_creality (512K)
```

The first build may take several minutes because PlatformIO can download toolchains and libraries.

A successful build ends with:

```text
SUCCESS
```

The firmware binary is normally created at approximately:

```text
.pio\build\STM32F103RE_creality\firmware.bin
```

The resulting firmware file is expected to be only a few hundred kilobytes. That is normal.

---

## 10. Verify the firmware identity

Before flashing, confirm the build was generated from the intended configuration.

Expected machine/configuration identity:

```text
Ender3V2-422-BLTUBL-T13-MPC
```

This corresponds to:

```text
Ender3V2  = Ender 3 V2
422       = Creality V4.2.2 board
BLT       = CR Touch / BLTouch support
UBL       = Unified Bed Levelling
T13       = Marlin #13 / 3950 thermistor configuration
MPC       = Model Predictive Temperature Control
```

---

## 11. Prepare the microSD card

Use a small, reliable microSD card.

Format it as:

```text
FAT32
```

Put the compiled `.bin` file in the **root** of the card.

Use a short, unique filename, for example:

```text
FW260911.bin
```

Creality bootloaders may ignore a firmware filename that is identical to one previously flashed, so use a new name for future firmware updates.

Ideally keep only **one `.bin` firmware file** on the card while flashing.

---

## 12. Flash only after the hardware is installed

Before flashing, install:

- Sprite Extruder Pro
- CR Touch
- Dual-Z kit
- Silicone bed spacers
- PEI magnetic spring-steel bed

The first two are the critical items for this firmware configuration.

### Flash procedure

1. Power the Ender 3 V2 **off**.
2. Insert the prepared microSD card.
3. Power the printer **on**.
4. Allow the bootloader to flash the firmware.
5. Wait for the printer to reach the normal mriscoc interface.
6. Remove the card after confirming the firmware has loaded.

---

## 13. First-start checks

Do **not** immediately start a print.

Check these first:

- [ ] Display starts normally
- [ ] CR Touch powers up and self-tests
- [ ] X/Y/Z movement directions are correct
- [ ] Dual-Z moves smoothly
- [ ] Sprite extruder motor direction is correct
- [ ] Hot-end temperature reading is plausible at room temperature
- [ ] Bed temperature reading is plausible
- [ ] Fans operate correctly
- [ ] Homing is safe
- [ ] CR Touch deploys/retracts correctly

> [!CAUTION]
> Be ready to switch power off during the first homing test if the probe, Z direction, or toolhead position behaves unexpectedly.

---

## 14. Calibration after installation

Once the hardware and firmware are confirmed working, perform calibration in this general order:

### 1. Square the X gantry

With dual Z installed, make sure the X gantry is level relative to the frame.

### 2. Mechanically tram the bed

Use the silicone spacers / adjustment wheels to get the bed reasonably parallel to the gantry.

### 3. Configure CR Touch probe offsets

The Sprite Pro changes the probe position relative to the nozzle.

Set the correct:

```text
X probe offset
Y probe offset
```

### 4. Set Z-offset

Carefully set the nozzle-to-bed Z-offset using mriscoc's Z-offset tools.

### 5. Generate a UBL mesh

Probe the bed and save the mesh.

### 6. Calibrate extrusion

Set/check the Sprite Pro extruder steps.

Do not assume the old stock extruder value is correct.

### 7. Run MPC tuning

Run hot-end MPC calibration after the Sprite Pro is fitted.

This allows the firmware to characterise the new heater/hot-end thermal behaviour.

### 8. Test temperatures progressively

Start around:

```text
200 °C
```

then verify operation at:

```text
230-250 °C
```

before considering higher-temperature materials.

---

## 15. Optional later upgrades

Once the printer is mechanically stable and producing good prints, the firmware can be rebuilt with additional features.

### Input Shaping (`IS`)

Useful for reducing ringing and allowing higher acceleration.

### Linear Advance (`LA`)

Useful for improving extrusion control during acceleration/deceleration.

Do **not** enable/tune these at the same time as the initial hardware rebuild. Establish a reliable baseline first.

To add them later:

1. Reopen `Configurator.pyw`.
2. Select the same configuration.
3. Tick `IS` and/or `LA`.
4. Generate again.
5. Copy the generated configuration files.
6. Recompile.
7. Flash the new `.bin`.
8. Calibrate the new feature.

No additional printer hardware is necessarily required simply to rebuild the firmware.

---

## 16. Final build summary

```text
Printer:          Creality Ender 3 V2
Mainboard:        Creality V4.2.2
MCU:              STM32F103RET6
Build target:     STM32F103RE_creality (512K)
Display:          DWIN
Extruder:         Sprite Extruder Pro
Probe:            CR Touch
Levelling:        BLT + UBL
Thermistor:       T13
Thermal control:  MPC
Firmware:         mriscoc Professional Firmware
```

Generated configuration:

```text
Ender3V2-422-BLTUBL-T13-MPC
```

---

## 17. Keep a recovery copy

Keep copies of:

```text
firmware.bin
Configuration.h
Configuration_adv.h
Version.h
platformio.ini
```

along with a note of the exact hardware configuration.

A useful archive structure is:

```text
Ender3V2-Firmware\
├── README.md
├── firmware\
│   └── Ender3V2-422-BLTUBL-T13-MPC.bin
├── config\
│   ├── Configuration.h
│   ├── Configuration_adv.h
│   ├── Version.h
│   └── platformio.ini
└── notes\
    └── calibration-values.md
```

This makes future rebuilds, troubleshooting and upgrades much easier.

---

## Related projects

- **mriscoc Professional Firmware:**  
  https://github.com/mriscoc/Ender3V2S1

- **mriscoc Special Configurations:**  
  https://github.com/mriscoc/Special_Configurations

- **PlatformIO:**  
  https://platformio.org/

---

> **Build philosophy:** establish a stable hardware + firmware baseline first. Add Input Shaping, Linear Advance and speed optimisation only after the upgraded printer is printing reliably.
