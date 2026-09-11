# Ender 3 V2 — Custom mriscoc Professional Firmware

> **Tailored build:** Ender 3 V2 · Creality V4.2.2 · STM32F103RET6 · DWIN display · Sprite Extruder Pro · CR Touch · UBL · T13 · MPC

This README documents the **complete Windows workflow** used to create a current custom **mriscoc Professional Firmware** image for this specific Ender 3 V2, including Python installation, Windows setup, the mriscoc Configurator, VS Code, PlatformIO, compilation, SD-card preparation, flashing, and first-start checks.

---

## 1. Exact hardware profile

| Component | This printer |
|---|---|
| Printer | **Creality Ender 3 V2** |
| Mainboard | **Creality V4.2.2** |
| MCU | **STM32F103RET6** |
| Build target | **STM32F103RE_creality (512K)** |
| Display | **DWIN** |
| Extruder / hot end | **Sprite Extruder Pro** |
| Probe | **CR Touch** |
| Levelling | **BLT + UBL** |
| Thermistor configuration | **T13 / Marlin #13 (3950)** |
| Hot-end thermal control | **MPC** |
| Input Shaping | Not enabled initially |
| Linear Advance | Not enabled initially |

### Target firmware configuration

```text
Ender3V2-422-BLTUBL-T13-MPC
```

---

# 2. Safety before starting

> [!WARNING]
> **Do not flash or use this T13 firmware while the original Ender 3 V2 hot end is still fitted.**
>
> This firmware build is intended for the **Sprite Extruder Pro / compatible all-metal hot-end and thermistor arrangement**.

Before working on the printer:

- Turn the printer off.
- Unplug mains power.
- Do not disconnect or reconnect motherboard wiring while powered.
- Verify heater and thermistor wiring before first power-up.
- Do not immediately command 300 °C after installation.
- Verify temperature behaviour progressively at normal temperatures first.

---

# 3. Windows software required

You will install:

1. **Python 3**
2. **Visual Studio Code**
3. VS Code extension: **PlatformIO IDE**
4. VS Code extension: **Auto Build Marlin**

You will also download:

- **mriscoc Professional Firmware source**  
  `https://github.com/mriscoc/Ender3V2S1`

- **mriscoc Special Configurations**  
  `https://github.com/mriscoc/Special_Configurations`

---

# 4. Install Python on Windows

Download Python 3 for Windows from:

```text
https://www.python.org/downloads/windows/
```

Run the Python installer.

## Critical installer option

On the first installer screen, **tick**:

```text
☑ Add python.exe to PATH
```

This is important because it lets Windows recognise commands such as:

```cmd
python
```

from Command Prompt.

The option:

```text
Use admin privileges when installing py.exe
```

is **not required** for this workflow.

Then click:

```text
Install Now
```

## After Python finishes installing

On the final installation screen, click:

```text
Disable path length limit
```

This is recommended because PlatformIO and compiler toolchains can create deeply nested folder paths.

Then click:

```text
Close
```

## Verify Python

Open **Command Prompt** and run:

```cmd
python --version
```

You should get a result similar to:

```text
Python 3.x.x
```

If `python` is not recognised:

1. Close all existing Command Prompt windows.
2. Open a new Command Prompt.
3. Try again.
4. If necessary, restart Windows after installing Python.

---

# 5. Download mriscoc Special Configurations

Go to:

```text
https://github.com/mriscoc/Special_Configurations
```

Use:

```text
Code → Download ZIP
```

> [!IMPORTANT]
> Download the **current repository ZIP from the main repository page**.
>
> Do **not** use the old `Source code (zip)` attachment from the historical T13 release page. That release-page source archive is not the workflow being used here.

Extract the ZIP completely.

The extracted folder should contain files and directories similar to:

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

`Configurator.pyw` must remain together with these supporting files and folders.

---

# 6. Run the mriscoc Configurator

If double-clicking `Configurator.pyw` opens the graphical application, continue to the next section.

If Windows opens it as text, or nothing useful happens, run it manually.

## Easy way to open Command Prompt in the correct folder

1. Open the extracted `Special_Configurations` folder in File Explorer.
2. Click the File Explorer **address bar**.
3. Type:

```text
cmd
```

4. Press **Enter**.

A Command Prompt window will open already pointed at that folder.

Run:

```cmd
python Configurator.pyw
```

The window title should be:

```text
Professional Firmware Configurator
```

---

# 7. Exact Configurator selections for this Ender 3 V2

Select:

| Configurator section | Selection |
|---|---|
| Printer | `Ender3V2` |
| Board | `422` |
| Leveling | `BLT` |
| UBL | ✅ Checked |
| Display | `DWIN` |
| Thermistor | `T13` |
| MPC | ✅ Checked |

Leave the other optional feature boxes **unticked initially**, particularly:

```text
IS
LA
LockSteppers
HomeOffs
NP
Repeat&Depth
```

## Why these selections?

```text
Ender3V2 = Ender 3 V2 printer
422      = Creality V4.2.2 motherboard
BLT      = CR Touch / BLTouch support
UBL      = Unified Bed Levelling
DWIN     = this printer's confirmed display type
T13      = Marlin #13 / 3950 thermistor configuration
MPC      = Model Predictive temperature control
```

### DWIN note

The display on this specific Ender 3 V2 was physically checked and identified as **DWIN**, so leave `DWIN` selected.

---

# 8. Generate the configuration files

Click:

```text
Auto
```

The Configuration Name should become:

```text
Ender3V2-422-BLTUBL-T13-MPC
```

The command shown at the bottom should be equivalent to:

```python
CreateConfigs.Generate(
    'Ender3V2-422-BLTUBL-T13-MPC',
    ['Ender3V2', '422', 'BLT', 'UBL', 'T13', 'MPC']
)
```

Then click:

```text
Generate
```

The configurator should create a folder named approximately:

```text
Ender3V2-422-BLTUBL-T13-MPC
```

Inside it should be files including:

```text
Configuration.h
Configuration_adv.h
Version.h
platformio.ini
log.txt
```

> [!NOTE]
> Windows may hide known file extensions. `platformio.ini` may therefore appear simply as:
>
> ```text
> platformio
> ```
>
> with its Type shown as **Configuration settings**.

---

# 9. Download the current mriscoc firmware source

Go to:

```text
https://github.com/mriscoc/Ender3V2S1
```

Use:

```text
Code → Download ZIP
```

Extract it.

For simplicity, use a short Windows path if possible, for example:

```text
C:\mriscoc\
```

Long project paths can sometimes cause trouble for PlatformIO/compiler tooling.

The true **project root** is the folder that directly contains:

```text
Marlin\
buildroot\
ini\
platformio.ini
README.md
```

---

# 10. Copy the generated configuration into the current source

From the generated folder:

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
<mriscoc project root>\Marlin\
```

Choose **Replace** when Windows asks.

Then copy:

```text
platformio.ini
```

into the **project root**:

```text
<mriscoc project root>\
```

Again choose **Replace**.

Do **not** copy:

```text
log.txt
```

## Expected structure

```text
Ender3V2S1\
│
├── platformio.ini              <-- generated replacement
│
├── Marlin\
│   ├── Configuration.h         <-- generated replacement
│   ├── Configuration_adv.h     <-- generated replacement
│   ├── Version.h               <-- generated replacement
│   └── ...
│
├── buildroot\
├── ini\
└── ...
```

---

# 11. Install Visual Studio Code

Download VS Code from:

```text
https://code.visualstudio.com/Download
```

Install it normally.

Launch VS Code.

---

# 12. Install the required VS Code extensions

Open the **Extensions** panel:

```text
Ctrl + Shift + X
```

Install:

```text
PlatformIO IDE
```

and:

```text
Auto Build Marlin
```

Restart/reload VS Code if prompted.

### Recognising the icons

In the VS Code left sidebar:

- **Alien-head icon** = PlatformIO
- **M icon** = Auto Build Marlin

---

# 13. Open the correct project folder in VS Code

Use:

```text
File → Open Folder
```

Open the **project root**, not the `Marlin` subfolder.

The folder you open should directly contain:

```text
Marlin
buildroot
ini
platformio.ini
README.md
```

Do **not** open only:

```text
...\Marlin\
```

PlatformIO needs to see `platformio.ini` in the project root.

If your extracted ZIP created two similarly named nested folders, select the **inner folder that directly contains `platformio.ini`**.

Wait for PlatformIO to finish initialising. The first launch may take a little time while compiler packages and dependencies are installed.

---

# 14. Open Auto Build Marlin

Click the:

```text
M
```

icon on the left sidebar.

Auto Build Marlin should detect the custom project/configuration.

You should see a machine/configuration name corresponding to:

```text
Ender3V2-422-BLTUBL-T13-MPC
```

---

# 15. Confirm the MCU before choosing a build target

This printer was physically checked and has:

```text
STM32F103RET6
```

Therefore the correct Auto Build Marlin target is:

```text
STM32F103RE_creality (512K)
```

## Correct target

```text
STM32F103RE_creality (512K) → Build
```

## Do not use

```text
STM32F103RE_creality_xfer
STM32F103RC_creality
STM32F103RC_creality_xfer
STM32F103RE_creality_maple
```

### Why?

`RET6` corresponds to the `RE` target.

The `RC` targets are for an STM32F103RCT6 MCU.

The `_xfer` targets are not the normal SD-card firmware build.

The `maple` environment is not the normal target for this build.

---

# 16. Compile the firmware

In Auto Build Marlin, click:

```text
Build
```

beside:

```text
STM32F103RE_creality (512K)
```

The first compile can take longer because PlatformIO may download compiler/toolchain components.

A successful build should end with:

```text
SUCCESS
```

The compiled firmware will normally be created at approximately:

```text
.pio\build\STM32F103RE_creality\firmware.bin
```

A firmware size of only a few hundred kilobytes is normal.

---

# 17. Verify the compiled firmware

Expected firmware identity:

```text
Ender3V2-422-BLTUBL-T13-MPC
```

Expected feature meaning:

```text
Ender3V2  = Ender 3 V2
422       = Creality V4.2.2
BLT       = CR Touch / BLTouch
UBL       = Unified Bed Levelling
T13       = thermistor profile
MPC       = Model Predictive Control
```

Keep a copy of the final `firmware.bin`.

A useful renamed archival copy would be:

```text
Ender3V2-422-BLTUBL-T13-MPC.bin
```

Do not necessarily use that long filename for flashing; use a short unique filename on the SD card.

---

# 18. Prepare the microSD card

Format the microSD card as:

```text
FAT32
```

For the firmware flash:

- Put the `.bin` in the **root** of the card.
- Prefer only **one firmware `.bin`** on the card.
- Give the file a short, unique name.

Example:

```text
FW260911.bin
```

Creality bootloaders can ignore a firmware filename that exactly matches a previously flashed filename, so use a different filename on future firmware updates.

You can prepare the card **before the new parts arrive**, but do not boot the printer from it yet.

---

# 19. Hardware that must be installed before flashing this build

Install the relevant hardware first:

- **Sprite Extruder Pro**
- **CR Touch**

The rest of the planned mechanical upgrades can also be fitted at the same time:

- Dual-Z kit
- Silicone bed spacers
- PEI magnetic spring-steel build plate

The **Sprite Pro and CR Touch are the critical firmware-dependent items**.

---

# 20. Flash the firmware

After the Sprite Pro and CR Touch are installed and wired:

1. Turn the printer **off**.
2. Insert the prepared microSD card.
3. Turn the printer **on**.
4. Allow the bootloader to flash the `.bin`.
5. Wait for the printer to start into the mriscoc interface.
6. Confirm the firmware starts normally.
7. Remove the SD card after the flash is complete.

---

# 21. First-start checks

Do **not** immediately start a print.

Check:

- [ ] Display starts correctly
- [ ] mriscoc interface appears
- [ ] CR Touch powers up
- [ ] CR Touch deploys and retracts
- [ ] Room-temperature hot-end reading looks plausible
- [ ] Bed temperature looks plausible
- [ ] X movement is correct
- [ ] Y movement is correct
- [ ] Z movement is correct
- [ ] Both Z motors move smoothly
- [ ] Sprite extruder motor direction is correct
- [ ] Fans operate
- [ ] Toolhead does not hit the frame
- [ ] Homing behaves safely

> [!CAUTION]
> Keep a hand near the power switch during the first homing test. If Z direction, probe operation, or toolhead movement is wrong, switch the printer off immediately.

---

# 22. Calibration after the rebuild

Perform calibration in this general order.

## 1. Square the X gantry

With dual-Z fitted, make sure the X gantry is mechanically level/square.

## 2. Tram the bed mechanically

Use the silicone spacers and adjustment wheels to make the bed reasonably parallel to the gantry.

## 3. Configure CR Touch X/Y probe offsets

The Sprite Pro changes the physical probe position relative to the nozzle.

Configure the correct:

```text
X probe offset
Y probe offset
```

## 4. Set Z-offset

Use mriscoc's Z-offset tools to set the correct nozzle-to-bed relationship.

## 5. Generate a UBL mesh

Probe the bed and save the UBL mesh.

## 6. Calibrate the Sprite extruder

Check/calibrate extrusion steps.

Do not assume the old Ender 3 V2 stock extruder value is correct.

## 7. Run MPC tuning

Run the mriscoc hot-end MPC calibration for the Sprite Pro.

This lets the firmware characterise the new heater and hot-end thermal response.

## 8. Verify temperatures progressively

Start conservatively, for example:

```text
200 °C
```

Then verify around:

```text
230-250 °C
```

Only consider higher temperatures after normal-temperature operation is confirmed stable and accurate.

---

# 23. Input Shaping and Linear Advance later

For the first build, leave:

```text
IS
LA
```

disabled.

This keeps the initial troubleshooting baseline simple.

Once the rebuilt printer works reliably, these can be added later by recompiling the firmware.

## Input Shaping (`IS`)

Helps reduce ringing/ghosting when using higher acceleration.

## Linear Advance (`LA`)

Improves extrusion behaviour during acceleration and deceleration.

### To add either later

1. Open `Configurator.pyw`.
2. Re-select the existing configuration.
3. Tick `IS` and/or `LA`.
4. Click **Auto**.
5. Click **Generate**.
6. Copy the generated config files again.
7. Recompile.
8. Flash with a new unique `.bin` filename.
9. Calibrate the added feature.

No major mechanical rebuild is required merely to add these firmware features.

---

# 24. Final build reference

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

# 25. Recommended firmware archive

Keep the firmware and configuration files together.

Example:

```text
Ender3V2-Firmware\
│
├── README.md
│
├── firmware\
│   └── Ender3V2-422-BLTUBL-T13-MPC.bin
│
├── config\
│   ├── Configuration.h
│   ├── Configuration_adv.h
│   ├── Version.h
│   └── platformio.ini
│
└── notes\
    └── calibration-values.md
```

After final calibration, record values such as:

```text
X probe offset:
Y probe offset:
Z offset:
Extruder steps:
MPC values:
UBL mesh saved:
Nozzle size:
Firmware build date:
```

This makes future firmware rebuilds and troubleshooting substantially easier.

---

# Useful links

**mriscoc Professional Firmware**

```text
https://github.com/mriscoc/Ender3V2S1
```

**mriscoc Special Configurations**

```text
https://github.com/mriscoc/Special_Configurations
```

**Python for Windows**

```text
https://www.python.org/downloads/windows/
```

**Visual Studio Code**

```text
https://code.visualstudio.com/Download
```

**PlatformIO**

```text
https://platformio.org/
```

---

> **Recommended approach:** first establish a reliable Sprite Pro + CR Touch + UBL + T13 + MPC baseline. Only after the machine is mechanically and thermally stable should Input Shaping, Linear Advance, and higher-speed tuning be added.
