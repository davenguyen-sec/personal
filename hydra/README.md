# Hydroponics Controller

> Automated hydroponics control using an **Arduino Mega** for sensing and dosing, with an **ESP8266 / NodeMCU v2** bridge for MQTT and Home Assistant telemetry.

---

## Overview

This system has two main parts:

### Arduino Mega
- Reads **water temperature**, **pH**, and **EC**
- Runs the actual control logic
- Gives **EC correction priority over pH**
- Commands four I²C pump controllers
- Drives a 20×4 LCD
- Sends CSV telemetry over `Serial1`

### ESP8266 / NodeMCU v2
- Receives CSV telemetry from the Mega
- Connects to Wi-Fi
- Publishes retained MQTT topics
- Publishes Home Assistant MQTT auto-discovery configuration

> [!IMPORTANT]
> The **Arduino Mega is the controller**.  
> The ESP8266 is only the network/telemetry bridge. Loss of Wi-Fi, MQTT, or Home Assistant does not itself stop the Mega's local sensing and dosing logic.

---

## System Architecture

```text
pH probe ───────────────┐
DS18B20 temperature ────┼──> Arduino Mega ──I²C──> pH Down pump
Atlas EZO-EC ───────────┤                  ├──────> Nutrient A pump
                        │                  ├──────> Nutrient B pump
                        │                  └──────> EC Down pump
                        │
                        ├──> 20×4 LCD
                        │
                        └──Serial1 @ 115200──> ESP8266
                                                │
                                                └──Wi-Fi / MQTT
                                                     │
                                                     └──Home Assistant
```

---

## Hardware Map

| Function | Connection / Address |
|---|---:|
| pH analogue input | `A5` |
| DS18B20 1-Wire | Digital pin `4` |
| pH Down pump | `0x32` |
| Nutrient A pump | `0x33` |
| Nutrient B pump | `0x34` |
| EC Down pump | `0x35` |
| Atlas EZO-EC | `0x64` |
| 20×4 LCD | `0x27` |
| Mega USB debug serial | `9600 baud` |
| Mega → ESP8266 telemetry | `115200 baud` |

> [!CAUTION]
> The Mega TX1 output is **5 V logic** while the ESP8266 RX0 input is **3.3 V only**.  
> Use a voltage divider or proper logic-level shifter, and ensure both boards share a common ground.

---

## Key Control Settings

### pH

| Parameter | Value |
|---|---:|
| Setpoint | `5.80` |
| Deadband | `0.40` |
| Actual pH-down trigger | `ph_control > 6.20` |
| Dose per action | `4.0 mL` |
| Lockout | `30 min` |
| Hourly cap | `12.0 mL` |
| EMA alpha | `0.20` |

### EC

| Parameter | Value |
|---|---:|
| Setpoint | `1000 µS/cm` |
| Deadband | `±50 µS/cm` |
| Accepted range | `950–1050 µS/cm` |
| Total EC-up dose | `8.0 mL` |
| A:B ratio | `1:1` |
| Nutrient A | `4.0 mL` |
| Nutrient B | `4.0 mL` |
| A → B delay | `2 min` |
| EC-up mix lockout | `10 min` |
| A hourly cap | `40 mL` |
| B hourly cap | `40 mL` |
| EC-down dose | `0.0 mL` |

> [!NOTE]
> Automatic EC-down is currently **disabled** because `EC_DOWN_DOSE_ML = 0.0`.

---

## Main Control Cycle

The Mega performs one full sensor/control cycle every **10 seconds**.

```text
1. Read water temperature
2. Send "Sleep" to EZO-EC
3. Wait 500 ms
4. Read analogue pH
5. Mark EC cycle invalid
6. Send "R" to EZO-EC
7. Wait 600 ms
8. Read EC response
9. Send "Status" to EZO-EC
10. Run control logic
11. Print debug snapshot
12. Send CSV telemetry to ESP8266
```

The EC circuit is deliberately quieted before reading pH to reduce electrical interference with the analogue pH measurement.

---

## pH Measurement and Filtering

The Mega converts the analogue value using:

```text
voltage_mV = analogRead(A5) / 1024 × 5000 mV
```

The DFRobot pH library then calculates pH using:
- probe voltage
- water temperature

Two pH values are retained:

- `ph_raw` — instantaneous probe reading
- `ph_control` — EMA-smoothed pH used for control

The EMA is:

```text
new ph_control =
    0.20 × new ph_raw
  + 0.80 × previous ph_control
```

### Practical effect

A single high raw pH sample usually does **not** cause an immediate dose.

Example:

```text
Previous ph_control = 5.80
New ph_raw          = 6.10

ph_control =
0.20 × 6.10 + 0.80 × 5.80
= 5.86
```

Result: **no pH-down dose yet**.

---

## Actual pH Dosing Threshold

Configured values:

```text
PH_SETPOINT = 5.80
PH_DEADBAND = 0.40
```

The controller has a pH-down pump only.

The actual trigger is therefore:

```text
ph_control > 5.80 + 0.40
ph_control > 6.20
```

At exactly `6.20`, the controller does **not** dose because the comparison is strict `>` rather than `>=`.

> [!IMPORTANT]
> `6.20` is the **upper dosing threshold**, not the configured pH setpoint.

There is no pH-up action below:

```text
5.80 - 0.40 = 5.40
```

So this is not a symmetric two-sided pH controller.

---

## Conditions Required for pH-Down

A pH-down dose requires **all** of the following:

1. Latest EC cycle is valid
2. EC is inside `950–1050 µS/cm`
3. No Nutrient B dose is pending
4. Previous EC correction has finished settling
5. `ph_control` is valid and within `0–14`
6. `ph_control > 6.20`
7. 30-minute pH lockout has expired
8. pH-down hourly total is below `12.0 mL`
9. Remaining hourly allowance is positive
10. The pH-down pump acknowledges the I²C command

A successful action normally commands:

```text
4.0 mL pH-down
```

Then the controller:
- starts the 30-minute lockout
- adds `4.0 mL` to the pH-down hourly total
- updates `LAST = pHD`

---

## EC-First Control Logic

The controller calculates:

```text
ec_error = EC_SETPOINT - ec_value
```

Meaning:

| EC error | Meaning |
|---|---|
| Positive | EC is too low |
| Negative | EC is too high |
| Zero | Exactly at setpoint |

EC is considered in range when:

```text
abs(ec_error) <= 50
```

Therefore:

```text
950 <= EC <= 1050 µS/cm
```

If EC is outside this range, or Nutrient B is pending, the controller exits before reaching pH control.

> [!IMPORTANT]
> pH correction is intentionally deferred until EC is stable.

---

## Low EC: Nutrient A/B Sequence

When:

```text
EC < 950 µS/cm
```

the controller attempts EC-up.

Normal dose split:

```text
Total = 8.0 mL
A     = 4.0 mL
B     = 4.0 mL
```

### Sequence

```text
1. Send D,4.00 to Nutrient A
2. If A is acknowledged:
   - Add 4 mL to A hourly total
   - Schedule B
   - Set B volume to 4 mL
   - Set B ready time = now + 2 min
   - Start EC-up cycle timer

3. After 2 min:
   - If latest EC state is valid
   - and B hourly cap allows
   - send B

4. If B succeeds:
   - Add 4 mL to B hourly total
   - Clear pending_B

5. If B fails:
   - Keep B pending
   - Retry after 5 s
```

B is only scheduled if A was successfully acknowledged.

---

## Why A and B Are Staggered

The controller deliberately waits:

```text
120 seconds
```

between A and B.

This gives Nutrient A time to disperse before Nutrient B is introduced, reducing the chance of concentrated interaction between the two nutrient parts.

The overall EC-up settling period is:

```text
10 minutes
```

measured from the accepted A command.

---

## Pending B Expiry

A pending B dose is cancelled if:

```text
now - accepted_A_time >= 10 minutes
```

The controller logs:

```text
Pending B dose expired and was cancelled.
```

Practical consequence:
- A may have been added
- B may never be delivered
- the software does not reverse or compensate for that unmatched A dose

---

## High EC Behaviour

When:

```text
EC > 1050 µS/cm
```

the controller enters the EC-down branch.

However:

```cpp
EC_DOWN_DOSE_ML = 0.0;
```

so no EC-down pump command occurs.

> [!WARNING]
> High EC can block automatic pH-down indefinitely because EC has priority over pH, while EC-down is currently disabled.

Once EC is manually diluted back into range, pH control can resume after the relevant settling logic permits it.

---

## EC Settling Before pH Control

The controller remembers the latest EC correction direction:

```text
+1 = EC-up
-1 = EC-down
 0 = no previous EC correction
```

Required settling time:

| Previous EC action | Settle time |
|---|---:|
| EC-up | `10 min` |
| EC-down | `10 min` |
| None | `0 min` |

pH is therefore prevented from reacting while the reservoir is expected to still be mixing.

---

## Pump Command Accounting

Each dosing command is sent with up to:

```text
3 I²C attempts
```

The controller only counts a dose when `Wire.endTransmission()` returns success.

Only then does it update:
- dose totals
- lockout timestamps
- pending A/B state
- `last_action`

> [!CAUTION]
> An I²C acknowledgement only proves that the pump controller received the command.

It does **not** prove:
- the motor turned
- liquid flowed
- the requested mL was delivered
- the tube was connected
- the tank contained chemical
- the pump calibration was accurate

All dose totals are **commanded/acknowledged volume**, not measured physical flow.

---

## EC Validity and Fail-Safe Behaviour

The EC handling has been updated so that a **genuine `0 µS/cm` reading remains valid**, while communication or parsing failures are treated as invalid.

### Why this matters

A real EC value of zero can legitimately occur:
- at startup
- with very low-conductivity water
- before nutrients have been added

Therefore the controller must not use:

```text
EC > 0
```

as a validity test.

Instead, it separates:

```text
ec_value
```

from:

```text
ec_read_valid
```

### Current behaviour

A fresh, valid EZO response containing:

```text
0
```

produces:

```text
ec_value      = 0
ec_read_valid = true
```

This means the controller can correctly recognise that EC is far below the `1000 µS/cm` setpoint and begin EC-up dosing once the normal timing and cap conditions allow it.

By contrast, an invalid transaction now returns:

```text
NAN
```

rather than `0.0`.

Examples include:
- no bytes returned by `Wire.requestFrom()`
- EZO response status other than `1`
- empty response payload
- malformed/non-numeric payload
- non-finite parsed value

Those cases result in:

```text
ec_read_valid = false
```

and the controller inhibits all new dosing for that sensor cycle.

### Startup behaviour

At startup:

```text
ec_value      = 0
ec_read_valid = false
```

The displayed value may therefore be zero before the first valid EC measurement, but this startup zero cannot authorise dosing because the validity flag is false.

After the first successful EC read:

```text
ec_read_valid = true
```

and the returned EC value becomes eligible for control decisions.

### Stale EC display behaviour

If a later EC read fails after at least one successful reading:

```text
ec_value
```

retains the last good EC value for LCD and telemetry continuity.

However:

```text
ec_read_valid = false
```

so that stale EC value cannot authorise:
- EC-up
- EC-down
- pH-down

because EC validity gates the entire control loop.

### Practical result

The controller now distinguishes these states correctly:

| Situation | `ec_value` | `ec_read_valid` | Dosing allowed? |
|---|---:|---|---|
| Startup, no valid EC yet | `0` | `false` | No |
| Genuine fresh EC = `0` | `0` | `true` | Yes, subject to normal control rules |
| Valid EC = `980` | `980` | `true` | Yes |
| Communication/parsing failure after good read | Last good EC | `false` | No |

This preserves automatic nutrient build-up from very low EC while preventing failed EC transactions from masquerading as a low-EC condition.

---

## EC Response Parsing

The EZO numeric response parser now uses `strtod()` rather than relying solely on `atof()`.

This allows the code to distinguish:

```text
"0"
```

from malformed or empty text.

The parser accepts:
- a normal numeric EC value
- a valid first numeric EC field followed by a comma-separated EZO field

It rejects:
- empty payloads
- non-numeric text
- non-finite values
- unexpected trailing text

Invalid responses return:

```text
NAN
```

and therefore cannot authorise dosing.

---

## EC Temperature Compensation

At startup the Mega sends:

```text
T,25.0
```

to the Atlas EZO-EC, then checks:

```text
T,?
```

The Mega also performs software compensation:

```text
compensated EC =
measured EC /
(1 + 0.02 × (water_temperature - 25))
```

Design intent appears to be:
- pin the EZO at `25 °C`
- perform temperature compensation in the Mega

Verify that no other component is also applying compensation, otherwise EC may be double-compensated.

---

## Startup Behaviour

Several dose timestamps initialise to zero.

Because lockouts are tested against `millis()`, a reboot effectively creates an initial no-dose period.

Approximate startup delays:

| Action | First eligible time after reboot |
|---|---:|
| EC-up | `~10 min` |
| EC-down | `~10 min` |
| pH-down | `~30 min` |

This happens even if no dose occurred before restart.

---

## Hourly Dose Caps

The controller maintains separate one-hour accounting windows for:

- Nutrient A
- Nutrient B
- EC-down
- pH-down

These are **not rolling 60-minute windows**.

They reset once:

```text
3,600,000 ms
```

has elapsed since the relevant channel's current window start.

Example:

```text
Window resets at 14:17
Next reset occurs around 15:17
```

A reboot clears these counters because they are stored only in RAM.

---

## Current pH Rate Limit

Configured:

```text
Dose         = 4.0 mL
Lockout      = 30 min
Hourly cap   = 12.0 mL
```

Practical maximum:
- normally no more than one accepted pH dose every 30 minutes
- the software cap allows up to 12 mL in one accounting window
- because each dose is 4 mL, the cap corresponds to at most three accepted pH-down doses per accounting window
- the 30-minute lockout usually limits the practical rate before the 12 mL cap does

Actual dosing still depends on EC state, EMA pH, and all other interlocks.

---

## Current EC Rate Limit

Each normal EC-up cycle:

```text
4 mL A
4 mL B
```

Hourly caps:

```text
40 mL A
40 mL B
```

The caps alone would allow up to ten complete A/B cycles per window.

In practice, the `10 min` EC lockout is usually more restrictive than the volume caps.

---

## LCD Pages

The LCD is:

```text
20 columns × 4 rows
```

Timing:

| Function | Interval |
|---|---:|
| Page rotation | `3 s` |
| In-page refresh | `1.5 s` |
| LCD re-init | `5 min` |

### Page A — Live Overview

```text
pHc:<filtered> pHr:<raw>
pH SP:<setpoint>
EC:<current> SP:<setpoint>
T:<temperature> C
```

### Page B — Hourly Totals

```text
A:     current/cap
B:     current/cap
EC Dn: current/cap
pH Dn: current/cap
```

### Page C — Status

```text
Last: <action> <mL>


Uptime: <hours>
```

---

## Last Action

Possible values:

```text
A
B
ECD
pHD
None
```

`last_action_ml` records the volume of the most recently acknowledged pump command.

Example:

```text
After A:
LAST = A
LAST_ML = 4.0

After B:
LAST = B
LAST_ML = 4.0
```

It does **not** represent the total A+B cycle volume.

---

## Mega → ESP8266 Telemetry

The Mega sends one key/value CSV line every sensor cycle.

Keys:

```text
PH_RAW
PH_C
EC
T
PH_SP
EC_SP
PH_ERR
EC_ERR
A_HR
B_HR
ECD_HR
PHD_HR
LAST
LAST_ML
UPTIME_MS
```

Example:

```text
PH_RAW,5.92,PH_C,5.88,EC,1580,T,20.4,PH_SP,5.8,...
```

---

## MQTT Topics

### Sensor States

```text
hydro/pH
hydro/ph_filt
hydro/ec_uScm
hydro/temp_C
```

### Setpoints

```text
hydro/ph_sp
hydro/ec_sp_uScm
```

### Error Terms

```text
hydro/ph_err
hydro/ec_err_uScm
```

### Dose Totals

```text
hydro/dose_a_ml_hr
hydro/dose_b_ml_hr
hydro/dose_ec_down_ml_hr
hydro/dose_ph_down_ml_hr
```

### Status

```text
hydro/last_action
hydro/last_action_ml
hydro/uptime_ms
hydro/status
```

### Diagnostics

```text
hydro/raw
```

---

## Home Assistant

The ESP8266 publishes MQTT auto-discovery under:

```text
homeassistant/...
```

Declared device:

| Field | Value |
|---|---|
| Name | `Hydroponics Controller` |
| Manufacturer | `DIY` |
| Model | `Mega+ESP8266` |
| Software version | `1.3` |
| Identifier | `hydro_controller_001` |

Discovery is:
- published after MQTT connection
- republished every `5 min`

---

## MQTT Availability

When connected:

```text
hydro/status = online
```

The MQTT Last Will publishes:

```text
hydro/status = offline
```

if the broker detects an ungraceful ESP8266 disconnect.

Use `hydro/status` to distinguish live telemetry from retained stale state.

---

## Retained MQTT Behaviour

State topics are retained.

That means Home Assistant immediately receives the latest published value after reconnecting.

This is useful, but retained values may appear valid even when the bridge has stopped updating.

Always consider:

```text
hydro/status
```

alongside retained sensor data.

---

## ESP8266 CSV Parser Behaviour

The bridge:
1. splits the line on commas
2. processes alternating `KEY,VALUE` pairs
3. ignores unknown keys
4. skips numeric fields containing `NaN`

If a `NaN` value is skipped, the previous retained MQTT value remains.

So a failed sensor value does not automatically clear its Home Assistant entity.

---

## Important Telemetry Limitation

The Mega does **not** send `ec_read_valid`.

Therefore Home Assistant cannot distinguish:
- fresh EC
- stale held-last-good EC

A future telemetry extension could include:

```text
EC_VALID
EC_AGE
PENDING_B
CONTROL_STATE
PH_LOCKOUT_REMAINING
EC_LOCKOUT_REMAINING
```

---

## Wi-Fi / MQTT Failure Behaviour

The ESP8266 reconnect routines are blocking.

If Wi-Fi or MQTT fails:
- the Mega continues local control
- Home Assistant telemetry stops updating
- MQTT availability should eventually go offline
- UART data may not be preserved during long network outages

The ESP8266 is not a store-and-forward logger.

---

## ESP8266 Reboot

An ESP8266 reboot does not inherently reset Mega control state.

After reconnecting, it:
- publishes `online`
- republishes discovery
- resumes forwarding new Mega telemetry

---

## Mega Reboot

A Mega reboot clears volatile state including:

```text
ph_control
last dose timestamps
hourly dose totals
pending_B
last_action
ec_last_good
```

The supplied code includes `EEPROM.h`, but does not actually persist any of this state.

---

## Watchdog

On AVR:

```text
WDTO_4S
```

The watchdog is reset:
- in the main loop
- during `safe_delay()`

A watchdog trip performs a full controller reset and therefore clears volatile state.

---

## `millis()` Wraparound

Timing comparisons use the standard unsigned subtraction pattern:

```cpp
now - previous >= interval
```

This is generally safe across `millis()` wraparound.

However, displayed uptime is calculated directly from:

```cpp
millis() / 3600000.0
```

so displayed uptime wraps after roughly **49.7 days**.

---

## Practical Examples

### pH = 6.00, EC = 1000

```text
pH is above setpoint
but below 6.20 trigger
```

Result:

```text
No pH dose
```

### pH = 6.21, EC = 1000

Assuming all interlocks are satisfied:

```text
Command 4.0 mL pH-down
```

### EC = 900

```text
EC error = +100
```

Result:

```text
4 mL A
then 4 mL B after 2 min
```

### EC = 1070

```text
EC error = -70
```

Result:

```text
Controller wants EC-down
but EC-down dose = 0
```

Therefore:
- no water dose occurs
- pH control remains blocked

### EC = 950

```text
Error = +50
```

Result:

```text
In range
```

### EC = 1050

```text
Error = -50
```

Result:

```text
In range
```

### pH = exactly 6.20

Result:

```text
No pH dose
```

because the code requires the error magnitude to be strictly greater than the deadband.

---

## Troubleshooting: Why Isn't It Dosing?

Check these in order:

1. **Has the Mega just rebooted?**
   - EC-up waits about `10 min`
   - pH-down waits about `30 min`

2. **Is EC above 1050?**
   - EC-down is disabled
   - pH is blocked

3. **Is EC below 950 but inside its EC lockout?**

4. **Is Nutrient B pending?**

5. **Is the EC settle period still active?**

6. **Is raw pH high but `PH_C <= 6.20`?**

7. **Is the 30-minute pH lockout active?**

8. **Has the pH hourly cap reached 12 mL?**

9. **Has an EC nutrient cap been reached?**

10. **Did the pump I²C command fail?**

11. **Is the latest EC cycle invalid?**
    - invalid EC cycles inhibit new dosing until a fresh valid reading is obtained

---

## Tuning Guide

### pH Parameters

Tune these together:

```text
PH_DOWN_DOSE_ML
PH_DOWN_MIX_LOCKOUT_MS
PH_DEADBAND
PH_CONTROL_EMA
PH_DOWN_MAX_ML_PER_HR
```

General effect:

| Change | Result |
|---|---|
| Larger dose | Stronger correction |
| Longer lockout | More mixing time |
| Wider deadband | Fewer corrections |
| Higher EMA alpha | Faster response |
| Lower EMA alpha | More smoothing |

### EC Parameters

Tune these together:

```text
EC_UP_TOTAL_DOSE_ML
EC_A_TO_B_RATIO
EC_AB_STAGGER_MS
EC_UP_MIX_LOCKOUT_MS
EC_DEADBAND
EC_A_MAX_ML_PER_HR
EC_B_MAX_ML_PER_HR
```

Actual values depend heavily on:
- reservoir volume
- circulation rate
- nutrient concentration
- plant uptake
- top-up water
- pump calibration

---

## Reliability Limitations

The controller does not directly verify:

- actual liquid flow
- pump stall
- tube disconnection
- siphoning
- leaks
- chemical container level
- reservoir level
- actual delivered mL
- pH calibration quality
- EC calibration quality

The system is closed-loop through later pH/EC measurements, but pump delivery itself is not independently measured.

---

## Operational Checklist

### Before unattended operation

- [ ] Calibrate pH probe
- [ ] Calibrate EC probe
- [ ] Calibrate each dosing pump in mL
- [ ] Confirm A/B pump addresses
- [ ] Confirm pH-down pump address
- [ ] Confirm tubing destinations
- [ ] Verify no siphoning
- [ ] Verify strong reservoir mixing
- [ ] Verify commanded volume matches physical dose
- [ ] Verify EC communication failures are reported as invalid rather than as numeric zero

### During operation

- [ ] Watch `PH_C`, not only `PH_RAW`
- [ ] Check EC is within `950–1050 µS/cm` before expecting pH dosing
- [ ] Check A/B/pH hourly totals when diagnosing lockouts
- [ ] Use `LAST` and `LAST_ML` to see the last acknowledged pump action
- [ ] Use `hydro/status` to identify stale MQTT data
- [ ] Periodically compare software dose totals with actual chemical usage

---

## Quick Reference

```text
pH setpoint              5.80
pH-down trigger          > 6.20
pH dose                  4.0 mL
pH lockout               30 min
pH hourly cap            12.0 mL
pH EMA alpha             0.20

EC setpoint              1000 µS/cm
EC range                 950–1050 µS/cm
EC-up total              8.0 mL
Nutrient A               4.0 mL
Nutrient B               4.0 mL
A → B delay              2 min
EC-up lockout            10 min
A hourly cap             40 mL
B hourly cap             40 mL
Automatic EC-down        Disabled
EC failure sentinel        NAN
EC validity gate          Required for all new dosing

Sensor cycle             10 s
LCD page rotation        3 s
LCD refresh              1.5 s
LCD re-init              5 min
Mega watchdog            4 s
HA discovery refresh     5 min
MQTT                     TCP 1883
```

---

## EC Safety Status

> [!NOTE]
> The previous EC-validity ambiguity has been corrected.

The current implementation:
- accepts a genuine `0 µS/cm` reading as valid
- returns `NAN` for EC transport/parsing failures
- rejects non-finite EC values
- keeps stale EC values for display only
- requires `ec_read_valid == true` before any new dosing can occur

This is the intended fail-safe behaviour.

---

## Control Philosophy

This controller is deliberately conservative.

It is **not PID control**.

It uses:

```text
measure
→ decide
→ command a fixed dose
→ wait for mixing
→ measure again
```

Control is based on:
- deadbands
- fixed-volume dosing
- long settling periods
- hourly command limits
- EMA-filtered pH
- EC priority over pH

---

## One-Sentence System Model

> The Mega measures temperature, pH and EC, gives EC correction priority, applies fixed-volume doses only when safety/timing rules permit, waits for mixing before further correction, and sends telemetry through the ESP8266 to MQTT and Home Assistant.
