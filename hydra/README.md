HYDRA HYDROPONICS CONTROLLER
Practical Operating README
========================================

PURPOSE
-------
This system is an automated hydroponics controller built around:

1. Arduino Mega
   - Reads water temperature, pH and EC.
   - Applies the dosing logic.
   - Commands four I2C pump controllers.
   - Drives a 20x4 LCD.
   - Sends one CSV telemetry line to the ESP8266 every sensor cycle.

2. ESP8266 / NodeMCU v2
   - Receives the Mega's CSV telemetry over UART.
   - Connects to Wi-Fi.
   - Publishes retained MQTT topics.
   - Creates Home Assistant entities using MQTT auto-discovery.

The Arduino Mega is the actual controller. The ESP8266 is a telemetry/network
bridge. Loss of Wi-Fi, MQTT or Home Assistant does not itself stop the Mega's
local sensor reading or dosing logic.


======================================================================
1. HARDWARE MAP
======================================================================

Arduino Mega inputs
-------------------
pH analogue input:
  PH_ANALOG_PIN = A5

Water temperature:
  DS18B20_PIN = digital pin 4

I2C devices
-----------
pH-down pump:       0x32
Nutrient-A pump:    0x33
Nutrient-B pump:    0x34
EC-down pump:       0x35
Atlas EZO-EC:       0x64
20x4 LCD:           0x27

Serial links
------------
Mega USB/debug serial:
  Serial = 9600 baud

Mega -> ESP8266 telemetry:
  Mega Serial1 = 115200 baud
  ESP8266 Serial/RX0 = 115200 baud

IMPORTANT ELECTRICAL DETAIL:
The Mega TX1 output is 5 V logic and the ESP8266 RX0 input is 3.3 V only.
Use a suitable voltage divider or logic-level shifter between Mega TX1 and
ESP8266 RX0. The two boards must share a common ground.


======================================================================
2. MAIN CONTROL SETTINGS
======================================================================

Sensor/control cycle:
  10 seconds

pH
--
Nominal pH setpoint:
  5.80

pH deadband:
  0.15 pH

pH-down dose:
  1.0 mL per accepted command

pH-down lockout:
  30 minutes after an accepted pH-down command

pH-down hourly cap:
  2.0 mL per controller hourly accounting window

pH control filtering:
  Exponential moving average, alpha = 0.20

EC
--
Nominal EC setpoint:
  1600 uS/cm

EC deadband:
  +/- 50 uS/cm

Therefore the controller considers EC in range from:
  1550 to 1650 uS/cm inclusive

EC-up action:
  8.0 mL total nutrient solution command per cycle

A:B ratio:
  1:1

Therefore each normal EC-up cycle is:
  Nutrient A = 4.0 mL
  Nutrient B = 4.0 mL

A-to-B delay:
  2 minutes

EC-up mix/settle lockout:
  10 minutes from the accepted Nutrient-A command

Maximum A per hourly window:
  40 mL

Maximum B per hourly window:
  40 mL

EC-down dose:
  0.0 mL

This means AUTOMATIC EC-DOWN IS CURRENTLY DISABLED.

EC-down hourly cap:
  100 mL

The 100 mL cap has no practical effect while EC_DOWN_DOSE_ML remains 0.0.


======================================================================
3. WHAT HAPPENS EVERY 10 SECONDS
======================================================================

The Mega runs this sequence:

1. Read water temperature.

2. Send "Sleep" to the Atlas EZO-EC circuit.

3. Wait 500 ms.

4. Read the analogue pH probe.

5. Mark the current EC cycle invalid before requesting a new EC value.

6. Send "R" to the EZO-EC probe.

7. Wait 600 ms.

8. Read the EC response if the "R" command was acknowledged.

9. Send "Status" to the EZO-EC circuit to keep it awake.

10. Run the dosing control loop.

11. Print a debug snapshot to the USB serial port.

12. Send one CSV telemetry line to the ESP8266.

The EC board is intentionally quieted before the analogue pH reading to reduce
electrical interference between the EC circuitry and pH measurement.


======================================================================
4. TEMPERATURE HANDLING
======================================================================

The DS18B20 is requested once per sensor cycle.

Accepted range:
  -50 C to +125 C

If the returned temperature is outside this range, the controller rejects the
new reading and keeps the previous water_temperature value.

Practical consequence:
A disconnected or failed temperature probe may leave the controller operating
with an old temperature value rather than clearly switching temperature to an
invalid state.

The water temperature is used for:
  - DFRobot pH compensation/library calculation.
  - Software EC compensation.


======================================================================
5. pH MEASUREMENT AND FILTERING
======================================================================

Analogue conversion
-------------------
The Mega reads A5 and calculates:

  voltage_mV = analogRead(A5) / 1024 * 5000 mV

The code assumes a 5.0 V ADC reference.

The DFRobot pH library then calculates pH using:
  - measured pH-board voltage
  - water temperature

Two pH values are retained:

  ph_raw
    Instantaneous pH reading.

  ph_control
    EMA-smoothed pH used by the dosing logic.

EMA equation
------------
With PH_CONTROL_EMA = 0.20:

  new ph_control =
      0.20 * new ph_raw
    + 0.80 * previous ph_control

At startup, the first valid calculation simply seeds ph_control with ph_raw.

Practical meaning
-----------------
The controller does not normally dose from a single raw pH sample.

A persistent change gradually pulls ph_control toward the raw reading.

Higher alpha:
  - responds faster
  - follows noise more closely

Lower alpha:
  - responds more slowly
  - smooths noise more heavily


======================================================================
6. THE ACTUAL pH DOSING THRESHOLD
======================================================================

The nominal pH setpoint is:
  5.80

The deadband is:
  0.15

The controller only owns a pH-DOWN pump.

The code doses only when:

  ph_control > 5.80 + 0.15

Therefore the practical pH-down trigger is:

  ph_control > 5.95

At exactly 5.95, the controller does NOT dose because the code uses a strict
"> deadband" comparison.

Important:
5.95 is the upper dosing threshold, not the configured pH setpoint.

The lower side:
  5.80 - 0.15 = 5.65

has no active correction because there is no pH-up pump.

Therefore this is not a conventional symmetric two-sided pH controller.

Practical pH behaviour:
  <= 5.95:
    no pH-down dose

  > 5.95:
    pH-down may be requested, but only if all other interlocks allow it


======================================================================
7. CONDITIONS REQUIRED BEFORE pH-DOWN CAN OCCUR
======================================================================

A pH-down dose requires ALL of the following:

1. Latest EC cycle is considered valid.

2. EC is presently inside its accepted band:
     1550 to 1650 uS/cm

3. No Nutrient-B dose is pending.

4. The previous EC correction has completed its required settling period.

5. ph_control is valid:
     0.0 <= pH <= 14.0
   and is not NaN.

6. ph_control is above 5.95.

7. At least 30 minutes have elapsed since the last successfully acknowledged
   pH-down command.

8. The pH-down hourly total is below 2.0 mL.

9. The remaining hourly allowance is greater than zero.

10. The pH-down pump's I2C dosing command is acknowledged.

Normal successful pH-down action:
  1.0 mL

After that acknowledgement:
  - last pH dose time is updated
  - the 30-minute lockout starts
  - hourly pH-down total increases by 1.0 mL
  - LCD/telemetry "last action" becomes pHD


======================================================================
8. EC CONTROL HAS PRIORITY OVER pH
======================================================================

The controller is deliberately EC-first.

It calculates:

  ec_error = EC_SETPOINT - ec_value

Therefore:

  positive ec_error:
    EC is low

  negative ec_error:
    EC is high

  zero:
    exactly at setpoint

EC is considered in range when:

  abs(ec_error) <= 50

which is:
  1550 <= EC <= 1650 uS/cm

If EC is outside that range, or Nutrient B is still pending, control returns
before reaching pH control.

Therefore pH correction waits until EC is both:
  - in range
  - not in the middle of an A/B nutrient cycle


======================================================================
9. LOW EC: NUTRIENT A/B SEQUENCE
======================================================================

When:
  EC < 1550 uS/cm

the controller wants EC-up.

Normal calculation:
  total dose = 8.0 mL
  A:B ratio = 1:1

So:
  A = 4.0 mL
  B = 4.0 mL

Before starting A, the controller requires:

  - no B already pending
  - 10-minute EC-up lockout has elapsed
  - at least 4 mL remains in the A hourly allowance
  - at least 4 mL remains in the B hourly allowance

Sequence
--------
1. Send "D,4.00" to Nutrient-A pump.

2. Only if that I2C command is acknowledged:
   - add 4 mL to A hourly commanded total
   - schedule B
   - set B volume to 4 mL
   - B becomes eligible 2 minutes later
   - mark the EC-up cycle start time
   - set last EC direction to "up"

3. B remains pending during the 2-minute stagger.

4. Once its ready time arrives, B can be sent if:
   - latest EC state is considered valid
   - B hourly total is still below its cap
   - B volume is positive

5. If the B command is acknowledged:
   - B hourly commanded total increases
   - pending_B becomes false
   - last action becomes B

6. If B I2C transmission fails:
   - B remains pending
   - retry is delayed by 5 seconds


======================================================================
10. WHY A AND B ARE STAGGERED
======================================================================

The two-part nutrient is not commanded simultaneously.

The controller waits:
  120 seconds

between accepted A and B commands.

This is intended to provide a mixing interval and reduce concentrated A/B
interaction before dilution in the reservoir.

The entire EC-up control cycle is governed by a:
  10-minute mix/settle lockout

measured from the accepted A command.


======================================================================
11. PENDING B EXPIRY
======================================================================

Pending B is cancelled when:

  now - accepted A time >= 10 minutes

If that happens before B succeeds, the code prints:

  "Pending B dose expired and was cancelled."

Practical consequence:
The controller will not keep trying to send the paired B dose forever.

A failed or blocked B can therefore leave an A-only addition from that cycle.
The software does not automatically reverse A or otherwise compensate for an
unmatched A dose.


======================================================================
12. HIGH EC / EC-DOWN BEHAVIOUR
======================================================================

When:
  EC > 1650 uS/cm

the controller enters the EC-down branch.

However:

  EC_DOWN_DOSE_ML = 0.0

Therefore no EC-down pump command is currently issued.

Practical consequence:
If EC remains above 1650 uS/cm, the controller remains in the EC-control stage
and keeps returning before pH control.

So HIGH EC CAN BLOCK AUTOMATIC pH-DOWN INDEFINITELY while automatic EC-down is
disabled.

This is intentional according to the current EC-first structure, but is
important operationally.

If you manually dilute the reservoir and EC returns to 1650 uS/cm or below,
pH control can resume after the relevant EC settling logic permits it.


======================================================================
13. EC SETTLING BEFORE pH CONTROL
======================================================================

Even when EC has returned inside 1550-1650 uS/cm, pH control may still wait.

The controller remembers the most recent EC direction:

  +1 = EC-up
  -1 = EC-down
   0 = no EC dose yet

Required settling time:
  after EC-up:   10 minutes
  after EC-down: 10 minutes
  none:          0 minutes

pH is therefore prevented from reacting while the reservoir is expected to
still be mixing after an EC correction.


======================================================================
14. PUMP COMMAND ACKNOWLEDGEMENT
======================================================================

Each dosing command is sent over I2C with up to:
  3 attempts

A command is considered successful when Wire.endTransmission() returns zero.

Only after an acknowledged pump command does the Mega update:
  - dose counters
  - relevant lockout timestamp
  - last action
  - pending A/B sequence state

This is a good software-level safeguard against counting an I2C transmission
that clearly failed.

VERY IMPORTANT LIMITATION:
An I2C acknowledgement proves only that the pump controller received the
command.

It does NOT prove:
  - the pump motor turned
  - tubing was connected
  - liquid actually flowed
  - the requested mL was physically delivered
  - the chemical container was not empty
  - a tube was not blocked
  - the pump calibration was accurate

All dose totals in this program are COMMANDED/ACKNOWLEDGED VOLUME, not directly
measured liquid volume.


======================================================================
15. I2C RETRIES AND TIMING
======================================================================

Normal command path:
  - up to 3 I2C transmission attempts
  - 5 ms delay after each failed attempt
  - once successful, send_command() waits 300 ms

The delay uses watchdog-safe polling rather than one long blocking delay.

The AVR watchdog is configured for:
  4 seconds

The software periodically resets it during controlled waits.


======================================================================
16. CRITICAL EC VALIDITY IMPLEMENTATION ISSUE
======================================================================

The comments say an invalid Atlas EC frame must never authorise dosing.

However, in the code as currently written, there is an important mismatch
between the comments and actual behaviour.

read_response_ec_numeric() returns:

  0.0

when:
  - Wire.requestFrom() returns no bytes
  - Atlas response status is not 1
  - text parsing effectively converts an invalid/non-numeric response to zero

Then read_ec_sensor() accepts any raw EC satisfying:

  raw >= 0.0 && raw < 1000000.0

Therefore a returned error value of 0.0 is treated as a VALID EC reading.

After compensation, it can set:

  ec_value = 0
  ec_read_valid = true

This means the current code does NOT fully implement the fail-safe behaviour
claimed by the comments.

Practical consequence:
An EC read failure could be interpreted as EC = 0 uS/cm and could authorise an
EC-up nutrient dose once the other timing/cap conditions allow it.

This is the most important control-safety issue in the current source.

Recommended design principle for a future fix:
read_response_ec_numeric() should return an unmistakably invalid sentinel such
as NAN, and read_ec_sensor() should explicitly reject NAN/non-finite values.

Until that is corrected, do not assume ec_read_valid is a perfect fresh-read
safety gate.


======================================================================
17. EC TEMPERATURE COMPENSATION
======================================================================

At startup the Mega commands the Atlas EZO-EC circuit:

  T,25.0

and attempts a readback using:
  T,?

The program then also performs its own software compensation:

  compensated EC =
    measured EC /
    (1 + ALPHA * (water_temperature - 25))

where:
  ALPHA = 0.02 per degree C

The practical design intent appears to be:
  - keep the EZO internally referenced at 25 C
  - apply temperature compensation in the Mega

This avoids dynamically sending the actual reservoir temperature into the EZO
each cycle.

Operationally, verify that the EZO's own output mode/settings match this
assumption and that you are not unintentionally applying temperature
compensation twice elsewhere in the system.


======================================================================
18. STARTUP BEHAVIOUR THAT MATTERS
======================================================================

Several timestamps start at zero:

  last_dose_ms_ec_cycle = 0
  last_dose_ms_ec_down  = 0
  last_dose_ms_ph_down  = 0

The code tests lockouts using:

  millis() - last_timestamp >= lockout

Therefore, after a reboot:

EC-up is not eligible until approximately:
  10 minutes uptime

EC-down is not eligible until approximately:
  10 minutes uptime

pH-down is not eligible until approximately:
  30 minutes uptime

This occurs even though no actual dose happened immediately before boot.

Practical consequence:
A controller restart creates a deliberate/effective startup no-dose period.

This may be desirable as a conservative boot behaviour, but it is important
when diagnosing "why isn't it dosing?" after power-up.


======================================================================
19. HOURLY DOSE CAPS
======================================================================

The controller maintains four separate accounting windows:

  Nutrient A
  Nutrient B
  EC-down
  pH-down

Each window starts from boot and is reset when at least:
  3,600,000 ms

has elapsed since that channel's window_start.

This is NOT a rolling previous-60-minutes calculation.

It is a fixed controller-runtime window.

Example:
If a channel window resets at 14:17, its next reset is around 15:17, not at
15:00.

A reboot resets all in-memory dose accounting.

The dose totals are not stored in EEPROM.

Therefore:
A reboot clears the software's knowledge of how much chemical was commanded in
the previous hour.

The startup lockouts partly reduce immediate post-reboot dosing risk, but the
hourly totals themselves do not survive power loss.


======================================================================
20. pH HOURLY LIMIT IN THE CURRENT CONFIGURATION
======================================================================

pH-down settings:

  per action = 1.0 mL
  lockout = 30 minutes
  hourly cap = 2.0 mL

So the system can normally accept at most two 1 mL pH-down commands during one
hourly accounting window.

Because a 30-minute lockout also applies, the practical maximum frequency is
one accepted pH-down action every 30 minutes.

Actual timing depends on:
  - sensor cycles
  - EC being valid
  - EC being in range
  - EC settling
  - EMA pH remaining above 5.95
  - the hourly window state


======================================================================
21. EC HOURLY LIMIT IN THE CURRENT CONFIGURATION
======================================================================

Each normal EC-up cycle commands:

  4 mL A
  4 mL B

Hourly cap:
  40 mL A
  40 mL B

Therefore the volume caps alone allow up to ten complete 4+4 mL cycles per
accounting window.

However, the 10-minute EC-up lockout is more restrictive in normal continuous
operation.

The lockout begins when A is acknowledged.

So dose frequency, sensor feedback and the reservoir response generally limit
actual EC-up actions before the 40 mL caps do.


======================================================================
22. WHAT LAST ACTION MEANS
======================================================================

last_action starts as:
  None

Possible values:
  A
  B
  ECD
  pHD
  None

last_action_ml records the volume associated with the most recently
successfully acknowledged pump command.

This is not the total volume of a full A+B EC cycle.

For example:
  after A 4 mL:
    LAST = A
    LAST_ML = 4.0

  after later B 4 mL:
    LAST = B
    LAST_ML = 4.0


======================================================================
23. LCD OPERATION
======================================================================

LCD:
  20 columns x 4 rows

I2C address:
  0x27

Page rotation:
  every 3 seconds

Numeric/in-page refresh:
  every 1.5 seconds

Periodic LCD reinitialisation:
  every 5 minutes

The reinitialisation is intended to recover from LCD/I2C display lockups.

There are three pages.


PAGE A - LIVE OVERVIEW
----------------------

Row 0:
  pHc = filtered/control pH
  pHr = raw pH

Row 1:
  configured pH setpoint

Row 2:
  current displayed EC
  configured EC setpoint

Row 3:
  water temperature


PAGE B - CURRENT HOURLY ACCOUNTING WINDOW
-----------------------------------------

A:
  acknowledged Nutrient-A mL / A cap

B:
  acknowledged Nutrient-B mL / B cap

EC Dn:
  acknowledged EC-down mL / cap

pH Dn:
  acknowledged pH-down mL / cap


PAGE C - OPERATING STATUS
-------------------------

Row 0:
  last acknowledged pump action and mL

Rows 1-2:
  blank

Row 3:
  controller uptime in hours


======================================================================
24. STALE DISPLAY VALUES
======================================================================

The design deliberately allows the LCD/telemetry EC value to retain the last
good EC measurement when a later EC reading fails.

This avoids the display jumping unnecessarily.

Conceptually there are two different states:

  ec_value
    value shown/reported

  ec_read_valid
    whether the newest EC cycle is usable for control

That is a sound architecture.

However, remember the implementation issue described in Section 16: some EC
communication failures currently become numeric zero and can therefore be
incorrectly marked valid.


======================================================================
25. MEGA -> ESP8266 TELEMETRY FORMAT
======================================================================

Every sensor cycle the Mega sends one comma-separated key/value line.

Keys:

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

Example structure:

  PH_RAW,5.92,PH_C,5.88,EC,1580,T,20.4,PH_SP,5.8,...

The bridge expects pairs:

  KEY,VALUE,KEY,VALUE,...


======================================================================
26. TELEMETRY VALUES
======================================================================

PH_RAW
  instantaneous pH

PH_C
  EMA-filtered pH used by the Mega's pH control

EC
  displayed/current retained EC value, in uS/cm

T
  water temperature in C

PH_SP
  5.8

EC_SP
  1600 uS/cm

PH_ERR
  PH_SP - PH_C

Interpretation:
  negative PH_ERR means pH is above target
  positive PH_ERR means pH is below target

EC_ERR
  EC_SP - EC

Interpretation:
  positive EC_ERR means EC is below target
  negative EC_ERR means EC is above target

A_HR / B_HR / ECD_HR / PHD_HR
  acknowledged commanded volumes in the current controller accounting windows

LAST
  most recent acknowledged pump command type

LAST_ML
  volume of that last command

UPTIME_MS
  Mega uptime according to millis()


======================================================================
27. IMPORTANT TELEMETRY LIMITATION
======================================================================

The Mega's telemetry does NOT currently send ec_read_valid.

Therefore Home Assistant can receive a plausible retained EC value without
knowing whether the newest EC measurement was fresh or stale.

Also, because ec_ok in send_line_to_esp8266() only checks whether ec_value is
within a numeric range, a held last-good EC value still appears as an ordinary
EC number.

Practical consequence:
Home Assistant's EC entity is useful for monitoring, but it is not a complete
representation of the Mega's internal EC freshness/safety state.

A future improvement would be to transmit fields such as:

  EC_VALID
  EC_AGE
  PENDING_B
  CONTROL_STATE
  PH_LOCKOUT_REMAINING
  EC_LOCKOUT_REMAINING

but these are not present in the supplied code.


======================================================================
28. ESP8266 BRIDGE ROLE
======================================================================

The ESP8266 does not make dosing decisions.

Its job is:

1. Read the Mega CSV line at 115200 baud.

2. Parse recognised key/value pairs.

3. Connect to Wi-Fi.

4. Connect to the MQTT broker.

5. Publish each value as a retained MQTT state.

6. Publish Home Assistant discovery configuration.

7. Publish an MQTT Last Will and Testament availability state.

8. Optionally mirror the complete raw CSV message.


======================================================================
29. ESP8266 NETWORK SETTINGS
======================================================================

The bridge uses:

Wi-Fi mode:
  station

MQTT:
  TCP port 1883

Configured broker:
  192.168.2.16

MQTT client ID:
  hydro-nodemcu

MQTT availability topic:
  hydro/status

The broker connection uses a username and password.

Port 1883 is normal unencrypted MQTT.

Practical security consequence:
MQTT credentials and telemetry are not TLS-encrypted on the LAN.

For a trusted, isolated local IoT VLAN this may be acceptable depending on your
threat model, but it should not be exposed directly to the internet.


======================================================================
30. HOME ASSISTANT MQTT TOPICS
======================================================================

Main values:

  hydro/pH
  hydro/ph_filt
  hydro/ec_uScm
  hydro/temp_C

Setpoints:

  hydro/ph_sp
  hydro/ec_sp_uScm

Errors:

  hydro/ph_err
  hydro/ec_err_uScm

Hourly/controller-window dose totals:

  hydro/dose_a_ml_hr
  hydro/dose_b_ml_hr
  hydro/dose_ec_down_ml_hr
  hydro/dose_ph_down_ml_hr

Last action:

  hydro/last_action
  hydro/last_action_ml

Runtime:

  hydro/uptime_ms

Availability:

  hydro/status

Raw diagnostic CSV, when enabled:

  hydro/raw


======================================================================
31. MQTT RETAINED MESSAGES
======================================================================

The bridge publishes the state topics as retained messages.

Practical meaning:
Home Assistant or another MQTT client can immediately receive the last known
value after reconnecting, even if the Mega has not produced a new cycle yet.

This is useful for dashboard continuity.

However, retained data can also look current when the physical controller or
bridge has only recently gone offline.

Use:
  hydro/status

as the availability indicator rather than assuming a retained sensor value is
necessarily fresh.


======================================================================
32. MQTT AVAILABILITY / LWT
======================================================================

When the ESP8266 successfully connects to MQTT it publishes:

  hydro/status = online

The MQTT Last Will is configured to publish:

  hydro/status = offline

if the broker detects an ungraceful connection loss.

This is used by Home Assistant discovery entities as their availability topic.


======================================================================
33. HOME ASSISTANT AUTO-DISCOVERY
======================================================================

The ESP8266 publishes Home Assistant discovery payloads under:

  homeassistant/...

Discovery is:
  - published after MQTT connection
  - republished every 5 minutes

The declared Home Assistant device is:

  Name:
    Hydroponics Controller

  Manufacturer:
    DIY

  Model:
    Mega+ESP8266

  Software version:
    1.3

  Identifier:
    hydro_controller_001

The periodic discovery republish helps recover entities after Home Assistant
restarts or loses discovery configuration.


======================================================================
34. ESP8266 CSV PARSER BEHAVIOUR
======================================================================

The bridge tokenises each received line on commas.

It processes alternating:
  key
  value

Unknown keys are ignored.

For numeric values:
  "NaN" is skipped and not published.

Because the MQTT state is retained, skipping a NaN means the previous retained
MQTT value remains visible.

Practical consequence:
A failed value does not automatically clear the corresponding Home Assistant
sensor state.

String fields such as LAST are published directly.


======================================================================
35. RAW MQTT DIAGNOSTICS
======================================================================

Current setting:

  ENABLE_RAW_MQTT = 1

Therefore the entire received CSV line is also published to:

  hydro/raw

This is useful when debugging:
  - UART corruption
  - missing fields
  - parser issues
  - Mega telemetry behaviour

Heartbeat is currently disabled:

  ENABLE_HEARTBEAT = 0

If enabled, it would publish:
  hydro/beat = 1

every 10 seconds.


======================================================================
36. WHAT HAPPENS IF WI-FI OR MQTT FAILS
======================================================================

The ESP8266's connectWiFi() and ensureMQTT() routines use blocking reconnect
loops.

The Mega controller is separate, so Mega local control can continue.

However, while the ESP8266 is disconnected or blocked reconnecting:
  - Home Assistant telemetry stops updating
  - MQTT availability should eventually indicate offline
  - the ESP8266 may not consume UART continuously

The ESP8266 line buffer only protects the current software-side line. It is not
a guaranteed store-and-forward telemetry logger.

Do not rely on the bridge to preserve every sensor cycle during prolonged
network outages.


======================================================================
37. WHAT HAPPENS IF THE ESP8266 REBOOTS
======================================================================

The Mega does not depend on the ESP8266 for control.

Therefore an ESP8266 reboot should not reset the Mega's dosing state unless the
hardware power arrangement causes both devices to reset together.

After reconnecting, the bridge:
  - publishes online
  - republishes discovery
  - begins publishing fresh Mega CSV data again


======================================================================
38. WHAT HAPPENS IF THE MEGA REBOOTS
======================================================================

A Mega reboot clears all volatile runtime state, including:

  ph_control EMA history
  last dosing timestamps
  hourly dose totals
  pending B
  last action state
  EC last-good state

It then:
  - initialises probes
  - sets EZO temperature reference to 25.0 C
  - initialises LCD
  - starts its runtime clocks again

As noted earlier, the zero-initialised timestamps effectively prevent:
  EC-up for about 10 minutes
  EC-down for about 10 minutes
  pH-down for about 30 minutes

after restart.


======================================================================
39. EEPROM
======================================================================

EEPROM.h is included in the Mega source, but the supplied code does not
actually use EEPROM to save control state.

Therefore the following do NOT survive reboot:
  - hourly dose totals
  - lockout timestamps
  - EMA state
  - pending-B state
  - last action
  - last-good EC


======================================================================
40. WATCHDOG
======================================================================

On AVR the Mega watchdog is enabled with:
  WDTO_4S

The code resets the watchdog:
  - during the main loop
  - during safe_delay()

This is intended to recover the controller if the main software becomes stuck
for long enough.

A watchdog reset is still a full controller restart and therefore has the same
volatile-state implications described above.


======================================================================
41. millis() AND LONG UPTIMES
======================================================================

The timing comparisons use the standard unsigned subtraction pattern:

  now - previous >= interval

This is generally suitable for millis() wraparound.

The displayed uptime is:

  millis() / 3600000.0

Because millis() on a normal 32-bit Arduino timebase wraps after roughly
49.7 days, the displayed uptime will also wrap rather than representing
permanent cumulative lifetime.


======================================================================
42. PRACTICAL CONTROL EXAMPLES
======================================================================

EXAMPLE A - pH = 5.90, EC = 1600
---------------------------------
Assume ph_control = 5.90.

EC is in range.

pH is above nominal setpoint 5.80, but it is NOT above 5.95.

Result:
  no pH dose


EXAMPLE B - pH = 5.96, EC = 1600
---------------------------------
Assume ph_control = 5.96 and all lockouts/caps allow dosing.

pH exceeds the upper trigger 5.95.

Result:
  command 1.0 mL pH-down

If acknowledged:
  start 30-minute pH lockout


EXAMPLE C - raw pH jumps to 6.10
--------------------------------
If the previous ph_control was 5.80:

  new ph_control
  = 0.20 * 6.10 + 0.80 * 5.80
  = 5.86

Result:
  no immediate pH-down dose

The raw reading must remain elevated for the EMA to rise past 5.95.


EXAMPLE D - EC = 1500
---------------------
EC error:
  1600 - 1500 = +100

This is outside the +/-50 band on the low side.

If the EC-up lockout and caps allow:
  A 4 mL is commanded

If A is acknowledged:
  B 4 mL is scheduled 2 minutes later

pH control is deferred.


EXAMPLE E - EC = 1670
---------------------
EC error:
  1600 - 1670 = -70

The controller wants EC-down.

But:
  EC_DOWN_DOSE_ML = 0.0

Result:
  no water command is made

Because EC remains out of range:
  pH control is also deferred


EXAMPLE F - EC = 1550
---------------------
Error:
  +50

The comparison is <= EC_DEADBAND.

Result:
  EC is in range


EXAMPLE G - EC = 1650
---------------------
Error:
  -50

Result:
  EC is in range


EXAMPLE H - pH = exactly 5.95
-----------------------------
Error magnitude:
  0.15

The code requires:
  magnitude > 0.15

Result:
  no pH-down dose


======================================================================
43. WHY THE SYSTEM MAY APPEAR TO "DO NOTHING"
======================================================================

Before assuming the pump or controller has failed, check:

1. Has the Mega just rebooted?
   - EC-up effectively waits ~10 min.
   - pH-down effectively waits ~30 min.

2. Is EC above 1650?
   - automatic EC-down is disabled
   - pH is consequently blocked

3. Is EC below 1550 but still in its 10-minute lockout?

4. Is Nutrient B pending?

5. Is EC waiting for the post-dose settle period?

6. Is pH raw high but EMA ph_control still <=5.95?

7. Is the pH 30-minute lockout active?

8. Has the 2 mL pH hourly cap been reached?

9. Has the relevant nutrient hourly cap been reached?

10. Did the pump I2C command fail?

11. Has an EC communication fault been misinterpreted as zero?
    See the critical issue in Section 16.


======================================================================
44. INTERPRETING pH RAW VS pH CONTROL
======================================================================

For control diagnosis, PH_C matters more than PH_RAW.

PH_RAW:
  what the probe reported in this cycle

PH_C:
  what the controller is actually using to make pH dosing decisions

If Home Assistant shows:
  PH_RAW = 6.05
  PH_C   = 5.91

then the controller will not yet dose pH-down because PH_C has not crossed
5.95.

The filtering is intentional.


======================================================================
45. INTERPRETING THE ERROR TOPICS
======================================================================

PH_ERR:
  PH_SP - PH_C

Examples:
  PH_C = 6.00
  PH_ERR = -0.20
  pH is above target

  PH_C = 5.70
  PH_ERR = +0.10
  pH is below target


EC_ERR:
  EC_SP - EC

Examples:
  EC = 1500
  EC_ERR = +100
  EC is low

  EC = 1700
  EC_ERR = -100
  EC is high


======================================================================
46. CONTROL PHILOSOPHY
======================================================================

This is a deliberately conservative, discrete-dose controller.

It is not PID control.

It uses:
  - threshold/deadband decisions
  - fixed dose quantities
  - long mixing lockouts
  - hourly software caps
  - EMA filtering for pH
  - EC priority over pH

The basic philosophy is:

  measure
  -> decide
  -> command a small fixed correction
  -> wait for mixing
  -> measure again

This is appropriate for avoiding rapid repeated dosing into an incompletely
mixed reservoir, provided dose sizes and lockout times are well tuned to the
actual reservoir volume and mixing rate.


======================================================================
47. SETTINGS THAT MOST DIRECTLY CHANGE BEHAVIOUR
======================================================================

PH_SETPOINT
  Changes nominal pH target.

PH_DEADBAND
  Changes how far above the setpoint pH must rise before pH-down can trigger.

With down-only control:
  trigger = PH_SETPOINT + PH_DEADBAND

PH_CONTROL_EMA
  Changes how quickly control pH follows the raw pH reading.

PH_DOWN_DOSE_ML
  Changes each pH-down command volume.

PH_DOWN_MIX_LOCKOUT_MS
  Changes minimum time between accepted pH-down commands.

PH_DOWN_MAX_ML_PER_HR
  Changes the controller's hourly pH-down command cap.

EC_SETPOINT
  Changes nominal EC target.

EC_DEADBAND
  Changes accepted EC range.

EC_UP_TOTAL_DOSE_ML
  Changes total A+B volume for a normal EC-up cycle.

EC_A_TO_B_RATIO
  Changes how the total EC-up dose is divided between A and B.

EC_AB_STAGGER_MS
  Changes delay from accepted A to B eligibility.

EC_UP_MIX_LOCKOUT_MS
  Changes the EC-up settling period.

EC_A_MAX_ML_PER_HR / EC_B_MAX_ML_PER_HR
  Change hourly nutrient command caps.

EC_DOWN_DOSE_ML
  Enables and sizes automatic EC-down when greater than zero.

SENSOR_INTERVAL
  Changes how often a new full measurement/control cycle occurs.


======================================================================
48. PARAMETERS THAT SHOULD BE TUNED TOGETHER
======================================================================

pH dosing
---------
Treat these as one set:

  PH_DOWN_DOSE_ML
  PH_DOWN_MIX_LOCKOUT_MS
  PH_DEADBAND
  PH_CONTROL_EMA
  PH_DOWN_MAX_ML_PER_HR

A larger pH dose generally requires enough mixing time before judging whether
another dose is needed.

A very narrow deadband plus fast EMA plus short lockout can create frequent
corrections.

A slow EMA plus long lockout is more conservative but can respond slowly.


EC dosing
---------
Treat these as one set:

  EC_UP_TOTAL_DOSE_ML
  EC_A_TO_B_RATIO
  EC_AB_STAGGER_MS
  EC_UP_MIX_LOCKOUT_MS
  EC_DEADBAND
  EC_A_MAX_ML_PER_HR
  EC_B_MAX_ML_PER_HR

The appropriate numbers depend strongly on:
  reservoir volume
  nutrient strength
  circulation/mixing rate
  plant uptake
  top-up-water behaviour


======================================================================
49. IMPORTANT SAFETY / RELIABILITY LIMITATIONS
======================================================================

The current system does NOT directly verify:

  physical liquid flow
  chemical tank level
  pump stall
  tube disconnection
  leak
  siphoning
  reservoir level
  actual delivered mL
  pH probe calibration quality
  EC probe calibration quality

The controller is therefore open-loop with respect to physical pump output and
closed-loop only through later pH/EC measurements.

A robust installation should rely on:
  - correctly calibrated pumps
  - conservative dose sizes
  - adequate mixing
  - correctly calibrated probes
  - physical reservoir inspection
  - sensible chemical concentration
  - mechanical protection against siphoning/leaks
  - independent high/low alarms where appropriate


======================================================================
50. MAIN CODE/COMMENT DISCREPANCIES TO REMEMBER
======================================================================

A. EC read failure handling
---------------------------
The comments describe a fail-safe invalid-read gate.

Actual implementation can convert several EC read failures to 0.0 and then
mark 0.0 as a valid EC measurement.

This should be corrected before relying on the fail-safe claim.


B. "Hourly" totals
------------------
They are one-hour runtime accounting windows, not rolling 60-minute totals and
not aligned to wall-clock hours.


C. Dose totals
--------------
They represent acknowledged commands, not measured physical dosing.


D. EC telemetry freshness
-------------------------
Home Assistant does not receive an EC-valid/freshness flag.


E. Automatic high-EC correction
-------------------------------
It is disabled because EC_DOWN_DOSE_ML = 0.0, but high EC still blocks pH
control because EC has priority.


F. Reboot state
---------------
Dose counters and prior-action history are volatile and reset on reboot.


======================================================================
51. NORMAL OPERATING CHECKLIST
======================================================================

At startup
----------
[ ] Confirm Mega boots normally.
[ ] Confirm LCD updates.
[ ] Confirm DS18B20 temperature is plausible.
[ ] Confirm raw and filtered pH are plausible.
[ ] Confirm EC is plausible.
[ ] Remember dosing is initially inhibited by the zero-based lockout timers.
[ ] Confirm ESP8266 shows online in MQTT/Home Assistant.


Before enabling unattended chemical dosing
-------------------------------------------
[ ] Calibrate pH probe.
[ ] Calibrate EC probe.
[ ] Calibrate each dosing pump in mL.
[ ] Confirm A/B pump addresses are not swapped.
[ ] Confirm pH-down pump address.
[ ] Confirm tubing destination for every pump.
[ ] Confirm no siphoning occurs when pumps stop.
[ ] Confirm reservoir circulation is strong enough for configured lockouts.
[ ] Verify a commanded 1 mL / 4 mL dose physically matches calibration.
[ ] Verify the EC failure handling issue has been corrected or otherwise
    mitigated.


During operation
----------------
[ ] Watch PH_C rather than PH_RAW when predicting pH control.
[ ] Check EC is inside 1550-1650 before expecting pH dosing.
[ ] Check A_HR/B_HR/PHD_HR when diagnosing lockouts/caps.
[ ] Use LAST and LAST_ML to see the most recently acknowledged pump command.
[ ] Use hydro/status to distinguish stale retained MQTT data from an online
    bridge.
[ ] Periodically compare software dose totals with actual chemical-container
    consumption.


======================================================================
52. QUICK REFERENCE
======================================================================

pH setpoint:
  5.80

Actual pH-down trigger:
  ph_control > 5.95

pH dose:
  1.0 mL

pH lockout:
  30 min

pH hourly cap:
  2.0 mL

pH EMA alpha:
  0.20

EC setpoint:
  1600 uS/cm

EC accepted band:
  1550-1650 uS/cm

EC-up total:
  8.0 mL

A dose:
  4.0 mL

B dose:
  4.0 mL

A -> B delay:
  2 min

EC-up lockout:
  10 min

A hourly cap:
  40 mL

B hourly cap:
  40 mL

Automatic EC-down:
  DISABLED

Main control cycle:
  every 10 s

LCD page:
  every 3 s

LCD refresh:
  every 1.5 s

LCD reinitialise:
  every 5 min

Mega watchdog:
  4 s

ESP8266 MQTT discovery refresh:
  every 5 min

MQTT:
  TCP 1883, unencrypted

Most important known issue:
  An invalid EC response can currently become numeric 0.0 and be accepted as a
  valid reading. Correct this before treating the EC-validity gate as fail-safe.


======================================================================
53. ONE-SENTENCE SYSTEM MODEL
======================================================================

The Mega repeatedly measures temperature, pH and EC, gives EC correction
priority, doses fixed amounts only after timing/cap checks, waits for mixing
before allowing further corrections, reports everything to the ESP8266, and
the ESP8266 exposes those values to Home Assistant over MQTT.

END OF README
