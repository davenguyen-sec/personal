/**
 * @file    hydra_main.ino
 * @brief   Hydroponics controller: EC-first dosing with EMA-smoothed pH control and
 *          a simple, evenly timed 20×4 LCD UI.
 *
 * @details
 * Control policy:
 *   1) Electrical Conductivity (EC) is corrected first. If EC is outside its band,
 *      the controller will adjust EC and defer any pH action until EC has settled.
 *   2) pH dosing uses an exponential moving average (EMA) of the raw pH reading
 *      to reduce action on noise. The EMA factor (alpha) is user-tunable.
 *
 * UI:
 *   - 20×4 I²C LCD rotates through three pages at fixed intervals.
 *   - EC line omits the “µS/cm” unit per requirement.
 *   - Uptime is shown in hours on the third page.
 *   - Page changes trigger exactly one clear to avoid visible flicker.
 *   - A 5-minute “keep-alive” re-initialises the LCD without showing the splash,
 *     which prevents some I²C LCDs from dimming or corrupting over time.
 *
 * Safety limits:
 *   - Per-action lockouts provide mix/settle time before the next dose.
 *   - Hourly per-channel caps (mL/h) bound total dosing per hour.
 *
 * Sensors and compensation:
 *   - DS18B20 for water temperature.
 *   - EZO-EC probe is pinned to 25.0 °C, then software-compensated to 25 °C using ALPHA.
 *   - pH from DFRobot_PH using ADC voltage and measured water temperature.
 *
 * Telemetry:
 *   - ESP8266 bridge on Serial1 outputs CSV once per sensor cycle for HA/MQTT.
 *
 * Notes:
 *   - Set PH_CONTROL_EMA = 1.0 to disable smoothing (EMA equals raw).
 *   - All times are in milliseconds. All volumes are millilitres. Metric only.
 *   - This sketch assumes pumps accept ASCII “D,<mL>” dose commands on I²C.
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "DFRobot_PH.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <string.h>

#if defined(ARDUINO_ARCH_AVR)
  #include <avr/wdt.h>
#endif

/* ────────────────────────────────────────────────────────────
 *                   USER-SETTABLE PARAMETERS
 *   Change values here. Everything else adapts automatically.
 * ──────────────────────────────────────────────────────────── */

// Analogue / digital pins
#define PH_ANALOG_PIN A5          // MCU ADC pin wired to pH interface output
#define DS18B20_PIN   4           // OneWire bus pin for DS18B20

// I²C slave addresses for pumps and EZO-EC
#define PH_DOWN_PUMP_ADDRESS      0x32
#define NUTRIENT_A_PUMP_ADDRESS   0x33
#define NUTRIENT_B_PUMP_ADDRESS   0x34
#define EC_DOWN_PUMP_ADDRESS      0x35
#define EC_PROBE_ADDRESS          0x64

// LCD geometry, address, and UI timing
#define LCD_I2C_ADDRESS 0x27
#define LCD_COLS        20
#define LCD_ROWS        4
static const unsigned long LCD_UPDATE_MS = 1000UL;   // in-page refresh cadence
static const unsigned long LCD_PAGE_MS   = 3000UL;   // dwell time per page
static const unsigned long LCD_REINIT_MS = 300000UL; // 5 min keep-alive; no splash

// ADC calibration (mV at ADC reference). Use 3300.0 on 3V3 MCUs.
static const double ANALOG_REF_MV = 5000.0;

// Control set-points
static const double PH_SETPOINT = 5.8;        // target pH
static const double EC_SETPOINT = 1600.0;     // target EC in µS/cm

// Deadbands to reduce chattering around set-points
static const double PH_DEADBAND = 0.15;       // ± around pH set-point
static const double EC_DEADBAND = 50.0;       // ± around EC set-point (µS/cm)

// EC temperature compensation coefficient (fraction per °C)
static const double ALPHA = 0.02;             // typical ionic solution value

// pH control smoothing. 0→no update, 1→raw only. 0.2 = 20% new, 80% history.
static const double PH_CONTROL_EMA = 0.20;

// EC-Up dose split across two parts (A then B). Total per EC cycle.
static const double EC_UP_TOTAL_DOSE_ML = 8.0;
static const double EC_A_TO_B_RATIO     = 1.0;      // 1:1 split between A and B
static const unsigned long EC_AB_STAGGER_MS = 120000UL; // wait between A and B

// EC-Down (water) per-dose. Set to 0 to disable EC-Down.
static const double EC_DOWN_DOSE_ML = 0.0;

// pH-Down per-dose size
static const double PH_DOWN_DOSE_ML = 0.5;

// Post-dose lockouts to allow mixing/settling before re-measuring or dosing
static const unsigned long EC_UP_MIX_LOCKOUT_MS   = 600000UL;  // 10 min
static const unsigned long EC_DOWN_MIX_LOCKOUT_MS = 600000UL;  // 10 min
static const unsigned long PH_DOWN_MIX_LOCKOUT_MS = 900000UL;  // 15 min

// Hourly dosing caps per channel (mL per rolling hour window)
static const double  EC_A_MAX_ML_PER_HR    = 40.0;
static const double  EC_B_MAX_ML_PER_HR    = 40.0;
static const double  EC_DOWN_MAX_ML_PER_HR = 100.0; // used if EC_DOWN_DOSE_ML > 0
static const double  PH_DOWN_MAX_ML_PER_HR = 2.0;

// Sensor read cadence. Also drives CSV telemetry cadence.
static const unsigned long SENSOR_INTERVAL = 10000UL; // 10 s

/* ────────────────────────────────────────────────────────────
 *                       Globals and instances
 * ──────────────────────────────────────────────────────────── */

DFRobot_PH ph_sensor;                // pH helper from DFRobot
OneWire oneWire(DS18B20_PIN);        // OneWire bus
DallasTemperature waterTemp(&oneWire);
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);

// Live readings
static double ph_raw = 0.0;          // instantaneous pH from ADC→DFRobot_PH
static double ph_control = NAN;      // EMA-smoothed pH used for decisions
static double ec_value = 0.0;        // EC after software temp compensation
static double water_temperature = 0.0;

// Timers for cadence and UI
static unsigned long previous_millis = 0;
static unsigned long last_lcd_ms = 0;
static unsigned long last_page_ms = 0;
static unsigned long last_lcd_reinit_ms = 0;
static uint8_t lcd_page = 0;         // 0..2 (A,B,C pages)

// UI page-flip housekeeping to prevent flicker
static bool page_changed = false;
static uint8_t prev_page = 255;
static bool boot_splash_shown = false;

// EC dosing state
static unsigned long last_dose_ms_ec_cycle = 0; // last A/B action time
static unsigned long last_dose_ms_ec_down  = 0; // last EC-Down action time
static int last_ec_direction = 0;               // +1 up, −1 down, 0 none

// Deferred B dose bookkeeping
static bool   pending_B = false;
static double pending_B_ml = 0.0;
static unsigned long pending_B_ready_ms = 0;

// pH dosing state
static unsigned long last_dose_ms_ph_down = 0;

// Rolling hourly windows and accumulators
static unsigned long ec_a_window_start    = 0;
static unsigned long ec_b_window_start    = 0;
static unsigned long ec_down_window_start = 0;
static unsigned long ph_down_window_start = 0;
static double ec_a_dosed_this_window    = 0.0;
static double ec_b_dosed_this_window    = 0.0;
static double ec_down_dosed_this_window = 0.0;
static double ph_down_dosed_this_window = 0.0;

// Last action for UI
static char   last_action[8] = "None"; // "A","B","ECD","pHD","None"
static double last_action_ml = 0.0;

/* ────────────────────────────────────────────────────────────
 *                  Forward declarations
 *   Each function has a short contract for maintainability.
 * ──────────────────────────────────────────────────────────── */
static void initialise_pins();
static void initialise_devices();
static void initialise_display(bool showSplash);

static double read_water_temperature();
static double read_ph_sensor();
static double read_ec_sensor();
static double compensate_ec(double ec_measured, double temperature);

static void   send_command(uint8_t address, const char *command);
static float  read_response_ec_numeric(uint8_t address);
static size_t read_response_ascii(uint8_t address, char *buf, size_t buflen);

static void dose_nutrient_A(double ml);
static void dose_nutrient_B(double ml);
static void dose_ec_down(double ml);
static void dose_ph_down(double ml);

static void control_loop();
static void maybe_reset_hourly_caps();

static void update_lcd();
static void lcd_screen_A();
static void lcd_screen_B();
static void lcd_screen_C();

static void send_line_to_esp8266();
static const char* ec_status_text();

/* ────────────────────────────────────────────────────────────
 *                              setup
 *   One-time initialisation. Keeps splash short to reduce blink.
 * ──────────────────────────────────────────────────────────── */
void setup() {
  Serial.begin(9600);         // for wired debug
  Serial1.begin(115200);      // for ESP8266 CSV bridge

  Wire.begin();
  #if defined(TWOWIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(25000, true);
  #elif defined(ARDUINO_ARCH_AVR)
    Wire.setWireTimeout(25000, true);
  #endif

  #if defined(ARDUINO_ARCH_AVR)
    wdt_enable(WDTO_4S);      // recover from I²C stalls
  #endif

  initialise_pins();
  initialise_devices();
  initialise_display(true);   // boot splash only once
  boot_splash_shown = true;

  // Seed UI timers so page 0 gets a full dwell
  last_page_ms = millis();
  lcd_page = 0;
  page_changed = true;        // force one clean draw
  update_lcd();

  Serial.println(F("Hydroponics controller initialised."));
}

/* ────────────────────────────────────────────────────────────
 *                               loop
 *   Non-blocking scheduler: sensors, control, UI, keep-alive.
 * ──────────────────────────────────────────────────────────── */
void loop() {
  #if defined(ARDUINO_ARCH_AVR)
    wdt_reset();
  #endif
  yield();

  const unsigned long now = millis();

  // 1) Sensor cadence and control
  if (now - previous_millis >= SENSOR_INTERVAL) {
    previous_millis = now;

    // Temperature first to feed pH and EC compensation
    read_water_temperature();

    // Avoid EC measurement noise while sampling pH
    send_command(EC_PROBE_ADDRESS, "Sleep");
    delay(10);
    read_ph_sensor();

    // Wake and read EC; compensate to 25 °C
    read_ec_sensor();

    // Run EC-first control
    control_loop();

    // Minimal serial snapshot for wired debug
    Serial.print(F("Snapshot | pHraw: "));
    Serial.print(ph_raw, 2);
    Serial.print(F(" | pHc: "));
    Serial.print(isnan(ph_control) ? ph_raw : ph_control, 2);
    Serial.print(F(" | EC: "));
    Serial.print(ec_value, 0);
    Serial.print(F(" µS/cm | T: "));
    Serial.print(water_temperature, 1);
    Serial.println(F(" °C"));

    // Push CSV row for ESP8266/HA
    send_line_to_esp8266();
  }

  // 2) Page rotation: flip exactly once per dwell, no catch-up bursts
  if (now - last_page_ms >= LCD_PAGE_MS) {
    last_page_ms = now;
    lcd_page = (lcd_page + 1) % 3;
    page_changed = true;      // allow a single clear on flip
    last_lcd_ms = now;        // prevent immediate extra refresh
    update_lcd();             // draw once at flip
  }

  // 3) In-page refresh for changing numbers
  if (now - last_lcd_ms >= LCD_UPDATE_MS) {
    last_lcd_ms = now;
    update_lcd();
  }

  // 4) Periodic LCD keep-alive. No splash to avoid visible blink.
  if (now - last_lcd_reinit_ms >= LCD_REINIT_MS) {
    initialise_display(false);
    last_lcd_reinit_ms = now;
    page_changed = true;      // one clean redraw after reinit
    update_lcd();
  }

  // 5) Deferred EC-B dose after stagger delay and within cap
  if (pending_B && now >= pending_B_ready_ms) {
    if ((now - last_dose_ms_ec_cycle) >= EC_UP_MIX_LOCKOUT_MS) {
      // If we have waited too long, cancel the deferred B
      pending_B = false;
    } else if (ec_b_dosed_this_window < EC_B_MAX_ML_PER_HR && pending_B_ml > 0.0) {
      dose_nutrient_B(pending_B_ml);
      ec_b_dosed_this_window += pending_B_ml;
      pending_B = false;
    }
  }
}

/* ────────────────────────────────────────────────────────────
 *                        Initialisation
 * ──────────────────────────────────────────────────────────── */

/**
 * @brief Configure MCU pins used by sensors/actuators.
 */
static void initialise_pins() {
  pinMode(PH_ANALOG_PIN, INPUT);
}

/**
 * @brief Bring up sensors and pin EZO-EC temperature to 25.0 °C.
 * @remarks Pinning the probe avoids double compensation. We then apply
 *          software compensation using ALPHA for display and control.
 */
static void initialise_devices() {
  ph_sensor.begin();
  waterTemp.begin();
  Serial.println(F("Devices initialised."));

  // Pin EZO-EC to 25.0 °C and verify
  send_command(EC_PROBE_ADDRESS, "T,25.0");
  delay(10);
  char tbuf[32];
  send_command(EC_PROBE_ADDRESS, "T,?");
  size_t n = read_response_ascii(EC_PROBE_ADDRESS, tbuf, sizeof(tbuf));
  if (n > 0) {
    Serial.print(F("EZO-EC temperature readback: "));
    Serial.println(tbuf);
  } else {
    Serial.println(F("EZO-EC temperature readback: no response"));
  }
}

/**
 * @brief Initialise LCD and optionally show a short splash.
 * @param showSplash True to show splash at boot only.
 */
static void initialise_display(bool showSplash) {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  if (showSplash) {
    lcd.setCursor(0, 0); lcd.print(F("Hydro Controller"));
    lcd.setCursor(0, 1); lcd.print(F("Init..."));
    delay(400); // short to minimise boot blink
    lcd.clear();
  }
}

/* ────────────────────────────────────────────────────────────
 *                           Sensors
 * ──────────────────────────────────────────────────────────── */

/**
 * @brief Read DS18B20 water temperature in °C.
 * @return Latest valid temperature. Falls back to previous if out-of-range.
 */
static double read_water_temperature() {
  waterTemp.requestTemperatures();
  const double t = waterTemp.getTempCByIndex(0);
  if (t < -50.0 || t > 125.0) return water_temperature; // reject nonsense
  water_temperature = t;
  return water_temperature;
}

/**
 * @brief Read raw pH and update EMA control value.
 * @return Raw pH value.
 * @notes EMA: phc_k = a*raw + (1-a)*phc_(k-1); a = PH_CONTROL_EMA in [0,1].
 *        a=1.0 → no smoothing; a→0 → very slow update.
 */
static double read_ph_sensor() {
  // ADC reading in mV, feed to DFRobot_PH with temperature
  const double voltage_mV = (analogRead(PH_ANALOG_PIN) / 1023.0) * ANALOG_REF_MV;
  ph_raw = ph_sensor.readPH(voltage_mV, water_temperature);

  // Initialise EMA with first valid reading
  const double a = constrain(PH_CONTROL_EMA, 0.0, 1.0);
  if (isnan(ph_control)) ph_control = ph_raw;
  else ph_control = a * ph_raw + (1.0 - a) * ph_control;

  return ph_raw;
}

/**
 * @brief Trigger EZO-EC reading and apply software temp compensation to 25 °C.
 * @return Compensated EC in µS/cm.
 */
static double read_ec_sensor() {
  send_command(EC_PROBE_ADDRESS, "R");
  delay(10);
  const double raw = read_response_ec_numeric(EC_PROBE_ADDRESS);
  ec_value = compensate_ec(raw, water_temperature);
  return ec_value;
}

/**
 * @brief Linear EC temperature compensation to reference 25 °C.
 * @param ec_measured  Probe-reported EC (assumed already pinned to 25 °C)
 * @param temperature  Measured water temperature in °C
 * @return EC corrected to 25 °C using ALPHA.
 */
static double compensate_ec(double ec_measured, double temperature) {
  const double T_REF = 25.0;
  return ec_measured / (1.0 + ALPHA * (temperature - T_REF));
}

/* ────────────────────────────────────────────────────────────
 *                          I²C helpers
 * ──────────────────────────────────────────────────────────── */

/**
 * @brief Send a null-terminated ASCII command to an I²C device.
 * @param address  7-bit I²C address.
 * @param command  ASCII command, e.g. "D,1.25".
 * @remarks Adds a short fixed delay to allow device processing.
 */
static void send_command(uint8_t address, const char *command) {
  Wire.beginTransmission(address);
  const char* p = command; while (*p) Wire.write(*p++);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.print(F("I2C TX error ")); Serial.print(err);
    Serial.print(F(" to 0x")); Serial.println(address, HEX);
  }
  delay(300);
}

/**
 * @brief Read a numeric ASCII response from EZO-EC and parse to float.
 * @return Parsed value or 0.0 on error. Status byte is read and ignored.
 */
static float read_response_ec_numeric(uint8_t address) {
  const uint8_t bytes = Wire.requestFrom(address, (uint8_t)32);
  if (bytes == 0) {
    Serial.print(F("I2C read error from 0x")); Serial.println(address, HEX);
    return 0.0f;
  }
  const uint8_t status = Wire.read(); (void)status;
  char buf[31]; uint8_t i = 0;
  while (Wire.available() && i < sizeof(buf) - 1) {
    const char c = Wire.read();
    if (c >= 32 && c <= 126) buf[i++] = c;
  }
  buf[i] = '\0';
  return atof(buf);
}

/**
 * @brief Read an ASCII response into caller buffer.
 * @return Number of bytes written excluding the terminator.
 */
static size_t read_response_ascii(uint8_t address, char *buf, size_t buflen) {
  if (!buf || buflen == 0) return 0;
  const uint8_t bytes = Wire.requestFrom(address, (uint8_t)32);
  if (bytes == 0) return 0;
  const uint8_t status = Wire.read(); (void)status;
  size_t i = 0;
  while (Wire.available() && i < buflen - 1) {
    const char c = Wire.read();
    if (c >= 32 && c <= 126) buf[i++] = c;
  }
  buf[i] = '\0';
  return i;
}

/* ────────────────────────────────────────────────────────────
 *                        Dosing primitives
 *   Thin wrappers around pump “D,<mL>” commands with logging.
 * ──────────────────────────────────────────────────────────── */

static void dose_nutrient_A(double ml) {
  if (ml <= 0.0) return;
  String cmd = "D," + String(ml, 2);
  send_command(NUTRIENT_A_PUMP_ADDRESS, cmd.c_str());
  Serial.print(F("Dose A (mL): ")); Serial.println(ml, 2);
  strcpy(last_action, "A"); last_action_ml = ml;
}

static void dose_nutrient_B(double ml) {
  if (ml <= 0.0) return;
  String cmd = "D," + String(ml, 2);
  send_command(NUTRIENT_B_PUMP_ADDRESS, cmd.c_str());
  Serial.print(F("Dose B (mL): ")); Serial.println(ml, 2);
  strcpy(last_action, "B"); last_action_ml = ml;
}

static void dose_ec_down(double ml) {
  if (ml <= 0.0) return;
  String cmd = "D," + String(ml, 2);
  send_command(EC_DOWN_PUMP_ADDRESS, cmd.c_str());
  Serial.print(F("EC Down (mL): ")); Serial.println(ml, 2);
  strcpy(last_action, "ECD"); last_action_ml = ml;
}

static void dose_ph_down(double ml) {
  if (ml <= 0.0) return;
  String cmd = "D," + String(ml, 2);
  send_command(PH_DOWN_PUMP_ADDRESS, cmd.c_str());
  Serial.print(F("pH Down dose (mL): ")); Serial.println(ml, 2);
  strcpy(last_action, "pHD"); last_action_ml = ml;
}

/* ────────────────────────────────────────────────────────────
 *                  Control loop (EC-first policy)
 *   1) Adjust EC to within deadband and let it settle.
 *   2) Then adjust pH using EMA-smoothed value.
 * ──────────────────────────────────────────────────────────── */

static void control_loop() {
  const unsigned long now = millis();
  maybe_reset_hourly_caps();

  const bool ph_ok = !(isnan(ph_control) || ph_control < 0.0 || ph_control > 14.0);

  // ── EC phase (priority) ────────────────────────────────────
  const double ec_err = EC_SETPOINT - ec_value;   // + means too low → EC up
  const bool   ec_in_range   = fabs(ec_err) <= EC_DEADBAND;
  const bool   ec_cycle_busy = pending_B;         // mid A→B cycle

  if (!ec_in_range || ec_cycle_busy) {
    if (!ec_cycle_busy) {
      const bool need_ec_down = (ec_err < 0.0);
      const unsigned long required_lockout = need_ec_down ? EC_DOWN_MIX_LOCKOUT_MS
                                                          : EC_UP_MIX_LOCKOUT_MS;
      const unsigned long last_ms = need_ec_down ? last_dose_ms_ec_down
                                                 : last_dose_ms_ec_cycle;

      if ((now - last_ms) >= required_lockout) {
        if (need_ec_down) {
          // EC-Down path (if enabled)
          if (EC_DOWN_DOSE_ML > 0.0 && ec_down_dosed_this_window < EC_DOWN_MAX_ML_PER_HR) {
            const double room = EC_DOWN_MAX_ML_PER_HR - ec_down_dosed_this_window;
            const double dose = min(EC_DOWN_DOSE_ML, room);
            if (dose > 0.0) {
              dose_ec_down(dose);
              last_dose_ms_ec_down = now;
              last_ec_direction = -1;
              ec_down_dosed_this_window += dose;
            }
          }
        } else {
          // EC-Up path using two-part A→B with stagger
          const double ratioA = EC_A_TO_B_RATIO;
          const double a_part = (ratioA / (ratioA + 1.0)) * EC_UP_TOTAL_DOSE_ML;
          const double b_part = EC_UP_TOTAL_DOSE_ML - a_part;

          const double a_room = max(0.0, EC_A_MAX_ML_PER_HR - ec_a_dosed_this_window);
          const double b_room = max(0.0, EC_B_MAX_ML_PER_HR - ec_b_dosed_this_window);

          if (a_part > 0.0 && b_part > 0.0 && a_room >= a_part && b_room >= b_part) {
            dose_nutrient_A(a_part);
            ec_a_dosed_this_window += a_part;

            pending_B = true;
            pending_B_ml = b_part;
            pending_B_ready_ms = now + EC_AB_STAGGER_MS;

            last_dose_ms_ec_cycle = now;
            last_ec_direction = +1;
          }
        }
      }
    }
    return; // Defer pH actions until EC within band and settled
  }

  // ── pH phase (after EC settle) ─────────────────────────────
  const unsigned long ec_required_settle =
      (last_ec_direction < 0) ? EC_DOWN_MIX_LOCKOUT_MS :
      (last_ec_direction > 0) ? EC_UP_MIX_LOCKOUT_MS   : 0UL;

  const unsigned long last_ec_any = max(last_dose_ms_ec_cycle, last_dose_ms_ec_down);
  const bool ec_has_settled = (millis() - last_ec_any) >= ec_required_settle;

  if (ph_ok && ec_has_settled) {
    const double ph_err = PH_SETPOINT - ph_control; // negative → too low pH set-point? we dose Down only
    const bool need_ph_down = (fabs(ph_err) > PH_DEADBAND && ph_err < 0.0);

    if (need_ph_down) {
      const bool lockout_ok = (millis() - last_dose_ms_ph_down) >= PH_DOWN_MIX_LOCKOUT_MS;
      const bool cap_ok = ph_down_dosed_this_window < PH_DOWN_MAX_ML_PER_HR;

      if (lockout_ok && cap_ok) {
        const double room = PH_DOWN_MAX_ML_PER_HR - ph_down_dosed_this_window;
        const double dose = min(PH_DOWN_DOSE_ML, room);
        if (dose > 0.0) {
          dose_ph_down(dose);
          last_dose_ms_ph_down = millis();
          ph_down_dosed_this_window += dose;
        }
      }
    }
  }
}

/* ────────────────────────────────────────────────────────────
 *                   Hourly caps reset
 *   Resets rolling windows every 60 minutes independently.
 * ──────────────────────────────────────────────────────────── */
static void maybe_reset_hourly_caps() {
  const unsigned long now = millis();
  const unsigned long HOUR_MS = 3600000UL;

  if (now - ec_a_window_start    >= HOUR_MS) { ec_a_window_start    = now; ec_a_dosed_this_window    = 0.0; }
  if (now - ec_b_window_start    >= HOUR_MS) { ec_b_window_start    = now; ec_b_dosed_this_window    = 0.0; }
  if (now - ec_down_window_start >= HOUR_MS) { ec_down_window_start = now; ec_down_dosed_this_window = 0.0; }
  if (now - ph_down_window_start >= HOUR_MS) { ph_down_window_start = now; ph_down_dosed_this_window = 0.0; }
}

/* ────────────────────────────────────────────────────────────
 *                   EC status helper
 *   Short text for LCD column fit. No units printed.
 * ──────────────────────────────────────────────────────────── */
static const char* ec_status_text() {
  if (pending_B) return "A->B";
  const double err = EC_SETPOINT - ec_value;
  if (fabs(err) <= EC_DEADBAND) return "OK";
  return (err > 0.0) ? "ECup" : "ECdn";
}

/* ────────────────────────────────────────────────────────────
 *                           LCD UI (3 pages)
 *   A: pH raw, pHc+alpha, EC+SP+status, Temp
 *   B: EC usage (A,B,ECdn) and last action
 *   C: pH-Down usage, SP/DB, Uptime (hours)
 * ──────────────────────────────────────────────────────────── */

static void lcd_pad_to_eol(uint8_t n) {
  while (n < LCD_COLS) { lcd.print(' '); n++; }
}

/**
 * @brief Draw current page. Clears once on page flip or after reinit only.
 */
static void update_lcd() {
  const bool full = page_changed || (lcd_page != prev_page);
  if (full) {
    lcd.clear();               // single clear at flip or after reinit
    prev_page = lcd_page;
    page_changed = false;
  }
  switch (lcd_page) {
    case 0: lcd_screen_A(); break;
    case 1: lcd_screen_B(); break;
    default: lcd_screen_C(); break;
  }
}

/**
 * @brief Page A: pH raw + SP, pHc + alpha, EC + SP + status, Temp.
 */
static void lcd_screen_A() {
  char b1[12], b2[12]; uint8_t n;

  // Row 0: pH raw + SP
  lcd.setCursor(0, 0); n = 0;
  lcd.print(F("pH:")); n += 3;
  dtostrf(ph_raw, 5, 2, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(F("  SP:")); n += 5;
  dtostrf(PH_SETPOINT, 3, 1, b2); lcd.print(b2); n += strlen(b2);
  lcd_pad_to_eol(n);

  // Row 1: pHc + alpha
  lcd.setCursor(0, 1); n = 0;
  lcd.print(F("pHc:")); n += 4;
  dtostrf(isnan(ph_control) ? ph_raw : ph_control, 5, 2, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(F(" a=")); n += 3;
  dtostrf(PH_CONTROL_EMA, 3, 2, b2); lcd.print(b2); n += strlen(b2);
  lcd_pad_to_eol(n);

  // Row 2: EC + SP + status (no units printed)
  lcd.setCursor(0, 2); n = 0;
  lcd.print(F("EC:")); n += 3;
  dtostrf(ec_value, 6, 0, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(F(" SP:")); n += 4;
  dtostrf(EC_SETPOINT, 4, 0, b2); lcd.print(b2); n += strlen(b2);
  {
    const char* s = ec_status_text();
    uint8_t sLen = strlen(s);
    if (n + 1 + sLen <= LCD_COLS) {
      lcd.print(' '); n += 1;
      while (n + sLen < LCD_COLS) { lcd.print(' '); n++; }
      lcd.print(s); n += sLen;
    } else {
      lcd_pad_to_eol(n);
    }
  }

  // Row 3: temperature
  lcd.setCursor(0, 3); n = 0;
  lcd.print(F("Temp:")); n += 5;
  dtostrf(water_temperature, 4, 1, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(F(" C")); n += 2;
  lcd_pad_to_eol(n);
}

/**
 * @brief Page B: EC channel usage and last action summary.
 */
static void lcd_screen_B() {
  char b1[12], b2[12]; uint8_t n;

  // Row 0: A usage
  lcd.setCursor(0, 0); n = 0;
  lcd.print(F("A: ")); n += 3;
  ltoa(lround(ec_a_dosed_this_window), b1, 10); lcd.print(b1); n += strlen(b1);
  lcd.print(F("/")); n += 1;
  ltoa(lround(EC_A_MAX_ML_PER_HR), b2, 10);    lcd.print(b2); n += strlen(b2);
  lcd.print(F(" mL")); n += 3; lcd_pad_to_eol(n);

  // Row 1: B usage
  lcd.setCursor(0, 1); n = 0;
  lcd.print(F("B: ")); n += 3;
  ltoa(lround(ec_b_dosed_this_window), b1, 10); lcd.print(b1); n += strlen(b1);
  lcd.print(F("/")); n += 1;
  ltoa(lround(EC_B_MAX_ML_PER_HR), b2, 10);    lcd.print(b2); n += strlen(b2);
  lcd.print(F(" mL")); n += 3; lcd_pad_to_eol(n);

  // Row 2: EC-Down usage
  lcd.setCursor(0, 2); n = 0;
  lcd.print(F("ECD: ")); n += 5;
  ltoa(lround(ec_down_dosed_this_window), b1, 10); lcd.print(b1); n += strlen(b1);
  lcd.print(F("/")); n += 1;
  ltoa(lround(EC_DOWN_MAX_ML_PER_HR), b2, 10);     lcd.print(b2); n += strlen(b2);
  lcd.print(F(" mL")); n += 3; lcd_pad_to_eol(n);

  // Row 3: last action
  lcd.setCursor(0, 3); n = 0;
  lcd.print(F("Last ")); n += 5;
  lcd.print(last_action); n += strlen(last_action);
  lcd.print(F(" ")); n += 1;
  dtostrf(last_action_ml, 4, 1, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(F("mL")); n += 2; lcd_pad_to_eol(n);
}

/**
 * @brief Page C: pH-Down usage, pH set-point and deadband, uptime hours.
 */
static void lcd_screen_C() {
  char b1[16], b2[16]; uint8_t n;

  // Row 0: pH-Down used/cap
  lcd.setCursor(0, 0); n = 0;
  lcd.print(F("pHD: ")); n += 5;
  dtostrf(ph_down_dosed_this_window, 0, 1, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(F("/")); n += 1;
  dtostrf(PH_DOWN_MAX_ML_PER_HR,     0, 1, b2); lcd.print(b2); n += strlen(b2);
  lcd.print(F(" mL")); n += 3; lcd_pad_to_eol(n);

  // Row 1: pH SP and deadband
  lcd.setCursor(0, 1); n = 0;
  lcd.print(F("SP:")); n += 3;
  dtostrf(PH_SETPOINT, 3, 1, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(F("  DB:")); n += 5;
  dtostrf(PH_DEADBAND, 3, 2, b2); lcd.print(b2); n += strlen(b2);
  lcd_pad_to_eol(n);

  // Row 2: reserved for future use

  // Row 3: Uptime (hours)
  lcd.setCursor(0, 3); n = 0;
  lcd.print(F("Uptime: ")); n += 8;
  const double up_h = millis() / 3600000.0;
  dtostrf(up_h, 0, 1, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(F(" hours")); n += 6;
  lcd_pad_to_eol(n);
}

/* ────────────────────────────────────────────────────────────
 * ESP8266 CSV bridge
 *   Emits one line per SENSOR_INTERVAL for HA/MQTT ingestion.
 * ──────────────────────────────────────────────────────────── */
static void send_line_to_esp8266() {
  const bool ph_ok = !(isnan(ph_control) || ph_control < 0.0 || ph_control > 14.0);
  const bool t_ok  = !(isnan(water_temperature) || water_temperature < -50.0 || water_temperature > 125.0);
  const bool ec_ok = (ec_value >= 0.0 && ec_value < 1000000.0);
  const long  ec_i = ec_ok ? lround(ec_value) : -1;

  const double ph_err = ph_ok ? (PH_SETPOINT - ph_control) : NAN;
  const long   ec_err = ec_ok ? lround(EC_SETPOINT - ec_value) : 0;

  Serial1.print(F("PH_RAW,"));    Serial1.print(ph_raw, 2);
  Serial1.print(F(",PH_C,"));     if (ph_ok) Serial1.print(ph_control, 2); else Serial1.print(F("NaN"));
  Serial1.print(F(",EC,"));       if (ec_ok) Serial1.print(ec_i); else Serial1.print(F("NaN"));
  Serial1.print(F(",T,"));        if (t_ok)  Serial1.print(water_temperature, 1); else Serial1.print(F("NaN"));
  Serial1.print(F(",PH_SP,"));    Serial1.print(PH_SETPOINT, 1);
  Serial1.print(F(",EC_SP,"));    Serial1.print(lround(EC_SETPOINT));
  Serial1.print(F(",PH_ERR,"));   if (ph_ok) Serial1.print(ph_err, 2); else Serial1.print(F("NaN"));
  Serial1.print(F(",EC_ERR,"));   if (ec_ok) Serial1.print(ec_err); else Serial1.print(F("NaN"));
  Serial1.print(F(",A_HR,"));     Serial1.print(lround(ec_a_dosed_this_window));
  Serial1.print(F(",B_HR,"));     Serial1.print(lround(ec_b_dosed_this_window));
  Serial1.print(F(",ECD_HR,"));   Serial1.print(lround(ec_down_dosed_this_window));
  Serial1.print(F(",PHD_HR,"));   Serial1.print(ph_down_dosed_this_window, 1);
  Serial1.print(F(",LAST,"));     Serial1.print(last_action);
  Serial1.print(F(",LAST_ML,"));  Serial1.print(last_action_ml, 1);
  Serial1.print(F(",UPTIME_MS,"));Serial1.print(millis());
  Serial1.print("\n");
}
