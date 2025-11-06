/**
 * @file    hydra_main.ino
 * @brief   Hydroponics controller with EC-first dosing and EMA-smoothed pH control.
 *          20×4 I²C LCD UI with fixed-width rows that avoid flicker and label creep.
 *
 * @details
 * System overview
 *  - Sensors:
 *      • pH probe via DFRobot_PH library (analogue read -> volts -> pH).
 *      • DS18B20 water temperature for display and EC compensation reference checks.
 *      • Atlas Scientific EZO-EC in I²C mode for EC. Probe is kept “awake” between cycles.
 *  - Actuators (I²C slave pump controllers; addresses below):
 *      • Nutrient A (EC Up part A), Nutrient B (EC Up part B), EC Down (water), pH Down.
 *  - Control policy:
 *      1) EC is corrected first. pH control is deferred until EC is within a deadband and the
 *         solution has had time to mix and settle (separate lockouts for EC Up/Down).
 *      2) pH control uses an exponential moving average (EMA) of pH (“ph_control”) to reduce
 *         noise-driven dosing. Raw pH (“ph_raw”) is still displayed for visibility.
 *  - Safety:
 *      • Per-action mix/settle lockouts to avoid stacking doses into unmixed solution.
 *      • Hourly ml/h caps per channel to bound maximum chemical addition.
 *      • “Last valid EC” hold to hide transient 0 reads on the UI/telemetry.
 *  - Telemetry:
 *      • One CSV line per sample on Serial1 for ESP8266 bridging (Home Assistant/MQTT).
 *  - LCD UI (2004A @ 0x27):
 *      • Three pages rotated on a fixed schedule. Labels drawn once per page to prevent flicker.
 *      • Critical rows (Row0 pH and Row2 EC) are rendered as single 20-char strings each refresh
 *        to prevent values and labels “butting” into each other.
 *
 * Units and locale
 *  - Metric units. EC displayed as µS/cm (label omitted to fit 20-col layout cleanly).
 *  - Temperatures in °C. Australia/Sydney timezone is irrelevant here; uptime is device-local.
 *
 * Editing guidance
 *  - To tune responsiveness vs stability, adjust:
 *      • PH_CONTROL_EMA ∈ [0,1]: higher → faster reaction, lower → smoother control signal.
 *      • *_DEADBAND: widen to reduce chatter, narrow to track setpoint more tightly.
 *      • *_MIX_LOCKOUT_MS: increase if your reservoir mixes slowly.
 *      • *_MAX_ML_PER_HR: safety envelope for each channel.
 *  - Any change that affects row widths must keep the 20-character guarantee on the LCD helpers.
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
 *   These are the primary knobs you’ll tune in deployment.
 * ────────────────────────────────────────────────────────────
 */

// Analogue and 1-Wire pins
#define PH_ANALOG_PIN A5           // pH signal input (after signal conditioning)
#define DS18B20_PIN   4            // 1-Wire data pin for the DS18B20 probe

// I²C addresses for pumps and EC probe
#define PH_DOWN_PUMP_ADDRESS      0x32
#define NUTRIENT_A_PUMP_ADDRESS   0x33
#define NUTRIENT_B_PUMP_ADDRESS   0x34
#define EC_DOWN_PUMP_ADDRESS      0x35
#define EC_PROBE_ADDRESS          0x64   // Atlas EZO-EC I²C address

// LCD geometry and timing (20 columns × 4 rows)
#define LCD_I2C_ADDRESS 0x27
#define LCD_COLS        20
#define LCD_ROWS        4

// UI cadence controls
static const unsigned long LCD_UPDATE_MS = 1500UL;   // In-page numeric refresh; slower reduces I²C contention
static const unsigned long LCD_PAGE_MS   = 3000UL;   // Dwell time per page before rotating
static const unsigned long LCD_REINIT_MS = 300000UL; // Periodic re-init to avoid I²C lockups (no splash)

// ADC reference (mV). For 3V3 MCUs use 3300.0. For 5V Arduinos use 5000.0.
static const double ANALOG_REF_MV = 5000.0;

// Control set-points
static const double PH_SETPOINT = 5.8;       // Target pH
static const double EC_SETPOINT = 1600.0;    // Target EC in µS/cm (displayed as integer)

// Deadbands around set-points to prevent chatter
static const double PH_DEADBAND = 0.15;
static const double EC_DEADBAND = 50.0;      // µS/cm

// EC temperature compensation coefficient (fraction/°C relative to 25 °C)
static const double ALPHA = 0.02;

// pH control smoothing (EMA factor α ∈ [0,1])
// α=1.0 → no smoothing (control uses raw pH). α=0.2 is a common starting point.
static const double PH_CONTROL_EMA = 0.20;

// EC Up (two-part A then B) total dose per action and stagger between parts
static const double EC_UP_TOTAL_DOSE_ML = 8.0;
static const double EC_A_TO_B_RATIO     = 1.0;        // 1:1 split between A and B
static const unsigned long EC_AB_STAGGER_MS = 120000UL; // Wait time between A and B (mix window)

// EC Down (water) per-dose; set 0.0 to disable automatic EC down-dosing
static const double EC_DOWN_DOSE_ML = 0.0;

// pH Down per-dose
static const double PH_DOWN_DOSE_ML = 0.5;

// Mix/settle lockouts to avoid dosing into an unmixed reservoir
static const unsigned long EC_UP_MIX_LOCKOUT_MS   = 600000UL; // 10 min after any EC Up action
static const unsigned long EC_DOWN_MIX_LOCKOUT_MS = 600000UL; // 10 min after any EC Down action
static const unsigned long PH_DOWN_MIX_LOCKOUT_MS = 900000UL; // 15 min after any pH Down action

// Hourly caps to bound chemical addition rates (ml per hour)
static const double  EC_A_MAX_ML_PER_HR    = 40.0;
static const double  EC_B_MAX_ML_PER_HR    = 40.0;
static const double  EC_DOWN_MAX_ML_PER_HR = 100.0; // Only meaningful if EC_DOWN_DOSE_ML > 0
static const double  PH_DOWN_MAX_ML_PER_HR = 2.0;

// Sensor cadence: how often a full read/control cycle runs
static const unsigned long SENSOR_INTERVAL = 10000UL; // 10 s is conservative for hobby reservoirs

/* ────────────────────────────────────────────────────────────
 *                       GLOBALS AND INSTANCES
 *   These are shared across the module for performance and clarity.
 * ────────────────────────────────────────────────────────────
 */

DFRobot_PH ph_sensor;
OneWire oneWire(DS18B20_PIN);
DallasTemperature waterTemp(&oneWire);
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);

// Live measurements
static double ph_raw = 0.0;          // Instantaneous pH from the probe
static double ph_control = NAN;      // EMA-smoothed pH used for control
static double ec_value = 0.0;        // Temperature-compensated EC (µS/cm)
static double ec_last_good = NAN;    // Last valid EC to mask transient zeros
static double water_temperature = 0.0;

// Timing state
static unsigned long previous_millis = 0;
static unsigned long last_lcd_ms = 0;
static unsigned long last_page_ms = 0;
static unsigned long last_lcd_reinit_ms = 0;
static uint8_t lcd_page = 0; // 0..2

// LCD page-change and flicker control
static bool page_changed = false;
static uint8_t prev_page = 255;
static bool lcd_full_redraw = true;  // True → print static labels; False → update numbers only

// EC dosing state
static unsigned long last_dose_ms_ec_cycle = 0; // Time of last EC Up action (A dose)
static unsigned long last_dose_ms_ec_down  = 0; // Time of last EC Down dose
static int last_ec_direction = 0;               // +1 Up, −1 Down, 0 none

// Deferred EC B dose (staggered after A)
static bool   pending_B = false;
static double pending_B_ml = 0.0;
static unsigned long pending_B_ready_ms = 0;

// pH dosing state
static unsigned long last_dose_ms_ph_down = 0;

// Hourly accounting windows
static unsigned long ec_a_window_start    = 0;
static unsigned long ec_b_window_start    = 0;
static unsigned long ec_down_window_start = 0;
static unsigned long ph_down_window_start = 0;
static double ec_a_dosed_this_window    = 0.0;
static double ec_b_dosed_this_window    = 0.0;
static double ec_down_dosed_this_window = 0.0;
static double ph_down_dosed_this_window = 0.0;

// Last action summary for UI
static char   last_action[8] = "None"; // One of: "A","B","ECD","pHD","None"
static double last_action_ml = 0.0;

/* ────────────────────────────────────────────────────────────
 *                    WATCHDOG-SAFE HELPERS
 * ────────────────────────────────────────────────────────────
 */

static inline void watchdog_kick() {
  #if defined(ARDUINO_ARCH_AVR)
    wdt_reset();
  #endif
  yield();
}

static void safe_delay(unsigned long ms) {
  const unsigned long start = millis();
  while ((millis() - start) < ms) {
    watchdog_kick();
    delay(10);
  }
}

/* ────────────────────────────────────────────────────────────
 *                  FORWARD DECLARATIONS
 * ────────────────────────────────────────────────────────────
 */
static void initialise_pins();
static void initialise_devices();
static void initialise_display(bool showSplash);

static double read_water_temperature();
static double read_ph_sensor();
static double read_ec_sensor();                 // Parses current EC frame after an earlier "R"
static double compensate_ec(double ec_measured, double temperature);

static bool   i2c_send_with_retries(uint8_t address, const char* command, uint8_t max_attempts = 3);
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
static void lcd_screen_A();  // Overview: Row0 pHc+pHr, Row1 pH SP, Row2 EC+SP, Row3 Temp
static void lcd_screen_B();  // Hourly totals vs caps
static void lcd_screen_C();  // Last action + uptime

static void send_line_to_esp8266();

// LCD helpers
static void lcd_pad_to_eol(uint8_t n);
static void print_row20(const char* s);

/* ────────────────────────────────────────────────────────────
 *                              SETUP
 * ────────────────────────────────────────────────────────────
 */
void setup() {
  Serial.begin(9600);     // USB serial for debugging
  Serial1.begin(115200);  // ESP8266 bridge for telemetry

  Wire.begin();
  #if defined(TWOWIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(25000, true);
  #elif defined(ARDUINO_ARCH_AVR)
    Wire.setWireTimeout(25000, true);
  #endif

  #if defined(ARDUINO_ARCH_AVR)
    wdt_enable(WDTO_4S); // 4 s watchdog as a last resort
  #endif

  initialise_pins();
  initialise_devices();
  initialise_display(true);   // Short splash only at boot

  // Seed UI timing and force first draw
  last_page_ms = millis();
  lcd_page = 0;
  page_changed = true;
  update_lcd();

  Serial.println(F("Hydroponics controller initialised."));
}

/* ────────────────────────────────────────────────────────────
 *                               LOOP
 *   Sensor cadence and control ordering:
 *     Sleep EC → read pH → trigger EC "R" → wait → parse EC → keep EC awake → control.
 * ────────────────────────────────────────────────────────────
 */
void loop() {
  watchdog_kick();

  const unsigned long now = millis();

  if (now - previous_millis >= SENSOR_INTERVAL) {
    previous_millis = now;

    read_water_temperature();           // 1) T
    send_command(EC_PROBE_ADDRESS, "Sleep"); // 2) Quiet EC before pH
    safe_delay(500);
    read_ph_sensor();                   // 3) pH
    send_command(EC_PROBE_ADDRESS, "R");     // 4) Trigger EC
    safe_delay(600);
    read_ec_sensor();                   // 5) Parse EC
    send_command(EC_PROBE_ADDRESS, "Status"); // 6) Keep EC awake
    control_loop();                     // 7) Control

    // Debug snapshot
    Serial.print(F("Snapshot | pHraw: "));
    Serial.print(ph_raw, 2);
    Serial.print(F(" | pHc: "));
    Serial.print(isnan(ph_control) ? ph_raw : ph_control, 2);
    Serial.print(F(" | EC: "));
    Serial.print(ec_value, 0);
    Serial.print(F(" | T: "));
    Serial.print(water_temperature, 1);
    Serial.println(F(" C"));

    send_line_to_esp8266();             // 8) CSV telemetry
  }

  // UI cadence
  if (now - last_page_ms >= LCD_PAGE_MS) {
    last_page_ms = now;
    lcd_page = (lcd_page + 1) % 3;
    page_changed = true;
    last_lcd_ms = now;
    update_lcd();
  }

  if (now - last_lcd_ms >= LCD_UPDATE_MS) {
    last_lcd_ms = now;
    update_lcd();
  }

  if (now - last_lcd_reinit_ms >= LCD_REINIT_MS) {
    initialise_display(false);
    last_lcd_reinit_ms = now;
    page_changed = true;
    update_lcd();
  }

  // Deferred EC-B
  if (pending_B && now >= pending_B_ready_ms) {
    if ((now - last_dose_ms_ec_cycle) >= EC_UP_MIX_LOCKOUT_MS) {
      pending_B = false; // expired safely
    } else if (ec_b_dosed_this_window < EC_B_MAX_ML_PER_HR && pending_B_ml > 0.0) {
      dose_nutrient_B(pending_B_ml);
      ec_b_dosed_this_window += pending_B_ml;
      pending_B = false;
    }
  }
}

/* ────────────────────────────────────────────────────────────
 *                        INITIALISATION
 * ────────────────────────────────────────────────────────────
 */

static void initialise_pins() {
  pinMode(PH_ANALOG_PIN, INPUT);
}

static void initialise_devices() {
  ph_sensor.begin();
  waterTemp.begin();
  Serial.println(F("Devices initialised."));

  // Pin the EZO-EC temperature to 25.0 °C and verify readback
  send_command(EC_PROBE_ADDRESS, "T,25.0");
  safe_delay(300);
  char tbuf[32];
  send_command(EC_PROBE_ADDRESS, "T,?");
  size_t n = read_response_ascii(EC_PROBE_ADDRESS, tbuf, sizeof(tbuf));
  if (n > 0) {
    Serial.print(F("EZO-EC temperature readback: "));
    Serial.println(tbuf); // e.g. "?T,25.0"
  } else {
    Serial.println(F("EZO-EC temperature readback: no response"));
  }
}

static void initialise_display(bool showSplash) {
  lcd.init();
  lcd.noBlink();
  lcd.noCursor();
  lcd.noAutoscroll();
  lcd.leftToRight();
  lcd.backlight();
  lcd.clear();
  if (showSplash) {
    // 20-char padded splash
    lcd.setCursor(0, 0); lcd.print(F("Hydro Controller   "));
    lcd.setCursor(0, 1); lcd.print(F("Init...            "));
    safe_delay(400);
    lcd.clear();
  }
  lcd_full_redraw = true; // Force label redraw on next update
}

/* ────────────────────────────────────────────────────────────
 *                           SENSORS
 * ────────────────────────────────────────────────────────────
 */

static double read_water_temperature() {
  waterTemp.requestTemperatures();
  const double t = waterTemp.getTempCByIndex(0);
  if (t < -50.0 || t > 125.0) return water_temperature; // reject implausible read
  water_temperature = t;
  return water_temperature;
}

static double read_ph_sensor() {
  const double voltage_mV = (analogRead(PH_ANALOG_PIN) / 1024.0) * ANALOG_REF_MV;
  ph_raw = ph_sensor.readPH(voltage_mV, water_temperature);

  const double a = constrain(PH_CONTROL_EMA, 0.0, 1.0);
  if (isnan(ph_control)) ph_control = ph_raw; // seed EMA
  else ph_control = a * ph_raw + (1.0 - a) * ph_control;

  return ph_raw;
}

// Parses the already-ready EC frame triggered in loop()
static double read_ec_sensor() {
  const double raw = read_response_ec_numeric(EC_PROBE_ADDRESS);
  if (raw > 0.0) {
    ec_last_good = compensate_ec(raw, water_temperature);
    ec_value = ec_last_good;
  } else if (!isnan(ec_last_good)) {
    ec_value = ec_last_good;  // mask transient zero on UI/CSV
  } else {
    ec_value = 0.0;
  }
  return ec_value;
}

static double compensate_ec(double ec_measured, double temperature) {
  const double T_REF = 25.0;
  return ec_measured / (1.0 + ALPHA * (temperature - T_REF));
}

/* ────────────────────────────────────────────────────────────
 *                          I²C HELPERS
 * ────────────────────────────────────────────────────────────
 */

static bool i2c_send_with_retries(uint8_t address, const char* command, uint8_t max_attempts) {
  for (uint8_t attempt = 0; attempt < max_attempts; ++attempt) {
    Wire.beginTransmission(address);
    const char* p = command; while (*p) Wire.write(*p++);
    uint8_t err = Wire.endTransmission();
    if (err == 0) return true;
    Serial.print(F("I2C TX error ")); Serial.print(err);
    Serial.print(F(" to 0x")); Serial.println(address, HEX);
    safe_delay(5);
  }
  return false;
}

static void send_command(uint8_t address, const char *command) {
  if (!i2c_send_with_retries(address, command, 3)) {
    Serial.print(F("I2C send failed permanently to 0x"));
    Serial.println(address, HEX);
  }
  safe_delay(300); // device processing time
}

static float read_response_ec_numeric(uint8_t address) {
  const uint8_t bytes = Wire.requestFrom(address, (uint8_t)32);
  if (bytes == 0) {
    Serial.print(F("I2C read error from 0x")); Serial.println(address, HEX);
    return 0.0f;
  }
  const uint8_t status = Wire.read(); (void)status; // 1=OK, 254=pending
  char buf[31]; uint8_t i = 0;
  while (Wire.available() && i < sizeof(buf) - 1) {
    const char c = Wire.read();
    if (c >= 32 && c <= 126) buf[i++] = c;
  }
  buf[i] = '\0';
  return atof(buf);
}

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
 *                        DOSING PRIMITIVES
 * ────────────────────────────────────────────────────────────
 */

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
 *                  CONTROL LOOP (EC-FIRST)
 * ────────────────────────────────────────────────────────────
 */

static void control_loop() {
  const unsigned long now = millis();
  maybe_reset_hourly_caps();

  const bool ph_ok = !(isnan(ph_control) || ph_control < 0.0 || ph_control > 14.0);

  // Stage 1: EC priority
  const double ec_err = EC_SETPOINT - ec_value;   // +low→Up, −high→Down
  const bool   ec_in_range   = fabs(ec_err) <= EC_DEADBAND;
  const bool   ec_cycle_busy = pending_B;

  if (!ec_in_range || ec_cycle_busy) {
    if (!ec_cycle_busy) {
      const bool need_ec_down = (ec_err < 0.0);
      const unsigned long required_lockout = need_ec_down ? EC_DOWN_MIX_LOCKOUT_MS
                                                          : EC_UP_MIX_LOCKOUT_MS;
      const unsigned long last_ms = need_ec_down ? last_dose_ms_ec_down
                                                 : last_dose_ms_ec_cycle;

      if ((now - last_ms) >= required_lockout) {
        if (need_ec_down) {
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
    return; // Defer pH until EC is stable/in-range
  }

  // Stage 2: pH after EC settles
  const unsigned long ec_required_settle =
      (last_ec_direction < 0) ? EC_DOWN_MIX_LOCKOUT_MS :
      (last_ec_direction > 0) ? EC_UP_MIX_LOCKOUT_MS   : 0UL;

  const unsigned long last_ec_any = max(last_dose_ms_ec_cycle, last_dose_ms_ec_down);
  const bool ec_has_settled = (millis() - last_ec_any) >= ec_required_settle;

  if (ph_ok && ec_has_settled) {
    const double ph_err = PH_SETPOINT - ph_control; // negative → need pH Down
    const bool need_ph_down = (fabs(ph_err) > PH_DEADBAND && ph_err < 0.0);

    if (need_ph_down) {
      const bool lockout_ok = (millis() - last_dose_ms_ph_down) >= PH_DOWN_MIX_LOCKOUT_MS;
      const bool cap_ok = (ph_down_dosed_this_window < PH_DOWN_MAX_ML_PER_HR);

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
 *                   HOURLY CAP WINDOWS
 * ────────────────────────────────────────────────────────────
 */

static void maybe_reset_hourly_caps() {
  const unsigned long now = millis();
  const unsigned long HOUR_MS = 3600000UL;

  if (now - ec_a_window_start    >= HOUR_MS) { ec_a_window_start    = now; ec_a_dosed_this_window    = 0.0; }
  if (now - ec_b_window_start    >= HOUR_MS) { ec_b_window_start    = now; ec_b_dosed_this_window    = 0.0; }
  if (now - ec_down_window_start >= HOUR_MS) { ec_down_window_start = now; ec_down_dosed_this_window = 0.0; }
  if (now - ph_down_window_start >= HOUR_MS) { ph_down_window_start = now; ph_down_dosed_this_window = 0.0; }
}

/* ────────────────────────────────────────────────────────────
 *                           LCD HELPERS
 * ────────────────────────────────────────────────────────────
 */

static void lcd_pad_to_eol(uint8_t n) {
  while (n < LCD_COLS) { lcd.print(' '); n++; }
}

static void print_row20(const char* s) {
  char buf[21];
  size_t len = strnlen(s, 20);
  memcpy(buf, s, len);
  while (len < 20) buf[len++] = ' ';
  buf[20] = '\0';
  lcd.print(buf);
}

/* ────────────────────────────────────────────────────────────
 *                           LCD UI PAGES
 * ────────────────────────────────────────────────────────────
 */

static void update_lcd() {
  const bool full = page_changed || (lcd_page != prev_page);
  if (full) {
    lcd.clear();
    prev_page = lcd_page;
    page_changed = false;
    lcd_full_redraw = true;
  }
  switch (lcd_page) {
    case 0: lcd_screen_A(); break;
    case 1: lcd_screen_B(); break;
    default: lcd_screen_C(); break;
  }
}

/**
 * Page A — Overview
 * Row 0: "pHc:%5.2f  pHr:%5.2f"
 * Row 1: "pH SP:<value>"
 * Row 2: "EC:%6ld  SP:%5ld"
 * Row 3: "T: <xx.x> C"  ← note the space after the colon; value starts at column 3.
 */
static void lcd_screen_A() {
  char cbuf[6], rbuf[6], row0[32], row2[32], b1[16];

  // Row 0: pH control and raw as one fixed-width line
  dtostrf(isnan(ph_control) ? ph_raw : ph_control, 5, 2, cbuf);
  dtostrf(ph_raw, 5, 2, rbuf);
  snprintf(row0, sizeof(row0), "pHc:%s  pHr:%s", cbuf, rbuf);
  lcd.setCursor(0,0);
  print_row20(row0);

  if (lcd_full_redraw) {
    // Row 1 scaffold: pH SP label with a reserved value slot
    lcd.setCursor(0,1); lcd.print(F("pH SP:           ")); // value at col 7..10
    // Row 3 scaffold: temperature line; leave at least one space after "T:"
    lcd.setCursor(0,3); lcd.print(F("T:     C          "));
    lcd_full_redraw = false;
  }

  // Row 1 value: pH setpoint
  dtostrf(PH_SETPOINT, 4, 1, b1);
  lcd.setCursor(7,1);  lcd.print(b1);

  // Row 2: EC current and setpoint rendered as one fixed-width line
  long ec_cur = lround(ec_value);
  long ec_sp  = lround(EC_SETPOINT);
  snprintf(row2, sizeof(row2), "EC:%6ld  SP:%5ld", ec_cur, ec_sp);
  lcd.setCursor(0,2);
  print_row20(row2);

  // Row 3 value: water temperature
  dtostrf(water_temperature, 4, 1, b1);
  lcd.setCursor(3,3);  // start at column 3, so "T:" then a space, then the number
  lcd.print(b1);
}

/**
 * Page B — Hourly totals vs caps (units intentionally omitted).
 */
static void lcd_screen_B() {
  char b1[16], b2[16]; uint8_t n = 0;

  // Row 0: Nutrient A hourly
  lcd.setCursor(0, 0); n = 0;
  lcd.print(F("A: ")); n += 3;
  ltoa(lround(ec_a_dosed_this_window), b1, 10); lcd.print(b1); n += strlen(b1);
  lcd.print(F("/")); n += 1;
  ltoa(lround(EC_A_MAX_ML_PER_HR), b2, 10);     lcd.print(b2); n += strlen(b2);
  lcd_pad_to_eol(n);

  // Row 1: Nutrient B hourly
  lcd.setCursor(0, 1); n = 0;
  lcd.print(F("B: ")); n += 3;
  ltoa(lround(ec_b_dosed_this_window), b1, 10); lcd.print(b1); n += strlen(b1);
  lcd.print(F("/")); n += 1;
  ltoa(lround(EC_B_MAX_ML_PER_HR), b2, 10);     lcd.print(b2); n += strlen(b2);
  lcd_pad_to_eol(n);

  // Row 2: EC Down hourly
  lcd.setCursor(0, 2); n = 0;
  lcd.print(F("EC Dn: ")); n += 7;
  ltoa(lround(ec_down_dosed_this_window), b1, 10); lcd.print(b1); n += strlen(b1);
  lcd.print(F("/")); n += 1;
  ltoa(lround(EC_DOWN_MAX_ML_PER_HR), b2, 10);     lcd.print(b2); n += strlen(b2);
  lcd_pad_to_eol(n);

  // Row 3: pH Down hourly
  lcd.setCursor(0, 3); n = 0;
  lcd.print(F("pH Dn: ")); n += 7;
  dtostrf(ph_down_dosed_this_window, 0, 1, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(F("/")); n += 1;
  dtostrf(PH_DOWN_MAX_ML_PER_HR, 0, 1, b2);     lcd.print(b2); n += strlen(b2);
  lcd_pad_to_eol(n);
}

/**
 * Page C — Operational status: last action and uptime.
 */
static void lcd_screen_C() {
  char b1[16]; uint8_t n = 0;

  // Row 0: Last action summary
  lcd.setCursor(0, 0); n = 0;
  lcd.print(F("Last: ")); n += 6;
  lcd.print(last_action); n += strlen(last_action);
  lcd.print(F(" ")); n += 1;
  dtostrf(last_action_ml, 4, 1, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(F(" mL")); n += 3; lcd_pad_to_eol(n);

  // Rows 1–2: cleared
  for (uint8_t row = 1; row <= 2; row++) {
    lcd.setCursor(0, row);
    for (uint8_t i = 0; i < LCD_COLS; i++) lcd.print(' ');
  }

  // Row 3: Uptime (hours)
  lcd.setCursor(0, 3); n = 0;
  lcd.print(F("Uptime: ")); n += 8;
  const double up_h = millis() / 3600000.0;
  dtostrf(up_h, 0, 1, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(F(" h")); n += 2; lcd_pad_to_eol(n);
}

/* ────────────────────────────────────────────────────────────
 *                        ESP8266 CSV BRIDGE
 * ────────────────────────────────────────────────────────────
 */

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
