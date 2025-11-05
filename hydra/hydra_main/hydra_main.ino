/**
 * @file    hydra_main.ino
 * @brief   Hydroponics controller with EC-first strategy and EMA-driven pH dosing.
 *
 * @details
 * - Prioritises EC correction (Up via two-part A/B or Down via water) before pH dosing.
 * - pH control uses an exponential moving average (EMA) to reduce noise-induced dosing.
 * - LCD shows both RAW pH and EMA (control) pH simultaneously.
 * - Hourly caps per direction and post-dose lockouts for mixing safety.
 * - EC temperature compensation to 25 °C with linear coefficient ALPHA.
 * - Atlas EZO-EC is pinned to 25.0 °C at boot for stable compensation.
 * - ESP8266 serial bridge outputs compact CSV for Home Assistant/MQTT ingestion.
 *
 * @note
 *   Set PH_CONTROL_EMA to 1.0 to disable smoothing (EMA equals the instantaneous reading).
 *   Keep sensor cadence and lockouts conservative to avoid oscillation and chemical overshoot.
 *
 * @hardware
 *   - pH: DFRobot pH analog board into PH_ANALOG_PIN.
 *   - EC: Atlas Scientific EZO-EC on I²C.
 *   - Temp: DS18B20 on OneWire.
 *   - Dosing pumps: I²C peristaltic modules at the addresses below.
 *   - LCD: 20x4 I²C (2004A).
 *
 * @safety
 *   - Verify I²C pump addresses and flow calibration before enabling dosing.
 *   - Set realistic hourly caps and lockouts to match reservoir volume and chemistry.
 *   - pH-Up is intentionally disabled. Only pH-Down is supported here.
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "DFRobot_PH.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>

#if defined(ARDUINO_ARCH_AVR)
  #include <avr/wdt.h>
#endif

/* ────────────────────────────────────────────────────────────
 *                     USER-SETTABLE PARAMETERS
 *   (ALL AND ONLY THE LINES BELOW USE ALL-CAPS COMMENTS)
 * ──────────────────────────────────────────────────────────── */

// ---------- Pins ----------
#define PH_ANALOG_PIN A5          // CHANGE IF YOUR PH ANALOG OUTPUT IS ON A DIFFERENT ANALOG PIN
#define DS18B20_PIN   4           // CHANGE IF YOUR DS18B20 DATA LINE IS ON A DIFFERENT DIGITAL PIN

// ---------- I2C addresses ----------
#define PH_DOWN_PUMP_ADDRESS      0x32  // CHANGE IF YOUR PH-DOWN PUMP USES A DIFFERENT I2C ADDRESS
#define NUTRIENT_A_PUMP_ADDRESS   0x33  // CHANGE IF YOUR NUTRIENT-A PUMP (FORMER PH-UP) USES A DIFFERENT I2C ADDRESS
#define NUTRIENT_B_PUMP_ADDRESS   0x34  // CHANGE IF YOUR NUTRIENT-B PUMP USES A DIFFERENT I2C ADDRESS
#define EC_DOWN_PUMP_ADDRESS      0x35  // CHANGE IF YOUR WATER (EC-DOWN) PUMP USES A DIFFERENT I2C ADDRESS
#define EC_PROBE_ADDRESS          0x64  // CHANGE IF YOUR ATLAS EZO-EC ADDRESS IS DIFFERENT

// ---------- LCD (2004A) ----------
#define LCD_I2C_ADDRESS 0x27      // COMMON: 0x27 OR 0x3F — CHANGE TO MATCH YOUR MODULE
#define LCD_COLS        20        // CHANGE IF USING A DIFFERENT LCD WIDTH
#define LCD_ROWS        4         // CHANGE IF USING A DIFFERENT LCD HEIGHT
#define LCD_UPDATE_MS   1000UL    // CHANGE TO ADJUST LCD REFRESH PERIOD (MS)

// Page timing: make one full page cycle match the 10 s sensor cadence.
#define LCD_PAGES                     3       // NUMBER OF LCD PAGES (A,B,C)
#define LCD_SYNC_TO_SENSOR_INTERVAL   1       // 1=ON: auto size page time to SENSOR_INTERVAL/LCD_PAGES
#define LCD_PAGE_MS_USER              3000UL  // ONLY USED IF LCD_SYNC_TO_SENSOR_INTERVAL==0

// ---------- ADC reference ----------
#define ANALOG_REF_MV   5000.0    // CHANGE TO 3300.0 IF YOUR MCU ADC REFERENCE IS 3.3 V

// ---------- Setpoints ----------
static const double PH_SETPOINT = 5.8;    // CHANGE TO YOUR TARGET PH (TYPICALLY 5.5–6.2)
static const double EC_SETPOINT = 1600.0; // CHANGE TO YOUR TARGET EC IN µS/cm (E.G., 800–1800)

// ---------- Control bands ----------
static const double PH_DEADBAND = 0.15;   // CHANGE TO WIDEN/NARROW PH ALLOWED ERROR (PH UNITS, ±)
static const double EC_DEADBAND = 50.0;   // CHANGE TO WIDEN/NARROW EC ALLOWED ERROR (µS/cm, ±)

// ---------- EC temperature compensation ----------
static const double ALPHA = 0.02;         // CHANGE TEMP COEFFICIENT FRACTION PER °C (TYPICAL 0.02)

// ---------- pH control smoothing ----------
static const double PH_CONTROL_EMA = 0.20; // CHANGE EMA FOR CONTROL pH (0..1; 1.0 = no smoothing)

// ---------- Two-part nutrient dosing (per action) ----------
static const double EC_UP_TOTAL_DOSE_ML = 8.0;            // CHANGE SINGLE "EC-UP TOTAL" DOSE SIZE (A+B SUM, ML)
static const double EC_A_TO_B_RATIO     = 1.0;            // CHANGE A:B RATIO (1.0=EQUAL PARTS; 2.0=TWICE A AS B, ETC.)
static const unsigned long EC_AB_STAGGER_MS = 120000UL;   // CHANGE MIX TIME BETWEEN A AND B DOSES (MS)

// ---------- EC-Down (water) per-dose amount ----------
static const double EC_DOWN_DOSE_ML = 0.0;                // SET TO >0 TO ENABLE EC-DOWN (WATER) DOSING

// ---------- pH dosing (Down only) ----------
static const double PH_DOWN_DOSE_ML = 0.5;                // CHANGE SINGLE PH-DOWN DOSE SIZE (ML)

// ---------- Per-direction settle lockouts (ms) ----------
static const unsigned long EC_UP_MIX_LOCKOUT_MS   = 600000UL; // CHANGE MIX TIME AFTER A->B NUTRIENT CYCLE (MS)
static const unsigned long EC_DOWN_MIX_LOCKOUT_MS = 600000UL; // CHANGE MIX TIME AFTER WATER (MS)
static const unsigned long PH_DOWN_MIX_LOCKOUT_MS = 900000UL; // CHANGE LOCKOUT AFTER PH-DOWN (MS)

// ---------- Hourly caps (mL/h) per direction ----------
static const double  EC_A_MAX_ML_PER_HR    = 40.0;  // CHANGE MAXIMUM NUTRIENT-A ML PER HOUR
static const double  EC_B_MAX_ML_PER_HR    = 40.0;  // CHANGE MAXIMUM NUTRIENT-B ML PER HOUR
static const double  EC_DOWN_MAX_ML_PER_HR = 100.0; // SET TO >0 IF ENABLING EC-DOWN
static const double  PH_DOWN_MAX_ML_PER_HR = 2.0;   // CHANGE MAXIMUM PH-DOWN ML PER HOUR

// ---------- Sensor / control cadence ----------
static const unsigned long SENSOR_INTERVAL = 10000UL; // CHANGE HOW OFTEN TO READ/CONTROL (MS)

/* ────────────────────────────────────────────────────────────
 *                END OF USER-SETTABLE PARAMETERS
 * ──────────────────────────────────────────────────────────── */


/* ============================================================
 *                   Globals and Instances
 * ============================================================ */

#if LCD_SYNC_TO_SENSOR_INTERVAL
  #define LCD_PAGE_MS  (SENSOR_INTERVAL / LCD_PAGES)   // e.g., 10000/3 = 3333 ms
#else
  #define LCD_PAGE_MS  LCD_PAGE_MS_USER
#endif
#define LCD_REINIT_MS  300000UL

DFRobot_PH ph_sensor;
OneWire oneWire(DS18B20_PIN);
DallasTemperature waterTemp(&oneWire);
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);

// Live readings
static double ph_raw = 0.0;            // Instantaneous pH from sensor
static double ph_control = NAN;         // EMA used for CONTROL decisions
static double ec_value = 0.0;           // µS/cm @ 25 °C
static double water_temperature = 0.0;  // °C

// Timing
static unsigned long previous_millis = 0;
static unsigned long last_lcd_ms = 0;
static unsigned long last_page_ms = 0;
static unsigned long last_lcd_reinit_ms = 0;
static uint8_t lcd_page = 0;            // 0 = A, 1 = B, 2 = C

// EC dosing state
static unsigned long last_dose_ms_ec_cycle = 0;  // Time of last A dose (cycle start)
static unsigned long last_dose_ms_ec_down  = 0;  // Time of last EC-Down dose
static int last_ec_direction = 0;                // +1 up (A/B), -1 down (water), 0 none

// Deferred B dose after A
static bool   pending_B = false;
static double pending_B_ml = 0.0;
static unsigned long pending_B_ready_ms = 0;

// pH dosing state
static unsigned long last_dose_ms_ph_down = 0;

// Hourly caps windows
static unsigned long ec_a_window_start    = 0;
static unsigned long ec_b_window_start    = 0;
static unsigned long ec_down_window_start = 0;
static unsigned long ph_down_window_start = 0;

static double ec_a_dosed_this_window    = 0.0;
static double ec_b_dosed_this_window    = 0.0;
static double ec_down_dosed_this_window = 0.0;
static double ph_down_dosed_this_window = 0.0;

// Last action indicator
static char   last_action[8] = "None";   // "A", "B", "ECD", "pHD", "None"
static double last_action_ml = 0.0;

/* ============================================================
 *                         Support utilities
 * ============================================================ */

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

static bool i2c_send_with_retries(uint8_t address, const char* command, uint8_t max_attempts = 3) {
  for (uint8_t attempt = 0; attempt < max_attempts; ++attempt) {
    Wire.beginTransmission(address);
    const char* p = command;
    while (*p) Wire.write(*p++);
    uint8_t err = Wire.endTransmission();
    if (err == 0) return true;
    Serial.print(F("I2C TX error ")); Serial.print(err);
    Serial.print(F(" to 0x")); Serial.println(address, HEX);
    safe_delay(5);
  }
  return false;
}

/* ============================================================
 *                    Function Declarations
 * ============================================================ */

// Initialisation
static void initialise_pins();
static void initialise_devices();
static void initialise_display();

// Sensors
static double read_water_temperature();
static double read_ph_sensor();
static double read_ec_sensor();
static double compensate_ec(double ec_measured, double temperature);

// I²C helpers
static void   send_command(uint8_t address, const char *command);
static float  read_response_ec_numeric(uint8_t address);
static size_t read_response_ascii(uint8_t address, char *buf, size_t buflen);

// Dosing primitives
static void dose_nutrient_A(double ml);
static void dose_nutrient_B(double ml);
static void dose_ec_down(double ml);
static void dose_ph_down(double ml);

// Control loop and caps
static void control_loop();
static void maybe_reset_hourly_caps();

// LCD UI
static void update_lcd();
static void lcd_screen_A();
static void lcd_screen_B();
static void lcd_screen_C();

// ESP8266 bridge
static void send_line_to_esp8266();

// EC status text helper
static const char* ec_status_text();

/* ============================================================
 *                              setup
 * ============================================================ */
void setup() {
  Serial.begin(9600);          // Host serial
  Serial1.begin(115200);       // ESP8266 bridge

  Wire.begin();
  #if defined(TWOWIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(25000, true);
  #elif defined(ARDUINO_ARCH_AVR)
    Wire.setWireTimeout(25000, true);
  #endif

  #if defined(ARDUINO_ARCH_AVR)
    wdt_enable(WDTO_4S);
  #endif

  initialise_pins();
  initialise_devices();
  initialise_display();

  Serial.println(F("Hydroponics controller initialised."));
}

/* ============================================================
 *                               loop
 * ============================================================ */
void loop() {
  watchdog_kick();
  const unsigned long now = millis();

  // Sensor cadence + control (10 s default)
  if (now - previous_millis >= SENSOR_INTERVAL) {
    previous_millis = now;

    read_water_temperature();

    // Suspend EC measurement noise before pH sample
    send_command(EC_PROBE_ADDRESS, "Sleep");
    safe_delay(500);
    read_ph_sensor();

    // Wake and read EC
    read_ec_sensor();

    // Decision and dosing
    control_loop();

    // Snapshot to host
    Serial.print(F("Snapshot | pHraw: "));
    Serial.print(ph_raw, 2);
    Serial.print(F(" | pHc: "));
    Serial.print(isnan(ph_control) ? ph_raw : ph_control, 2);
    Serial.print(F(" | EC: "));
    Serial.print(ec_value, 0);
    Serial.print(F(" µS/cm | T: "));
    Serial.print(water_temperature, 1);
    Serial.println(F(" °C"));

    // CSV to ESP8266
    send_line_to_esp8266();
  }

  // LCD page rotation and refresh
  if (now - last_page_ms >= LCD_PAGE_MS) {
    last_page_ms = now;
    lcd_page = (lcd_page + 1) % LCD_PAGES; // 0..(LCD_PAGES-1)

    // Optional sync: when rolling over to page 0, align next sensor cycle
    #if LCD_SYNC_TO_SENSOR_INTERVAL
      if (lcd_page == 0) {
        // Force next sensor cycle immediately after completing 3 pages.
        // This keeps the "read → show A,B,C → read" rhythm.
        if ((now - previous_millis) < SENSOR_INTERVAL) {
          previous_millis = now - SENSOR_INTERVAL;
        }
      }
    #endif
  }

  if (now - last_lcd_reinit_ms >= LCD_REINIT_MS) {
    initialise_display();
    last_lcd_reinit_ms = now;
  }
  if (now - last_lcd_ms >= LCD_UPDATE_MS) {
    last_lcd_ms = now;
    update_lcd();
  }

  // Handle deferred B dose (non-blocking)
  if (pending_B && now >= pending_B_ready_ms) {
    if ((now - last_dose_ms_ec_cycle) >= EC_UP_MIX_LOCKOUT_MS) {
      // Missed window; expire pending B and allow new cycle.
      pending_B = false;
    } else if (ec_b_dosed_this_window < EC_B_MAX_ML_PER_HR && pending_B_ml > 0.0) {
      dose_nutrient_B(pending_B_ml);
      ec_b_dosed_this_window += pending_B_ml;
      pending_B = false;
    }
  }
}

/* ============================================================
 *                        Initialisation
 * ============================================================ */

static void initialise_pins() {
  pinMode(PH_ANALOG_PIN, INPUT);
}

static void initialise_devices() {
  ph_sensor.begin();
  waterTemp.begin();
  Serial.println(F("Devices initialised."));

  // Fix EZO-EC internal temperature to T=25.0 for stable compensation.
  send_command(EC_PROBE_ADDRESS, "T,25.0");
  safe_delay(300);

  // Confirm set temperature.
  char tbuf[32];
  send_command(EC_PROBE_ADDRESS, "T,?");
  size_t n = read_response_ascii(EC_PROBE_ADDRESS, tbuf, sizeof(tbuf));
  if (n > 0) {
    Serial.print(F("EZO-EC temperature readback: "));
    Serial.println(tbuf); // e.g., "?T,25.0"
  } else {
    Serial.println(F("EZO-EC temperature readback: no response"));
  }
}

static void initialise_display() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(F("Hydro Controller"));
  lcd.setCursor(0, 1); lcd.print(F("Init..."));
  safe_delay(800);
  lcd.clear();
}

/* ============================================================
 *                           Sensors
 * ============================================================ */

static double read_water_temperature() {
  waterTemp.requestTemperatures();
  const double t = waterTemp.getTempCByIndex(0);
  if (t < -50.0 || t > 125.0) {
    Serial.println(F("DS18B20 out of range; keeping last value."));
    return water_temperature;
  }
  water_temperature = t;
  return water_temperature;
}

static double read_ph_sensor() {
  const double voltage_mV = (analogRead(PH_ANALOG_PIN) / 1023.0) * ANALOG_REF_MV;
  ph_raw = ph_sensor.readPH(voltage_mV, water_temperature);

  // EMA for control: ph_control_k = a*raw + (1-a)*ph_control_(k-1)
  const double a = constrain(PH_CONTROL_EMA, 0.0, 1.0);
  if (isnan(ph_control)) ph_control = ph_raw;  // Seed on first run
  else ph_control = a * ph_raw + (1.0 - a) * ph_control;

  return ph_raw;
}

static double read_ec_sensor() {
  send_command(EC_PROBE_ADDRESS, "R");
  safe_delay(600);
  const double raw = read_response_ec_numeric(EC_PROBE_ADDRESS);
  ec_value = compensate_ec(raw, water_temperature);
  return ec_value;
}

static double compensate_ec(double ec_measured, double temperature) {
  const double T_REF = 25.0;
  return ec_measured / (1.0 + ALPHA * (temperature - T_REF));
}

/* ============================================================
 *                          I²C helpers
 * ============================================================ */

static void send_command(uint8_t address, const char *command) {
  if (!i2c_send_with_retries(address, command, 3)) {
    Serial.print(F("I2C send failed to 0x")); Serial.println(address, HEX);
  }
  safe_delay(300);
}

static float read_response_ec_numeric(uint8_t address) {
  const uint8_t bytes = Wire.requestFrom(address, (uint8_t)32);
  if (bytes == 0) {
    Serial.print(F("I2C read error from 0x")); Serial.println(address, HEX);
    return 0.0f;
  }
  const uint8_t status = Wire.read(); (void)status; // discard status byte
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
  const uint8_t status = Wire.read(); (void)status; // discard status byte
  size_t i = 0;
  while (Wire.available() && i < buflen - 1) {
    const char c = Wire.read();
    if (c >= 32 && c <= 126) buf[i++] = c;
  }
  buf[i] = '\0';
  return i;
}

/* ============================================================
 *                        Dosing primitives
 * ============================================================ */

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

/* ============================================================
 *                 EC-first conservative control loop
 * ============================================================ */

static void control_loop() {
  const unsigned long now = millis();

  // Maintain hourly windows
  maybe_reset_hourly_caps();

  // Validate control pH
  const bool ph_ok = !(isnan(ph_control) || ph_control < 0.0 || ph_control > 14.0);

  // ----- EC priority phase -----
  const double ec_err = EC_SETPOINT - ec_value;    // + low -> up; - high -> down
  const bool   ec_in_range   = fabs(ec_err) <= EC_DEADBAND;
  const bool   ec_cycle_busy = pending_B;

  if (!ec_in_range || ec_cycle_busy) {
    if (!ec_cycle_busy) {
      const bool need_ec_down = (ec_err < 0.0);
      const unsigned long required_lockout = need_ec_down ? EC_DOWN_MIX_LOCKOUT_MS
                                                          : EC_UP_MIX_LOCKOUT_MS;
      const unsigned long last_ms = need_ec_down ? last_dose_ms_ec_down
                                                 : last_dose_ms_ec_cycle;

      // Lockout respected?
      if ((now - last_ms) >= required_lockout) {
        if (need_ec_down) {
          // EC-Down via water, if enabled and within hourly cap
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
          // EC-Up via two-part A then delayed B
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
    return; // Do not evaluate pH until EC is resolved and settled
  }

  // ----- pH phase (Down only) after EC has settled -----
  const unsigned long ec_required_settle =
      (last_ec_direction < 0) ? EC_DOWN_MIX_LOCKOUT_MS :
      (last_ec_direction > 0) ? EC_UP_MIX_LOCKOUT_MS   : 0UL;

  const unsigned long last_ec_any = max(last_dose_ms_ec_cycle, last_dose_ms_ec_down);
  const bool ec_has_settled = (millis() - last_ec_any) >= ec_required_settle;

  if (ph_ok && ec_has_settled) {
    // Control uses EMA signal
    const double ph_err = PH_SETPOINT - ph_control; // negative => need Down
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

/* ============================================================
 *                     Hourly caps window reset
 * ============================================================ */

static void maybe_reset_hourly_caps() {
  const unsigned long now = millis();
  const unsigned long HOUR_MS = 3600000UL;

  if (now - ec_a_window_start    >= HOUR_MS) { ec_a_window_start    = now; ec_a_dosed_this_window    = 0.0; }
  if (now - ec_b_window_start    >= HOUR_MS) { ec_b_window_start    = now; ec_b_dosed_this_window    = 0.0; }
  if (now - ec_down_window_start >= HOUR_MS) { ec_down_window_start = now; ec_down_dosed_this_window = 0.0; }
  if (now - ph_down_window_start >= HOUR_MS) { ph_down_window_start = now; ph_down_dosed_this_window = 0.0; }
}

/* ============================================================
 *                  EC status text helper (for LCD)
 * ============================================================ */
static const char* ec_status_text() {
  if (pending_B) return "A->B";                 // Stagger in progress
  const double err = EC_SETPOINT - ec_value;    // + low -> need up; - high -> need down
  if (fabs(err) <= EC_DEADBAND) return "OK";    // In band
  return (err > 0.0) ? "ECup" : "ECdn";
}

/* ============================================================
 *                           LCD UI (3 pages)
 *   Page A: RAW pH + EMA pH + setpoints + temp + EC status
 *   Page B: EC usage (A/B and ECD) + last action
 *   Page C: pHD usage with 0.1 mL precision (+ SP/DB/EMA)
 * ============================================================ */

static void update_lcd() {
  switch (lcd_page) {
    case 0: lcd_screen_A(); break;
    case 1: lcd_screen_B(); break;
    default: lcd_screen_C(); break;
  }
}

static void lcd_screen_A() {
  char b1[12], b2[12];
  uint8_t n;
  lcd.clear();

  // Row 0: raw pH and setpoint
  lcd.setCursor(0, 0); n = 0;
  lcd.print(F("pH:")); n += 3;
  dtostrf(ph_raw, 5, 2, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(F("  SP:")); n += 5;
  dtostrf(PH_SETPOINT, 3, 1, b2); lcd.print(b2); n += strlen(b2);
  while (n < LCD_COLS) lcd.print(' '), n++;

  // Row 1: control pH (EMA) with alpha
  lcd.setCursor(0, 1); n = 0;
  lcd.print(F("pHc:")); n += 4;
  dtostrf(isnan(ph_control) ? ph_raw : ph_control, 5, 2, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(F(" a=")); n += 3;
  dtostrf(PH_CONTROL_EMA, 3, 2, b2); lcd.print(b2); n += strlen(b2);
  while (n < LCD_COLS) lcd.print(' '), n++;

  // Row 2: EC, setpoint, status
  lcd.setCursor(0, 2); n = 0;
  lcd.print(F("EC:")); n += 3;
  dtostrf(ec_value, 6, 0, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(F(" uS SP:")); n += 7;
  dtostrf(EC_SETPOINT, 4, 0, b2); lcd.print(b2); n += strlen(b2);
  const char* s = ec_status_text();
  uint8_t sLen = strlen(s);
  if (n + 1 + sLen <= LCD_COLS) {
    lcd.print(' '); n += 1;
    while (n + sLen < LCD_COLS) { lcd.print(' '); n++; }
    lcd.print(s); n += sLen;
  } else {
    while (n < LCD_COLS) lcd.print(' '), n++;
  }

  // Row 3: temperature
  lcd.setCursor(0, 3); n = 0;
  lcd.print(F("Temp:")); n += 5;
  dtostrf(water_temperature, 4, 1, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(F(" C")); n += 2;
  while (n < LCD_COLS) lcd.print(' '), n++;
}

static void lcd_screen_B() {
  char b1[12], b2[12]; uint8_t n;
  lcd.clear();

  // Row 0: A usage
  lcd.setCursor(0, 0); n = 0;
  lcd.print(F("A: ")); n += 3;
  ltoa(lround(ec_a_dosed_this_window), b1, 10); lcd.print(b1); n += strlen(b1);
  lcd.print(F("/")); n += 1;
  ltoa(lround(EC_A_MAX_ML_PER_HR), b2, 10);    lcd.print(b2); n += strlen(b2);
  lcd.print(F(" mL")); n += 3; while (n < LCD_COLS) lcd.print(' '), n++;

  // Row 1: B usage
  lcd.setCursor(0, 1); n = 0;
  lcd.print(F("B: ")); n += 3;
  ltoa(lround(ec_b_dosed_this_window), b1, 10); lcd.print(b1); n += strlen(b1);
  lcd.print(F("/")); n += 1;
  ltoa(lround(EC_B_MAX_ML_PER_HR), b2, 10);    lcd.print(b2); n += strlen(b2);
  lcd.print(F(" mL")); n += 3; while (n < LCD_COLS) lcd.print(' '), n++;

  // Row 2: EC-Down usage (shorthand)
  lcd.setCursor(0, 2); n = 0;
  lcd.print(F("ECD: ")); n += 5;
  ltoa(lround(ec_down_dosed_this_window), b1, 10); lcd.print(b1); n += strlen(b1);
  lcd.print(F("/")); n += 1;
  ltoa(lround(EC_DOWN_MAX_ML_PER_HR), b2, 10);     lcd.print(b2); n += strlen(b2);
  lcd.print(F(" mL")); n += 3;
  while (n < LCD_COLS) lcd.print(' '), n++;

  // Row 3: last action
  lcd.setCursor(0, 3); n = 0;
  lcd.print(F("Last ")); n += 5;
  lcd.print(last_action); n += strlen(last_action);
  lcd.print(F(" ")); n += 1;
  dtostrf(last_action_ml, 4, 1, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(F("mL")); n += 2;
  while (n < LCD_COLS) lcd.print(' '), n++;
}

static void lcd_screen_C() {
  char b1[16], b2[16]; uint8_t n;
  lcd.clear();

  // Row 0: short title
  lcd.setCursor(0, 0); n = 0;
  lcd.print(F("pHD mL/h")); n = LCD_COLS;

  // Row 1: pH-Down used/cap (shorthand)
  lcd.setCursor(0, 1); n = 0;
  lcd.print(F("pHD: ")); n += 5;
  dtostrf(ph_down_dosed_this_window, 0, 1, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(F("/")); n += 1;
  dtostrf(PH_DOWN_MAX_ML_PER_HR,     0, 1, b2); lcd.print(b2); n += strlen(b2);
  while (n < LCD_COLS) lcd.print(' '), n++;

  // Row 2: setpoint and deadband (shorthand)
  lcd.setCursor(0, 2); n = 0;
  lcd.print(F("SP:")); n += 3;
  dtostrf(PH_SETPOINT, 3, 1, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(F("  DB:")); n += 5;
  dtostrf(PH_DEADBAND, 3, 2, b2); lcd.print(b2); n += strlen(b2);
  while (n < LCD_COLS) lcd.print(' '), n++;

  // Row 3: EMA factor (shorthand)
  lcd.setCursor(0, 3); n = 0;
  lcd.print(F("EMA a=")); n += 6;
  dtostrf(PH_CONTROL_EMA, 3, 2, b1); lcd.print(b1); n += strlen(b1);
  while (n < LCD_COLS) lcd.print(' '), n++;
}

/* ============================================================
 * ESP8266 bridge — compact CSV for HA/MQTT
 * ============================================================ */

static void send_line_to_esp8266() {
  const bool ph_ok = !(isnan(ph_control) || ph_control < 0.0 || ph_control > 14.0);
  const bool t_ok  = !(isnan(water_temperature) || water_temperature < -50.0 || water_temperature > 125.0);
  const bool ec_ok = (ec_value >= 0.0 && ec_value < 1000000.0);
  const long  ec_i = ec_ok ? lround(ec_value) : -1;

  const double ph_err = ph_ok ? (PH_SETPOINT - ph_control) : NAN; // control uses EMA pH
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
  Serial1.print(F(",PHD_HR,"));   Serial1.print(ph_down_dosed_this_window, 1); // 0.1 mL precision
  Serial1.print(F(",LAST,"));     Serial1.print(last_action);
  Serial1.print(F(",LAST_ML,"));  Serial1.print(last_action_ml, 1);
  Serial1.print(F(",UPTIME_MS,"));Serial1.print(millis());
  Serial1.print("\n");
}
