/* ============================================================
   Hydroponics Controller (single-file build)
   - EC-first logic: correct EC until within range; only then correct pH
   - Two-part nutrients: Nutrient A (repurposed former pH-Up pump) then Nutrient B
   - Safe sequencing: A -> mix/settle -> B (non-blocking stagger)
   - Independent hourly caps for Nutrient A, Nutrient B, and EC-Down (water)
   - pH-Up removed; pH-Down only (pH tends to rise naturally)
   - DS18B20-based software EC temperature compensation to 25 °C
   - Atlas EZO-EC temp pinned to 25.0 °C at boot; verified once at boot
   - 20x4 I2C LCD (2004A) with three rotating screens + LCD watchdog
   - ESP8266 bridge: CSV over Serial1 each sensor cycle for MQTT/HA
   ============================================================ */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "DFRobot_PH.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>

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
#define LCD_PAGE_MS     3000UL    // CHANGE TO ADJUST TIME EACH SCREEN IS SHOWN (MS)
#define LCD_REINIT_MS   300000UL  // CHANGE TO ADJUST LCD RE-INIT INTERVAL (MS)

// ---------- Setpoints ----------
double ph_setpoint = 5.7;         // CHANGE TO YOUR TARGET PH (TYPICALLY 5.5–6.2)
double ec_setpoint = 1600.0;      // CHANGE TO YOUR TARGET EC IN uS/CM (E.G., 800–1800)

// ---------- Control bands ----------
static const double PH_DEADBAND = 0.3;   // CHANGE TO WIDEN/NARROW PH ALLOWED ERROR (PH UNITS, ±)
static const double EC_DEADBAND = 50.0;  // CHANGE TO WIDEN/NARROW EC ALLOWED ERROR (uS/CM, ±)

// ---------- Two-part nutrient dosing (per action) ----------
static const double EC_UP_TOTAL_DOSE_ML = 8.0;      // CHANGE SINGLE "EC-UP TOTAL" DOSE SIZE (A+B SUM, ML)
static const double EC_A_TO_B_RATIO     = 1.0;      // CHANGE A:B RATIO (1.0=EQUAL PARTS; 2.0=TWICE A AS B, ETC.)
static const unsigned long EC_AB_STAGGER_MS = 120000UL; // CHANGE MIX TIME BETWEEN A AND B DOSES (MS)

// ---------- EC-Down (water) per-dose amount ----------
static const double EC_DOWN_DOSE_ML = 0.0;          // SET TO >0 TO ENABLE EC-DOWN (WATER) DOSING (CURRENTLY DISABLED)

// ---------- pH dosing (Down only) ----------
static const double PH_DOWN_DOSE_ML = 1.0;          // CHANGE SINGLE PH-DOWN DOSE SIZE (ML)

// ---------- Per-direction settle lockouts (ms) ----------
static const unsigned long EC_UP_MIX_LOCKOUT_MS   = 600000UL; // CHANGE MIX TIME AFTER A->B NUTRIENT CYCLE (MS)
static const unsigned long EC_DOWN_MIX_LOCKOUT_MS = 600000UL; // CHANGE MIX TIME AFTER WATER (MS)
static const unsigned long PH_DOWN_MIX_LOCKOUT_MS = 600000UL; // CHANGE LOCKOUT AFTER PH-DOWN (MS)

// ---------- Hourly caps (mL/h) per direction ----------
static const double  EC_A_MAX_ML_PER_HR    = 60.0;  // CHANGE MAXIMUM NUTRIENT-A ML PER HOUR
static const double  EC_B_MAX_ML_PER_HR    = 60.0;  // CHANGE MAXIMUM NUTRIENT-B ML PER HOUR
static const double  EC_DOWN_MAX_ML_PER_HR = 0.0;   // SET TO >0 IF ENABLING EC-DOWN
static const double  PH_DOWN_MAX_ML_PER_HR = 60.0;  // CHANGE MAXIMUM PH-DOWN ML PER HOUR

// ---------- Sensor / control cadence ----------
const unsigned long sensor_interval = 10000UL;      // CHANGE HOW OFTEN TO READ/CONTROL (MS)

/* ────────────────────────────────────────────────────────────
 *                END OF USER-SETTABLE PARAMETERS
 * ──────────────────────────────────────────────────────────── */


/* ============================================================
 *                   Globals and Instances
 * ============================================================ */

DFRobot_PH ph_sensor;
OneWire oneWire(DS18B20_PIN);
DallasTemperature waterTemp(&oneWire);
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);

// Live readings
double ph_value = 0.0;
double ec_value = 0.0;
double water_temperature = 0.0;

// Timing helpers
unsigned long previous_millis = 0;
static unsigned long last_lcd_ms = 0;
static unsigned long last_page_ms = 0;
static unsigned long last_lcd_reinit_ms = 0;
static uint8_t lcd_page = 0; // 0=A, 1=B, 2=C

// EC dose timing / state
static unsigned long last_dose_ms_ec_cycle = 0; // time when latest Nutrient A->B cycle started
static unsigned long last_dose_ms_ec_down  = 0; // time of last EC-Down dose
static int last_ec_direction = 0;               // +1 = raising EC (A/B), -1 = lowering EC (water), 0 = none

// Non-blocking stagger for B after A
static bool   pending_B = false;                // true if a B dose is scheduled after an A dose
static double pending_B_ml = 0.0;               // B dose amount waiting to be delivered
static unsigned long pending_B_ready_ms = 0;    // earliest time to deliver B

// pH timing
static unsigned long last_dose_ms_ph_down = 0;

// Hourly windows (independent counters)
static unsigned long ec_a_window_start    = 0;
static unsigned long ec_b_window_start    = 0;
static unsigned long ec_down_window_start = 0;
static unsigned long ph_down_window_start = 0;

static double        ec_a_dosed_this_window    = 0.0;
static double        ec_b_dosed_this_window    = 0.0;
static double        ec_down_dosed_this_window = 0.0;
static double        ph_down_dosed_this_window = 0.0;

// LCD “last action” indicator
static char   last_action[8] = "None"; // "A","B","ECD","pHD","None"
static double last_action_ml = 0.0;

/* ============================================================
 *                    Freeze-safety additions
 *   - Hardware watchdog (AVR)
 *   - I²C timeouts and bounded retries
 *   - Non-blocking delay wrapper that yields/kicks WDT
 * ============================================================ */
#if defined(ARDUINO_ARCH_AVR)
  #include <avr/wdt.h>
#endif

// Yield + pet HW WDT where available.
static inline void watchdog_kick() {
  #if defined(ARDUINO_ARCH_AVR)
    wdt_reset();
  #endif
  // Allow other cores (ESP, etc.) to breathe
  yield();
}

// Replace long blocking delays with a watchdog-safe, chunked delay.
static void safe_delay(unsigned long ms) {
  const unsigned long start = millis();
  while ((millis() - start) < ms) {
    watchdog_kick();
    delay(10);
  }
}

// Small helper for bounded I²C retries to avoid indefinite wait loops.
static bool i2c_send_with_retries(uint8_t address, const char* command, uint8_t max_attempts = 3) {
  for (uint8_t attempt = 0; attempt < max_attempts; ++attempt) {
    Wire.beginTransmission(address);
    const char* p = command;
    while (*p) { Wire.write(*p++); }
    uint8_t err = Wire.endTransmission();
    if (err == 0) return true;
    Serial.print("I2C TX error "); Serial.print(err);
    Serial.print(" to 0x"); Serial.println(address, HEX);
    // brief backoff; keep kicking the watchdog
    safe_delay(5);
  }
  return false;
}

/* ============================================================
 *                    Function Declarations
 * ============================================================ */
// Initialization
void initialise_pins();
void initialise_devices();
void initialise_display();

// Sensors
double read_water_temperature();
double read_ph_sensor();
double read_ec_sensor();
double compensate_ec(double ec_measured, double temperature);

// I2C helpers
void   send_command(uint8_t address, const char *command);
float  read_response_ec_numeric(uint8_t address);
size_t read_response_ascii(uint8_t address, char *buf, size_t buflen);

// Dosing primitives
void dose_nutrient_A(double ml); // uses NUTRIENT_A_PUMP_ADDRESS
void dose_nutrient_B(double ml); // uses NUTRIENT_B_PUMP_ADDRESS
void dose_ec_down(double ml);    // uses EC_DOWN_PUMP_ADDRESS (water)
void dose_ph_down(double ml);    // uses PH_DOWN_PUMP_ADDRESS

// Control loop
void control_loop();
static void maybe_reset_hourly_caps();

// LCD UI
void update_lcd();
void lcd_screen_A();
void lcd_screen_B();
void lcd_screen_C();

// ===== ESP8266 BRIDGE =====
void send_line_to_esp8266();

/* ============================================================
 *                         Arduino setup
 * ============================================================ */
void setup() {
  Serial.begin(9600);
  // ===== ESP8266 BRIDGE ===== use Serial1 (Mega hardware UART) to NodeMCU/Wi-Fi bridge
  Serial1.begin(115200);

  // I²C begin before devices; set robust timeouts to prevent stalls.
  Wire.begin();

  // Ensure I²C operations time out rather than hang (units are microseconds on AVR cores).
  // 25,000 µs (~25 ms) is a good default; auto-reset I²C on timeout to recover bus.
  #if defined(TWOWIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(25000, true);
  #elif defined(ARDUINO_ARCH_AVR)
    // Older AVR cores use setWireTimeout name too:
    Wire.setWireTimeout(25000, true);
  #endif

  // Enable hardware watchdog on AVR so any unexpected hard lock resets cleanly.
  #if defined(ARDUINO_ARCH_AVR)
    wdt_enable(WDTO_4S); // 4 s window; main loop kicks it frequently
  #endif

  initialise_pins();
  initialise_devices();
  initialise_display();
  Serial.println("Hydroponics controller initialised (two-part nutrients A->B; pH-Down only).");
  Serial.println();
}

/* ============================================================
 *                          Arduino loop
 * ============================================================ */
void loop() {
  watchdog_kick(); // keep the watchdog happy at the top of each pass
  const unsigned long now = millis();

  // Cadenced sensor read + control
  if (now - previous_millis >= sensor_interval) {
    previous_millis = now;

    read_water_temperature();

    // --- quiet EC before pH ---
    send_command(EC_PROBE_ADDRESS, "Sleep");
    safe_delay(500);

    // --- pH phase ---
    read_ph_sensor();

    // --- wake EC and read ---
    send_command(EC_PROBE_ADDRESS, "R");
    safe_delay(600);
    read_ec_sensor();   // uses I²C + compensation

    // --- return EC to normal run mode ---
    send_command(EC_PROBE_ADDRESS, "Status"); // harmless wake; ensures next loop works

    // --- normal control ---
    control_loop();

    // Snapshot to aid tuning
    Serial.print("Snapshot | pH: ");
    Serial.print(ph_value, 2);
    Serial.print(" | EC: ");
    Serial.print(ec_value, 0);
    Serial.print(" uS/cm | Water T: ");
    Serial.print(water_temperature, 1);
    Serial.println(" C");
    Serial.println();

    // ===== ESP8266 BRIDGE ===== publish one CSV line for MQTT bridge
    send_line_to_esp8266();
  }

  // LCD page rotation and watchdog
  if (now - last_page_ms >= LCD_PAGE_MS) {
    last_page_ms = now;
    lcd_page = (lcd_page + 1) % 3;
  }
  if (now - last_lcd_reinit_ms >= LCD_REINIT_MS) {
    initialise_display();
    last_lcd_reinit_ms = now;
  }
  if (now - last_lcd_ms >= LCD_UPDATE_MS) {
    last_lcd_ms = now;
    update_lcd();
  }

  // Handle pending Nutrient B dose (non-blocking stagger after A)
  if (pending_B && now >= pending_B_ready_ms) {
    // If the overall Nutrient A->B lockout elapsed before B, drop B to keep sequence integrity.
    if ((now - last_dose_ms_ec_cycle) >= EC_UP_MIX_LOCKOUT_MS) {
      pending_B = false; // expired; next control pass will schedule a fresh A->B if still needed
    } else if (ec_b_dosed_this_window < EC_B_MAX_ML_PER_HR && pending_B_ml > 0.0) {
      dose_nutrient_B(pending_B_ml);
      ec_b_dosed_this_window += pending_B_ml;
      pending_B = false;
      // The cycle’s settle time continues from the original A-dose start.
    }
  }
}

/* ============================================================
 *                        Initialization
 * ============================================================ */
void initialise_pins() {
  pinMode(PH_ANALOG_PIN, INPUT);
}

void initialise_devices() {
  ph_sensor.begin();
  waterTemp.begin();
  Serial.println("Devices initialised.");

  // Pin EZO-EC temperature to 25.0 °C to avoid double-compensation.
  send_command(EC_PROBE_ADDRESS, "T,25.0");
  safe_delay(300);
  char tbuf[32];
  send_command(EC_PROBE_ADDRESS, "T,?");
  size_t n = read_response_ascii(EC_PROBE_ADDRESS, tbuf, sizeof(tbuf));
  if (n > 0) {
    Serial.print("EZO-EC temperature readback: ");
    Serial.println(tbuf); // e.g., "?T,25.0"
  } else {
    Serial.println("EZO-EC temperature readback: no response");
  }
}

void initialise_display() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Hydro Controller");
  lcd.setCursor(0, 1); lcd.print("Init...");
  safe_delay(800);
  lcd.clear();
}

/* ============================================================
 *                           Sensors
 * ============================================================ */
double read_water_temperature() {
  waterTemp.requestTemperatures();
  const double t = waterTemp.getTempCByIndex(0);
  if (t < -50.0 || t > 125.0) {
    Serial.println("DS18B20 reading out of range; keeping last value.");
    return water_temperature;
  }
  water_temperature = t;
  return water_temperature;
}

double read_ph_sensor() {
  // If your MCU analog reference is 3.3 V, change 5000.0 -> 3300.0.
  const double voltage_mV = (analogRead(PH_ANALOG_PIN) / 1024.0) * 5000.0;
  const double pH = ph_sensor.readPH(voltage_mV, water_temperature);
  ph_value = pH;
  return ph_value;
}

double read_ec_sensor() {
  // EZO-EC: "R" = read (uS/cm)
  send_command(EC_PROBE_ADDRESS, "R");
  safe_delay(600);
  const double raw = read_response_ec_numeric(EC_PROBE_ADDRESS);
  const double ec_comp = compensate_ec(raw, water_temperature);
  ec_value = ec_comp;
  return ec_value;
}

double compensate_ec(double ec_measured, double temperature) {
  // Normalize EC to 25 °C using a typical 2%/°C coefficient.
  const double T_REF = 25.0;
  const double ALPHA = 0.02;
  return ec_measured / (1.0 + ALPHA * (temperature - T_REF));
}

/* ============================================================
 *                          I²C helpers
 * ============================================================ */
void send_command(uint8_t address, const char *command) {
  // Bounded retries to avoid indefinite stalls on a flaky bus.
  if (!i2c_send_with_retries(address, command, 3)) {
    Serial.print("I2C send failed permanently to 0x");
    Serial.println(address, HEX);
  }
  safe_delay(300);
}

float read_response_ec_numeric(uint8_t address) {
  // requestFrom honours Wire timeouts; we still guard and parse defensively.
  const uint8_t bytes = Wire.requestFrom(address, (uint8_t)32);
  if (bytes == 0) {
    Serial.print("I2C read error from 0x"); Serial.println(address, HEX);
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

size_t read_response_ascii(uint8_t address, char *buf, size_t buflen) {
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

/* ============================================================
 *                        Dosing primitives
 * ============================================================ */
void dose_nutrient_A(double ml) {
  if (ml <= 0.0) return;
  String cmd = "D," + String(ml, 2);
  send_command(NUTRIENT_A_PUMP_ADDRESS, cmd.c_str()); // nutrient A pump
  Serial.print("Dose A (mL): "); Serial.println(ml, 2);
  strcpy(last_action, "A"); last_action_ml = ml;
}

void dose_nutrient_B(double ml) {
  if (ml <= 0.0) return;
  String cmd = "D," + String(ml, 2);
  send_command(NUTRIENT_B_PUMP_ADDRESS, cmd.c_str()); // nutrient B pump
  Serial.print("Dose B (mL): "); Serial.println(ml, 2);
  strcpy(last_action, "B"); last_action_ml = ml;
}

void dose_ec_down(double ml) {
  if (ml <= 0.0) return;
  String cmd = "D," + String(ml, 2);
  send_command(EC_DOWN_PUMP_ADDRESS, cmd.c_str()); // water
  Serial.print("EC Down (water) mL: "); Serial.println(ml, 2);
  strcpy(last_action, "ECD"); last_action_ml = ml;
}

void dose_ph_down(double ml) {
  if (ml <= 0.0) return;
  String cmd = "D," + String(ml, 2);
  send_command(PH_DOWN_PUMP_ADDRESS, cmd.c_str());
  Serial.print("pH Down dose (mL): "); Serial.println(ml, 2);
  strcpy(last_action, "pHD"); last_action_ml = ml;
}

/* ============================================================
 *                 EC-first conservative control loop
 * ============================================================ */
void control_loop() {
  const unsigned long now = millis();

  // Reset hourly windows if an hour has passed.
  maybe_reset_hourly_caps();

  // Sanity guard for pH reading.
  const bool ph_ok = !(isnan(ph_value) || ph_value < 0.0 || ph_value > 14.0);
  if (!ph_ok) {
    Serial.println("pH sanity check failed — skipping pH control (check probe/wiring).");
  }

  // EC priority: correct EC until within deadband.
  const double ec_err = ec_setpoint - ec_value;          // + -> EC low (need Up A/B), - -> EC high (need Down water)
  const bool   ec_in_range = fabs(ec_err) <= EC_DEADBAND;

  // Consider the A->B sequence "busy" while B is pending.
  const bool ec_cycle_busy = pending_B;

  if (!ec_in_range || ec_cycle_busy) {
    if (!ec_cycle_busy) {
      const bool need_ec_down = (ec_err < 0.0);
      const unsigned long required_lockout = need_ec_down ? EC_DOWN_MIX_LOCKOUT_MS
                                                          : EC_UP_MIX_LOCKOUT_MS;
      const unsigned long last_ms = need_ec_down ? last_dose_ms_ec_down
                                                 : last_dose_ms_ec_cycle;

      if ((now - last_ms) >= required_lockout) {
        if (need_ec_down) {
          // EC DOWN (water)
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
          // EC UP (two-part: A then B)
          const double ratioA = EC_A_TO_B_RATIO;
          const double a_part = (ratioA / (ratioA + 1.0)) * EC_UP_TOTAL_DOSE_ML;
          const double b_part = EC_UP_TOTAL_DOSE_ML - a_part;

          const double a_room = max(0.0, EC_A_MAX_ML_PER_HR - ec_a_dosed_this_window);
          const double b_room = max(0.0, EC_B_MAX_ML_PER_HR - ec_b_dosed_this_window);

          // Start an A->B cycle only if both parts fit under their hourly caps.
          if (a_part > 0.0 && b_part > 0.0 && a_room >= a_part && b_room >= b_part) {
            dose_nutrient_A(a_part);
            ec_a_dosed_this_window += a_part;

            pending_B = true;
            pending_B_ml = b_part;
            pending_B_ready_ms = now + EC_AB_STAGGER_MS;

            last_dose_ms_ec_cycle = now; // lockout covers the whole A->B event
            last_ec_direction = +1;
          }
        }
      }
    }
    // Do not attempt pH while EC is out of range or A->B is in progress.
    return;
  }

  // EC is in range and no active A->B cycle; consider pH (Down only).
  const unsigned long ec_required_settle =
      (last_ec_direction < 0) ? EC_DOWN_MIX_LOCKOUT_MS :
      (last_ec_direction > 0) ? EC_UP_MIX_LOCKOUT_MS   : 0UL;

  const unsigned long last_ec_any = max(last_dose_ms_ec_cycle, last_dose_ms_ec_down);
  const bool ec_has_settled = (millis() - last_ec_any) >= ec_required_settle;

  if (ph_ok && ec_has_settled) {
    const double ph_err = ph_setpoint - ph_value; // + => would need pH-Up (not used), - => need pH-Down
    if (fabs(ph_err) > PH_DEADBAND && ph_err < 0.0) {
      if ((millis() - last_dose_ms_ph_down) >= PH_DOWN_MIX_LOCKOUT_MS &&
          ph_down_dosed_this_window < PH_DOWN_MAX_ML_PER_HR) {
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
 *                           LCD UI (3 pages)
 *   A: live readings + setpoints + EC state
 *   B: per-direction hourly usage vs caps
 *   C: last action summary
 * ============================================================ */
void update_lcd() {
  switch (lcd_page) {
    case 0: lcd_screen_A(); break;
    case 1: lcd_screen_B(); break;
    default: lcd_screen_C(); break;
  }
}

void lcd_screen_A() {
  char b1[12], b2[12]; uint8_t n;
  lcd.clear();

  // Line 0: "pH:xx.xx  SP:x.x"
  lcd.setCursor(0, 0); n = 0;
  lcd.print("pH:"); n += 3;
  dtostrf(ph_value, 5, 2, b1); lcd.print(b1); n += strlen(b1);
  lcd.print("  SP:"); n += 5;
  dtostrf(ph_setpoint, 3, 1, b2); lcd.print(b2); n += strlen(b2);
  while (n < LCD_COLS) lcd.print(' '), n++;

  // Line 1: "EC:xxxxx uS SP:xxxx"
  lcd.setCursor(0, 1); n = 0;
  lcd.print("EC:"); n += 3;
  dtostrf(ec_value, 6, 0, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(" uS SP:"); n += 7;
  dtostrf(ec_setpoint, 4, 0, b2); lcd.print(b2); n += strlen(b2);
  while (n < LCD_COLS) lcd.print(' '), n++;

  // Line 2: "Temp:yy.y C"
  lcd.setCursor(0, 2); n = 0;
  lcd.print("Temp:"); n += 5;
  dtostrf(water_temperature, 4, 1, b1); lcd.print(b1); n += strlen(b1);
  lcd.print(" C"); n += 2;
  while (n < LCD_COLS) lcd.print(' '), n++;

  // Line 3: "State: EC OK/UP/DN"
  lcd.setCursor(0, 3); n = 0;
  lcd.print("State: "); n += 7;
  bool in_range = (fabs(ec_setpoint - ec_value) <= EC_DEADBAND);
  if (in_range)                 { lcd.print("EC OK"); n += 5; }
  else if (ec_setpoint > ec_value) { lcd.print("EC UP"); n += 5; }
  else                          { lcd.print("EC DN"); n += 5; }
  while (n < LCD_COLS) lcd.print(' '), n++;
}

void lcd_screen_B() {
  // Show hourly usage vs caps for A, B, and EC-Down, plus pH-Down
  char b1[12], b2[12]; uint8_t n;
  lcd.clear();

  // Line 0: "A:  xx/xxx mL"
  lcd.setCursor(0, 0); n = 0;
  lcd.print("A: "); n += 3;
  ltoa(lround(ec_a_dosed_this_window), b1, 10); lcd.print(b1); n += strlen(b1);
  lcd.print("/"); n += 1;
  ltoa(lround(EC_A_MAX_ML_PER_HR), b2, 10); lcd.print(b2); n += strlen(b2);
  lcd.print(" mL"); n += 3; while (n < LCD_COLS) lcd.print(' '), n++;

  // Line 1: "B:  xx/xxx mL"
  lcd.setCursor(0, 1); n = 0;
  lcd.print("B: "); n += 3;
  ltoa(lround(ec_b_dosed_this_window), b1, 10); lcd.print(b1); n += strlen(b1);
  lcd.print("/"); n += 1;
  ltoa(lround(EC_B_MAX_ML_PER_HR), b2, 10); lcd.print(b2); n += strlen(b2);
  lcd.print(" mL"); n += 3; while (n < LCD_COLS) lcd.print(' '), n++;

  // Line 2: "EC Dn:  xx/xxx mL"
  lcd.setCursor(0, 2); n = 0;
  lcd.print("EC Dn: "); n += 7;
  ltoa(lround(ec_down_dosed_this_window), b1, 10); lcd.print(b1); n += strlen(b1);
  lcd.print("/"); n += 1;
  ltoa(lround(EC_DOWN_MAX_ML_PER_HR), b2, 10); lcd.print(b2); n += strlen(b2);
  lcd.print(" mL"); n += 3; while (n < LCD_COLS) lcd.print(' '), n++;

  // Line 3: "pH Dn:  xx/xxx mL"
  lcd.setCursor(0, 3); n = 0;
  lcd.print("pH Dn: "); n += 7;
  ltoa(lround(ph_down_dosed_this_window), b1, 10); lcd.print(b1); n += strlen(b1);
  lcd.print("/"); n += 1;
  ltoa(lround(PH_DOWN_MAX_ML_PER_HR), b2, 10); lcd.print(b2); n += strlen(b2);
  lcd.print(" mL"); n += 3; while (n < LCD_COLS) lcd.print(' '), n++;
}

void lcd_screen_C() {
  char b1[12]; uint8_t n;
  lcd.clear();

  // Line 0: "Last: X 5.0mL"
  lcd.setCursor(0, 0); n = 0;
  lcd.print("Last: "); n += 6;
  lcd.print(last_action); n += strlen(last_action);
  lcd.print(" "); n += 1;
  dtostrf(last_action_ml, 4, 1, b1); lcd.print(b1); n += strlen(b1);
  lcd.print("mL"); n += 2;
  while (n < LCD_COLS) lcd.print(' '), n++;

  // Clear remaining lines for a stable look
  for (uint8_t row = 1; row < 4; row++) {
    lcd.setCursor(0, row);
    for (uint8_t i = 0; i < LCD_COLS; i++) lcd.print(' ');
  }
}

/* ============================================================
 * ESP8266 bridge helper — minimal but useful CSV for HA
 * Example:
 * PH,5.78,EC,1605,T,26.9,PH_SP,5.8,EC_SP,1600,EC_STATE,OK,
 * PH_ERR,0.02,EC_ERR,-5,A_HR,8,B_HR,8,ECD_HR,0,PHD_HR,1,
 * LAST,A,LAST_ML,4.0,UPTIME_MS,1234567
 * ============================================================ */
void send_line_to_esp8266() {
  // Sanity guards
  const bool ph_ok = !(isnan(ph_value) || ph_value < 0.0 || ph_value > 14.0);
  const bool t_ok  = !(isnan(water_temperature) || water_temperature < -50.0 || water_temperature > 125.0);
  const bool ec_ok = (ec_value >= 0.0 && ec_value < 1000000.0);
  const long  ec_i = ec_ok ? lround(ec_value) : -1;

  // Derivatives (only if the base values are sane)
  const double ph_err = ph_ok ? (ph_setpoint - ph_value) : NAN;
  const long   ec_err = ec_ok ? lround(ec_setpoint - ec_value) : 0;

  // EC state (for a simple status chip/timeline)
  const bool in_range = ec_ok && (fabs(ec_setpoint - ec_value) <= EC_DEADBAND);
  const char* ec_state =
      !ec_ok ? "NA" :
      in_range ? "OK" :
      (ec_setpoint > ec_value ? "UP" : "DOWN");

  // Keep the legacy trio first for backward compatibility
  Serial1.print("PH,");
  if (ph_ok) Serial1.print(ph_value, 2); else Serial1.print("NaN");

  Serial1.print(",EC,");
  if (ec_ok) Serial1.print(ec_i); else Serial1.print("NaN");

  Serial1.print(",T,");
  if (t_ok) Serial1.print(water_temperature, 1); else Serial1.print("NaN");

  // Setpoints
  Serial1.print(",PH_SP,"); Serial1.print(ph_setpoint, 1);
  Serial1.print(",EC_SP,"); Serial1.print(lround(ec_setpoint));

  // State
  Serial1.print(",EC_STATE,"); Serial1.print(ec_state);

  // Errors (handy for trend graphs/alerts)
  Serial1.print(",PH_ERR,");
  if (ph_ok) Serial1.print(ph_err, 2); else Serial1.print("NaN");

  Serial1.print(",EC_ERR,");
  if (ec_ok) Serial1.print(ec_err); else Serial1.print("NaN");

  // Hourly dosing totals (window counters)
  Serial1.print(",A_HR,");    Serial1.print(lround(ec_a_dosed_this_window));
  Serial1.print(",B_HR,");    Serial1.print(lround(ec_b_dosed_this_window));
  Serial1.print(",ECD_HR,");  Serial1.print(lround(ec_down_dosed_this_window)); // water (EC down) — will be 0 if disabled
  Serial1.print(",PHD_HR,");  Serial1.print(lround(ph_down_dosed_this_window));

  // Last action snapshot
  Serial1.print(",LAST,");    Serial1.print(last_action);
  Serial1.print(",LAST_ML,"); Serial1.print(last_action_ml, 1);

  // Uptime (diagnostics)
  Serial1.print(",UPTIME_MS,"); Serial1.print(millis());

  Serial1.print("\n");
}
