/*
 * file: DFRobot_PH_DS18B20_Compensated.ino
 * purpose: Gravity Analog pH Sensor (SEN0161-V2) with automatic temperature compensation
 *          using a DS18B20 temperature probe.
 *
 * Hardware:
 *   - Arduino Mega 2560 (or compatible 5 V Arduino)
 *   - Gravity Analog pH Sensor V2 on analog pin A5 by default
 *   - DS18B20 temperature probe on digital pin D4 by default
 *   - REQUIRED: 4.7 kΩ pull-up resistor between DS18B20 DATA and 5 V
 *
 * Notes:
 *   - Temperature compensation materially improves pH accuracy. The pH library expects °C.
 *   - This sketch scans for DS18B20 devices at boot. If none are found, it falls back to 25.0 °C.
 *   - If a DS18B20 exists, it reads the first sensor found each cycle.
 *   - The sampling interval is 10 s (matching the original example’s 10000 ms).
 *
 * Serial Commands for pH calibration (send via Serial Monitor, “Both NL & CR”):
 *   enterph  -> enter calibration mode
 *   calph    -> calibrate with buffer solution (auto-detects 4.0 and 7.0)
 *   exitph   -> save calibration and exit
 *
 * Library licences:
 *   - DFRobot_PH: GNU LGPL (see original repository)
 *   - DallasTemperature / OneWire: respective open-source licences
 */

#include "DFRobot_PH.h"
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/* ────────────────────────────────────────────────────────────
 *                       USER SETTINGS
 *   Edit these to match your wiring. Keep pins consistent with
 *   your build documentation for easy maintenance.
 * ──────────────────────────────────────────────────────────── */

// pH input
#define PH_PIN            A5          // Analog input for Gravity pH probe signal

// DS18B20 temperature sensor
#ifndef DS18B20_PIN
#define DS18B20_PIN       4           // Digital pin for DS18B20 DATA
#endif

// Sampling and behaviour
#define SAMPLE_INTERVAL_MS 10000UL    // 10 s between pH/temperature reads
#define DEFAULT_TEMP_C     25.0f      // Fallback when no DS18B20 is detected
#define DS18B20_RES_BITS   12         // DS18B20 resolution: 9..12 bits; 12 gives best precision

/* ────────────────────────────────────────────────────────────
 *                    GLOBAL OBJECTS/STATE
 * ──────────────────────────────────────────────────────────── */

DFRobot_PH ph;                        // DFRobot pH helper
float voltage = 0.0f;
float phValue = 7.00f;
float temperature = DEFAULT_TEMP_C;   // Current temperature used for compensation (°C)

OneWire oneWire(DS18B20_PIN);         // OneWire bus on the chosen pin
DallasTemperature sensors(&oneWire);  // DallasTemperature wrapper

static uint8_t dsDeviceCount = 0;     // Number of DS18B20 devices found at boot
static float lastGoodTempC = DEFAULT_TEMP_C; // Sticky last known valid temperature

/* ────────────────────────────────────────────────────────────
 *                       FORWARD DECLARES
 * ──────────────────────────────────────────────────────────── */

static float readTemperatureC_DS18B20(void);
static void printAddress(const DeviceAddress a);

/* ────────────────────────────────────────────────────────────
 *                              SETUP
 * ──────────────────────────────────────────────────────────── */

void setup()
{
  Serial.begin(115200);
  while (!Serial) { /* wait for USB on boards with native USB */ }

  Serial.println(F("\nDFRobot pH with DS18B20 Temperature Compensation"));
  Serial.print(F("pH analog pin : A")); Serial.println(PH_PIN - A0);
  Serial.print(F("DS18B20 pin   : D")); Serial.println(DS18B20_PIN);
  Serial.println(F("Ensure a 4.7 kΩ pull-up from DS18B20 DATA to 5 V.\n"));

  // Initialise pH helper
  ph.begin();

  // Initialise DS18B20
  sensors.begin();
  sensors.setWaitForConversion(true);       // Block until conversion completes after request
  dsDeviceCount = sensors.getDeviceCount(); // Count devices once at boot
  Serial.print(F("DS18B20 devices found: ")); Serial.println(dsDeviceCount);

  if (dsDeviceCount == 0) {
    Serial.println(F("No DS18B20 detected. Using DEFAULT_TEMP_C for compensation.\n"));
  } else {
    // Optional: set per-device resolution. We set bus-wide expectations then verify.
    // If you wanted per-device control, enumerate and call setResolution(addr, bits).
    sensors.setResolution(DS18B20_RES_BITS);

    // One initial read to seed lastGoodTempC
    float t = readTemperatureC_DS18B20();
    if (!isnan(t)) {
      lastGoodTempC = t;
      temperature = t;
    }
  }
}

/* ────────────────────────────────────────────────────────────
 *                              LOOP
 * ──────────────────────────────────────────────────────────── */

void loop()
{
  static unsigned long lastSample = 0;
  const unsigned long now = millis();

  // Periodic sample block
  if (now - lastSample >= SAMPLE_INTERVAL_MS) {   // interval = 10 s
    lastSample = now;

    // 1) Read temperature (DS18B20 if present; otherwise fallback)
    temperature = (dsDeviceCount > 0) ? readTemperatureC_DS18B20() : DEFAULT_TEMP_C;
    if (isnan(temperature)) {
      // Sensor missing or momentary disconnect; hold last good or DEFAULT
      temperature = lastGoodTempC;
    } else {
      lastGoodTempC = temperature; // update sticky value on success
    }

    // 2) Read pH analog voltage in millivolts
    //    The Gravity pH adaptor outputs 0–5 V; Arduino ADC is 10-bit (0..1023)
    //    Using 1024.0 keeps continuity with many DFRobot examples.
    voltage = analogRead(PH_PIN) / 1024.0f * 5000.0f; // mV

    // 3) Convert voltage to pH with temperature compensation
    phValue = ph.readPH(voltage, temperature);

    // 4) Report
    Serial.print(F("T: "));
    Serial.print(temperature, 2);
    Serial.print(F(" °C  |  pH: "));
    Serial.print(phValue, 2);
    Serial.print(F("  |  V: "));
    Serial.print(voltage, 1);
    Serial.println(F(" mV"));
  }

  // Calibration handler. Leave this always reachable so you can type commands any time.
  // Commands: enterph, calph, exitph
  ph.calibration(voltage, temperature);
}

/* ────────────────────────────────────────────────────────────
 *                  TEMPERATURE READ IMPLEMENTATION
 * ──────────────────────────────────────────────────────────── */

/**
 * @brief Read °C from the first DS18B20 on the bus.
 *
 * Behaviour:
 *  - Requests a conversion, waits until complete (blocking), then reads index 0.
 *  - Returns NaN on any error so caller can fall back gracefully.
 *  - Keeps logic simple and predictable at a 10 s cadence.
 */
static float readTemperatureC_DS18B20(void)
{
  if (dsDeviceCount == 0) return NAN;

  // Request a conversion for all devices on the bus
  sensors.requestTemperatures();

  // Read first device on the bus by index
  float tC = sensors.getTempCByIndex(0);

  // Handle disconnection sentinel from DallasTemperature
  if (tC == DEVICE_DISCONNECTED_C) {
    // Provide a concise debug note once per failure case if desired
    // Serial.println(F("DS18B20 read failed (disconnected)."));
    return NAN;
  }

  // Sanity-check typical hydroponic range to catch corrupt reads
  if (tC < -10.0f || tC > 85.0f) { // DS18B20 rated range is wider, this narrows to practical limits
    // Serial.println(F("DS18B20 read out of expected range."));
    return NAN;
  }

  return tC;
}

/* ────────────────────────────────────────────────────────────
 *                     OPTIONAL: ADDRESS UTILITY
 *   Useful if you want to enumerate sensors and pick a specific
 *   address rather than index 0. Not used by default.
 * ──────────────────────────────────────────────────────────── */

static void printAddress(const DeviceAddress a)
{
  for (uint8_t i = 0; i < 8; i++) {
    if (a[i] < 16) Serial.print('0');
    Serial.print(a[i], HEX);
    if (i < 7) Serial.print(':');
  }
}
