/* DS18B20_Tester_Mega.ino
   Arduino Mega 2560 (SDA=20, SCL=21 not used here)
   DS18B20 DATA -> D4  (default)
   4.7k pull-up between DATA and 5V is REQUIRED
   Prints device count, addresses, power mode, resolution, and temperature (°C).
*/

#include <OneWire.h>
#include <DallasTemperature.h>

#ifndef DS18B20_PIN
#define DS18B20_PIN 4   // change this if you moved the data wire
#endif

OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

DeviceAddress addr;
uint8_t deviceCount = 0;

void printAddress(const DeviceAddress a) {
  for (uint8_t i = 0; i < 8; i++) {
    if (a[i] < 16) Serial.print('0');
    Serial.print(a[i], HEX);
    if (i < 7) Serial.print(':');
  }
}

void printPowerMode() {
  bool parasite = sensors.isParasitePowerMode();
  Serial.print(F("Power Mode   : "));
  Serial.println(parasite ? F("Parasite") : F("Normal (VCC)"));
}

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }

  Serial.println(F("\nDS18B20 Tester (Mega 2560)"));
  Serial.print(F("OneWire pin  : D")); Serial.println(DS18B20_PIN);
  Serial.println(F("NOTE: Ensure a 4.7k resistor from DATA to 5V.\n"));

  sensors.begin();
  sensors.setWaitForConversion(true); // block until conversion done
  deviceCount = sensors.getDeviceCount();

  Serial.print(F("Devices found: ")); Serial.println(deviceCount);
  printPowerMode();
  Serial.println();

  if (deviceCount == 0) {
    Serial.println(F("No devices detected. Check wiring, pull-up, and pin."));
    Serial.println(F("Re-scanning every 3 seconds...\n"));
  }
}

void loop() {
  // Re-scan each cycle to catch hot-plugging
  uint8_t count = sensors.getDeviceCount();
  if (count != deviceCount) {
    deviceCount = count;
    Serial.print(F("\nDevices found (rescan): ")); Serial.println(deviceCount);
    printPowerMode();
    Serial.println();
  }

  if (deviceCount == 0) {
    delay(3000);
    return;
  }

  // Iterate over all devices
  for (uint8_t i = 0; i < deviceCount; i++) {
    DeviceAddress a;
    if (!sensors.getAddress(a, i)) {
      Serial.print(F("Sensor #")); Serial.print(i);
      Serial.println(F(": address read failed."));
      continue;
    }

    // Set and report resolution (12-bit)
    sensors.setResolution(a, 12);
    uint8_t res = sensors.getResolution(a);

    // Request temperature for this specific sensor
    sensors.requestTemperaturesByAddress(a);
    float tC = sensors.getTempC(a);

    Serial.print(F("Sensor #")); Serial.print(i);
    Serial.print(F(" | Addr: "));
    printAddress(a);
    Serial.print(F(" | Res: ")); Serial.print(res); Serial.print(F("-bit"));
    Serial.print(F(" | Temp: "));

    if (tC == DEVICE_DISCONNECTED_C) {
      Serial.println(F("DISCONNECTED"));
    } else {
      Serial.print(tC, 3); Serial.println(F(" °C"));
    }
  }

  Serial.println(F("-----"));
  delay(2000); // read every ~2 s
}
