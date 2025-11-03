/* I2C_Scanner_Mega.ino
   Scans the I2C bus and reports all device addresses.
   Arduino Mega 2560: SDA=20, SCL=21
*/

#include <Wire.h>

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; } // for boards with native USB; harmless on Mega

  Wire.begin();                 // Mega: SDA=20, SCL=21
  Wire.setClock(100000);        // 100 kHz (standard mode)
  delay(200);

  Serial.println(F("I2C Scanner (Mega 2560)"));
  Serial.println(F("SDA: pin 20, SCL: pin 21"));
  Serial.println(F("Scanning addresses 0x03 to 0x77...\n"));
}

void loop() {
  uint8_t count = 0;

  for (uint8_t address = 1; address < 127; address++) {
    if (address < 0x03 || address > 0x77) continue;  // skip reserved

    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      // Device acknowledged
      Serial.print(F("Found device at 0x"));
      if (address < 16) Serial.print('0');
      Serial.print(address, HEX);
      Serial.println(F("  (ACK)"));
      count++;
    } else if (error == 4) {
      // Other error
      Serial.print(F("Unknown error at 0x"));
      if (address < 16) Serial.print('0');
      Serial.println(address, HEX);
    }

    delay(2); // small gap keeps the bus happy
  }

  if (count == 0) {
    Serial.println(F("\nNo I2C devices found."));
  } else {
    Serial.print(F("\nScan complete. Devices found: "));
    Serial.println(count);
  }

  Serial.println(F("\nTip: Common addresses you might see:"));
  Serial.println(F("  EZO-EC        : 0x64"));
  Serial.println(F("  Atlas pumps   : 0x32, 0x33, 0x34, 0x35"));
  Serial.println(F("Run will repeat in 5 s...\n"));

  delay(5000); // rescan every 5 seconds
}
