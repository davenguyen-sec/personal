/* ============================================================
   Peristaltic Pump Tester (I2C, manual control via Serial)
   - Test each pump individually by volume (mL) or time (ms)
   - Quick "prime" and "stop" commands
   - Matches your previous controller addresses/roles
   ============================================================ */

// ---------- Includes ----------
#include <Arduino.h>
#include <Wire.h>

// ---------- I2C addresses (CHANGE IF YOUR HARDWARE DIFFERS) ----------
#define PH_DOWN_ADDR  0x32   // Pump 1: pH Down
#define PH_UP_ADDR    0x33   // Pump 2: pH Up
#define EC_UP_ADDR    0x34   // Pump 3: Nutrients (EC up)
#define EC_DOWN_ADDR  0x35   // Pump 4: Water (EC down)

// ---------- SAFETY/TUNABLES (YOU CAN CHANGE THESE) ----------
static const float  MAX_TEST_ML   = 50.0;      // MAX single dose when using "dose" (mL)
static const unsigned long MAX_TEST_MS = 60000UL; // MAX single run when using "time"/"prime" (ms)
static const unsigned long SERIAL_BAUD = 9600;    // Serial speed for the monitor

// ---------- HELP TEXT ----------
const char *HELP_TEXT =
  "\nPump Tester — commands (type then press Enter):\n"
  "  help\n"
  "  list                         # show pump names and I2C addresses\n"
  "  dose <pump> <mL>             # e.g. dose ecup 5.0\n"
  "  time <pump> <ms>             # e.g. time ecdn 2500   (run by time)\n"
  "  prime <pump> <ms>            # alias of 'time', intended for priming lines\n"
  "  stop <pump|all>              # sends STOP to one or all pumps\n"
  "  testall                      # runs a short test on all pumps in sequence\n"
  "\nPumps (case-insensitive): phup, phdn, ecup, ecdn\n";

// ---------- Simple map from text token -> address ----------
struct PumpDesc {
  const char *name;
  uint8_t addr;
};

PumpDesc PUMPS[] = {
  {"phdn", PH_DOWN_ADDR},   // pH Down
  {"phup", PH_UP_ADDR},     // pH Up
  {"ecup", EC_UP_ADDR},     // EC Up (nutrients)
  {"ecdn", EC_DOWN_ADDR}    // EC Down (water)
};

static const size_t NUM_PUMPS = sizeof(PUMPS)/sizeof(PUMPS[0]);

// ---------- I2C helpers ----------
void i2cSend(const uint8_t addr, const char *cmd) {
  Wire.beginTransmission(addr);
  while (*cmd) Wire.write(*cmd++);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.print("I2C error to 0x"); Serial.print(addr, HEX);
    Serial.print(" (code "); Serial.print(err); Serial.println(").");
  }
  delay(10);
}

void pumpDose_mL(const uint8_t addr, float mL) {
  if (mL <= 0.0f) { Serial.println("Dose volume must be > 0 mL."); return; }
  if (mL > MAX_TEST_ML) {
    Serial.print("Clamping to MAX_TEST_ML="); Serial.print(MAX_TEST_ML); Serial.println(" mL.");
    mL = MAX_TEST_ML;
  }
  String cmd = "D," + String(mL, 2);
  i2cSend(addr, cmd.c_str());
}

void pumpRun_timeMs(const uint8_t addr, unsigned long ms) {
  if (ms == 0) { Serial.println("Time must be > 0 ms."); return; }
  if (ms > MAX_TEST_MS) {
    Serial.print("Clamping to MAX_TEST_MS="); Serial.print(MAX_TEST_MS); Serial.println(" ms.");
    ms = MAX_TEST_MS;
  }
  String cmd = "T," + String(ms);
  i2cSend(addr, cmd.c_str());
}

void pumpStop(const uint8_t addr) {
  // Common firmwares accept a stop command like 'X' — adjust if your modules differ
  i2cSend(addr, "X");
}

// ---------- Utility: find address by token ----------
bool lookupPumpAddr(const String &token, uint8_t &addrOut, const char* &stdNameOut) {
  String t = token; t.trim(); t.toLowerCase();
  for (size_t i = 0; i < NUM_PUMPS; ++i) {
    String n = PUMPS[i].name;
    if (t == n) {
      addrOut = PUMPS[i].addr;
      stdNameOut = PUMPS[i].name;
      return true;
    }
  }
  return false;
}

void printList() {
  Serial.println("\nPumps:");
  for (size_t i = 0; i < NUM_PUMPS; ++i) {
    Serial.print("  ");
    Serial.print(PUMPS[i].name);
    Serial.print(" @ 0x");
    Serial.println(PUMPS[i].addr, HEX);
  }
  Serial.println();
}

void quickTestAll() {
  Serial.println("Quick test: dosing each pump briefly...");
  // Adjust these defaults if you prefer time or volume
  const float test_mL = 2.0;             // small, visible dose
  const unsigned long settle_ms = 500;   // short gap between pumps

  for (size_t i = 0; i < NUM_PUMPS; ++i) {
    Serial.print("  -> "); Serial.print(PUMPS[i].name);
    Serial.print(" dose "); Serial.print(test_mL, 1); Serial.println(" mL");
    pumpDose_mL(PUMPS[i].addr, test_mL);
    delay(settle_ms);
  }
  Serial.println("Quick test complete.");
}

// ---------- Command parsing ----------
void printHelp() { Serial.print(HELP_TEXT); }

void parseLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  // Tokenise (space-separated)
  String cmd, arg1, arg2;
  int sp1 = line.indexOf(' ');
  if (sp1 < 0) {
    cmd = line;
  } else {
    cmd = line.substring(0, sp1);
    String rest = line.substring(sp1 + 1);
    rest.trim();
    int sp2 = rest.indexOf(' ');
    if (sp2 < 0) {
      arg1 = rest;
    } else {
      arg1 = rest.substring(0, sp2);
      arg2 = rest.substring(sp2 + 1);
      arg2.trim();
    }
  }

  cmd.toLowerCase();

  if (cmd == "help") { printHelp(); return; }
  if (cmd == "list") { printList(); return; }
  if (cmd == "testall") { quickTestAll(); return; }

  if (cmd == "stop") {
    if (arg1.length() == 0 || arg1.equalsIgnoreCase("all")) {
      for (size_t i = 0; i < NUM_PUMPS; ++i) pumpStop(PUMPS[i].addr);
      Serial.println("Sent STOP to all pumps.");
      return;
    } else {
      uint8_t addr; const char* name;
      if (!lookupPumpAddr(arg1, addr, name)) {
        Serial.println("Unknown pump. Use: phup, phdn, ecup, ecdn");
        return;
      }
      pumpStop(addr);
      Serial.print("Sent STOP to "); Serial.println(name);
      return;
    }
  }

  if (cmd == "dose") {
    if (arg1.length() == 0 || arg2.length() == 0) {
      Serial.println("Usage: dose <pump> <mL>   e.g. dose ecup 5.0");
      return;
    }
    uint8_t addr; const char* name;
    if (!lookupPumpAddr(arg1, addr, name)) {
      Serial.println("Unknown pump. Use: phup, phdn, ecup, ecdn");
      return;
    }
    float mL = arg2.toFloat();
    if (mL <= 0) { Serial.println("Enter a positive mL value."); return; }
    Serial.print("Dosing "); Serial.print(name);
    Serial.print(": "); Serial.print(mL, 2); Serial.println(" mL");
    pumpDose_mL(addr, mL);
    return;
  }

  if (cmd == "time" || cmd == "prime") {
    if (arg1.length() == 0 || arg2.length() == 0) {
      Serial.println(String("Usage: ") + cmd + " <pump> <ms>   e.g. " + cmd + " ecdn 2500");
      return;
    }
    uint8_t addr; const char* name;
    if (!lookupPumpAddr(arg1, addr, name)) {
      Serial.println("Unknown pump. Use: phup, phdn, ecup, ecdn");
      return;
    }
    unsigned long ms = (unsigned long)arg2.toInt();
    if (ms == 0) { Serial.println("Enter a positive time in ms."); return; }
    Serial.print("Running "); Serial.print(name);
    Serial.print(" for "); Serial.print(ms); Serial.println(" ms");
    pumpRun_timeMs(addr, ms);
    return;
  }

  Serial.println("Unknown command. Type 'help' for usage.");
}

// ---------- Arduino setup/loop ----------
void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  delay(100);
  Serial.println("\nPeristaltic Pump Tester ready.");
  printHelp();
  printList();
}

void loop() {
  // Read a whole line from Serial (blocking only while characters arrive)
  static String buffer;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;         // ignore CR
    if (c == '\n') {
      parseLine(buffer);
      buffer = "";
    } else {
      buffer += c;
    }
  }
}
