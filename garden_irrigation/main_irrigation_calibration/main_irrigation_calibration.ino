#include <Arduino.h>

#define FLOW_PIN 14   // GPIO14 = D5 (ESP8266)

// ---------------- FLOW ----------------
volatile uint32_t pulseCount = 0;

void ICACHE_RAM_ATTR flowISR() {
  pulseCount++;
}

// ---------------- PRESSURE ----------------
bool haveP1 = false;
int raw1 = 0, raw2 = 0;
float P1_kPa = 0, P2_kPa = 0;

// ---------------- UTILS ----------------
String readLine() {
  String s;
  while (true) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') return s;
      if (c != '\r') s += c;
    }
    yield();
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(FLOW_PIN, INPUT);   // external pull-up fitted
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowISR, FALLING);

  Serial.println();
  Serial.println("FLOW + PRESSURE CALIBRATION (kPa)");
  Serial.println("Commands:");
  Serial.println("  flow_start");
  Serial.println("  flow_stop <litres>");
  Serial.println("  press <kPa>");
  Serial.println();
}

void loop() {
  Serial.print("> ");
  String cmd = readLine();
  cmd.trim();

  // -------- FLOW START --------
  if (cmd == "flow_start") {
    pulseCount = 0;
    Serial.println("Flow calibration started. Collect water.");
    return;
  }

  // -------- FLOW STOP --------
  if (cmd.startsWith("flow_stop")) {
    float litres = cmd.substring(10).toFloat();
    if (litres <= 0) {
      Serial.println("Usage: flow_stop <litres>");
      return;
    }

    uint32_t pulses = pulseCount;
    float ppl = pulses / litres;

    Serial.println();
    Serial.println("Flow calibration result:");
    Serial.print("  Pulses counted: "); Serial.println(pulses);
    Serial.print("  Volume entered: "); Serial.print(litres, 3); Serial.println(" L");
    Serial.print("  ==> PULSES_PER_LITRE = "); Serial.println(ppl, 6);
    Serial.println("Use in code:");
    Serial.print("  const float PULSES_PER_LITRE = ");
    Serial.print(ppl, 6);
    Serial.println(";");
    Serial.println();
    return;
  }

  // -------- PRESSURE --------
  if (cmd.startsWith("press")) {
    float kPa = cmd.substring(6).toFloat();
    int raw = analogRead(A0);

    if (!haveP1) {
      raw1 = raw;
      P1_kPa = kPa;
      haveP1 = true;
      Serial.print("Captured P1: ");
      Serial.print(P1_kPa); Serial.print(" kPa at ADC=");
      Serial.println(raw1);
      Serial.println("Set a different pressure and repeat: press <kPa>");
      return;
    }

    raw2 = raw;
    P2_kPa = kPa;

    int dRaw = raw2 - raw1;
    float dP = P2_kPa - P1_kPa;

    if (dRaw == 0) {
      Serial.println("Error: ADC readings identical.");
      haveP1 = false;
      return;
    }

    float m = dP / dRaw;
    float b = P1_kPa - m * raw1;

    Serial.println();
    Serial.println("Pressure calibration result (kPa):");
    Serial.print("  m = "); Serial.print(m, 8); Serial.println(" kPa/count");
    Serial.print("  b = "); Serial.print(b, 4); Serial.println(" kPa");
    Serial.println("Use in code:");
    Serial.print("  const float PRESS_M = ");
    Serial.print(m, 8);
    Serial.println(";");
    Serial.print("  const float PRESS_B = ");
    Serial.print(b, 4);
    Serial.println(";");
    Serial.println();

    haveP1 = false;
    return;
  }

  Serial.println("Unknown command.");
}
