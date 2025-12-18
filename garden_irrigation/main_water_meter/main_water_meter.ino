#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// --------- User settings ----------
const char* WIFI_SSID = "";
const char* WIFI_PASS = "";

const char* MQTT_HOST = "192.168.2.16";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "mqttuser";   // optional
const char* MQTT_PASS = "";   // optional

const char* DEVICE_ID = "main_water_meter";

// --------- Pins ----------
const uint8_t FLOW_PIN = 14;   // GPIO14 = D5 (interrupt capable)

// --------- Calibration (yours) ----------
const float PULSES_PER_LITRE = 380.000000f; //332 low //432 high

const float PRESS_M = 1.12944162f;   // kPa/count
const float PRESS_B = -131.0152f;    // kPa

//Previously overestimated (higher than true value):
//const float PRESS_M = 1.22807014f;   // kPa/count
//const float PRESS_B = -157.1930f;    // kPa

// --------- Timing ----------
const uint32_t SAMPLE_MS = 1000;
const uint32_t PUB_MS    = 5000;

// --------- MQTT topics ----------
String baseTopic, tTotal, tFlow, tPress, tAvail;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

volatile uint32_t pulseCount = 0;
void ICACHE_RAM_ATTR flowISR() { pulseCount++; }

double totalLitres = 0.0;
uint32_t lastPulseSnapshot = 0;
float flowLpm = 0.0f;
float pressureKPa = 0.0f;

uint32_t lastSampleMs = 0;
uint32_t lastPubMs = 0;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("BOOT: starting water meter MQTT publisher");

  baseTopic = String("water/") + DEVICE_ID;
  tTotal = baseTopic + "/total_l";
  tFlow  = baseTopic + "/flow_lpm";
  tPress = baseTopic + "/pressure_kpa";
  tAvail = baseTopic + "/availability";

  Serial.println("Topics:");
  Serial.println("  " + tTotal);
  Serial.println("  " + tFlow);
  Serial.println("  " + tPress);
  Serial.println("  " + tAvail);

  pinMode(FLOW_PIN, INPUT); // external pull-up recommended
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowISR, FALLING);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void loop() {
  // Wi-Fi connect (non-blocking)
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t last = 0;
    if (millis() - last > 1000) {
      Serial.println("WiFi: connecting...");
      last = millis();
    }
  } else {
    static bool printedIP = false;
    if (!printedIP) {
      Serial.print("WiFi: connected, IP=");
      Serial.println(WiFi.localIP());
      printedIP = true;
    }
  }

  // MQTT connect
  if (WiFi.status() == WL_CONNECTED && !mqtt.connected()) {
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    Serial.println("MQTT: connecting...");
    bool ok = mqtt.connect(DEVICE_ID, MQTT_USER, MQTT_PASS, tAvail.c_str(), 1, true, "offline");
    Serial.println(ok ? "MQTT: connected" : "MQTT: connect failed");
    if (ok) mqtt.publish(tAvail.c_str(), "online", true);
    delay(200);
  }

  mqtt.loop();

  // Sample once per second
  uint32_t now = millis();
  if (now - lastSampleMs >= SAMPLE_MS) {
    uint32_t pNow;
    noInterrupts();
    pNow = pulseCount;
    interrupts();

    uint32_t dp = pNow - lastPulseSnapshot;
    lastPulseSnapshot = pNow;

    totalLitres += (double)dp / (double)PULSES_PER_LITRE;

    float hz = (float)dp / (SAMPLE_MS / 1000.0f);
    flowLpm = (hz * 60.0f) / PULSES_PER_LITRE;

    int raw = analogRead(A0);
    pressureKPa = PRESS_M * (float)raw + PRESS_B;
    if (pressureKPa < 0) pressureKPa = 0;

    Serial.printf("Sample: dp=%u  total=%.3f L  flow=%.3f L/min  press=%.1f kPa\n",
                  dp, totalLitres, flowLpm, pressureKPa);

    lastSampleMs = now;
  }

  // Publish every 5 seconds
  if (mqtt.connected() && now - lastPubMs >= PUB_MS) {
    mqtt.publish(tTotal.c_str(), String(totalLitres, 3).c_str(), true);
    mqtt.publish(tFlow.c_str(),  String(flowLpm, 3).c_str(), false);
    mqtt.publish(tPress.c_str(), String(pressureKPa, 1).c_str(), false);
    Serial.println("MQTT: published");
    lastPubMs = now;
  }
}