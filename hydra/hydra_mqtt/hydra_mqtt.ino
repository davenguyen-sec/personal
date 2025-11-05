/* ============================================================================
 * Project: Hydroponics Controller MQTT Bridge (NodeMCU v2 / ESP8266)
 * Purpose: Receive CSV telemetry from Arduino Mega via Serial, parse, and
 *          publish retained MQTT topics with Home Assistant auto-discovery.
 *
 * Summary
 * - Reads one CSV line per cycle from the Mega controller (Serial@115200).
 * - Parses sensor values, setpoints, status fields, and hourly dosing totals.
 * - Publishes retained MQTT states under the "hydro/*" namespace (retained).
 * - Republishes Home Assistant discovery payloads every 5 min for resilience.
 * - Optional raw CSV mirror and heartbeat topics for diagnostics.
 *
 * Data contract (2025-11)
 * - Keys: PH_RAW, PH_C, EC, T, PH_SP, EC_SP, PH_ERR, EC_ERR,
 *         A_HR, B_HR, ECD_HR, PHD_HR, LAST, LAST_ML, UPTIME_MS
 * - Unknown keys are ignored safely.
 *
 * Hardware
 * - Board: NodeMCU v2 (ESP8266)
 * - Serial link: NodeMCU RX0 ← Mega TX1 (Serial1 on Mega)
 * - Logic levels: Mega TX1 is 5 V; ESP8266 RX0 is 3.3 V only.
 *                 Use a voltage divider or level shifter on the RX line.
 * - Power: 5 V via USB or shared supply (common GND).
 *
 * Networking
 * - WPA/WPA2 Wi-Fi, MQTT over TCP (port 1883), retained messages, LWT on "hydro/status".
 *
 * Notes
 * - Expects well-formed CSV from the Mega; NaN values are dropped.
 * - Home Assistant discovery topics published under "homeassistant/*".
 * ===========================================================================*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// -----------------------------------------------------------------------------
// Network credentials (edit)
// -----------------------------------------------------------------------------
const char* WIFI_SSID  = "WIFI_SSID";         // Wi-Fi SSID
const char* WIFI_PASS  = "WIFI_PASS";         // Wi-Fi password

// -----------------------------------------------------------------------------
// MQTT broker settings
// -----------------------------------------------------------------------------
const char* MQTT_HOST   = "192.168.2.16";  // MQTT broker hostname or IP
const uint16_t MQTT_PORT = 1883;           // Broker port (1883 = unencrypted)
const char* MQTT_USER   = "mqttuser";      // MQTT username
const char* MQTT_PASSWD = "MQTT_PASSWD";            // MQTT password
const char* CLIENT_ID    = "hydro-nodemcu";

// MQTT buffer size for HA discovery JSON
const uint16_t MQTT_BUF  = 1024;

// -----------------------------------------------------------------------------
// MQTT topics (all retained)
// -----------------------------------------------------------------------------
const char* TOPIC_PH        = "hydro/pH";                 // pH raw (instantaneous), 2 d.p.
const char* TOPIC_PH_FILT   = "hydro/ph_filt";            // pH filtered (EMA), 2 d.p.
const char* TOPIC_EC        = "hydro/ec_uScm";            // EC in µS/cm, integer
const char* TOPIC_TEMP      = "hydro/temp_C";             // Water temp °C, 1 d.p.
const char* TOPIC_AVAIL     = "hydro/status";             // LWT/online

// Setpoints and error terms (from Mega)
const char* TOPIC_PH_SP     = "hydro/ph_sp";              // pH setpoint
const char* TOPIC_EC_SP     = "hydro/ec_sp_uScm";         // EC setpoint (µS/cm)
const char* TOPIC_PH_ERR    = "hydro/ph_err";             // PH_SP − PH_CONTROL (EMA)
const char* TOPIC_EC_ERR    = "hydro/ec_err_uScm";        // EC_SP − EC (µS/cm)

// Hourly dosing totals
const char* TOPIC_A_HR      = "hydro/dose_a_ml_hr";       // Nutrient A this hour (mL)
const char* TOPIC_B_HR      = "hydro/dose_b_ml_hr";       // Nutrient B this hour (mL)
const char* TOPIC_ECD_HR    = "hydro/dose_ec_down_ml_hr"; // Water (EC-down) this hour (mL)
const char* TOPIC_PHD_HR    = "hydro/dose_ph_down_ml_hr"; // pH-down this hour (mL)

// Last dosing action snapshot
const char* TOPIC_LAST      = "hydro/last_action";        // A|B|ECD|pHD|None
const char* TOPIC_LAST_ML   = "hydro/last_action_ml";     // Volume of last action (mL)

// Runtime
const char* TOPIC_UPTIME    = "hydro/uptime_ms";          // Controller uptime in ms

// -----------------------------------------------------------------------------
// Debug toggles
// -----------------------------------------------------------------------------
#define ENABLE_RAW_MQTT   1   // 1=mirror raw CSV to hydro/raw
#define ENABLE_HEARTBEAT  0   // 1=publish hydro/beat every 10 s

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------
WiFiClient espClient;
PubSubClient mqtt(espClient);
String lineBuf;
bool discoverySent = false;
unsigned long lastBeat = 0;

// -----------------------------------------------------------------------------
// Wi-Fi / MQTT helpers
// -----------------------------------------------------------------------------
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.hostname(CLIENT_ID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(250); yield(); }
}

inline void publishAvailability(const char* s) { mqtt.publish(TOPIC_AVAIL, s, true); }
inline void pubRetained(const char* topic, const String& val) { mqtt.publish(topic, val.c_str(), true); }

inline void pubNum(const char* topic, float v, uint8_t dp) {
  char buf[24];
  dtostrf(v, 0, dp, buf);
  mqtt.publish(topic, buf, true);
}

// -----------------------------------------------------------------------------
// Home Assistant Discovery
// -----------------------------------------------------------------------------
void publishDiscovery() {
  const char* device = R"json(
    "device":{
      "identifiers":["hydro_controller_001"],
      "name":"Hydroponics Controller",
      "manufacturer":"DIY",
      "model":"Mega+ESP8266",
      "sw_version":"1.3"
    },
    "availability_topic":"hydro/status",
    "availability_mode":"latest"
  )json";

  auto disc = [&](const char* comp, const char* slug, const String& name,
                  const char* state_topic, const char* unit = nullptr,
                  const char* device_class = nullptr,
                  const char* state_class = nullptr,
                  const char* icon = nullptr,
                  const char* entity_category = nullptr) {
    const String topic = String("homeassistant/") + comp + "/hydro_" + slug + "/config";
    String p = String("{") +
      "\"name\":\"" + name + "\"," +
      "\"unique_id\":\"hydro_" + String(slug) + "_001\"," +
      "\"state_topic\":\"" + String(state_topic) + "\",";
    if (unit)            p += "\"unit_of_measurement\":\"" + String(unit) + "\",";
    if (device_class)    p += "\"device_class\":\"" + String(device_class) + "\",";
    if (state_class)     p += "\"state_class\":\"" + String(state_class) + "\",";
    if (icon)            p += "\"icon\":\"" + String(icon) + "\",";
    if (entity_category) p += "\"entity_category\":\"" + String(entity_category) + "\",";
    p += String(device) + "}";
    bool ok = mqtt.publish(topic.c_str(), p.c_str(), true);
    if (!ok) { Serial.print("Discovery publish failed: "); Serial.println(topic); }
  };

  // Primary sensors
  disc("sensor","ph",        "Hydro pH",             TOPIC_PH,       "pH",    nullptr, "measurement", "mdi:alpha-p-circle-outline");
  disc("sensor","ph_filt",   "Hydro pH (filtered)",  TOPIC_PH_FILT,  "pH",    nullptr, "measurement", "mdi:alpha-p-circle");
  disc("sensor","ec",        "Hydro EC",             TOPIC_EC,       "µS/cm", nullptr, "measurement", "mdi:flash");
  disc("sensor","temp",      "Hydro Water Temp",     TOPIC_TEMP,     "°C",    "temperature","measurement");

  // Setpoints
  disc("sensor","ph_sp",     "Hydro pH Setpoint",    TOPIC_PH_SP,    "pH",    nullptr, "measurement", "mdi:target", "config");
  disc("sensor","ec_sp",     "Hydro EC Setpoint",    TOPIC_EC_SP,    "µS/cm", nullptr, "measurement", "mdi:target", "config");

  // Errors
  disc("sensor","ph_err",    "Hydro pH Error",       TOPIC_PH_ERR,   "pH",    nullptr, "measurement", "mdi:sigma");
  disc("sensor","ec_err",    "Hydro EC Error",       TOPIC_EC_ERR,   "µS/cm", nullptr, "measurement", "mdi:sigma");

  // Hourly dosing
  disc("sensor","dose_a_hr",   "Dose A this hour",   TOPIC_A_HR,     "mL",    nullptr, "total_increasing", "mdi:beaker");
  disc("sensor","dose_b_hr",   "Dose B this hour",   TOPIC_B_HR,     "mL",    nullptr, "total_increasing", "mdi:beaker");
  disc("sensor","dose_ecd_hr", "EC-Down this hour",  TOPIC_ECD_HR,   "mL",    nullptr, "total_increasing", "mdi:water-plus");
  disc("sensor","dose_phd_hr", "pH-Down this hour",  TOPIC_PHD_HR,   "mL",    nullptr, "total_increasing", "mdi:flask-round-bottom");

  // Status
  disc("sensor","last_act",    "Last Action",        TOPIC_LAST,     nullptr, nullptr, nullptr, "mdi:history");
  disc("sensor","last_act_ml", "Last Action mL",     TOPIC_LAST_ML,  "mL",    nullptr, "measurement", "mdi:history");
  disc("sensor","uptime_ms",   "Controller Uptime",  TOPIC_UPTIME,   "ms",    nullptr, "measurement", "mdi:clock-outline", "diagnostic");

  discoverySent = true;
}

void ensureMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setKeepAlive(30);
  mqtt.setSocketTimeout(5);
  mqtt.setBufferSize(MQTT_BUF);

  while (!mqtt.connected()) {
    if (mqtt.connect(CLIENT_ID, MQTT_USER, MQTT_PASSWD,
                     TOPIC_AVAIL, 1, true, "offline")) {
      publishAvailability("online");
      discoverySent = false;
      publishDiscovery();
      break;
    }
    delay(1500);
    yield();
  }
}

// -----------------------------------------------------------------------------
// CSV parsing
// -----------------------------------------------------------------------------
struct KeyMap { const char* key; const char* topic; uint8_t decimals; bool isString; };

const KeyMap MAP[] = {
  // Current keys only
  {"PH_RAW",    TOPIC_PH,        2, false},
  {"PH_C",      TOPIC_PH_FILT,   2, false},
  {"EC",        TOPIC_EC,        0, false},
  {"T",         TOPIC_TEMP,      1, false},
  {"PH_SP",     TOPIC_PH_SP,     1, false},
  {"EC_SP",     TOPIC_EC_SP,     0, false},
  {"PH_ERR",    TOPIC_PH_ERR,    2, false},
  {"EC_ERR",    TOPIC_EC_ERR,    0, false},
  {"A_HR",      TOPIC_A_HR,      0, false},
  {"B_HR",      TOPIC_B_HR,      0, false},
  {"ECD_HR",    TOPIC_ECD_HR,    0, false},
  {"PHD_HR",    TOPIC_PHD_HR,    0, false},
  {"LAST",      TOPIC_LAST,      0, true },
  {"LAST_ML",   TOPIC_LAST_ML,   1, false},
  {"UPTIME_MS", TOPIC_UPTIME,    0, false},
};

void processLine(const String& s) {
#if ENABLE_RAW_MQTT
  mqtt.publish("hydro/raw", s.c_str(), true);
#endif

  // Tokenise by comma
  String tok; String arr[96]; int i = 0;
  for (char ch : s) {
    if (ch == ',') { if (i < 96) arr[i++] = tok; tok = ""; }
    else           { tok += ch; }
  }
  if (tok.length() && i < 96) arr[i++] = tok;
  for (int k = 0; k < i; k++) arr[k].trim();

  if (!discoverySent) publishDiscovery();

  // Map to topics
  for (int j = 0; j < i - 1; j += 2) {
    const String key = arr[j];
    const String val = arr[j + 1];

    for (auto &m : MAP) {
      if (key.equals(m.key)) {
        if (m.isString) {
          pubRetained(m.topic, val);
        } else {
          if (val.equalsIgnoreCase("NaN")) break;
          char buf[24];
          dtostrf(val.toFloat(), 0, m.decimals, buf);
          mqtt.publish(m.topic, buf, true);
        }
        break;
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Arduino entry points
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200); // UART0 listens to Mega Serial1 TX
  connectWiFi();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected()) ensureMQTT();
  mqtt.loop();

#if ENABLE_HEARTBEAT
  if (millis() - lastBeat > 10000UL) {
    mqtt.publish("hydro/beat", "1", true);
    lastBeat = millis();
  }
#endif

  // Refresh discovery every 5 min for resilience after HA restarts
  static unsigned long lastDisc = 0;
  if (millis() - lastDisc > 300000UL) {
    publishDiscovery();
    lastDisc = millis();
  }

  // Line-oriented reader from the Mega (ends on '\n')
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (lineBuf.length()) { processLine(lineBuf); lineBuf = ""; }
    } else if (isPrintable(c)) {
      lineBuf += c;
      if (lineBuf.length() > 480) lineBuf = ""; // guard
    }
  }
}

