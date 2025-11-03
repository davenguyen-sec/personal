// NodeMCU v2 (ESP8266) — MQTT bridge with Home Assistant Discovery
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const char* WIFI_SSID = "WIFI_SSID";
const char* WIFI_PASS = "WIFI_PASS";

const char* MQTT_HOST   = "192.168.2.16";
const uint16_t MQTT_PORT= 1883;
const char* MQTT_USER   = "mqttuser";
const char* MQTT_PASSWD = "MQTT_PASSWD";

const char* CLIENT_ID   = "hydro-nodemcu";

// Increase buffer for large discovery JSON payloads
const uint16_t MQTT_BUF = 1024;

// ---- State & availability topics (existing) ----
const char* TOPIC_PH     = "hydro/pH";
const char* TOPIC_EC     = "hydro/ec_uScm";
const char* TOPIC_TEMP   = "hydro/temp_C";
const char* TOPIC_AVAIL  = "hydro/status";

// ---- New topics (match Arduino CSV keys) ----
const char* TOPIC_PH_SP     = "hydro/ph_sp";
const char* TOPIC_EC_SP     = "hydro/ec_sp_uScm";
const char* TOPIC_EC_STATE  = "hydro/ec_state";              // OK|UP|DOWN|NA
const char* TOPIC_PH_ERR    = "hydro/ph_err";
const char* TOPIC_EC_ERR    = "hydro/ec_err_uScm";
const char* TOPIC_A_HR      = "hydro/dose_a_ml_hr";
const char* TOPIC_B_HR      = "hydro/dose_b_ml_hr";
const char* TOPIC_ECD_HR    = "hydro/dose_ec_down_ml_hr";
const char* TOPIC_PHD_HR    = "hydro/dose_ph_down_ml_hr";
const char* TOPIC_LAST      = "hydro/last_action";           // A|B|ECD|pHD|None
const char* TOPIC_LAST_ML   = "hydro/last_action_ml";
const char* TOPIC_UPTIME    = "hydro/uptime_ms";

// Debug toggles
#define ENABLE_RAW_MQTT   1   // mirror raw CSV to hydro/raw
#define ENABLE_HEARTBEAT  0   // publish hydro/beat every 10 s

// ==== GLOBALS ====
WiFiClient espClient;
PubSubClient mqtt(espClient);
String lineBuf;
bool discoverySent = false;
unsigned long lastBeat = 0;

// ===== Wi-Fi / MQTT helpers =====
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.hostname(CLIENT_ID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(250); yield(); }
}

void publishAvailability(const char* s) {
  mqtt.publish(TOPIC_AVAIL, s, true); // retained
}

// Small helper to publish retained numeric/string
inline void pubRetained(const char* topic, const String& val) {
  mqtt.publish(topic, val.c_str(), true);
}

// ===== Home Assistant Discovery =====
void publishDiscovery() {
  // Shared device block (included in each entity)
  const char* device = R"json(
    "device":{
      "identifiers":["hydro_controller_001"],
      "name":"Hydroponics Controller",
      "manufacturer":"DIY",
      "model":"Mega+ESP8266",
      "sw_version":"1.1"
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
    bool ok = mqtt.publish(topic.c_str(), p.c_str(), true); // retained
    if (!ok) {
      Serial.print("Discovery publish failed: ");
      Serial.println(topic);
    }
  };

  // Existing three
  disc("sensor","ph","Hydro pH",              TOPIC_PH,  "pH",    nullptr, "measurement", "mdi:alpha-p-circle-outline");
  disc("sensor","ec","Hydro EC",              TOPIC_EC,  "µS/cm", nullptr, "measurement", "mdi:flash");
  disc("sensor","temp","Hydro Water Temp",    TOPIC_TEMP,"°C",    "temperature","measurement");

  // New, minimal + useful set
  disc("sensor","ph_sp","Hydro pH Setpoint",     TOPIC_PH_SP, "pH",    nullptr,"measurement","mdi:target","config");
  disc("sensor","ec_sp","Hydro EC Setpoint",     TOPIC_EC_SP, "µS/cm", nullptr,"measurement","mdi:target","config");

  disc("sensor","ec_state","Hydro EC State",     TOPIC_EC_STATE, nullptr,nullptr,nullptr,"mdi:flash");

  disc("sensor","ph_err","Hydro pH Error",       TOPIC_PH_ERR, "pH",    nullptr,"measurement","mdi:sigma");
  disc("sensor","ec_err","Hydro EC Error",       TOPIC_EC_ERR, "µS/cm", nullptr,"measurement","mdi:sigma");

  disc("sensor","dose_a_hr","Dose A this hour",   TOPIC_A_HR,   "mL", nullptr,"total_increasing","mdi:beaker");
  disc("sensor","dose_b_hr","Dose B this hour",   TOPIC_B_HR,   "mL", nullptr,"total_increasing","mdi:beaker");
  disc("sensor","dose_ecd_hr","EC-Down this hour",TOPIC_ECD_HR, "mL", nullptr,"total_increasing","mdi:water-plus");
  disc("sensor","dose_phd_hr","pH-Down this hour",TOPIC_PHD_HR, "mL", nullptr,"total_increasing","mdi:flask-round-bottom");

  disc("sensor","last_act","Last Action",         TOPIC_LAST,   nullptr,nullptr,nullptr,"mdi:history");
  disc("sensor","last_act_ml","Last Action mL",   TOPIC_LAST_ML,"mL",  nullptr,"measurement","mdi:history");

  disc("sensor","uptime_ms","Controller Uptime",  TOPIC_UPTIME,"ms", nullptr,"measurement","mdi:clock-outline","diagnostic");

  discoverySent = true;
}

void ensureMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setKeepAlive(30);
  mqtt.setSocketTimeout(5);
  mqtt.setBufferSize(MQTT_BUF);   // important for discovery JSON

  while (!mqtt.connected()) {
    if (mqtt.connect(CLIENT_ID, MQTT_USER, MQTT_PASSWD,
                     TOPIC_AVAIL, 1, true, "offline")) {
      publishAvailability("online");   // birth
      discoverySent = false;           // re-send on each reconnect
      publishDiscovery();              // retained discovery payloads
      break;
    }
    delay(1500);
    yield();
  }
}

inline void pubNum(const char* topic, float v, uint8_t dp) {
  char buf[24];
  dtostrf(v, 0, dp, buf);
  mqtt.publish(topic, buf, true);
}

// ===== CSV parsing =====
// Accepts lines like:
// PH,5.78,EC,1605,T,26.9,PH_SP,5.8,EC_SP,1600,EC_STATE,OK,PH_ERR,0.02,EC_ERR,-5,A_HR,8,B_HR,8,ECD_HR,0,PHD_HR,1,LAST,A,LAST_ML,4.0,UPTIME_MS,1234567
struct KeyMap { const char* key; const char* topic; uint8_t decimals; bool isString; };

const KeyMap MAP[] = {
  {"PH_SP",     TOPIC_PH_SP,    1, false},
  {"EC_SP",     TOPIC_EC_SP,    0, false},
  {"EC_STATE",  TOPIC_EC_STATE, 0, true },
  {"PH_ERR",    TOPIC_PH_ERR,   2, false},
  {"EC_ERR",    TOPIC_EC_ERR,   0, false},
  {"A_HR",      TOPIC_A_HR,     0, false},
  {"B_HR",      TOPIC_B_HR,     0, false},
  {"ECD_HR",    TOPIC_ECD_HR,   0, false},
  {"PHD_HR",    TOPIC_PHD_HR,   0, false},
  {"LAST",      TOPIC_LAST,     0, true },
  {"LAST_ML",   TOPIC_LAST_ML,  1, false},
  {"UPTIME_MS", TOPIC_UPTIME,   0, false},
};

void processLine(const String& s) {
#if ENABLE_RAW_MQTT
  mqtt.publish("hydro/raw", s.c_str(), true);  // mirror raw CSV (retained)
#endif

  // Split to tokens (64 tokens max)
  String tok; String arr[64]; int i = 0;
  for (char ch : s) {
    if (ch == ',') { if (i < 64) arr[i++] = tok; tok = ""; }
    else           { tok += ch; }
  }
  if (tok.length() && i < 64) arr[i++] = tok;

  for (int k = 0; k < i; k++) arr[k].trim();

  // Legacy values for the trio
  float ph = NAN, tC = NAN; long ec = -1;

  if (!discoverySent) publishDiscovery();

  for (int j = 0; j < i - 1; j += 2) {
    const String key = arr[j];
    const String val = arr[j + 1];

    // Legacy trio
    if      (key == "PH") { ph = val.toFloat(); continue; }
    else if (key == "EC") { ec = val.toInt();   continue; }
    else if (key == "T")  { tC = val.toFloat(); continue; }

    // New keys via map
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

  // Publish legacy trio last (retained)
  if (!isnan(ph))  pubRetained(TOPIC_PH,   String(ph, 2));
  if (ec >= 0)     pubRetained(TOPIC_EC,   String(ec));
  if (!isnan(tC))  pubRetained(TOPIC_TEMP, String(tC, 1));
}

// ===== Arduino entry points =====
void setup() {
  Serial.begin(115200); // Hardware UART0 (RX0=GPIO3, TX0=GPIO1)
  connectWiFi();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected()) ensureMQTT();
  mqtt.loop();

#if ENABLE_HEARTBEAT
  if (millis() - lastBeat > 10000) {
    mqtt.publish("hydro/beat", "1", true);
    lastBeat = millis();
  }
#endif

  // Resend discovery every 5 minutes (covers HA restarts)
  static unsigned long lastDisc = 0;
  if (millis() - lastDisc > 300000UL) {
    publishDiscovery();
    lastDisc = millis();
  }

  // Line-oriented reader from the Mega/UNO
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (lineBuf.length()) { processLine(lineBuf); lineBuf = ""; }
    } else if (isPrintable(c)) {
      lineBuf += c;
      if (lineBuf.length() > 480) lineBuf = ""; // guard against overflow
    }
  }
}