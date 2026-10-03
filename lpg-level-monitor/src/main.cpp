/*
  LPG level monitor
  Board : Wemos/LOLIN D1 mini Pro (ESP8266EX)
  Sensor: DYP L062MUW ultrasonic liquefied-gas level sensor, UART auto output

  Sensor frame (115200 8N1, sent continuously, ~800 ms in processed mode):
    FF  LvlH LvlL  TmpH TmpL  SigH SigL  AngH AngL  SUM
    level mm, temp 0.1 C (signed), echo signal mV, tilt 0.1 deg,
    SUM = low 8 bits of the sum of the 9 previous bytes.

  Features
    - Web UI: live status, settings (WiFi, MQTT, tank), WiFi scan, OTA firmware upload
    - Setup access point + captive portal when WiFi isn't configured or is unreachable
    - MQTT with username/password, optional TLS, LWT, retained state,
      command topic, optional Home Assistant discovery
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <stdarg.h>

// ---------------------------------------------------------------- serial ports
#ifdef SENSOR_SOFTSERIAL
  #include <SoftwareSerial.h>
  SoftwareSerial sensorSerial(D7, -1);   // RX only, sensor green wire -> D7
  #define SENSOR sensorSerial
  #define DBG Serial                     // debug on USB
#else
  #define SENSOR Serial                  // hardware UART0, swapped to D7 (RX) / D8 (TX)
  #define DBG Serial1                    // debug output (TX only) on D4 / GPIO2
#endif

// ---------------------------------------------------------------- constants
static const char *FW_VERSION         = "1.4.0";
static const char *CONFIG_FILE        = "/config.json";
static const char *AP_PASSWORD        = "lpgsetup";      // setup AP password (min 8 chars)
static const char *WEB_USER           = "admin";
static const uint32_t SENSOR_TIMEOUT_MS   = 5000;
static const uint32_t WIFI_CONNECT_MS     = 20000;
static const uint32_t WIFI_LOST_TO_AP_MS  = 60000;
static const uint32_t AP_AUTO_OFF_MS      = 300000;
static const uint32_t MQTT_RETRY_MS       = 15000;
static const uint8_t  ALERT_HYST_PCT      = 5;       // must rise this far above the threshold to clear
static const uint32_t ALERT_CONFIRM_MS    = 30000;   // condition must hold this long before changing

// ---------------------------------------------------------------- configuration
struct Config {
  String   wifiSsid;
  String   wifiPass;
  String   deviceName   = "lpg-sensor";
  String   webPass;                       // empty = no login required
  String   mqttHost;
  uint16_t mqttPort     = 1883;
  String   mqttUser;
  String   mqttPass;
  String   mqttClientId;                  // empty = deviceName-chipid
  String   mqttTopic    = "lpg/tank1";
  bool     mqttTls      = false;
  String   mqttFingerprint;               // optional SHA1 fingerprint for TLS
  bool     mqttRetain   = true;
  bool     haDiscovery  = false;
  uint32_t publishSec   = 60;
  int16_t  levelOffsetMm = 0;             // added to the sensor reading
  uint16_t fullLevelMm  = 0;              // level that counts as 100 %, 0 = no %
  uint8_t  lowAlertPct  = 0;              // low-level alert threshold %, 0 = off
  String   tankName;                      // friendly name, e.g. "Farmhouse tank"
} cfg;

#define MAX_SUPPLIERS 3
struct Supplier { String name, phone, email, web, account, address; };
Supplier suppliers[MAX_SUPPLIERS];

String tankLabel() { return cfg.tankName.length() ? cfg.tankName : cfg.deviceName; }
void publishInfo();

// ---------------------------------------------------------------- globals
ESP8266WebServer        server(80);
ESP8266HTTPUpdateServer httpUpdater;
DNSServer               dns;
WiFiClient              plainClient;
BearSSL::WiFiClientSecure secureClient;
PubSubClient            mqtt;

bool     apActive        = false;
uint32_t apStartMs       = 0;
uint32_t wifiLostSinceMs = 0;
uint32_t lastMqttAttempt = 0;
uint32_t lastPublishMs   = 0;
uint32_t publishCount    = 0;
bool     publishNow      = false;
bool     tlsProbed       = false;
uint32_t restartAt       = 0;
bool     lowAlert        = false;
bool     alertKnown      = false;
uint32_t alertPendingMs  = 0;
String   chipId;

struct Reading {
  bool     valid    = false;   // at least one good frame received
  bool     liquid   = false;   // sensor reports a liquid level
  uint16_t raw      = 0;
  int32_t  levelMm  = 0;
  float    tempC    = NAN;
  uint16_t signalMv = 0;
  float    angleDeg = NAN;
  uint32_t atMs     = 0;
} last;

uint32_t goodFrames = 0, badFrames = 0;
uint32_t accLevel = 0, accCount = 0;     // averaging between publishes

uint8_t  frameBuf[10];
uint8_t  framePos = 0;

// ---------------------------------------------------------------- logging
#define LOG_LINES 25
String  logBuf[LOG_LINES];
uint8_t logHead = 0, logCount = 0;

void logf(const char *fmt, ...) {
  char msg[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);
  char line[180];
  snprintf(line, sizeof(line), "[%6lus] %s", (unsigned long)(millis() / 1000), msg);
  DBG.println(line);
  logBuf[logHead] = line;
  logHead = (logHead + 1) % LOG_LINES;
  if (logCount < LOG_LINES) logCount++;
}

// ---------------------------------------------------------------- helpers
String esc(const String &s) {
  String o;
  o.reserve(s.length() + 8);
  for (char c : s) {
    switch (c) {
      case '&':  o += F("&amp;");  break;
      case '<':  o += F("&lt;");   break;
      case '>':  o += F("&gt;");   break;
      case '"':  o += F("&quot;"); break;
      case '\'': o += F("&#39;");  break;
      default:   o += c;
    }
  }
  return o;
}

String topic(const char *sub) { return cfg.mqttTopic + "/" + sub; }

float round1(float v) { return isnan(v) ? v : roundf(v * 10.0f) / 10.0f; }

float fillPct(int32_t levelMm) {
  if (cfg.fullLevelMm == 0) return NAN;
  return round1(constrain(100.0f * levelMm / cfg.fullLevelMm, 0.0f, 100.0f));
}

const char *sensorStatus() {
  if (!last.valid || millis() - last.atMs > SENSOR_TIMEOUT_MS) return "sensor_offline";
  if (!last.liquid) return "no_liquid";
  return "ok";
}

const char *mqttStateText() {
  if (cfg.mqttHost.isEmpty()) return "not configured";
  switch (mqtt.state()) {
    case MQTT_CONNECTED:            return "connected";
    case MQTT_CONNECTION_TIMEOUT:   return "connection timeout";
    case MQTT_CONNECTION_LOST:      return "connection lost";
    case MQTT_CONNECT_FAILED:       return "connect failed (host/port/TLS?)";
    case MQTT_DISCONNECTED:         return "disconnected";
    case MQTT_CONNECT_BAD_PROTOCOL: return "bad protocol";
    case MQTT_CONNECT_BAD_CLIENT_ID:return "client ID rejected";
    case MQTT_CONNECT_UNAVAILABLE:  return "server unavailable";
    case MQTT_CONNECT_BAD_CREDENTIALS: return "bad username/password";
    case MQTT_CONNECT_UNAUTHORIZED: return "not authorised";
    default:                        return "unknown";
  }
}

// ---------------------------------------------------------------- config storage
void loadConfig() {
  File f = LittleFS.open(CONFIG_FILE, "r");
  if (!f) { logf("No config file, using defaults"); return; }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) { logf("Config parse error: %s", err.c_str()); return; }

  cfg.wifiSsid        = doc["wifiSsid"]        | "";
  cfg.wifiPass        = doc["wifiPass"]        | "";
  cfg.deviceName      = doc["deviceName"]      | "lpg-sensor";
  cfg.webPass         = doc["webPass"]         | "";
  cfg.mqttHost        = doc["mqttHost"]        | "";
  cfg.mqttPort        = doc["mqttPort"]        | 1883;
  cfg.mqttUser        = doc["mqttUser"]        | "";
  cfg.mqttPass        = doc["mqttPass"]        | "";
  cfg.mqttClientId    = doc["mqttClientId"]    | "";
  cfg.mqttTopic       = doc["mqttTopic"]       | "lpg/tank1";
  cfg.mqttTls         = doc["mqttTls"]         | false;
  cfg.mqttFingerprint = doc["mqttFingerprint"] | "";
  cfg.mqttRetain      = doc["mqttRetain"]      | true;
  cfg.haDiscovery     = doc["haDiscovery"]     | false;
  cfg.publishSec      = doc["publishSec"]      | 60;
  cfg.levelOffsetMm   = doc["levelOffsetMm"]   | 0;
  cfg.fullLevelMm     = doc["fullLevelMm"]     | 0;
  cfg.lowAlertPct     = doc["lowAlertPct"]     | 0;
  cfg.tankName        = doc["tankName"]        | "";
  JsonArrayConst sup  = doc["suppliers"];
  for (uint8_t i = 0; i < MAX_SUPPLIERS; i++) {
    JsonObjectConst o = sup[i];
    suppliers[i].name    = o["name"]    | "";
    suppliers[i].phone   = o["phone"]   | "";
    suppliers[i].email   = o["email"]   | "";
    suppliers[i].web     = o["web"]     | "";
    suppliers[i].account = o["account"] | "";
    suppliers[i].address = o["address"] | "";
  }
  logf("Config loaded");
}

bool saveConfig() {
  JsonDocument doc;
  doc["wifiSsid"]        = cfg.wifiSsid;
  doc["wifiPass"]        = cfg.wifiPass;
  doc["deviceName"]      = cfg.deviceName;
  doc["webPass"]         = cfg.webPass;
  doc["mqttHost"]        = cfg.mqttHost;
  doc["mqttPort"]        = cfg.mqttPort;
  doc["mqttUser"]        = cfg.mqttUser;
  doc["mqttPass"]        = cfg.mqttPass;
  doc["mqttClientId"]    = cfg.mqttClientId;
  doc["mqttTopic"]       = cfg.mqttTopic;
  doc["mqttTls"]         = cfg.mqttTls;
  doc["mqttFingerprint"] = cfg.mqttFingerprint;
  doc["mqttRetain"]      = cfg.mqttRetain;
  doc["haDiscovery"]     = cfg.haDiscovery;
  doc["publishSec"]      = cfg.publishSec;
  doc["levelOffsetMm"]   = cfg.levelOffsetMm;
  doc["fullLevelMm"]     = cfg.fullLevelMm;
  doc["lowAlertPct"]     = cfg.lowAlertPct;
  doc["tankName"]        = cfg.tankName;
  JsonArray sup = doc["suppliers"].to<JsonArray>();
  for (uint8_t i = 0; i < MAX_SUPPLIERS; i++) {
    JsonObject o = sup.add<JsonObject>();
    o["name"]    = suppliers[i].name;
    o["phone"]   = suppliers[i].phone;
    o["email"]   = suppliers[i].email;
    o["web"]     = suppliers[i].web;
    o["account"] = suppliers[i].account;
    o["address"] = suppliers[i].address;
  }
  File f = LittleFS.open(CONFIG_FILE, "w");
  if (!f) { logf("Could not write config"); return false; }
  serializeJson(doc, f);
  f.close();
  logf("Config saved");
  return true;
}

// ---------------------------------------------------------------- sensor
void processFrame(const uint8_t *f) {
  uint16_t lvl = (uint16_t(f[1]) << 8) | f[2];
  int16_t  tmp = (int16_t)((uint16_t(f[3]) << 8) | f[4]);
  uint16_t sig = (uint16_t(f[5]) << 8) | f[6];
  uint16_t ang = (uint16_t(f[7]) << 8) | f[8];

  last.raw      = lvl;
  last.liquid   = lvl < 0xFF00;          // 0xFFFx = no liquid / no echo
  last.levelMm  = last.liquid ? max((int32_t)0, (int32_t)lvl + cfg.levelOffsetMm) : 0;
  last.tempC    = tmp / 10.0f;
  last.signalMv = sig;
  last.angleDeg = ang / 10.0f;
  last.atMs     = millis();
  if (!last.valid) logf("First sensor frame: level %u mm, signal %u mV", lvl, sig);
  last.valid    = true;
  goodFrames++;

  if (last.liquid) { accLevel += last.levelMm; accCount++; }
}

void pollSensor() {
  while (SENSOR.available()) {
    uint8_t b = SENSOR.read();
    if (framePos == 0 && b != 0xFF) continue;      // wait for start byte
    frameBuf[framePos++] = b;
    if (framePos < sizeof(frameBuf)) continue;

    uint8_t sum = 0;
    for (uint8_t i = 0; i < 9; i++) sum += frameBuf[i];
    if (sum == frameBuf[9]) {
      processFrame(frameBuf);
      framePos = 0;
    } else {
      // bad checksum: resync on the next 0xFF inside the buffer
      badFrames++;
      uint8_t k = 1;
      while (k < sizeof(frameBuf) && frameBuf[k] != 0xFF) k++;
      if (k < sizeof(frameBuf)) {
        memmove(frameBuf, frameBuf + k, sizeof(frameBuf) - k);
        framePos = sizeof(frameBuf) - k;
      } else {
        framePos = 0;
      }
    }
  }
}

// ---------------------------------------------------------------- MQTT
void publishDiscovery() {
  struct Item { const char *key, *name, *unit, *dclass, *tmpl; };
  const Item items[] = {
    {"level",  "Liquid level", "mm", "distance",    "{{ value_json.level_mm }}"},
    {"fill",   "Fill level",   "%",  nullptr,       "{{ value_json.fill_pct }}"},
    {"temp",   "Temperature",  "°C", "temperature", "{{ value_json.temp_c }}"},
    {"signal", "Echo signal",  "mV", "voltage",     "{{ value_json.signal_mv }}"},
    {"angle",  "Tilt angle",   "°",  nullptr,       "{{ value_json.angle_deg }}"},
  };
  String node = "lpg_" + chipId;
  for (const Item &it : items) {
    if (!strcmp(it.key, "fill") && cfg.fullLevelMm == 0) continue;
    JsonDocument d;
    d["name"]    = it.name;
    d["uniq_id"] = node + "_" + it.key;
    d["stat_t"]  = topic("state");
    d["val_tpl"] = it.tmpl;
    d["avty_t"]  = topic("status");
    d["stat_cla"] = "measurement";
    if (it.unit)   d["unit_of_meas"] = it.unit;
    if (it.dclass) d["dev_cla"] = it.dclass;
    JsonObject dev = d["dev"].to<JsonObject>();
    dev["ids"].to<JsonArray>().add(node);
    dev["name"] = tankLabel();
    dev["mf"]   = "DYP";
    dev["mdl"]  = "L062MUW";
    dev["sw"]   = FW_VERSION;
    String out;
    serializeJson(d, out);
    String t = "homeassistant/sensor/" + node + "/" + it.key + "/config";
    mqtt.publish(t.c_str(), out.c_str(), true);
  }
  {
    JsonDocument d;
    d["name"]     = "Low gas level";
    d["uniq_id"]  = node + "_low";
    d["stat_t"]   = topic("alert");
    d["pl_on"]    = "low";
    d["pl_off"]   = "ok";
    d["dev_cla"]  = "problem";
    d["avty_t"]   = topic("status");
    JsonObject dev = d["dev"].to<JsonObject>();
    dev["ids"].to<JsonArray>().add(node);
    String out;
    serializeJson(d, out);
    mqtt.publish(("homeassistant/binary_sensor/" + node + "/low/config").c_str(), out.c_str(), true);
  }
  logf("Home Assistant discovery published");
}

void onMqttMessage(char *t, byte *payload, unsigned int len) {
  String msg;
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  msg.trim();
  msg.toLowerCase();
  logf("MQTT command: %s", msg.c_str());
  if (msg == "publish")                           publishNow = true;
  else if (msg == "restart" || msg == "reboot")   restartAt = millis() + 500;
}

void setupMqtt() {
  if (cfg.mqttHost.isEmpty()) return;
  if (cfg.mqttTls) {
    if (cfg.mqttFingerprint.length()) secureClient.setFingerprint(cfg.mqttFingerprint.c_str());
    else                              secureClient.setInsecure();   // encrypts, no cert check
    mqtt.setClient(secureClient);
  } else {
    mqtt.setClient(plainClient);
  }
  mqtt.setServer(cfg.mqttHost.c_str(), cfg.mqttPort);
  mqtt.setBufferSize(1536);   // room for the info message with 3 suppliers
  mqtt.setKeepAlive(30);
  mqtt.setSocketTimeout(10);
  mqtt.setCallback(onMqttMessage);
}

void mqttLoop() {
  if (cfg.mqttHost.isEmpty() || WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) { mqtt.loop(); return; }
  if (lastMqttAttempt && millis() - lastMqttAttempt < MQTT_RETRY_MS) return;
  lastMqttAttempt = millis();

  if (cfg.mqttTls && !tlsProbed) {
    // Use small TLS buffers if the broker supports it (saves ~20 KB RAM)
    if (secureClient.probeMaxFragmentLength(cfg.mqttHost.c_str(), cfg.mqttPort, 1024)) {
      secureClient.setBufferSizes(1024, 1024);
      logf("TLS: broker supports MFLN, using 1 KB buffers");
    }
    tlsProbed = true;
  }

  String cid  = cfg.mqttClientId.length() ? cfg.mqttClientId : cfg.deviceName + "-" + chipId;
  String will = topic("status");
  const char *user = cfg.mqttUser.length() ? cfg.mqttUser.c_str() : nullptr;
  const char *pass = cfg.mqttUser.length() ? cfg.mqttPass.c_str() : nullptr;

  logf("MQTT connecting to %s:%u as %s", cfg.mqttHost.c_str(), cfg.mqttPort, cid.c_str());
  if (mqtt.connect(cid.c_str(), user, pass, will.c_str(), 1, true, "offline")) {
    logf("MQTT connected");
    mqtt.publish(will.c_str(), "online", true);
    mqtt.subscribe(topic("cmd").c_str());
    if (cfg.haDiscovery) publishDiscovery();
    publishInfo();
    if (alertKnown) mqtt.publish(topic("alert").c_str(), lowAlert ? "low" : "ok", true);
  } else {
    logf("MQTT failed: %s", mqttStateText());
  }
}

void buildSensorJson(JsonObject o, int32_t levelMm) {
  o["status"]    = sensorStatus();
  o["level_mm"]  = levelMm;
  o["level_cm"]  = round1(levelMm / 10.0f);
  o["fill_pct"]  = fillPct(levelMm);
  o["temp_c"]    = round1(last.tempC);
  o["signal_mv"] = last.signalMv;
  o["angle_deg"] = round1(last.angleDeg);
}

// Low-level alert with hysteresis and a confirmation delay, so a level
// hovering around the threshold doesn't flood the inbox.
// Retained <base>/info: tank name and supplier contacts, for Node-RED emails etc.
void publishInfo() {
  JsonDocument doc;
  doc["tank"]          = tankLabel();
  doc["device"]        = cfg.deviceName;
  doc["low_alert_pct"] = cfg.lowAlertPct;
  if (WiFi.status() == WL_CONNECTED) doc["url"] = "http://" + WiFi.localIP().toString() + "/";
  JsonArray arr = doc["suppliers"].to<JsonArray>();
  for (const Supplier &sp : suppliers) {
    if (sp.name.isEmpty() && sp.phone.isEmpty() && sp.email.isEmpty()) continue;
    JsonObject o = arr.add<JsonObject>();
    o["name"] = sp.name; o["phone"] = sp.phone; o["email"] = sp.email;
    o["web"]  = sp.web;  o["account"] = sp.account; o["address"] = sp.address;
  }
  String out;
  serializeJson(doc, out);
  if (!mqtt.publish(topic("info").c_str(), out.c_str(), true)) logf("Info publish failed (%u bytes)", out.length());
}

void setAlert(bool low) {
  bool changed = !alertKnown || low != lowAlert;
  lowAlert   = low;
  alertKnown = true;
  if (!changed) return;
  logf("Low level alert: %s", low ? "LOW" : "ok");
  if (mqtt.connected()) mqtt.publish(topic("alert").c_str(), low ? "low" : "ok", true);
}

void evaluateAlert() {
  if (cfg.lowAlertPct == 0 || cfg.fullLevelMm == 0) { alertPendingMs = 0; setAlert(false); return; }
  const char *st = sensorStatus();
  if (!strcmp(st, "sensor_offline")) { alertPendingMs = 0; return; }   // no data: keep state
  float pct = !strcmp(st, "no_liquid") ? 0.0f : fillPct(last.levelMm); // no echo = empty
  bool want = lowAlert ? pct < cfg.lowAlertPct + ALERT_HYST_PCT : pct < cfg.lowAlertPct;
  if (alertKnown && want == lowAlert) { alertPendingMs = 0; return; }
  if (!alertPendingMs) alertPendingMs = millis();
  if (millis() - alertPendingMs < ALERT_CONFIRM_MS) return;
  alertPendingMs = 0;
  setAlert(want);
}

void publishState() {
  int32_t level = accCount ? (int32_t)(accLevel / accCount) : last.levelMm;
  JsonDocument doc;
  buildSensorJson(doc.to<JsonObject>(), level);
  doc["tank"]      = tankLabel();
  doc["samples"]   = accCount;
  doc["alert"]     = lowAlert ? "low" : "ok";
  doc["low_alert_pct"] = cfg.lowAlertPct;
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["uptime_s"]  = millis() / 1000;
  String out;
  serializeJson(doc, out);
  if (mqtt.publish(topic("state").c_str(), out.c_str(), cfg.mqttRetain)) {
    publishCount++;
    logf("Published: %s", out.c_str());
  } else {
    logf("Publish failed");
  }
  accLevel = 0;
  accCount = 0;
  lastPublishMs = millis();
  publishNow = false;
}

// ---------------------------------------------------------------- WiFi
void startAP() {
  String ssid = "LPG-Setup-" + chipId;
  WiFi.mode(cfg.wifiSsid.length() ? WIFI_AP_STA : WIFI_AP);
  WiFi.softAP(ssid.c_str(), AP_PASSWORD);
  dns.start(53, "*", WiFi.softAPIP());
  apActive  = true;
  apStartMs = millis();
  logf("Setup AP '%s' (password '%s') -> http://%s", ssid.c_str(), AP_PASSWORD,
       WiFi.softAPIP().toString().c_str());
}

void stopAP() {
  dns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  apActive = false;
  logf("Setup AP stopped");
}

void setupWifi() {
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.hostname(cfg.deviceName);
  if (cfg.wifiSsid.isEmpty()) { startAP(); return; }

  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPass.c_str());
  logf("Connecting to WiFi '%s'", cfg.wifiSsid.c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_CONNECT_MS) {
    delay(100);
    pollSensor();
  }
  if (WiFi.status() == WL_CONNECTED) {
    logf("WiFi connected, IP %s, RSSI %d dBm", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    logf("WiFi connection failed, starting setup AP (will keep retrying)");
    startAP();
  }
}

void wifiWatchdog() {
  bool connected = WiFi.status() == WL_CONNECTED;
  if (connected) {
    wifiLostSinceMs = 0;
    if (apActive && cfg.wifiSsid.length() && millis() - apStartMs > AP_AUTO_OFF_MS &&
        WiFi.softAPgetStationNum() == 0) {
      stopAP();
    }
  } else if (!apActive && cfg.wifiSsid.length()) {
    if (!wifiLostSinceMs) wifiLostSinceMs = millis();
    if (millis() - wifiLostSinceMs > WIFI_LOST_TO_AP_MS) {
      logf("WiFi lost for 60 s");
      startAP();
    }
  }
}

// ---------------------------------------------------------------- web UI
static const char CSS[] PROGMEM = R"CSS(
:root{--bg:#f4f5f7;--card:#fff;--fg:#1d2330;--mut:#6b7280;--acc:#0f766e;--bd:#e5e7eb;--bad:#b91c1c}
@media(prefers-color-scheme:dark){:root{--bg:#111418;--card:#1b2027;--fg:#e6e8eb;--mut:#9aa3ad;--acc:#2dd4bf;--bd:#2b323c;--bad:#f87171}}
*{box-sizing:border-box}body{margin:0;font:15px/1.45 system-ui,-apple-system,Segoe UI,Roboto,sans-serif;background:var(--bg);color:var(--fg)}
header{display:flex;flex-wrap:wrap;gap:12px;align-items:center;padding:12px 16px;background:var(--card);border-bottom:1px solid var(--bd)}
header b{margin-right:auto}.logo{width:36px;height:36px;flex:none}nav a{color:var(--acc);text-decoration:none;margin-left:14px;font-weight:600}
main{max-width:760px;margin:0 auto;padding:16px}.card{background:var(--card);border:1px solid var(--bd);border-radius:10px;padding:16px;margin-bottom:16px}
h2{margin:0 0 12px;font-size:16px}table{width:100%;border-collapse:collapse}td{padding:5px 0;border-bottom:1px solid var(--bd)}td:first-child{color:var(--mut);width:45%}
.mut{color:var(--mut)}
.tankrow{display:flex;flex-wrap:wrap;gap:20px 28px;align-items:center}.tankrow table{flex:1;min-width:240px}
.tank{width:170px;height:auto;display:block;margin:0 auto}.tank .shell{fill:var(--bg);stroke:var(--mut);stroke-width:3}
.tank .liq{fill:var(--liq)}.tank .surf{fill:var(--liq2)}.tank .tick{stroke:var(--mut);stroke-width:1.5;opacity:.6}
.tank #t_lvl{transition:transform 1s ease}.tank.off .liq,.tank.off .surf{fill:var(--mut);opacity:.35}
.tank text{text-anchor:middle;fill:var(--fg);font-weight:700}.tank .badge{fill:var(--card);opacity:.92;stroke:var(--bd)}
.tank .tsub{font-size:13px;font-weight:600;fill:var(--mut)}
:root{--liq:var(--acc);--liq2:#14b8a6}@media(prefers-color-scheme:dark){:root{--liq2:#99f6e4}}
label{display:block;margin:10px 0 4px;font-weight:600}input[type=text],input[type=password],input[type=number],input[type=tel],input[type=email]{width:100%;padding:8px;border:1px solid var(--bd);border-radius:6px;background:var(--bg);color:var(--fg);font:inherit}
.chk{display:flex;gap:8px;align-items:center;font-weight:400}.hint{font-size:13px;color:var(--mut);margin-top:3px}
button{background:var(--acc);color:#fff;border:0;border-radius:6px;padding:9px 16px;font:inherit;font-weight:600;cursor:pointer;margin-top:12px}
button.sec{background:transparent;color:var(--acc);border:1px solid var(--acc)}pre{white-space:pre-wrap;font-size:12px;margin:0;max-height:300px;overflow:auto}
.ok{color:var(--acc)}.bad{color:var(--bad)}
.tank .outline{fill:none;stroke:var(--mut);stroke-width:3}.tank.low .outline{stroke:var(--bad)}
.tank .thr{stroke:var(--bad);stroke-width:2;stroke-dasharray:6 4}
.sup{padding:12px 0;border-top:1px solid var(--bd)}.sup:first-child{border-top:0;padding-top:0}.sup>span{display:block;font-size:12px;text-transform:uppercase;letter-spacing:.05em}
.acts{display:flex;flex-wrap:wrap;gap:8px;margin-top:8px}.btn{background:var(--acc);color:#fff;text-decoration:none;border-radius:6px;padding:8px 14px;font-weight:600}
.btn+.btn{background:transparent;color:var(--acc);border:1px solid var(--acc);padding:7px 13px}
details{border:1px solid var(--bd);border-radius:8px;padding:8px 12px;margin-top:10px}summary{cursor:pointer;font-weight:600}
textarea{width:100%;padding:8px;border:1px solid var(--bd);border-radius:6px;background:var(--bg);color:var(--fg);font:inherit;resize:vertical}
.alert{background:#b91c1c;color:#fff;border-radius:10px;padding:12px 16px;margin-bottom:16px}
)CSS";

String pageHead(const char *title) {
  String s;
  s.reserve(3000);
  s += F("<!doctype html><html><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'><title>");
  s += title;
  s += F("</title><style>");
  s += FPSTR(CSS);
  s += F("</style></head><body><header>"
         "<svg class=logo viewBox='0 0 40 40' role=img aria-label='DJF'>"
         "<circle cx=20 cy=20 r=20 fill='#14253e'/>"
         "<circle cx=20 cy=20 r=15.5 fill=none stroke='#c9d1db' stroke-width=1.15 />"
         "<text x=20 y=24.2 text-anchor=middle fill='#e6ebf0' font-size=11 font-weight=700 "
         "font-family='Helvetica Neue,Arial,sans-serif' letter-spacing=.3>DJF</text></svg><b>");
  s += esc(tankLabel());
  s += F("</b><nav><a href='/'>Status</a><a href='/info'>Info</a><a href='/config'>Settings</a><a href='/update'>Firmware</a></nav></header><main>");
  return s;
}

static const char STATUS_BODY[] PROGMEM = R"HTML(
<div class=alert id=s_alert hidden><b>Gas level is low</b><span id=s_alerttxt></span></div>
<section class=card><h2 id=s_tank>Tank</h2>
<div class=tankrow><div>
<svg class=tank id=t_svg viewBox="0 0 160 262" role=img aria-label="Tank fill level">
<defs><clipPath id=t_clip><path d="M20 78A60 46 0 0 1 140 78V200A60 40 0 0 1 20 200Z"/></clipPath></defs>
<rect class=shell x=56 y=6 width=48 height=26 rx=5 />
<rect class=shell x=42 y=232 width=76 height=24 rx=4 />
<path class=shell d="M20 78A60 46 0 0 1 140 78V200A60 40 0 0 1 20 200Z"/>
<g clip-path="url(#t_clip)"><g id=t_lvl style="transform:translateY(210px)">
<rect class=surf x=0 y=30 width=160 height=5 /><rect class=liq x=0 y=34 width=160 height=210 /></g>
<rect x=30 y=40 width=10 height=190 rx=5 fill="#fff" opacity=.12 />
<line id=t_thr class=thr x1=0 x2=160 y1=0 y2=0 visibility=hidden /></g>
<line class=tick x1=124 x2=140 y1=187.5 y2=187.5 /><line class=tick x1=118 x2=140 y1=135 y2=135 /><line class=tick x1=124 x2=140 y1=82.5 y2=82.5 />
<path class=outline d="M20 78A60 46 0 0 1 140 78V200A60 40 0 0 1 20 200Z"/>
<rect class=badge x=34 y=106 width=92 height=68 rx=12 />
<text id=t_pct x=80 y=144 font-size=34>&ndash;</text>
<text id=t_sub class=tsub x=80 y=164></text>
</svg><div class=mut id=s_fill style="text-align:center;font-size:13px;max-width:200px;margin:6px auto 0"></div></div>
<table>
<tr><td>Gas level</td><td id=s_level></td></tr>
<tr><td>Gas temperature</td><td id=s_temp></td></tr>
<tr><td>Low level warning</td><td id=s_warn></td></tr>
<tr><td>Sensor</td><td id=s_status></td></tr>
</table></div></section>
<section class=card id=s_order hidden><h2>Order gas</h2><div id=s_sup></div></section>
)HTML";

static const char INFO_BODY[] PROGMEM = R"HTML(
<section class=card><h2>Sensor details</h2><table>
<tr><td>Status</td><td id=s_status></td></tr>
<tr><td>Liquid level</td><td id=s_level></td></tr>
<tr><td>Echo signal</td><td id=s_signal></td></tr>
<tr><td>Tilt angle</td><td id=s_angle></td></tr>
<tr><td>Last frame</td><td id=s_age></td></tr>
<tr><td>Frames good / bad</td><td id=s_frames></td></tr>
</table></section>
<section class=card><h2>Connection</h2><table>
<tr><td>WiFi</td><td id=s_wifi></td></tr>
<tr><td>IP address</td><td id=s_ip></td></tr>
<tr><td>MQTT</td><td id=s_mqtt></td></tr>
<tr><td>Publishes</td><td id=s_pub></td></tr>
<tr><td>Low level alert</td><td id=s_warn></td></tr>
<tr><td>Uptime</td><td id=s_up></td></tr>
<tr><td>Free memory</td><td id=s_heap></td></tr>
<tr><td>Firmware</td><td id=s_ver></td></tr>
</table>
<button onclick="post('/api/publish')">Publish now</button>
<button class=sec onclick="if(confirm('Restart device?'))post('/api/restart')">Restart</button></section>
<section class=card><h2>Log</h2><pre id=s_log></pre></section>
)HTML";

// Shared by Status and Info: fills in whichever elements the page has
static const char PAGE_JS[] PROGMEM = R"HTML(
<script>
const $=i=>document.getElementById(i);
const set=(i,v,h)=>{const e=$(i);if(e)e[h?'innerHTML':'textContent']=v};
const n=(v,d,u)=>v==null?'–':(+v).toFixed(d)+(u||'');
function post(u){fetch(u,{method:'POST'}).then(r=>{if(!r.ok)alert('Failed: '+r.status)})}
function dur(s){const d=Math.floor(s/86400),h=Math.floor(s%86400/3600),m=Math.floor(s%3600/60);return (d?d+'d ':'')+h+'h '+m+'m'}
const FRIENDLY={ok:'Working normally',no_liquid:'No level detected',sensor_offline:'No signal from sensor'};
async function tick(){try{
 const j=await (await fetch('/api/status')).json(),s=j.sensor,a=j.alert;
 const ok=s.status=='ok',pc=s.fill_pct,hasP=ok&&pc!=null,info=!!$('s_log');
 set('s_level',ok?s.level_mm+' mm':'–');
 set('s_temp',n(s.temp_c,1,' °C'));
 set('s_status','<span class='+(ok?'ok':'bad')+'>'+(info?s.status.replace('_',' '):FRIENDLY[s.status]||s.status)+'</span>',1);
 set('s_warn',a.threshold?(a.low?'<span class=bad>Low – below '+a.threshold+'%</span>':'Alerts below '+a.threshold+'%'):'<span class=mut>Off</span>',1);
 if($('t_svg')){
  $('t_svg').classList.toggle('off',!ok);$('t_svg').classList.toggle('low',a.low);
  $('t_lvl').style.transform='translateY('+(210*(1-(hasP?Math.min(100,Math.max(0,pc))/100:0)))+'px)';
  set('t_pct',hasP?Math.round(pc)+'%':ok?s.level_mm:'–');
  set('t_sub',hasP?s.level_mm+' mm':ok?'mm':s.status=='no_liquid'?'no level':'offline');
  set('s_fill',ok&&pc==null?'Set "Full level" in Settings to show %':'');
  const y=240-2.1*a.threshold,t=$('t_thr');t.setAttribute('y1',y);t.setAttribute('y2',y);
  t.setAttribute('visibility',a.threshold?'visible':'hidden');
  $('s_alert').hidden=!a.low;set('s_alerttxt',' – below '+a.threshold+'%. Time to arrange a refill.');
 }
 set('s_signal',s.signal_mv+' mV'+(s.signal_mv&&s.signal_mv<300?' (weak)':''));
 set('s_angle',n(s.angle_deg,1,'°'));
 set('s_age',s.age_ms==null?'never':(s.age_ms/1000).toFixed(1)+' s ago');
 set('s_frames',s.good_frames+' / '+s.bad_frames);
 const w=j.wifi;set('s_wifi',w.connected?w.ssid+' ('+w.rssi+' dBm)':'not connected'+(w.ap?' – setup AP active':''));
 set('s_ip',(w.ip||'')+(w.ap?'  AP: '+w.ap_ip:''));
 set('s_mqtt','<span class='+(j.mqtt.connected?'ok':'bad')+'>'+j.mqtt.state+'</span>',1);
 set('s_pub',j.mqtt.count+(j.mqtt.last_s!=null?', last '+j.mqtt.last_s+' s ago':''));
 set('s_up',dur(j.uptime_s));set('s_heap',j.heap+' bytes');set('s_ver',j.version);
}catch(e){}}
async function lg(){if(!$('s_log'))return;try{set('s_log',await (await fetch('/api/log')).text())}catch(e){}}
let lastPct=null;
function sup(){const c=$('s_sup');if(!c||typeof SUP=='undefined')return;
 $('s_tank').textContent=TANK;const list=SUP.filter(x=>x.name||x.phone||x.email||x.web);
 $('s_order').hidden=!list.length;if(!list.length)return;
 const e=t=>(t||'').replace(/[&<>"']/g,m=>'&#'+m.charCodeAt(0)+';');
 c.innerHTML=list.map((x,i)=>{
  const body='Hello,\n\nPlease could I order an LPG refill'+(x.account?' for account '+x.account:'')+'.\n\nTank: '+TANK+(lastPct!=null?'\nCurrent level: '+Math.round(lastPct)+'%':'')+'\n\nThank you';
  const web=x.web&&!/^https?:/i.test(x.web)?'https://'+x.web:x.web;
  return '<div class=sup>'+(list.length>1?'<span class=mut>'+(i?'Alternative':'Preferred')+'</span>':'')+
  '<b>'+e(x.name||'Supplier')+'</b>'+(x.account?'<div class=mut>Account '+e(x.account)+'</div>':'')+
  (x.address?'<div class=mut style="white-space:pre-line">'+e(x.address)+'</div>':'')+'<div class=acts>'+
  (x.phone?'<a class=btn href="tel:'+e(x.phone.replace(/[^+0-9]/g,''))+'">Call '+e(x.phone)+'</a>':'')+
  (x.email?'<a class=btn href="mailto:'+e(x.email)+'?subject='+encodeURIComponent('LPG refill order \u2013 '+TANK)+'&body='+encodeURIComponent(body)+'">Email</a>':'')+
  (web?'<a class=btn href="'+e(web)+'" target=_blank rel=noopener>Website</a>':'')+'</div></div>'}).join('')}
const _t=tick;tick=async function(){await _t();const p=document.getElementById('t_pct');if(p&&/%$/.test(p.textContent))lastPct=parseFloat(p.textContent);sup()};
tick();lg();setInterval(tick,2000);setInterval(lg,5000);
</script>
)HTML";

bool checkAuth() {
  if (cfg.webPass.isEmpty()) return true;
  if (server.authenticate(WEB_USER, cfg.webPass.c_str())) return true;
  server.requestAuthentication(BASIC_AUTH, cfg.deviceName.c_str());
  return false;
}

void sendPage(const char *title, PGM_P body) {
  String s = pageHead(title);
  if (body == STATUS_BODY) {
    JsonDocument d;
    d["t"] = tankLabel();
    JsonArray a = d["s"].to<JsonArray>();
    for (const Supplier &sp : suppliers) {
      JsonObject o = a.add<JsonObject>();
      o["name"] = sp.name; o["phone"] = sp.phone; o["email"] = sp.email;
      o["web"]  = sp.web;  o["account"] = sp.account; o["address"] = sp.address;
    }
    String j;
    serializeJson(d, j);
    j.replace("<", "\\u003c");                 // can't close the <script> tag
    s += F("<script>const D=");
    s += j;
    s += F(",TANK=D.t,SUP=D.s;</script>");
  }
  s += FPSTR(body);
  s += FPSTR(PAGE_JS);
  s += F("</main></body></html>");
  server.send(200, "text/html", s);
}

void handleRoot() { sendPage("LPG level", STATUS_BODY); }
void handleInfo() { sendPage("Info", INFO_BODY); }

void handleStatus() {
  JsonDocument doc;
  JsonObject s = doc["sensor"].to<JsonObject>();
  buildSensorJson(s, last.levelMm);
  s["raw"]         = last.raw;
  s["good_frames"] = goodFrames;
  s["bad_frames"]  = badFrames;
  if (last.valid) s["age_ms"] = millis() - last.atMs;

  JsonObject w = doc["wifi"].to<JsonObject>();
  bool wc = WiFi.status() == WL_CONNECTED;
  w["connected"] = wc;
  w["ssid"]      = cfg.wifiSsid;
  if (wc) { w["rssi"] = WiFi.RSSI(); w["ip"] = WiFi.localIP().toString(); }
  w["ap"] = apActive;
  if (apActive) w["ap_ip"] = WiFi.softAPIP().toString();

  JsonObject m = doc["mqtt"].to<JsonObject>();
  m["connected"] = mqtt.connected();
  m["state"]     = mqttStateText();
  m["count"]     = publishCount;
  if (publishCount) m["last_s"] = (millis() - lastPublishMs) / 1000;

  JsonObject a = doc["alert"].to<JsonObject>();
  a["low"]       = lowAlert;
  a["threshold"] = (cfg.fullLevelMm && cfg.lowAlertPct) ? cfg.lowAlertPct : 0;
  doc["uptime_s"] = millis() / 1000;
  doc["heap"]     = ESP.getFreeHeap();
  doc["version"]  = FW_VERSION;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleLog() {
  String out;
  for (uint8_t i = 0; i < logCount; i++) {
    out += logBuf[(logHead + LOG_LINES - logCount + i) % LOG_LINES];
    out += '\n';
  }
  server.send(200, "text/plain", out);
}

void handleScan() {
  if (!checkAuth()) return;
  int n = WiFi.scanNetworks();
  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i).isEmpty()) continue;
    JsonObject o = arr.add<JsonObject>();
    o["ssid"] = WiFi.SSID(i);
    o["rssi"] = WiFi.RSSI(i);
  }
  WiFi.scanDelete();
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void field(String &s, const char *label, const char *name, const String &value,
           const char *type = "text", const char *hint = nullptr, const char *extra = "") {
  s += F("<label for='"); s += name; s += F("'>"); s += label; s += F("</label><input type='");
  s += type; s += F("' id='"); s += name; s += F("' name='"); s += name; s += F("' value='");
  s += esc(value); s += F("' "); s += extra; s += F(">");
  if (hint) { s += F("<div class=hint>"); s += hint; s += F("</div>"); }
}

void secretField(String &s, const char *label, const char *name, const String &current) {
  s += F("<label for='"); s += name; s += F("'>"); s += label; s += F("</label><input type='password' id='");
  s += name; s += F("' name='"); s += name; s += F("' autocomplete='new-password' placeholder='");
  s += current.length() ? F("(saved - leave blank to keep, '-' to clear)") : F("(not set)");
  s += F("'>");
}

void checkbox(String &s, const char *label, const char *name, bool on) {
  s += F("<label class=chk><input type=checkbox name='"); s += name; s += F("'");
  if (on) s += F(" checked");
  s += F("> "); s += label; s += F("</label>");
}

void handleConfigGet() {
  if (!checkAuth()) return;
  // Sent in chunks so the page never has to fit in RAM all at once
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  String s = pageHead("Settings");
  s += F("<form method=post action='/config'>");

  s += F("<section class=card><h2>Tank</h2>");
  field(s, "Tank name", "tankName", cfg.tankName, "text",
        "Shown at the top of every page and in alerts, e.g. <i>House tank</i> or <i>Barn tank</i>. "
        "Useful when you have more than one.", "maxlength=40");
  field(s, "Full level (mm)", "fullLevelMm", String(cfg.fullLevelMm), "number",
        "Liquid height that counts as 100 %. 0 = don't calculate %.", "min=0 max=1200");
  field(s, "Low level alert (%)", "lowAlertPct", String(cfg.lowAlertPct), "number",
        "Warn when the tank drops below this fill level (shown on the Status page and sent to "
        "&lt;base&gt;/alert as low / ok). 0 = off. Needs Full level to be set.", "min=0 max=95");
  field(s, "Level offset (mm)", "levelOffsetMm", String(cfg.levelOffsetMm), "number",
        "Added to every reading to calibrate against a known level.", "min=-500 max=500");
  s += F("</section>");
  server.sendContent(s); s = "";

  s += F("<section class=card><h2>Gas suppliers</h2><div class=hint>Shown on the Status page with "
         "call / email / website buttons. The first one is shown as preferred. Leave a supplier blank to hide it.</div>");
  for (uint8_t i = 0; i < MAX_SUPPLIERS; i++) {
    const Supplier &sp = suppliers[i];
    String k = "s" + String(i) + "_";
    s += F("<details");
    if (i == 0 || sp.name.length()) s += F(" open");
    s += F("><summary>");
    s += i == 0 ? F("Preferred supplier") : (i == 1 ? F("Alternative supplier 1") : F("Alternative supplier 2"));
    if (sp.name.length()) { s += F(" &ndash; "); s += esc(sp.name); }
    s += F("</summary>");
    field(s, "Company name", (k + "name").c_str(), sp.name, "text", nullptr, "maxlength=50");
    field(s, "Phone", (k + "phone").c_str(), sp.phone, "tel", nullptr, "maxlength=30");
    field(s, "Email", (k + "email").c_str(), sp.email, "email", nullptr, "maxlength=60");
    field(s, "Website", (k + "web").c_str(), sp.web, "text", nullptr, "maxlength=80 placeholder='www.example.co.uk'");
    field(s, "Your account / customer number", (k + "account").c_str(), sp.account, "text", nullptr, "maxlength=30");
    s += F("<label for='"); s += k; s += F("address'>Address</label><textarea rows=3 maxlength=150 id='");
    s += k; s += F("address' name='"); s += k; s += F("address'>"); s += esc(sp.address); s += F("</textarea>");
    s += F("</details>");
    server.sendContent(s); s = "";
  }
  s += F("</section>");

  s += F("<section class=card><h2>WiFi</h2>");
  field(s, "Network name (SSID)", "wifiSsid", cfg.wifiSsid, "text", nullptr, "list='nets' autocomplete='off'");
  s += F("<datalist id=nets></datalist><button type=button class=sec onclick='scan(this)'>Scan for networks</button>");
  secretField(s, "WiFi password", "wifiPass", cfg.wifiPass);
  field(s, "Device name", "deviceName", cfg.deviceName, "text",
        "Used as hostname (http://name.local) and in MQTT client ID. Letters, digits and - only.");
  s += F("</section>");

  s += F("<section class=card><h2>MQTT</h2>");
  field(s, "Broker host", "mqttHost", cfg.mqttHost, "text", "IP address or hostname. Leave blank to disable MQTT.");
  field(s, "Port", "mqttPort", String(cfg.mqttPort), "number", "1883 plain, 8883 TLS", "min=1 max=65535");
  field(s, "Username", "mqttUser", cfg.mqttUser);
  secretField(s, "Password", "mqttPass", cfg.mqttPass);
  field(s, "Client ID", "mqttClientId", cfg.mqttClientId, "text", "Leave blank for automatic.");
  field(s, "Base topic", "mqttTopic", cfg.mqttTopic, "text",
        "Publishes &lt;base&gt;/state (JSON) and &lt;base&gt;/status (online/offline). Listens on &lt;base&gt;/cmd.");
  field(s, "Publish interval (seconds)", "publishSec", String(cfg.publishSec), "number",
        "Readings between publishes are averaged.", "min=2 max=86400");
  checkbox(s, "Use TLS (encrypted connection)", "mqttTls", cfg.mqttTls);
  field(s, "TLS SHA1 fingerprint (optional)", "mqttFingerprint", cfg.mqttFingerprint, "text",
        "e.g. AB:CD:... If blank the connection is encrypted but the server certificate is not verified.");
  checkbox(s, "Retain state messages", "mqttRetain", cfg.mqttRetain);
  checkbox(s, "Home Assistant auto-discovery", "haDiscovery", cfg.haDiscovery);
  s += F("</section>");
  server.sendContent(s); s = "";

  s += F("<section class=card><h2>Web interface</h2>");
  secretField(s, "Admin password", "webPass", cfg.webPass);
  s += F("<div class=hint>Protects Settings, Firmware and actions. Username is <b>admin</b>.</div></section>");

  s += F("<button type=submit>Save and restart</button></form>"
         "<script>async function scan(b){b.textContent='Scanning...';try{const l=await (await fetch('/api/scan')).json();"
         "const d=document.getElementById('nets');d.innerHTML='';l.sort((a,b)=>b.rssi-a.rssi).forEach(n=>{const o=document.createElement('option');"
         "o.value=n.ssid;o.label=n.rssi+' dBm';d.appendChild(o)});b.textContent=l.length+' networks found - click the SSID box'}"
         "catch(e){b.textContent='Scan failed'}}</script></main></body></html>");
  server.sendContent(s);
  server.sendContent("");    // end of chunked response
}

void updateSecret(String &target, const char *arg) {
  if (!server.hasArg(arg)) return;
  String v = server.arg(arg);
  if (v == "-") target = "";
  else if (v.length()) target = v;
}

void handleConfigPost() {
  if (!checkAuth()) return;
  cfg.wifiSsid = server.arg("wifiSsid");  cfg.wifiSsid.trim();
  updateSecret(cfg.wifiPass, "wifiPass");
  String name = server.arg("deviceName");
  name.trim();
  name.replace(" ", "-");
  if (name.length()) cfg.deviceName = name;

  cfg.mqttHost = server.arg("mqttHost");  cfg.mqttHost.trim();
  long port = server.arg("mqttPort").toInt();
  cfg.mqttPort = (port >= 1 && port <= 65535) ? port : 1883;
  cfg.mqttUser = server.arg("mqttUser");  cfg.mqttUser.trim();
  updateSecret(cfg.mqttPass, "mqttPass");
  cfg.mqttClientId = server.arg("mqttClientId");  cfg.mqttClientId.trim();
  String t = server.arg("mqttTopic");
  t.trim();
  while (t.endsWith("/")) t.remove(t.length() - 1);
  if (t.length()) cfg.mqttTopic = t;
  cfg.publishSec = constrain(server.arg("publishSec").toInt(), 2L, 86400L);
  cfg.mqttTls = server.hasArg("mqttTls");
  cfg.mqttFingerprint = server.arg("mqttFingerprint");  cfg.mqttFingerprint.trim();
  cfg.mqttRetain  = server.hasArg("mqttRetain");
  cfg.haDiscovery = server.hasArg("haDiscovery");

  cfg.fullLevelMm   = constrain(server.arg("fullLevelMm").toInt(), 0L, 1200L);
  cfg.tankName = server.arg("tankName");  cfg.tankName.trim();
  for (uint8_t i = 0; i < MAX_SUPPLIERS; i++) {
    String k = "s" + String(i) + "_";
    auto get = [&](const char *f, uint16_t maxLen) {
      String v = server.arg(k + f); v.trim();
      if (v.length() > maxLen) v.remove(maxLen);
      return v;
    };
    suppliers[i].name    = get("name", 50);
    suppliers[i].phone   = get("phone", 30);
    suppliers[i].email   = get("email", 60);
    suppliers[i].web     = get("web", 80);
    suppliers[i].account = get("account", 30);
    suppliers[i].address = get("address", 150);
    suppliers[i].address.replace("\r", "");
  }
  cfg.lowAlertPct   = constrain(server.arg("lowAlertPct").toInt(), 0L, 95L);
  cfg.levelOffsetMm = constrain(server.arg("levelOffsetMm").toInt(), -500L, 500L);
  updateSecret(cfg.webPass, "webPass");

  bool ok = saveConfig();
  String s = pageHead("Saved");
  s += ok ? F("<section class=card><h2>Saved</h2><p>Restarting now. If you changed WiFi, reconnect to that network and open <b>http://")
          : F("<section class=card><h2 class=bad>Could not save settings</h2><p>Restarting anyway. Try <b>http://");
  s += esc(cfg.deviceName);
  s += F(".local</b> or the IP shown by your router.</p></section><script>setTimeout(()=>location='/',15000)</script></main></body></html>");
  server.send(200, "text/html", s);
  restartAt = millis() + 1500;
}

void handleNotFound() {
  // Captive portal: send any unknown host to the setup page while the AP is up
  if (apActive) {
    IPAddress ip;
    String host = server.hostHeader();
    int colon = host.indexOf(':');
    if (colon >= 0) host = host.substring(0, colon);
    if (!ip.fromString(host)) {
      server.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/config", true);
      server.send(302, "text/plain", "");
      return;
    }
  }
  server.send(404, "text/plain", "Not found");
}

void setupWeb() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/info", HTTP_GET, handleInfo);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/log", HTTP_GET, handleLog);
  server.on("/api/scan", HTTP_GET, handleScan);
  server.on("/config", HTTP_GET, handleConfigGet);
  server.on("/config", HTTP_POST, handleConfigPost);
  server.on("/api/publish", HTTP_POST, []() {
    if (!checkAuth()) return;
    publishNow = true;
    server.send(200, "text/plain", mqtt.connected() ? "ok" : "queued (MQTT not connected)");
  });
  server.on("/api/restart", HTTP_POST, []() {
    if (!checkAuth()) return;
    server.send(200, "text/plain", "restarting");
    restartAt = millis() + 500;
  });
  server.onNotFound(handleNotFound);

  if (cfg.webPass.length()) httpUpdater.setup(&server, "/update", WEB_USER, cfg.webPass);
  else                      httpUpdater.setup(&server, "/update");
  server.begin();
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  chipId = String(ESP.getChipId(), HEX);

#ifdef SENSOR_SOFTSERIAL
  Serial.begin(115200);
  SENSOR.begin(115200);
#else
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("LPG level monitor - sensor on D7, debug output continues on D4 (Serial1)"));
  Serial.flush();
  Serial.swap();            // UART0 now RX=GPIO13 (D7), TX=GPIO15 (D8)
  Serial1.begin(115200);    // debug, TX only on GPIO2 (D4)
#endif

  logf("LPG level monitor %s, chip %s", FW_VERSION, chipId.c_str());
  if (!LittleFS.begin()) logf("LittleFS mount failed");
  loadConfig();

  setupWifi();
  if (MDNS.begin(cfg.deviceName)) MDNS.addService("http", "tcp", 80);
  setupWeb();
  setupMqtt();
  logf("Ready: http://%s.local", cfg.deviceName.c_str());
}

void loop() {
  pollSensor();
  if (apActive) dns.processNextRequest();
  server.handleClient();
  MDNS.update();
  wifiWatchdog();
  mqttLoop();

  static uint32_t lastAlertCheck = 0;
  if (millis() - lastAlertCheck >= 1000) { lastAlertCheck = millis(); evaluateAlert(); }

  if (mqtt.connected()) {
    bool due = !lastPublishMs || millis() - lastPublishMs >= cfg.publishSec * 1000UL;
    bool sensorReady = last.valid || millis() > 15000;   // give sensor time after boot
    if ((due && sensorReady) || publishNow) publishState();
  }

  if (restartAt && (int32_t)(millis() - restartAt) >= 0) {
    if (mqtt.connected()) { mqtt.publish(topic("status").c_str(), "offline", true); mqtt.disconnect(); }
    delay(100);
    ESP.restart();
  }
}
