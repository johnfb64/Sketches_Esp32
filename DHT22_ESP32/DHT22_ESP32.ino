#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>
#include "DHT.h"

//================= CONFIG =================
#define DHTPIN   5 // pin gpio 5
#define DHTTYPE  DHT22
const char* WIFI_SSID = "your_wifi";
const char* WIFI_PASS = "your_pwd_wifi";

const char* MQTT_HOST = "192.168.1.111";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "user mqtt;
const char* MQTT_PASS = "pwd mqtt";

const char* DEVICE_ID = "esp32_dht22_1";  // identificador único

const unsigned long READ_MS   = 2000;     // periodo de lectura del DHT
const uint8_t       RETRIES   = 5;        // reintentos por lectura
const unsigned long RECHK_MS  = 5000;     // cada cuánto reviso WiFi/MQTT
const unsigned long BLINK_MS  = 500;      // parpadeo del LED
// (Opcional) IP estática — descomenta si quieres usarla
// IPAddress ip(192,168,1,54), gw(192,168,1,1), mask(255,255,255,0), dns(8,8,8,8);
//==========================================

// Topics para Home Assistant (discovery y estado)
WiFiClient espClient;
PubSubClient mqtt(espClient);
DHT dht(DHTPIN, DHTTYPE);

const String base = "homeassistant";
const String dev  = DEVICE_ID;
const String avail_topic = "home/" + dev + "/status";    // LWT/availability
const String state_topic = "home/" + dev + "/state";     // JSON con T/H
const String cfg_temp = base + "/sensor/" + dev + "/temperature/config";
const String cfg_hum  = base + "/sensor/" + dev + "/humidity/config";

float lastT = NAN, lastH = NAN;
unsigned long lastRead = 0, lastChk = 0, lastBlink = 0;
bool ledState = false;

void logln(const String& m){ Serial.println(m); }

//---------------- WiFi ----------------
void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  // (Opcional) IP estática
  // WiFi.config(ip, gw, mask, dns);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] Conectando");
  int t=0;
  while (WiFi.status() != WL_CONNECTED && t < 80) { // ~20 s
    delay(250); Serial.print(".");
    t++;
  }
  Serial.println();
  if (WiFi.status()==WL_CONNECTED){
    Serial.print("[WiFi] OK  IP="); Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] Falló (loop reintentará).");
  }
}

//------------- MQTT / Discovery -------------
void publishDiscovery() {
  String device = String("{\"identifiers\":[\"") + dev + "\"],"
                  "\"name\":\"ESP32 DHT22\","
                  "\"model\":\"ESP32 DevKit + DHT22\","
                  "\"manufacturer\":\"JohnLab\"}";

  String pTemp = String("{") +
    "\"name\":\"ESP32 Temperature\"," +
    "\"uniq_id\":\"" + dev + "_temp\"," +
    "\"stat_t\":\"" + state_topic + "\"," +
    "\"val_tpl\":\"{{ value_json.temperature }}\"," +
    "\"dev_cla\":\"temperature\"," +
    "\"unit_of_meas\":\"°C\"," +
    "\"stat_cla\":\"measurement\"," +
    "\"avty_t\":\"" + avail_topic + "\"," +
    "\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\"," +
    "\"dev\":" + device +
  "}";

  String pHum = String("{") +
    "\"name\":\"ESP32 Humidity\"," +
    "\"uniq_id\":\"" + dev + "_hum\"," +
    "\"stat_t\":\"" + state_topic + "\"," +
    "\"val_tpl\":\"{{ value_json.humidity }}\"," +
    "\"dev_cla\":\"humidity\"," +
    "\"unit_of_meas\":\"%\"," +
    "\"stat_cla\":\"measurement\"," +
    "\"avty_t\":\"" + avail_topic + "\"," +
    "\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\"," +
    "\"dev\":" + device +
  "}";

  mqtt.publish(cfg_temp.c_str(), pTemp.c_str(), true);
  mqtt.publish(cfg_hum .c_str(), pHum .c_str(), true);
  logln("[MQTT] Discovery publicado");
}

void mqttConnect() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(512); 
  while (!mqtt.connected()) {
    Serial.print("[MQTT] Conectando a "); Serial.print(MQTT_HOST); Serial.print(":"); Serial.println(MQTT_PORT);
    // LWT: offline retenido si se cae
    if (mqtt.connect(dev.c_str(), MQTT_USER, MQTT_PASS,
                     avail_topic.c_str(), 1, true, "offline")) {
      logln("[MQTT] Conectado");
      mqtt.publish(avail_topic.c_str(), "online", true);
      publishDiscovery();
    } else {
      Serial.print("[MQTT] Falló rc="); Serial.print(mqtt.state()); Serial.println(" (reintento en 2s)");
      delay(2000);
    }
  }
}

//---------------- DHT helpers ----------------
bool readDHTOnce(float &h, float &t){
  h = dht.readHumidity();
  t = dht.readTemperature();
  return !(isnan(h) || isnan(t));
}

bool readDHTStable(float &h, float &t){
  for (uint8_t i=0;i<RETRIES;i++){
    if (readDHTOnce(h,t)) return true;
    delay(250);
  }
  return false;
}

//---------------- SETUP / LOOP ----------------
void setup() {
  pinMode(2, OUTPUT);                 // LED on-board
  digitalWrite(2, LOW);
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] ESP32 MQTT + DHT22");

  pinMode(DHTPIN, INPUT_PULLUP);      // ayuda si el módulo trae pull-up débil
  dht.begin();
  delay(1500);                        // warm-up del DHT

  wifiConnect();
  mqttConnect();
}

void loop() {
  // Heartbeat LED (toggle cada BLINK_MS)
  if (millis() - lastBlink > BLINK_MS) {
    lastBlink = millis();
    ledState = !ledState;
    digitalWrite(2, ledState ? HIGH : LOW);
  }

  // Mantener WiFi/MQTT vivos y reconectar si caen
  if (millis() - lastChk > RECHK_MS) {
    lastChk = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Reconectando...");
      wifiConnect();
    }
    if (!mqtt.connected()) {
      Serial.println("[MQTT] Reconectando...");
      mqttConnect();
    }
  }
  mqtt.loop();

  // Lectura periódica del DHT y publicación
  if (millis() - lastRead >= READ_MS) {
    lastRead = millis();
    float h,t;
    if (readDHTStable(h,t)) {
      // Publica si cambió lo suficiente (para no inundar)
      if (isnan(lastT) || isnan(lastH) || fabsf(t-lastT)>0.1f || fabsf(h-lastH)>0.3f) {
        lastT = t; lastH = h;
        String payload = String("{\"temperature\":") + String(t,1) +
                         ",\"humidity\":" + String(h,1) + "}";
        mqtt.publish(state_topic.c_str(), payload.c_str(), false);
        Serial.println(String("[PUB] ")+payload);
      } else {
        Serial.println("[PUB] sin cambios");
      }
    } else {
      Serial.println("[DHT] NaN (revisa pin/cable/pull-up, se mantiene último válido)");
    }
  }
}
