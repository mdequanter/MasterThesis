// WEMOS D1 mini: MQTT client that receives heading/direction data
// Required libraries:
// - ESP8266 board package
// - PubSubClient
// - ArduinoJson

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// === WiFi config ===
const char* WIFI_SSID = "AILABO_24";
const char* WIFI_PASS = "geheim123$";

// === MQTT config ===
// Change these if your EMQX broker uses a different host/credentials.
const char* MQTT_HOST = "broker.emqx.io";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "";
const char* MQTT_PASS = "";
const char* MQTT_TOPIC = "ehb/pathnavigation/heading";
const char* MQTT_CLIENT_ID = "wemos-d1-mini-heading";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// === Sliding window (1s) for smoothing ===
struct Sample {
  uint32_t t_ms;
  float v;
};

static const uint16_t MAX_SAMPLES = 100;
Sample buf[MAX_SAMPLES];
uint16_t head = 0;
uint16_t countS = 0;

void addSample(float v) {
  uint32_t now = millis();
  buf[head] = {now, v};
  head = (head + 1) % MAX_SAMPLES;
  if (countS < MAX_SAMPLES) countS++;

  uint16_t kept = 0;
  uint32_t cutoff = now - 1000;
  for (uint16_t i = 0; i < countS; i++) {
    uint16_t idx = (head + MAX_SAMPLES - 1 - i) % MAX_SAMPLES;
    if (buf[idx].t_ms >= cutoff) {
      kept++;
    } else {
      break;
    }
  }
  countS = kept;
}

bool avgDirection(float& out) {
  if (countS == 0) return false;

  uint32_t now = millis();
  uint32_t cutoff = now - 1000;
  float sum = 0.0f;
  uint16_t n = 0;

  for (uint16_t i = 0; i < countS; i++) {
    uint16_t idx = (head + MAX_SAMPLES - 1 - i) % MAX_SAMPLES;
    if (buf[idx].t_ms >= cutoff) {
      sum += buf[idx].v;
      n++;
    }
  }

  if (n == 0) return false;
  out = sum / n;
  return true;
}

bool parseNumericPayload(const String& payload, float& direction) {
  char* endPtr = nullptr;
  direction = strtof(payload.c_str(), &endPtr);
  return endPtr != payload.c_str();
}

bool extractDirectionAngle(const String& payload, float& direction) {
  if (parseNumericPayload(payload, direction)) {
    return true;
  }

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.print("Payload parse error: ");
    Serial.println(err.c_str());
    return false;
  }

  if (doc.containsKey("direction_angle")) {
    direction = doc["direction_angle"].as<float>();
    return true;
  }
  if (doc.containsKey("heading")) {
    direction = doc["heading"].as<float>();
    return true;
  }
  if (doc.containsKey("angle")) {
    direction = doc["angle"].as<float>();
    return true;
  }
  if (doc.containsKey("value")) {
    direction = doc["value"].as<float>();
    return true;
  }

  return false;
}

void handleDirection(float dir) {
  addSample(dir);

  float avg = 0.0f;
  if (!avgDirection(avg)) {
    return;
  }

  Serial.print("Direction: ");
  Serial.print(dir, 1);
  Serial.print(" | Avg(1s): ");
  Serial.println(avg, 1);
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  String message;
  message.reserve(length);
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  float dir = 0.0f;
  if (!extractDirectionAngle(message, dir)) {
    Serial.print("Unhandled payload on ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
    return;
  }

  handleDirection(dir);
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }

  Serial.printf("\nWiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
}

bool connectMQTT() {
  Serial.printf("Connecting MQTT to %s:%u\n", MQTT_HOST, MQTT_PORT);

  bool connected = false;
  if (strlen(MQTT_USER) > 0) {
    connected = mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS);
  } else {
    connected = mqttClient.connect(MQTT_CLIENT_ID);
  }

  if (!connected) {
    Serial.printf("MQTT connect failed, rc=%d\n", mqttClient.state());
    return false;
  }

  Serial.println("MQTT connected");
  if (!mqttClient.subscribe(MQTT_TOPIC)) {
    Serial.println("MQTT subscribe failed");
    return false;
  }

  Serial.print("Subscribed to ");
  Serial.println(MQTT_TOPIC);
  return true;
}

void setup() {
  Serial.begin(9600);
  delay(200);

  connectWiFi();

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(onMqttMessage);
  mqttClient.setBufferSize(512);

  connectMQTT();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  if (!mqttClient.connected()) {
    static uint32_t lastTry = 0;
    if (millis() - lastTry > 3000) {
      lastTry = millis();
      connectMQTT();
    }
    delay(5);
    return;
  }

  mqttClient.loop();
  delay(5);
}
