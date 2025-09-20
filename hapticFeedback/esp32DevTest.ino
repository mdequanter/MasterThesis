// ESP32-CAM: WebSocket client die `direction_angle` ontvangt en print
// Benodigd: Arduino core for ESP32, libraries: ArduinoWebsockets, ArduinoJson

#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>
using namespace websockets;

// === Config ===
const char* WIFI_SSID = "NETGEAR24";
const char* WIFI_PASS = "";
const char* WS_URL    = "ws://94.111.36.87:9000";   // jouw signaling server

// === Sliding window (1s) voor average ===
struct Sample { uint32_t t_ms; float v; };
static const uint16_t MAX_SAMPLES = 100; // ~100 Hz
Sample buf[MAX_SAMPLES];
uint16_t head = 0, countS = 0;

WebsocketsClient ws;

void addSample(float v){
  uint32_t now = millis();
  buf[head] = { now, v };
  head = (head + 1) % MAX_SAMPLES;
  if (countS < MAX_SAMPLES) countS++;

  // verwijder ouder dan 1000 ms
  uint16_t i = 0;
  uint16_t kept = 0;
  uint32_t cutoff = now - 1000;
  for (i = 0; i < countS; i++){
    uint16_t idx = (head + MAX_SAMPLES - 1 - i) % MAX_SAMPLES; // teruglopend
    if (buf[idx].t_ms >= cutoff) { kept++; } else { break; }
  }
  countS = kept;
}

bool avgDirection(float& out){
  if (countS == 0) return false;
  uint32_t now = millis();
  uint32_t cutoff = now - 1000;
  float sum = 0.0f; uint16_t n = 0;
  for (uint16_t i = 0; i < countS; i++){
    uint16_t idx = (head + MAX_SAMPLES - 1 - i) % MAX_SAMPLES;
    if (buf[idx].t_ms >= cutoff){ sum += buf[idx].v; n++; }
  }
  if (n == 0) return false;
  out = sum / n;
  return true;
}

void onMessage(WebsocketsMessage msg){
  // Verwacht JSON zoals: {"direction_angle": 123.4}
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, msg.data());
  if (err) {
    Serial.print("JSON parse error: ");
    Serial.println(err.c_str());
    return;
  }
  if (doc.containsKey("direction_angle")){
    float dir = doc["direction_angle"];
    addSample(dir);
    float avg;
    if (avgDirection(avg)){
      int angle_int = (int)roundf(constrain(avg, 0.0f, 180.0f));
      Serial.printf("Ontvangen direction_angle: %.1f | Gemiddelde(1s): %d\n", dir, angle_int);
    } else {
      Serial.printf("Ontvangen direction_angle: %.1f | onvoldoende data voor gemiddelde\n", dir);
    }
  }
}

void onEvent(WebsocketsEvent event, String data){
  if (event == WebsocketsEvent::ConnectionOpened){
    Serial.println("WS open");
    //ws.send("{\"hello\":\"esp32-cam\",\"role\":\"listener\"}");
  } else if (event == WebsocketsEvent::ConnectionClosed){
    Serial.println("WS gesloten");
  } else if (event == WebsocketsEvent::GotPing){
    // optional
  } else if (event == WebsocketsEvent::GotPong){
    // optional
  }
}

void connectWiFi(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi verbinden");
  while (WiFi.status() != WL_CONNECTED){
    delay(400);
    Serial.print(".");
  }
  Serial.printf("\nWiFi OK, IP: %s\n", WiFi.localIP().toString().c_str());
}

void connectWS(){
  Serial.printf("Verbind met %s\n", WS_URL);
  if (!ws.connect(WS_URL)){
    Serial.println("WS connectie mislukt");
  } else {
    Serial.println("WS connectie gestart");
  }
}

void setup(){
  Serial.begin(115200);
  delay(200);

  connectWiFi();

  ws.onMessage(onMessage);
  ws.onEvent(onEvent);
  connectWS();
}

void loop(){
  // Houd WebSocket levend
  if (ws.available()){
    ws.poll();
  } else {
    static uint32_t lastTry = 0;
    if (millis() - lastTry > 3000){
      lastTry = millis();
      connectWS();
    }
  }
  delay(5);
}
