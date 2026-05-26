// Minimal WiFi connection script for Wemos D1 Mini
// Required libraries:
// - ESP8266 board package

#include <ESP8266WiFi.h>

// === WiFi config ===
const char* WIFI_SSID = "AILABO_24";
const char* WIFI_PASS = "geheim123$";

void setup() {
  Serial.begin(9600);
  delay(200);

  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nWiFi reconnected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  delay(1000);
}
