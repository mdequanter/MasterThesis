#include <ESP8266WiFi.h>
#include <ArduinoWebsockets.h>

using namespace websockets;

const char* WIFI_SSID = "AILABO";
const char* WIFI_PASS = "geheim123$";
const char* WS_URL = "wss://signaling.ehb.be";
const char* WS_ROOM = "/ws/pathnavigation";
const char* BEARER_TOKEN = "LTddk_ptxQX-omdw5B5rfpniA2wB-19KBxFaKuODMzw";

WebsocketsClient ws;

bool parseFloatAfterColon(const String& payload, int colonPos, float& value) {
  if (colonPos < 0) return false;

  int start = colonPos + 1;
  while (start < payload.length() && (payload[start] == ' ' || payload[start] == '\"')) {
    start++;
  }

  int end = start;
  while (end < payload.length()) {
    char c = payload[end];
    bool validChar =
      (c >= '0' && c <= '9') ||
      c == '-' ||
      c == '+' ||
      c == '.';

    if (!validChar) {
      break;
    }
    end++;
  }

  if (end <= start) return false;

  value = payload.substring(start, end).toFloat();
  return true;
}

bool extractHeading(const String& payload, float& heading) {
  int keyPos = payload.indexOf("\"heading\"");
  if (keyPos >= 0) {
    return parseFloatAfterColon(payload, payload.indexOf(':', keyPos), heading);
  }

  keyPos = payload.indexOf("\"direction_angle\"");
  if (keyPos >= 0) {
    return parseFloatAfterColon(payload, payload.indexOf(':', keyPos), heading);
  }

  return false;
}

void onMessage(WebsocketsMessage msg) {
  float heading = 0.0f;
  if (extractHeading(msg.data(), heading)) {
    Serial.print("Heading: ");
    Serial.println(heading, 1);
  } else {
    Serial.print("Message: ");
    Serial.println(msg.data());
  }
}

void onEvent(WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionOpened) {
    Serial.println("WebSocket connected");
  } else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println("WebSocket closed");
  } else if (event == WebsocketsEvent::GotPing) {
    Serial.println("WebSocket ping");
  } else if (event == WebsocketsEvent::GotPong) {
    Serial.println("WebSocket pong");
  }
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Connected. IP: ");
  Serial.println(WiFi.localIP());
}

void connectWS() {
  Serial.print("Connecting to ");
  Serial.println(WS_URL);

  ws.setInsecure();
  if (!ws.connect(WS_URL)) {
    Serial.println("WebSocket connect failed");
  } else {
    Serial.println("WebSocket connect started");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  connectWiFi();

  ws.onMessage(onMessage);
  ws.onEvent(onEvent);
  ws.addHeader("Authorization", String("Bearer ") + BEARER_TOKEN);
  ws.addHeader("X-Room", WS_ROOM);
  connectWS();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  if (ws.available()) {
    ws.poll();
  } else {
    static uint32_t lastTry = 0;
    if (millis() - lastTry > 3000) {
      lastTry = millis();
      connectWS();
    }
  }
}
