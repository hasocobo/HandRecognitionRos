/*
  Hand Gesture Robot - ESP32 MASTER
  
  Receives MQTT commands from server: {"left": -255..255, "right": -255..255}
  Forwards to Arduino via Serial:     <L:xxx,R:yyy>
  
  Wiring:
    ESP32 TX2 (GPIO17) -> Arduino RX
    ESP32 RX2 (GPIO16) -> Arduino TX
    ESP32 GND          -> Arduino GND
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ============== CONFIGURATION ==============
// WiFi
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// MQTT Server (your EC2 public IP)
const char* MQTT_SERVER = "YOUR_EC2_PUBLIC_IP";
const int MQTT_PORT = 1883;
const char* MQTT_TOPIC_CMD = "robot/cmd";
const char* MQTT_TOPIC_TELEMETRY = "robot/telemetry";

// Serial to Arduino (Hardware Serial2)
#define ARDUINO_SERIAL Serial2
const int ARDUINO_RX = 16;  // ESP32 RX2
const int ARDUINO_TX = 17;  // ESP32 TX2
const long ARDUINO_BAUD = 115200;

// Safety
const unsigned long TIMEOUT_MS = 500;
const unsigned long TELEMETRY_INTERVAL_MS = 5000;

// ============================================

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

unsigned long lastCommandTime = 0;
unsigned long lastTelemetryTime = 0;
int lastLeft = 0;
int lastRight = 0;
bool motorsRunning = false;

void setup() {
  // Debug serial
  Serial.begin(115200);
  Serial.println("\n=== ESP32 MQTT-Arduino Bridge ===");
  
  // Arduino serial
  ARDUINO_SERIAL.begin(ARDUINO_BAUD, SERIAL_8N1, ARDUINO_RX, ARDUINO_TX);
  Serial.println("✓ Arduino Serial initialized");
  
  // WiFi
  connectWiFi();
  
  // MQTT
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(256);
  
  // Send initial stop to Arduino
  sendToArduino(0, 0);
}

void loop() {
  // Maintain MQTT connection
  if (!mqtt.connected()) {
    reconnectMqtt();
  }
  mqtt.loop();
  
  // Safety timeout
  if (motorsRunning && (millis() - lastCommandTime > TIMEOUT_MS)) {
    Serial.println("⚠️ Timeout - stopping");
    sendToArduino(0, 0);
    motorsRunning = false;
  }
  
  // Periodic telemetry
  if (millis() - lastTelemetryTime > TELEMETRY_INTERVAL_MS) {
    sendTelemetry();
    lastTelemetryTime = millis();
  }
}

void connectWiFi() {
  Serial.printf("Connecting to WiFi '%s'", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n✓ WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n✗ WiFi failed! Restarting...");
    ESP.restart();
  }
}

void reconnectMqtt() {
  while (!mqtt.connected()) {
    Serial.printf("Connecting to MQTT %s:%d...", MQTT_SERVER, MQTT_PORT);
    
    String clientId = "esp32_robot_" + String(random(0xffff), HEX);
    
    if (mqtt.connect(clientId.c_str())) {
      Serial.println(" ✓ connected!");
      mqtt.subscribe(MQTT_TOPIC_CMD);
      Serial.printf("✓ Subscribed to '%s'\n", MQTT_TOPIC_CMD);
    } else {
      Serial.printf(" ✗ failed (rc=%d), retry in 3s...\n", mqtt.state());
      delay(3000);
    }
  }
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  // Parse JSON: {"left": X, "right": Y}
  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  
  if (err) {
    Serial.printf("JSON error: %s\n", err.c_str());
    return;
  }
  
  int left = doc["left"] | 0;
  int right = doc["right"] | 0;
  
  // Clamp values
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  
  // Forward to Arduino
  sendToArduino(left, right);
  
  lastLeft = left;
  lastRight = right;
  lastCommandTime = millis();
  motorsRunning = (left != 0 || right != 0);
  
  // Debug
  Serial.printf("L:%4d R:%4d\n", left, right);
}

void sendToArduino(int left, int right) {
  // Format: <L:xxx,R:yyy>
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "<L:%d,R:%d>", left, right);
  ARDUINO_SERIAL.print(buffer);
}

void sendTelemetry() {
  if (!mqtt.connected()) return;
  
  StaticJsonDocument<128> doc;
  doc["status"] = motorsRunning ? "running" : "idle";
  doc["left"] = lastLeft;
  doc["right"] = lastRight;
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["uptime_s"] = millis() / 1000;
  
  char buffer[128];
  serializeJson(doc, buffer);
  mqtt.publish(MQTT_TOPIC_TELEMETRY, buffer);
}
