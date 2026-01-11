/*
  ESP32-CAM + MQTT Bridge (Merged)

  - Runs the ESP32-CAM web server (startCameraServer()) for streaming/config UI
  - Connects to WiFi once
  - Subscribes to MQTT topic "robot/cmd" with JSON:
      {"left":-255..255,"right":-255..255,"gripper":0/1}
  - Forwards commands to Arduino over Serial2 as:
      <L:xxx,R:yyy,G:z>
  - Publishes telemetry on "robot/telemetry"

  Wiring (as you wrote):
    ESP32 TX2 (GPIO13) -> Arduino RX
    ESP32 RX2 (GPIO14) -> Arduino TX
    ESP32 GND          -> Arduino GND

  NOTE: On some ESP32-CAM boards, GPIO13/14 may conflict with SD card pins.
  If you use the SD slot, you will likely need different UART pins.
*/

#include "esp_camera.h"
#include <WiFi.h>

#include <PubSubClient.h>
#include <ArduinoJson.h>

// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"

// ===========================
// WiFi
// ===========================
const char* WIFI_SSID = "LILIA 7744";
const char* WIFI_PASS = "0784&v5J";

// ===========================
// Static IP (edit to match your router subnet)
// ===========================
IPAddress local_IP(192, 168, 137, 50);
IPAddress gateway(192, 168, 137, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(1, 1, 1, 1);
IPAddress secondaryDNS(8, 8, 8, 8);

// ===========================
// MQTT
// ===========================
const char* MQTT_SERVER = "13.62.11.187";
const int   MQTT_PORT   = 1883;

const char* MQTT_TOPIC_CMD       = "robot/cmd";
const char* MQTT_TOPIC_TELEMETRY = "robot/telemetry";

// ===========================
// Serial to Arduino (Hardware Serial2)
// ===========================
#define ARDUINO_SERIAL Serial2
const int  ARDUINO_RX   = 13;     // ESP32 RX2
const int  ARDUINO_TX   = 14;     // ESP32 TX2
const long ARDUINO_BAUD = 115200;

// ===========================
// Safety / Telemetry
// ===========================
const unsigned long TIMEOUT_MS            = 500;
const unsigned long TELEMETRY_INTERVAL_MS = 5000;

// ===========================
// Gripper / clipper
// ===========================
const int DEFAULT_GRIPPER = 0;  // 0=open, 1=closed, -1=unknown (omit)

// ===========================
// Camera server hooks (provided by ESP32-CAM webserver example)
// ===========================
void startCameraServer();
void setupLedFlash();

// ===========================
// Globals
// ===========================
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

unsigned long lastCommandTime   = 0;
unsigned long lastTelemetryTime = 0;
int lastLeft  = 0;
int lastRight = 0;
int lastGripper = DEFAULT_GRIPPER;
bool motorsRunning = false;

// ===========================
// Helpers
// ===========================
static void sendToArduino(int left, int right, int gripper) {
  char buffer[48];
  if (gripper >= 0) {
    snprintf(buffer, sizeof(buffer), "<L:%d,R:%d,G:%d>", left, right, gripper);
  } else {
    snprintf(buffer, sizeof(buffer), "<L:%d,R:%d>", left, right);
  }
  ARDUINO_SERIAL.print(buffer);
}

static void sendTelemetry() {
  if (!mqtt.connected()) return;

  StaticJsonDocument<160> doc;
  doc["status"]    = motorsRunning ? "running" : "idle";
  doc["left"]      = lastLeft;
  doc["right"]     = lastRight;
  if (lastGripper >= 0) {
    doc["gripper"] = lastGripper;
  }
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["uptime_s"]  = millis() / 1000;

  char out[160];
  serializeJson(doc, out);
  mqtt.publish(MQTT_TOPIC_TELEMETRY, out);
}

static void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<160> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.printf("[MQTT] JSON error: %s\n", err.c_str());
    return;
  }

  int left  = doc["left"]  | 0;
  int right = doc["right"] | 0;
  int gripper = lastGripper;

  if (doc.containsKey("gripper")) {
    gripper = doc["gripper"] | 0;
    gripper = (gripper != 0) ? 1 : 0;
    lastGripper = gripper;
  }

  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  sendToArduino(left, right, gripper);

  lastLeft = left;
  lastRight = right;
  lastCommandTime = millis();
  motorsRunning = (left != 0 || right != 0);

  Serial.printf("[CMD] L:%4d R:%4d G:%d\n", left, right, gripper);
}

static void connectWiFi() {
  Serial.printf("Connecting to WiFi '%s'", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("\n[WARN] Failed to configure static IP");
  }

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 60) {
    delay(250);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[OK] WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[ERR] WiFi failed! Restarting...");
    ESP.restart();
  }
}

static void reconnectMqtt() {
  while (!mqtt.connected()) {
    Serial.printf("Connecting to MQTT %s:%d...", MQTT_SERVER, MQTT_PORT);

    String clientId = "esp32_robot_" + String((uint32_t)esp_random(), HEX);

    if (mqtt.connect(clientId.c_str())) {
      Serial.println(" [OK] connected");
      mqtt.subscribe(MQTT_TOPIC_CMD);
      Serial.printf("[OK] Subscribed to '%s'\n", MQTT_TOPIC_CMD);
    } else {
      Serial.printf(" [ERR] failed (rc=%d), retry in 3s...\n", mqtt.state());
      delay(3000);
    }
  }
}

static bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.frame_size   = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // streaming
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count     = 1;

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();

  if (s && s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }

  if (config.pixel_format == PIXFORMAT_JPEG && s) {
    s->set_framesize(s, FRAMESIZE_QVGA); // faster initial stream
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  if (s) {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
  }
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  if (s) s->set_vflip(s, 1);
#endif

#if defined(LED_GPIO_NUM)
  setupLedFlash();
#endif

  return true;
}

// ===========================
// Arduino entry points
// ===========================
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("\n=== ESP32-CAM + MQTT-Arduino Bridge ===");

  // Arduino serial first (so you can see if it crashes before camera init)
  ARDUINO_SERIAL.begin(ARDUINO_BAUD, SERIAL_8N1, ARDUINO_RX, ARDUINO_TX);
  Serial.println("[OK] Arduino Serial2 initialized");

  // Camera
  if (!initCamera()) {
    Serial.println("[ERR] Camera init failed. Halting.");
    while (true) { delay(1000); }
  }
  Serial.println("[OK] Camera initialized");

  // WiFi
  connectWiFi();

  // Camera web server
  startCameraServer();
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  // MQTT
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(256);

  // Initial stop for safety
  sendToArduino(0, 0, lastGripper);
  motorsRunning = false;
  lastCommandTime = millis();
  lastTelemetryTime = millis();
}

void loop() {
  // Maintain MQTT connection
  if (!mqtt.connected()) {
    reconnectMqtt();
  }
  mqtt.loop();

  // Safety timeout: stop motors if no command recently
  if (motorsRunning && (millis() - lastCommandTime > TIMEOUT_MS)) {
    sendToArduino(0, 0, lastGripper);
    motorsRunning = false;
  }

  // Periodic telemetry
  if (millis() - lastTelemetryTime > TELEMETRY_INTERVAL_MS) {
    sendTelemetry();
    lastTelemetryTime = millis();
  }

  // Keep loop responsive (camera server runs in other tasks)
  delay(5);
}
