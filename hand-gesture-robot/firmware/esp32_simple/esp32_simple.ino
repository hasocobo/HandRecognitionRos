/*
 * ESP32 Simple Motor Controller
 * 
 * Receives MQTT commands from server and drives motors directly.
 * No Arduino slave needed - ESP32 does PWM directly.
 * 
 * Message format: {"left": -255..255, "right": -255..255}
 * Positive = forward, Negative = backward
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
const char* MQTT_TOPIC = "robot/cmd";

// Motor Pins (adjust for your wiring)
// Left Motor
const int LEFT_PWM = 25;    // PWM pin (must support PWM)
const int LEFT_DIR1 = 26;   // Direction pin 1
const int LEFT_DIR2 = 27;   // Direction pin 2 (for H-bridge)

// Right Motor  
const int RIGHT_PWM = 32;   // PWM pin (must support PWM)
const int RIGHT_DIR1 = 33;  // Direction pin 1
const int RIGHT_DIR2 = 14;  // Direction pin 2 (for H-bridge)

// PWM Settings
const int PWM_FREQ = 5000;
const int PWM_RES = 8;  // 8-bit = 0-255
const int LEFT_PWM_CH = 0;
const int RIGHT_PWM_CH = 1;

// Safety timeout (stop if no command for this many ms)
const unsigned long TIMEOUT_MS = 500;

// ============================================

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

unsigned long lastCommandTime = 0;
bool motorsRunning = false;

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32 Motor Controller ===");
    
    setupMotors();
    connectWiFi();
    
    mqtt.setServer(MQTT_SERVER, MQTT_PORT);
    mqtt.setCallback(onMqttMessage);
    mqtt.setBufferSize(256);
}

void loop() {
    // Reconnect MQTT if needed
    if (!mqtt.connected()) {
        reconnectMqtt();
    }
    mqtt.loop();
    
    // Safety timeout - stop motors if no command received
    if (motorsRunning && (millis() - lastCommandTime > TIMEOUT_MS)) {
        Serial.println("⚠️ Timeout - stopping motors");
        stopMotors();
        motorsRunning = false;
    }
}

void setupMotors() {
    // Configure motor pins
    pinMode(LEFT_DIR1, OUTPUT);
    pinMode(LEFT_DIR2, OUTPUT);
    pinMode(RIGHT_DIR1, OUTPUT);
    pinMode(RIGHT_DIR2, OUTPUT);
    
    // Setup PWM channels
    ledcSetup(LEFT_PWM_CH, PWM_FREQ, PWM_RES);
    ledcSetup(RIGHT_PWM_CH, PWM_FREQ, PWM_RES);
    ledcAttachPin(LEFT_PWM, LEFT_PWM_CH);
    ledcAttachPin(RIGHT_PWM, RIGHT_PWM_CH);
    
    stopMotors();
    Serial.println("✓ Motors initialized");
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
        Serial.println("\n✗ WiFi connection failed! Restarting...");
        ESP.restart();
    }
}

void reconnectMqtt() {
    while (!mqtt.connected()) {
        Serial.printf("Connecting to MQTT %s:%d...", MQTT_SERVER, MQTT_PORT);
        
        String clientId = "esp32_robot_" + String(random(0xffff), HEX);
        
        if (mqtt.connect(clientId.c_str())) {
            Serial.println(" ✓ connected!");
            mqtt.subscribe(MQTT_TOPIC);
            Serial.printf("✓ Subscribed to '%s'\n", MQTT_TOPIC);
        } else {
            Serial.printf(" ✗ failed (rc=%d), retrying in 3s...\n", mqtt.state());
            delay(3000);
        }
    }
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    // Parse JSON
    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    
    if (err) {
        Serial.printf("JSON parse error: %s\n", err.c_str());
        return;
    }
    
    int left = doc["left"] | 0;
    int right = doc["right"] | 0;
    
    // Clamp values
    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);
    
    // Drive motors
    setMotor(LEFT_PWM_CH, LEFT_DIR1, LEFT_DIR2, left);
    setMotor(RIGHT_PWM_CH, RIGHT_DIR1, RIGHT_DIR2, right);
    
    lastCommandTime = millis();
    motorsRunning = (left != 0 || right != 0);
    
    // Debug output (comment out for production)
    Serial.printf("L:%4d R:%4d\n", left, right);
}

void setMotor(int pwmChannel, int dir1, int dir2, int speed) {
    if (speed > 0) {
        // Forward
        digitalWrite(dir1, HIGH);
        digitalWrite(dir2, LOW);
        ledcWrite(pwmChannel, speed);
    } else if (speed < 0) {
        // Backward
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, HIGH);
        ledcWrite(pwmChannel, -speed);
    } else {
        // Stop (brake)
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, LOW);
        ledcWrite(pwmChannel, 0);
    }
}

void stopMotors() {
    ledcWrite(LEFT_PWM_CH, 0);
    ledcWrite(RIGHT_PWM_CH, 0);
    digitalWrite(LEFT_DIR1, LOW);
    digitalWrite(LEFT_DIR2, LOW);
    digitalWrite(RIGHT_DIR1, LOW);
    digitalWrite(RIGHT_DIR2, LOW);
}

