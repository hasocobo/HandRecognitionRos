/*
  Hand Gesture Robot - ARDUINO SLAVE
  
  Receives left/right PWM commands from ESP32 via Serial.
  Format: <L:xxx,R:yyy>  where xxx/yyy are -255 to 255
  
  Positive = Forward, Negative = Backward
*/

// --- PIN DEFINITIONS ---
// Right Motor
const int IN1 = 10;
const int IN2 = 9;
// Left Motor
const int IN3 = 5;
const int IN4 = 6;

// --- CONFIGURATION ---
const unsigned long TIMEOUT_MS = 500;  // Stop if no command for 500ms

// --- STATE ---
int leftPWM = 0;
int rightPWM = 0;
unsigned long lastCommandTime = 0;

// Serial Buffer
String inputString = "";
bool stringComplete = false;

void setup() {
  // Motor Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Start Serial (match ESP32 baud rate)
  Serial.begin(115200);
  inputString.reserve(64);
  
  stopMotors();
  Serial.println("Arduino Motor Controller Ready");
}

void loop() {
  // 1. Read Serial Data
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '<') {
      // Start of packet - reset buffer
      inputString = "";
    } else if (c == '>') {
      // End of packet - process it
      stringComplete = true;
    } else {
      inputString += c;
    }
  }

  // 2. Process Command
  if (stringComplete) {
    parseCommand(inputString);
    inputString = "";
    stringComplete = false;
    lastCommandTime = millis();
    
    // Drive motors immediately
    driveMotor(IN3, IN4, leftPWM);   // Left motor
    driveMotor(IN1, IN2, rightPWM);  // Right motor
  }

  // 3. Safety Timeout
  if (millis() - lastCommandTime > TIMEOUT_MS) {
    if (leftPWM != 0 || rightPWM != 0) {
      Serial.println("Timeout - stopping");
      leftPWM = 0;
      rightPWM = 0;
      stopMotors();
    }
  }
}

// Parse: L:100,R:100
void parseCommand(String cmd) {
  int lIdx = cmd.indexOf("L:");
  int rIdx = cmd.indexOf("R:");
  int commaIdx = cmd.indexOf(',');
  
  if (lIdx == -1 || rIdx == -1 || commaIdx == -1) {
    Serial.println("Parse error");
    return;
  }
  
  // Extract values
  String leftStr = cmd.substring(lIdx + 2, commaIdx);
  String rightStr = cmd.substring(rIdx + 2);
  
  leftPWM = constrain(leftStr.toInt(), -255, 255);
  rightPWM = constrain(rightStr.toInt(), -255, 255);
  
  // Debug (comment out for less serial traffic)
  // Serial.print("L:"); Serial.print(leftPWM);
  // Serial.print(" R:"); Serial.println(rightPWM);
}

// Drive a single motor
void driveMotor(int pinA, int pinB, int speed) {
  if (speed > 0) {
    // Forward
    analogWrite(pinA, min(speed, 255));
    digitalWrite(pinB, LOW);
  } else if (speed < 0) {
    // Backward
    digitalWrite(pinA, LOW);
    analogWrite(pinB, min(-speed, 255));
  } else {
    // Stop (brake)
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
