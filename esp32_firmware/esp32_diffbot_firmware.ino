/*
  ================================================================
  ESP32 Diff-Drive Robot Firmware
  ================================================================
  Protocol (must match ros2_modular_diffbot/kinematics_wifi_bridge.py):
    PC -> ESP32  : "<left_pwm>,<right_pwm>\n"        (int, -255..255)
    ESP32 -> PC  : "<left_rad_s>,<right_rad_s>\n"    (float, actual wheel speed)

  Hardware assumed:
    - L298N motor driver
    - 2x TT gear motor with 2-channel (A/B) quadrature encoder
    - ESP32 connects to your existing WiFi router (Station mode)

  BEFORE USE:
    1. Set WIFI_SSID / WIFI_PASSWORD below
    2. Set STATIC_IP to something free on your router's subnet
       (must match `esp_ip` parameter in kinematics_wifi_bridge.py)
    3. Wire pins according to the #define block below (change if needed)
    4. CALIBRATE PULSES_PER_REV (see calibration note below) - this is
       the single most important number, wrong value = wrong odometry
  ================================================================
*/

#include <WiFi.h>

// ---------------- WiFi settings ----------------
const char* WIFI_SSID     = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

IPAddress STATIC_IP(192, 168, 1, 100);   // must match esp_ip in kinematics_wifi_bridge.py
IPAddress GATEWAY(192, 168, 1, 1);       // usually your router's IP
IPAddress SUBNET(255, 255, 255, 0);

const uint16_t TCP_PORT = 8080;

// ---------------- Motor driver pins (L298N) ----------------
#define ENA_PIN 25   // left motor PWM (speed)
#define IN1_PIN 26   // left motor direction
#define IN2_PIN 27

#define ENB_PIN 33   // right motor PWM (speed)
#define IN3_PIN 32   // right motor direction
#define IN4_PIN 14

// ---------------- Encoder pins (quadrature: A + B per wheel) ----------------
#define ENC_LEFT_A  18
#define ENC_LEFT_B  19
#define ENC_RIGHT_A 34   // input-only pin, fine for encoders (no internal pull-up needed if module has its own)
#define ENC_RIGHT_B 35

// ---------------- Calibration ----------------
// এই সংখ্যাটা তোমার নির্দিষ্ট motor+gearbox+encoder অনুযায়ী আলাদা হবে।
// ক্যালিব্রেট করার সহজ উপায়: চাকা হাতে ধরে ঠিক ১ পুরো ঘুরিয়ে Serial Monitor এ
// leftPulseCount এর মান দেখো - সেই সংখ্যাটাই এখানে বসাও।
const long PULSES_PER_REV = 390;   // <-- placeholder, নিজে ক্যালিব্রেট করে বসাও
const float TWO_PI_F = 6.28318530718f;

// ---------------- PWM (LEDC) config ----------------
const int PWM_FREQ = 1000;
const int PWM_RES_BITS = 8;   // 0-255
const int PWM_CHANNEL_L = 0;
const int PWM_CHANNEL_R = 1;

WiFiServer server(TCP_PORT);
WiFiClient client;

volatile long leftPulseCount = 0;
volatile long rightPulseCount = 0;

long lastLeftCount = 0;
long lastRightCount = 0;
unsigned long lastFeedbackTime = 0;
const unsigned long FEEDBACK_INTERVAL_MS = 80;   // ~12.5 Hz feedback rate

String rxBuffer = "";

// ---------------- Encoder ISRs ----------------
void IRAM_ATTR leftEncoderISR() {
  bool a = digitalRead(ENC_LEFT_A);
  bool b = digitalRead(ENC_LEFT_B);
  if (a == b) leftPulseCount++; else leftPulseCount--;
}

void IRAM_ATTR rightEncoderISR() {
  bool a = digitalRead(ENC_RIGHT_A);
  bool b = digitalRead(ENC_RIGHT_B);
  if (a == b) rightPulseCount++; else rightPulseCount--;
}

// ---------------- Motor control ----------------
void setMotor(int pwmChannel, int in1, int in2, int pwmValue) {
  pwmValue = constrain(pwmValue, -255, 255);
  if (pwmValue >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  ledcWrite(pwmChannel, abs(pwmValue));
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  ledcAttach(ENA_PIN, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(ENB_PIN, PWM_FREQ, PWM_RES_BITS);
  // পুরনো Arduino-ESP32 core (< 3.0) হলে উপরের দুই লাইনের বদলে এভাবে লিখতে হবে:
  // ledcSetup(PWM_CHANNEL_L, PWM_FREQ, PWM_RES_BITS); ledcAttachPin(ENA_PIN, PWM_CHANNEL_L);
  // ledcSetup(PWM_CHANNEL_R, PWM_FREQ, PWM_RES_BITS); ledcAttachPin(ENB_PIN, PWM_CHANNEL_R);

  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT);
  pinMode(ENC_RIGHT_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderISR, CHANGE);

  WiFi.config(STATIC_IP, GATEWAY, SUBNET);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("TCP server started, waiting for ROS2 PC to connect...");

  lastFeedbackTime = millis();
}

void handleIncomingLine(String line) {
  line.trim();
  int commaIndex = line.indexOf(',');
  if (commaIndex == -1) return;

  int pwmLeft = line.substring(0, commaIndex).toInt();
  int pwmRight = line.substring(commaIndex + 1).toInt();

  setMotor(PWM_CHANNEL_L, IN1_PIN, IN2_PIN, pwmLeft);
  setMotor(PWM_CHANNEL_R, IN3_PIN, IN4_PIN, pwmRight);
}

void sendFeedback() {
  unsigned long now = millis();
  float dt = (now - lastFeedbackTime) / 1000.0f;
  if (dt <= 0) return;

  noInterrupts();
  long currentLeft = leftPulseCount;
  long currentRight = rightPulseCount;
  interrupts();

  long deltaLeft = currentLeft - lastLeftCount;
  long deltaRight = currentRight - lastRightCount;
  lastLeftCount = currentLeft;
  lastRightCount = currentRight;
  lastFeedbackTime = now;

  // pulses -> radians -> rad/s
  float leftRadS = (deltaLeft / (float)PULSES_PER_REV) * TWO_PI_F / dt;
  float rightRadS = (deltaRight / (float)PULSES_PER_REV) * TWO_PI_F / dt;

  if (client && client.connected()) {
    client.printf("%.4f,%.4f\n", leftRadS, rightRadS);
  }
}

void loop() {
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("ROS2 PC connected!");
    }
  }

  if (client && client.connected() && client.available()) {
    char c = client.read();
    if (c == '\n') {
      handleIncomingLine(rxBuffer);
      rxBuffer = "";
    } else {
      rxBuffer += c;
    }
  }

  if (millis() - lastFeedbackTime >= FEEDBACK_INTERVAL_MS) {
    sendFeedback();
  }
}
