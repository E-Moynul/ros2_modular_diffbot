/*
  ================================================================
  ESP32 Diff-Drive Robot Firmware (SIMPLIFIED - no encoder)
  ================================================================
  Protocol:
    PC -> ESP32  : "<left_pwm>,<right_pwm>\n"   (int, -200..200)
    (কোনো feedback ফেরত পাঠানো হয় না - open-loop)

  Hardware assumed:
    - L298N motor driver
    - 2x plain BO gear motor (NO encoder)
    - ESP32 connects to your existing WiFi router (Station mode)

  BEFORE USE:
    1. WIFI_SSID / WIFI_PASSWORD বসাও
    2. STATIC_IP নিজের router এর subnet অনুযায়ী ঠিক করো
       (kinematics_wifi_bridge.py এর esp_ip parameter এর সাথে মিলতে হবে)
  ================================================================
*/

#include <WiFi.h>

// ---------------- WiFi settings ----------------
const char* WIFI_SSID     = "Redmi 13";
const char* WIFI_PASSWORD = "00000000";

IPAddress STATIC_IP(10, 26, 161, 100);
IPAddress GATEWAY(10, 26, 161, 168);
IPAddress SUBNET(255, 255, 255, 0);

const uint16_t TCP_PORT = 8080;

// ---------------- Motor driver pins (L298N) ----------------
#define ENA_PIN 25   // left motor PWM (speed)
#define IN1_PIN 26   // left motor direction
#define IN2_PIN 27

#define ENB_PIN 33   // right motor PWM (speed)
#define IN3_PIN 32   // right motor direction
#define IN4_PIN 14

// ---------------- PWM (LEDC) config ----------------
const int PWM_FREQ = 1000;
const int PWM_RES_BITS = 8;   // 0-255
const int PWM_CHANNEL_L = 0;
const int PWM_CHANNEL_R = 1;

WiFiServer server(TCP_PORT);
WiFiClient client;
String rxBuffer = "";

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
  // পুরনো Arduino-ESP32 core (< 3.0) হলে বদলে এভাবে লিখতে হবে:
  // ledcSetup(PWM_CHANNEL_L, PWM_FREQ, PWM_RES_BITS); ledcAttachPin(ENA_PIN, PWM_CHANNEL_L);
  // ledcSetup(PWM_CHANNEL_R, PWM_FREQ, PWM_RES_BITS); ledcAttachPin(ENB_PIN, PWM_CHANNEL_R);

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
}

void handleIncomingLine(String line) {
  line.trim();
  int commaIndex = line.indexOf(',');
  if (commaIndex == -1) return;

  int pwmLeft = line.substring(0, commaIndex).toInt();
  int pwmRight = line.substring(commaIndex + 1).toInt();

  setMotor(PWM_CHANNEL_L, IN1_PIN, IN2_PIN, pwmLeft);
  setMotor(PWM_CHANNEL_R, IN3_PIN, IN4_PIN, pwmRight);

  Serial.print("PWM L,R = ");
  Serial.print(pwmLeft);
  Serial.print(",");
  Serial.println(pwmRight);
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
}
