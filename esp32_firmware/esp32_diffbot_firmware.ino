#include <WiFi.h>

// ---------------- WiFi ----------------
const char* WIFI_SSID     = "Redmi 13";
const char* WIFI_PASSWORD = "00000000";

IPAddress STATIC_IP(10, 26, 161, 100);
IPAddress GATEWAY(10, 26, 161, 168);
IPAddress SUBNET(255, 255, 255, 0);

const uint16_t TCP_PORT = 8080;

// ---------------- L298N ----------------
#define ENA_PIN 25   // left motor PWM speed
#define IN1_PIN 26   // left motor direction
#define IN2_PIN 27

#define ENB_PIN 33   // right motor PWM speed
#define IN3_PIN 32   // right motor direction
#define IN4_PIN 14

// ---------------- PWM (LEDC) config ----------------
const int PWM_FREQ = 1000;
const int PWM_RES_BITS = 8; 

WiFiServer server(TCP_PORT);
WiFiClient client;
String rxBuffer = "";

void setMotor(int pwmPin, int in1, int in2, int pwmValue) {
  pwmValue = constrain(pwmValue, -255, 255);
  if (pwmValue >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  //error fix: instead of channel , directly pin is being used here
  ledcWrite(pwmPin, abs(pwmValue));
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(2, OUTPUT); // LED

  ledcAttach(ENA_PIN, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(ENB_PIN, PWM_FREQ, PWM_RES_BITS);

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

  setMotor(ENA_PIN, IN1_PIN, IN2_PIN, pwmLeft);
  setMotor(ENB_PIN, IN3_PIN, IN4_PIN, pwmRight);
  
  if (pwmLeft > 0 || pwmRight > 0) {
    digitalWrite(2, HIGH);
  } else {
    digitalWrite(2, LOW);
  }

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
