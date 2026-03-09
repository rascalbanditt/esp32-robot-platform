#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <MPU9250.h>

// ================= I2C =================
#define SDA_PIN 1
#define SCL_PIN 2

MPU9250 imu;

// ================= Motor Pins =================
#define PWMA 4
#define AIN1 5
#define AIN2 6
#define EN1  7

#define PWMB 8
#define BIN1 9
#define BIN2 10
#define EN2  3

// ================= MASTER MAC (PUT YOUR S3 MAC HERE) =================
uint8_t masterMAC[] = {0x80,0xB5,0x4E,0xC1,0xB4,0xBC};

// ================= Control Packet =================
typedef struct {
  uint8_t state;
  float vx;
  float vy;
  float wz;
  uint32_t timestamp;
} ControlPacket;

// ================= Telemetry Packet =================
typedef struct {
  float x;
  float y;
  float yaw;
} TelemetryPacket;

float x_pos = 0;
float y_pos = 0;
float yaw = 0;

unsigned long lastTime = 0;
unsigned long lastPacketTime = 0;

ControlPacket currentCmd;

// ================= Motor Functions =================

void motorStop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void motorDrive(float vx, float wz) {
  if (vx > 0.1) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  }
  else if (vx < -0.1) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  }
  else if (wz > 0.1) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  }
  else if (wz < -0.1) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  }
  else {
    motorStop();
  }

  analogWrite(PWMA, 200);
  analogWrite(PWMB, 200);
}

// ================= Receive Control =================

void onReceive(const esp_now_recv_info *info, const uint8_t *data, int len) {
  memcpy(&currentCmd, data, sizeof(currentCmd));
  lastPacketTime = millis();
  motorDrive(currentCmd.vx, currentCmd.wz);
}

// ================= Setup =================

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(EN1, OUTPUT);

  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(EN2, OUTPUT);

  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);

  motorStop();

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!imu.setup(0x68)) {
    Serial.println("IMU not found!");
    while (1);
  }

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_now_init();

  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, masterMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  lastTime = millis();
}

// ================= Loop =================

void loop() {

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // Update IMU
  if (imu.update()) {
    yaw = imu.getYaw();
  }

  // Dead reckoning using commanded velocity
  float vx = currentCmd.vx;
  x_pos += vx * cos(yaw * DEG_TO_RAD) * dt;
  y_pos += vx * sin(yaw * DEG_TO_RAD) * dt;

  // Send telemetry
  TelemetryPacket telem;
  telem.x = x_pos;
  telem.y = y_pos;
  telem.yaw = yaw;

  esp_now_send(masterMAC, (uint8_t*)&telem, sizeof(telem));

  // Safety stop
  if (millis() - lastPacketTime > 2000) {
    motorStop();
  }

  delay(50);
}