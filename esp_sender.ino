#include <WiFi.h>
#include <esp_now.h>

// ================= Robot MAC =================
uint8_t robotMAC[] = {0xE8,0x06,0x90,0x64,0x32,0x10};

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

ControlPacket pkt;
unsigned long lastSend = 0;

void sendPacket() {
  pkt.timestamp = millis();
  esp_now_send(robotMAC, (uint8_t*)&pkt, sizeof(pkt));
}

// ================= Receive Telemetry =================

void onReceive(const esp_now_recv_info *info, const uint8_t *data, int len) {
  TelemetryPacket telem;
  memcpy(&telem, data, sizeof(telem));

  Serial.print("X: ");
  Serial.print(telem.x);
  Serial.print(" Y: ");
  Serial.print(telem.y);
  Serial.print(" Yaw: ");
  Serial.println(telem.yaw);
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_now_init();

  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, robotMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.println("Controls: w a s d x");
}

void loop() {

  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'w') { pkt.vx = 0.5; pkt.wz = 0; pkt.state = 1; }
    if (c == 's') { pkt.vx = -0.5; pkt.wz = 0; pkt.state = 1; }
    if (c == 'a') { pkt.wz = 0.6; pkt.vx = 0; pkt.state = 1; }
    if (c == 'd') { pkt.wz = -0.6; pkt.vx = 0; pkt.state = 1; }

    if (c == 'x') {
      pkt.vx = 0;
      pkt.wz = 0;
      pkt.state = 3;
    }
  }

  if (millis() - lastSend > 50) {
    sendPacket();
    lastSend = millis();
  }
}