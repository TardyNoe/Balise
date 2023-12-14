#include "WiFi.h"
#include "AsyncUDP.h"
#include <Wire.h>

const char* kSsid = "Raspb";
const char* kPassword = "12345678";

int a = 20;
int b = 0;
int c = 0;

AsyncUDP udp;


void setupWiFi() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(kSsid, kPassword);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("Connected to WiFi. IP: ");
  Serial.println(WiFi.localIP());
}

void setupUDP() {
  if (udp.listen(1234)) {
    Serial.println("UDP Listening...");
    udp.onPacket([](AsyncUDPPacket packet) {
      byte byteArray[6] = {
        a & 255, a >> 8 & 255,
        b & 255, b >> 8 & 255,
        c & 255, c >> 8 & 255
      };
      packet.write(byteArray, sizeof(byteArray));

      uint8_t Recived[6];
      size_t lengthToCopy = min(packet.length(), sizeof(Recived));
      memcpy(Recived, packet.data(), lengthToCopy);
      a = (Recived[0] << 8) + Recived[1];
      b = (Recived[2] << 8) + Recived[3];
      c = (Recived[4] << 8) + Recived[5];
    });
  }
}

void setup() {
  Serial.begin(115200);
  setupWiFi();
  setupUDP();
}

void loop() {
  delay(200);
}
