#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "ESP-S3-N16R8";
const char* password = "12345678";

WiFiUDP udp;
unsigned int localPort = 4210;  

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  Serial.print("Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(localPort);
  Serial.println("UDP slave ready");
}

void loop() {
  char packet[255];
  int len = udp.parsePacket();
  if (len) {
    udp.read(packet, 255);
    packet[len] = '\0';
    Serial.println("Received: " + String(packet));
  }
}