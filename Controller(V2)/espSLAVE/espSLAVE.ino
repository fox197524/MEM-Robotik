#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "Fox-2";
const char* password = "Kyra2bin9";

WiFiUDP udp;
unsigned int localPort = 4210;

unsigned long lastUpdate = 0;
const int updateInterval = 20; // ~50 Hz (20 ms)

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

    if (millis() - lastUpdate >= updateInterval) {
      lastUpdate = millis();

      String msg = String(packet);

      if (msg.startsWith("AXIS")) {
        int firstSpace = msg.indexOf(' ');
        int secondSpace = msg.indexOf(' ', firstSpace + 1);
        int axisId = msg.substring(firstSpace + 1, secondSpace).toInt();
        float val = msg.substring(secondSpace + 1).toFloat();

        int barLen = int((val + 1.0) * 10); 
        Serial.printf("Axis %d: [", axisId);
        for (int i = 0; i < 20; i++) {
          if (i < barLen) Serial.print("#");
          else Serial.print("-");
        }
        Serial.printf("] %.2f\n", val);
      }

      else if (msg.startsWith("BUTTON")) {
        int firstSpace = msg.indexOf(' ');
        int secondSpace = msg.indexOf(' ', firstSpace + 1);
        int btnId = msg.substring(firstSpace + 1, secondSpace).toInt();
        int state = msg.substring(secondSpace + 1).toInt();

        Serial.printf("Button %d: %s\n", btnId, state ? "PRESSED" : "released");
      }
      
      else if (msg.startsWith("HAT")) {
        Serial.println("D-pad: " + msg);
        
      }
    }
  }
}