#include <Arduino.h>

#define IBUS_FRAME_SIZE 32
#define IBUS_CHANNELS   14

uint8_t ibusFrame[IBUS_FRAME_SIZE];
uint16_t channels[IBUS_CHANNELS];

void setup() {
  Serial1.begin(115200); // PA9 TX, PA10 RX
}

bool readIbusFrame() {
  if (Serial1.available() >= IBUS_FRAME_SIZE) {

    for (int i = 0; i < IBUS_FRAME_SIZE; i++) {
      ibusFrame[i] = Serial1.read();
    }

    uint16_t checksum = 0xFFFF;
    for (int i = 0; i < IBUS_FRAME_SIZE - 2; i++) {
      checksum -= ibusFrame[i];
    }
    uint16_t received = ibusFrame[30] | (ibusFrame[31] << 8);
    if (checksum != received) return false;

    for (int ch = 0; ch < IBUS_CHANNELS; ch++) {
      channels[ch] = ibusFrame[2 + ch * 2] | (ibusFrame[3 + ch * 2] << 8);
    }
    return true;
  }
  return false;
}

unsigned long lastSend = 0;

void loop() {
  if (readIbusFrame()) {
    if (millis() - lastSend >= 50) { 
      lastSend = millis();
      for (int i = 0; i < IBUS_CHANNELS; i++) {
        Serial1.print(channels[i]);
        if (i < IBUS_CHANNELS - 1) Serial1.print(',');
      }
      Serial1.println();
    }
  }
}
