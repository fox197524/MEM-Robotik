#include <SPI.h>
#include <TFT_eSPI.h>
#include "map_image.h"

#define MAP_WIDTH  187
#define MAP_HEIGHT 180

TFT_eSPI tft = TFT_eSPI();

void setup() {
  Serial.begin(115200);
  // Give it a moment to connect the serial monitor
  delay(2000); 

  // --- START OF DIAGNOSTIC CODE ---
  Serial.println("--- Map Data Verification ---");

  // 1. Check the size of the array in bytes
  size_t expected_size = MAP_WIDTH * MAP_HEIGHT * 2;
  size_t actual_size = sizeof(map_data);
  Serial.print("Expected array size (bytes): ");
  Serial.println(expected_size);
  Serial.print("Actual array size (bytes):   ");
  Serial.println(actual_size);

  if (actual_size == expected_size) {
    Serial.println("SUCCESS: Array size is correct!");
  } else {
    Serial.println("ERROR: Array size is INCORRECT!");
  }

  // 2. Check the value of the very first pixel
  Serial.print("Value of first pixel (HEX): 0x");
  Serial.println(map_data[0], HEX);
  Serial.println("---------------------------");
  // --- END OF DIAGNOSTIC CODE ---


  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  // ... rest of your setup code ...
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(10, 10);
  tft.println("Loading map...");
  delay(1000);
  tft.pushImage(0, 0, MAP_WIDTH, MAP_HEIGHT, map_data);
}

void loop() {
  // ...
}