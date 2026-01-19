// Your main ESP32 sketch file: map_display.ino

#include <SPI.h>
#include <TFT_eSPI.h> // Include the graphics library

// Include the image data that you generated
#include "map_image.h"

// Define the dimensions of your map image
#define MAP_WIDTH  187
#define MAP_HEIGHT 180

TFT_eSPI tft = TFT_eSPI(); // Create an instance of the library

// THE FIX IS HERE: The file path has been removed from the line below
void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(1); // Adjust rotation as needed (0, 1, 2, 3)
  tft.fillScreen(TFT_BLACK);

  // Display a message
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(10, 10);
  tft.println("Loading map...");
  delay(1000);

  // Draw the map onto the screen at position (0, 0)
  // The pushImage function is very fast for this
  tft.pushImage(0, 0, MAP_WIDTH, MAP_HEIGHT, map_data);
  Serial.write("messy load on burak gumusoglu!");
}

void loop() {
  // The map is static, so nothing needs to be done in the loop.
  // You could add code here to draw a moving dot or other indicators on top.
}
