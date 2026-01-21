#include <SPI.h>
#include <SD.h>

// === MAP & HARDWARE CONFIGURATION ========================================
#define MAP_WIDTH 3744
#define MAP_HEIGHT 3600
#define BYTES_PER_PIXEL 2 // For RGB565 format

// The name of the binary file on the SD card
#define MAP_FILENAME "/map.bin"

// The GPIO pin for the SD card's Chip Select (CS).
// **CHANGE THIS TO MATCH YOUR WIRING!**
#define SD_CS_PIN 34
// =========================================================================

// Global File object. We open the file once and keep it open for fast access.
File mapFile;

/**
 * @brief Reads a rectangular region of the map from the SD card.
 * 
 * @param startX The top-left X coordinate of the region to read.
 * @param startY The top-left Y coordinate of the region to read.
 * @param regionWidth The width of the region to read.
 * @param regionHeight The height of the region to read.
 * @param buffer A pointer to a buffer large enough to hold the region data.
 * @return true if the read was successful, false otherwise.
 */
bool getMapRegion(int startX, int startY, int regionWidth, int regionHeight, uint8_t* buffer) {
  // --- Input Validation ---
  if (!mapFile) {
    Serial.println("Error: Map file is not open.");
    return false;
  }
  if (startX < 0 || startY < 0 || (startX + regionWidth) > MAP_WIDTH || (startY + regionHeight) > MAP_HEIGHT) {
    Serial.println("Error: Requested region is out of map bounds.");
    return false;
  }
  if (buffer == nullptr) {
    Serial.println("Error: Provided buffer is null.");
    return false;
  }

  size_t bytes_per_row_in_region = regionWidth * BYTES_PER_PIXEL;
  
  // The map data is not contiguous for a region, so we must read it row by row.
  for (int y = 0; y < regionHeight; y++) {
    // Calculate the absolute Y coordinate on the full map
    long currentY = startY + y;

    // Calculate the precise starting byte position in the 25.7MB file
    // This is the most critical calculation.
    size_t fileOffset = ((currentY * MAP_WIDTH) + startX) * BYTES_PER_PIXEL;
    
    // Seek to that position in the file
    if (!mapFile.seek(fileOffset)) {
      Serial.printf("Error: Failed to seek to offset %u\n", fileOffset);
      return false;
    }

    // Calculate where to put this row's data in our local buffer
    uint8_t* buffer_ptr = buffer + (y * bytes_per_row_in_region);

    // Read one full row of the region into the buffer
    size_t bytesRead = mapFile.read(buffer_ptr, bytes_per_row_in_region);

    if (bytesRead != bytes_per_row_in_region) {
      Serial.printf("Error: Read failed for row %d. Expected %d, got %d bytes.\n", y, bytes_per_row_in_region, bytesRead);
      return false;
    }
  }

  return true; // Success!
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- ESP32-S3 Robot Navigation Map Reader ---");
  
  // Initialize SD Card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("FATAL: SD Card Mount Failed!");
    while (1);
  }
  Serial.println("SD Card Initialized.");

  // Open the map file globally
  mapFile = SD.open(MAP_FILENAME, FILE_READ);
  if (!mapFile) {
    Serial.printf("FATAL: Failed to open '%s'. Check if file exists.\n", MAP_FILENAME);
    while (1);
  }

  // Verify file size to ensure we have the correct map
  long expectedSize = (long)MAP_WIDTH * (long)MAP_HEIGHT * BYTES_PER_PIXEL;
  long actualSize = mapFile.size();
  Serial.printf("Map File Size: %ld bytes (Expected: %ld bytes)\n", actualSize, expectedSize);

  if(actualSize != expectedSize) {
    Serial.println("FATAL: Map file size is incorrect!");
    while(1);
  }

  Serial.println("Map file is open and verified. Ready for navigation queries.");
}

void loop() {
  // --- SIMULATION of a robot querying the map ---
  Serial.println("\n-------------------------------------------");
  Serial.println("Simulating robot requesting local map data...");

  // Define the size of the robot's "view" (e.g., 32x32 pixels)
  const int VIEW_SIZE = 32;
  // Allocate a buffer for this view. PSRAM is great for this.
  uint8_t* localMapView = (uint8_t*) ps_malloc(VIEW_SIZE * VIEW_SIZE * BYTES_PER_PIXEL);

  if (localMapView == nullptr) {
    Serial.println("Failed to allocate memory for local map view!");
    delay(5000);
    return;
  }

  // --- Query 1: Center of the map ---
  int robotX = (MAP_WIDTH / 2) - (VIEW_SIZE / 2);
  int robotY = (MAP_HEIGHT / 2) - (VIEW_SIZE / 2);
  Serial.printf("Robot at (%d, %d). Requesting %dx%d region.\n", robotX, robotY, VIEW_SIZE, VIEW_SIZE);
  
  if (getMapRegion(robotX, robotY, VIEW_SIZE, VIEW_SIZE, localMapView)) {
    Serial.println("SUCCESS: Region data retrieved into buffer.");
    // Your navigation algorithm would now process the 'localMapView' buffer.
    // For example, let's check the first pixel of our retrieved tile.
    uint16_t* firstPixel = (uint16_t*)localMapView;
    Serial.printf("Value of first pixel in this region is: 0x%04X\n", firstPixel[0]);
  } else {
    Serial.println("FAILURE: Could not retrieve map region.");
  }

  // --- Query 2: A different location ---
  robotX = 100;
  robotY = 500;
  Serial.printf("\nRobot moved to (%d, %d). Requesting %dx%d region.\n", robotX, robotY, VIEW_SIZE, VIEW_SIZE);

  if (getMapRegion(robotX, robotY, VIEW_SIZE, VIEW_SIZE, localMapView)) {
      Serial.println("SUCCESS: Region data retrieved into buffer.");
  } else {
      Serial.println("FAILURE: Could not retrieve map region.");
  }

  // Clean up the buffer
  free(localMapView);

  Serial.println("-------------------------------------------");
  delay(10000); // Wait 10 seconds before the next simulated query
}