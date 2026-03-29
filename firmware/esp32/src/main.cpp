#include <Arduino.h>
#include <FastLED.h>

#define LED_PIN_LEFT 5
#define LED_PIN_RIGHT 18
#define NUM_LEDS 1024       // LEDs per pin
#define TOTAL_LEDS 2048     // Total LEDs (32 x 64)
#define MATRIX_WIDTH 32
#define MATRIX_HEIGHT 64
#define PANEL_SIZE 16
#define BAUD_RATE 460800

CRGB leds_left[NUM_LEDS];
CRGB leds_right[NUM_LEDS];

uint8_t packetBuffer[260];
uint16_t packetIndex = 0;
uint8_t currentPacketType = 0;

#define PKT_TYPE_PING 0x05
#define PKT_TYPE_LED_1BIT 0x03
#define PKT_TYPE_INFO 0x06
#define HEADER_1 0xAA
#define HEADER_2 0xBB

void processPing() {
  Serial.println("PONG");
  Serial.flush();
}

void processInfo() {
  Serial.println("MIRROR-LED-32x64");
  Serial.println("VERSION:2.0");
  Serial.println("PANELS:8");
  Serial.println("OK");
  Serial.flush();
}

// Convert (x, y) matrix coordinate to LED index for a specific pin
// Uses serpentine layout within 16x16 panels
// Physical wiring: Left screen column -> RIGHT pin, Right screen column -> LEFT pin
// Also mirror horizontally within each panel
uint16_t matrixToLedIndex(uint8_t x, uint8_t y, bool isLeftPin) {
  // Determine which panel column (0=left screen, 1=right screen)
  uint8_t panelCol = x / PANEL_SIZE;
  
  // SWAPPED: Left screen column (0) -> RIGHT pin, Right screen column (1) -> LEFT pin
  // isLeftPin=true means we want data for LEFT pin (which shows RIGHT screen column)
  // isLeftPin=false means we want data for RIGHT pin (which shows LEFT screen column)
  if ((panelCol == 0 && isLeftPin) || (panelCol == 1 && !isLeftPin)) {
    return 0xFFFF; // Invalid - this pixel belongs to other pin
  }
  
  // Local x within panel (0-15), then MIRROR it horizontally
  uint8_t localX = x % PANEL_SIZE;
  localX = (PANEL_SIZE - 1) - localX;  // Mirror: 0->15, 15->0
  
  // Determine which panel row (0-3 from top)
  uint8_t panelRow = y / PANEL_SIZE;
  
  // Local y within panel (0-15)  
  uint8_t localY = y % PANEL_SIZE;
  
  // Calculate LED index within the pin's LED array
  // Each panel has 256 LEDs, panels are wired top to bottom
  uint16_t panelOffset = panelRow * 256;  // Which panel (0, 256, 512, 768)
  
  // Within panel: serpentine layout (odd rows reversed)
  uint16_t pixelInPanel;
  if (localY & 1) {
    // Odd row: right to left
    pixelInPanel = localY * PANEL_SIZE + (PANEL_SIZE - 1 - localX);
  } else {
    // Even row: left to right
    pixelInPanel = localY * PANEL_SIZE + localX;
  }
  
  return panelOffset + pixelInPanel;
}

void processLED1BitPacket() {
  if (packetBuffer[2] != PKT_TYPE_LED_1BIT || packetIndex < 259) {
    return;
  }
  
  uint8_t *packed = &packetBuffer[3];
  
  // Clear all LEDs first
  fill_solid(leds_left, NUM_LEDS, CRGB::Black);
  fill_solid(leds_right, NUM_LEDS, CRGB::Black);
  
  // Process all 2048 pixels (32 x 64 matrix)
  // Packet format: row-major, MSB first
  // Byte 0 = pixels (0,0) to (7,0)
  // Byte 1 = pixels (8,0) to (15,0)
  // etc.
  
  for (uint16_t pixelIdx = 0; pixelIdx < TOTAL_LEDS; pixelIdx++) {
    uint16_t byteIdx = pixelIdx / 8;
    uint8_t bitIdx = 7 - (pixelIdx % 8);  // MSB first
    
    if (byteIdx >= 256) break;
    
    bool isOn = (packed[byteIdx] & (1 << bitIdx)) != 0;
    
    if (isOn) {
      // Convert linear index to x,y
      uint8_t x = pixelIdx % MATRIX_WIDTH;
      uint8_t y = pixelIdx / MATRIX_WIDTH;
      
      // Determine which pin and get LED index
      // SWAPPED: Left screen column -> RIGHT pin, Right screen column -> LEFT pin
      if (x < PANEL_SIZE) {
        // Left screen column -> RIGHT pin (isLeftPin=false)
        uint16_t ledIdx = matrixToLedIndex(x, y, false);
        if (ledIdx < NUM_LEDS) {
          leds_right[ledIdx] = CRGB(255, 255, 255);
        }
      } else {
        // Right screen column -> LEFT pin (isLeftPin=true)
        uint16_t ledIdx = matrixToLedIndex(x, y, true);
        if (ledIdx < NUM_LEDS) {
          leds_left[ledIdx] = CRGB(255, 255, 255);
        }
      }
    }
  }
  
  FastLED.show();
}

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(10);
  
  Serial.println("\n=== MINIMAL ESP32 TEST ===");
  Serial.println("WiFi: DISABLED");
  Serial.println("Servos: DISABLED");
  Serial.println("Only LEDs active");
  Serial.println("READY for PING...");
  
  FastLED.addLeds<WS2812B, LED_PIN_LEFT, GRB>(leds_left, NUM_LEDS);
  FastLED.addLeds<WS2812B, LED_PIN_RIGHT, GRB>(leds_right, NUM_LEDS);
  FastLED.setBrightness(255);
  
  delay(100);
  Serial.println("LEDs initialized");
}

void loop() {
  while (Serial.available() > 0) {
    uint8_t inByte = Serial.read();
    
    if (packetIndex == 0 && inByte == HEADER_1) {
      packetBuffer[packetIndex++] = inByte;
    } else if (packetIndex == 1) {
      if (inByte == HEADER_2) {
        packetBuffer[packetIndex++] = inByte;
      } else {
        packetIndex = 0;
        currentPacketType = 0;
      }
    } else if (packetIndex == 2) {
      if (inByte == PKT_TYPE_PING || inByte == PKT_TYPE_LED_1BIT || inByte == PKT_TYPE_INFO) {
        currentPacketType = inByte;
        packetBuffer[packetIndex++] = inByte;
        
        if (inByte == PKT_TYPE_PING) {
          processPing();
          packetIndex = 0;
          currentPacketType = 0;
        } else if (inByte == PKT_TYPE_INFO) {
          processInfo();
          packetIndex = 0;
          currentPacketType = 0;
        }
      } else {
        packetIndex = 0;
        currentPacketType = 0;
      }
    } else if (packetIndex > 2) {
      if (packetIndex >= 259) {
        if (currentPacketType == PKT_TYPE_LED_1BIT) {
          processLED1BitPacket();
        }
        packetIndex = 0;
        currentPacketType = 0;
      } else {
        packetBuffer[packetIndex++] = inByte;
      }
    }
  }
  
  delay(1);
}
