#include <Arduino.h>
/*
======================================================================================
MIRROR PROD ANIMATION - ESP32S3 UNIFIED CONTROLLER
======================================================================================
Combined body visualization (LEDs) + hand-controlled servos (mechanical mirror)

Hardware:
- ESP32S3 DevKit
- 32x64 WS2812B LED Matrix on GPIO 5 & 18 (Body visualization)
- PCA9685 + 6 Servos on I2C (Hand control)

Communication: 460800 baud
- LED Packet: 0xAA 0xBB 0x01 + 2048 bytes (body pose)
- Servo Packet: 0xAA 0xBB 0x02 + 12 bytes (hand angles)

Created: 2025-11-25
======================================================================================
*/

#include <Adafruit_PWMServoDriver.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <FastLED.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

// ==================== WIFI CONFIGURATION ====================
const char *ssid = "ACT_2563";
const char *password = "loki@1234";

// ==================== LED CONFIGURATION ====================
#define LED_PIN_LEFT 5
#define LED_PIN_RIGHT 18
#define LEDS_PER_PIN 1024
#define NUM_LEDS 2048

#define TOTAL_W 32
#define TOTAL_H 64
#define PANEL_W 16
#define PANEL_H 16

// ==================== SERVO CONFIGURATION ====================
#define NUM_SERVOS 64       // 64 servos (4x PCA9685 boards)
#define PCA9685_ADDR_1 0x40 // First PCA9685 (servos 0-15)
#define PCA9685_ADDR_2 0x41 // Second PCA9685 (servos 16-31)
#define PCA9685_ADDR_3 0x42 // Third PCA9685 (servos 32-47)
#define PCA9685_ADDR_4 0x43 // Fourth PCA9685 (servos 48-63)
#define SERVO_FREQ 50
#define OSC_FREQ 27000000

#define PWM_MIN 250
#define PWM_MAX 470
#define SMOOTH_ALPHA 1.0    // Optimized: 1.0 = instant response (no smoothing lag)

// ==================== SERIAL PROTOCOL ====================
#define BAUD_RATE 460800
#define PACKET_LED_SIZE 2051 // Header(2) + Type(1) + Data(2048)
#define PACKET_SERVO_SIZE 131 // Header(2) + Type(1) + ServoData(128) for 64 servos

#define PKT_TYPE_LED 0x01
#define PKT_TYPE_SERVO 0x02

// ==================== GLOBALS ====================
CRGB leds_left[LEDS_PER_PIN];
CRGB leds_right[LEDS_PER_PIN];

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(PCA9685_ADDR_1);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(PCA9685_ADDR_2);
Adafruit_PWMServoDriver pwm3 = Adafruit_PWMServoDriver(PCA9685_ADDR_3);
Adafruit_PWMServoDriver pwm4 = Adafruit_PWMServoDriver(PCA9685_ADDR_4);
float servoPositions[NUM_SERVOS];
float targetPositions[NUM_SERVOS];

uint8_t packetBuffer[PACKET_LED_SIZE];
uint16_t packetIndex = 0;
uint8_t currentPacketType = 0;

unsigned long lastLEDPacket = 0;
unsigned long lastServoPacket = 0;
unsigned long frameCount = 0;
unsigned long lastFPSUpdate = 0;
float currentFPS = 0.0;

bool wifiConnectedOnce = false;

// ==================== XY MAPPING ====================
inline uint16_t XY(uint8_t x, uint8_t y) {
  // Simple Linear Mapping for Troubleshooting
  // 32 LEDs wide.
  // Rows 0-31 (Indices 0-1023) -> First Pin (leds_left)
  // Rows 32-63 (Indices 1024-2047) -> Second Pin (leds_right)
  return (y * TOTAL_W) + x;
}

// ==================== SERVO CONTROL ====================
uint16_t angleToPWM(float angle) {
  if (angle < 0)
    angle = 0;
  if (angle > 180)
    angle = 180;
  return map((long)angle, 0, 180, PWM_MIN, PWM_MAX);
}

void setServoAngle(uint8_t id, float angle) {
  if (id >= NUM_SERVOS)
    return;
  targetPositions[id] = angle;
  servoPositions[id] +=
      SMOOTH_ALPHA * (targetPositions[id] - servoPositions[id]);

  uint16_t pwm_value = angleToPWM(servoPositions[id]);

  // Route to correct PCA9685 board (4 boards, 16 channels each)
  if (id < 16) {
    pwm1.setPWM(id, 0, pwm_value);      // Board 1: servos 0-15
  } else if (id < 32) {
    pwm2.setPWM(id - 16, 0, pwm_value); // Board 2: servos 16-31
  } else if (id < 48) {
    pwm3.setPWM(id - 32, 0, pwm_value); // Board 3: servos 32-47
  } else {
    pwm4.setPWM(id - 48, 0, pwm_value); // Board 4: servos 48-63
  }
}

void centerAllServos() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    targetPositions[i] = 90.0;
    servoPositions[i] = 90.0;
    setServoAngle(i, 90.0);
  }
}

// ==================== WIFI ====================
void startWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("WiFi: ");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
}

void setupOTA() {
  ArduinoOTA.setHostname("MirrorHybrid-ESP32");

  ArduinoOTA.onStart([]() {
    FastLED.clear();
    FastLED.show();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    uint16_t ledsLit = map(progress, 0, total, 0, NUM_LEDS);
    for (uint16_t i = 0; i < ledsLit; i++) {
      if (i < LEDS_PER_PIN) {
        leds_left[i] = CRGB::Blue;
      } else {
        leds_right[i - LEDS_PER_PIN] = CRGB::Blue;
      }
    }
    FastLED.show();
  });

  ArduinoOTA.onEnd([]() {
    FastLED.clear();
    FastLED.show();
  });

  ArduinoOTA.begin();
}

// ==================== PACKET PROCESSING ====================
void processLEDPacket() {
  if (packetBuffer[0] == 0xAA && packetBuffer[1] == 0xBB &&
      packetBuffer[2] == PKT_TYPE_LED) {
    uint8_t *data = &packetBuffer[3];

    for (uint8_t y = 0; y < TOTAL_H; y++) {
      for (uint8_t x = 0; x < TOTAL_W; x++) {
        uint16_t dataIdx = (y * TOTAL_W) + x;
        uint8_t value = data[dataIdx];
        uint16_t ledIdx = XY(x, y);

        if (value > 10) {
          if (ledIdx < LEDS_PER_PIN) {
            leds_left[ledIdx] = CRGB(value, value, value);
          } else {
            leds_right[ledIdx - LEDS_PER_PIN] = CRGB(value, value, value);
          }
        } else {
          if (ledIdx < LEDS_PER_PIN) {
            leds_left[ledIdx] = CRGB::Black;
          } else {
            leds_right[ledIdx - LEDS_PER_PIN] = CRGB::Black;
          }
        }
      }
    }

    FastLED.show();
    lastLEDPacket = millis();
    frameCount++;
  }
}

void processServoPacket() {
  if (packetBuffer[0] == 0xAA && packetBuffer[1] == 0xBB &&
      packetBuffer[2] == PKT_TYPE_SERVO) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      uint16_t value = (packetBuffer[3 + i * 2] << 8) | packetBuffer[4 + i * 2];
      float angle = map(value, 0, 1000, 0, 180);
      setServoAngle(i, angle);
    }
    lastServoPacket = millis();
  }
}

// ==================== SETUP ====================
void setup() {
  // CRITICAL: Disable watchdog timer to prevent crash during LED/Servo
  // processing. NOTE: disableCore1WDT() crashes on ESP32-S3 with USB CDC mode!
  disableCore0WDT();
  // disableCore1WDT();  // DISABLED - causes crash on ESP32-S3 USB mode

  Serial.begin(BAUD_RATE);
  Serial.setTimeout(1);
  Serial.println("\n=== MIRROR HYBRID - ESP32 ===");
  Serial.println("Body + Hand Tracking v1.0");
  Serial.println("Watchdog timer Core0 disabled for stability");

  // LEDs
  Serial.println("Init LEDs...");
  FastLED.addLeds<WS2812B, LED_PIN_LEFT, GRB>(leds_left, LEDS_PER_PIN);
  FastLED.addLeds<WS2812B, LED_PIN_RIGHT, GRB>(leds_right, LEDS_PER_PIN);
  FastLED.setBrightness(255);
  FastLED.setMaxRefreshRate(120);

  // LED test
  for (int i = 0; i < 2; i++) {
    fill_solid(leds_left, LEDS_PER_PIN, CRGB::Green);
    fill_solid(leds_right, LEDS_PER_PIN, CRGB::Green);
    FastLED.show();
    delay(100);
    FastLED.clear();
    FastLED.show();
    delay(100);
  }

  // Servos (4x PCA9685 for 64 servos)
  Serial.println("Init servos (64 channels, 4 boards)...");
  // Use custom I2C pins: SDA=GPIO18, SCL=GPIO19
  Wire.begin(18, 19);  // SDA=D18, SCL=D19
  Wire.setClock(400000); // Optimized: 400kHz Fast Mode

  // Initialize all 4 PCA9685 boards
  pwm1.begin();
  pwm1.setOscillatorFrequency(OSC_FREQ);
  pwm1.setPWMFreq(SERVO_FREQ);
  Serial.println("  PCA9685 #1 (0x40): OK");

  pwm2.begin();
  pwm2.setOscillatorFrequency(OSC_FREQ);
  pwm2.setPWMFreq(SERVO_FREQ);
  Serial.println("  PCA9685 #2 (0x41): OK");

  pwm3.begin();
  pwm3.setOscillatorFrequency(OSC_FREQ);
  pwm3.setPWMFreq(SERVO_FREQ);
  Serial.println("  PCA9685 #3 (0x42): OK");

  pwm4.begin();
  pwm4.setOscillatorFrequency(OSC_FREQ);
  pwm4.setPWMFreq(SERVO_FREQ);
  Serial.println("  PCA9685 #4 (0x43): OK");

  centerAllServos();
  delay(500);

  // Servo test - quick sweep (faster for 64 servos)
  Serial.println("Testing all 64 servos...");
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, 120);
    delay(20);  // Faster delay for 64 servos
  }
  delay(200);
  centerAllServos();

  // WiFi
  startWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    setupOTA();
    wifiConnectedOnce = true;
  }

  Serial.println("\nREADY! Waiting for data...");
  Serial.println("Packet Types:");
  Serial.println("  0x01 = Body/LED (2051 bytes)");
  Serial.println("  0x02 = Hand/Servo (67 bytes)");
  Serial.println();

  lastFPSUpdate = millis();
}

// ==================== LOOP ====================
void loop() {
  // OTA
  if (wifiConnectedOnce) {
    ArduinoOTA.handle();
  }

  // Serial packets
  // DEFENSIVE: Clear buffer if it gets too large (prevents crash from overflow)
  if (Serial.available() > 3000) {
    Serial.println("WARNING: Serial buffer overflow detected - clearing");
    while (Serial.available() > 0) {
      Serial.read();
    }
    packetIndex = 0;
    currentPacketType = 0;
  }

  while (Serial.available() > 0) {
    uint8_t inByte = Serial.read();

    if (packetIndex == 0 && inByte == 0xAA) {
      packetBuffer[packetIndex++] = inByte;
    } else if (packetIndex == 1) {
      // Expect second header byte
      if (inByte == 0xBB) {
        packetBuffer[packetIndex++] = inByte;
      } else {
        // Bad header, reset
        packetIndex = 0;
        currentPacketType = 0;
      }
    } else if (packetIndex == 2) {
      // Packet type
      if (inByte == PKT_TYPE_LED || inByte == PKT_TYPE_SERVO) {
        currentPacketType = inByte;
        packetBuffer[packetIndex++] = inByte;
      } else {
        // Unknown type, reset
        packetIndex = 0;
        currentPacketType = 0;
      }
    } else if (packetIndex > 2) {
      // Bounds guard to prevent buffer overrun on malformed packets
      uint16_t maxSize = (currentPacketType == PKT_TYPE_LED) ? PACKET_LED_SIZE
                         : (currentPacketType == PKT_TYPE_SERVO)
                             ? PACKET_SERVO_SIZE
                             : PACKET_LED_SIZE; // fallback largest
      if (packetIndex >= maxSize) {
        packetIndex = 0;
        currentPacketType = 0;
        continue;
      }

      packetBuffer[packetIndex++] = inByte;

      bool complete = false;
      if (currentPacketType == PKT_TYPE_LED && packetIndex >= PACKET_LED_SIZE) {
        complete = true;
      } else if (currentPacketType == PKT_TYPE_SERVO &&
                 packetIndex >= PACKET_SERVO_SIZE) {
        complete = true;
      }

      if (complete) {
        if (currentPacketType == PKT_TYPE_LED) {
          processLEDPacket();
        } else if (currentPacketType == PKT_TYPE_SERVO) {
          processServoPacket();
        }
        packetIndex = 0;
        currentPacketType = 0;
      }
    } else {
      packetIndex = 0;
      currentPacketType = 0;
    }
  }

  // Safety timeouts
  if (lastLEDPacket > 0 && millis() - lastLEDPacket > 2000) {
    FastLED.clear();
    FastLED.show();
    lastLEDPacket = 0;
  }

  if (lastServoPacket > 0 && millis() - lastServoPacket > 2000) {
    centerAllServos();
    lastServoPacket = 0;
  }

  // Smooth servo updates
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, targetPositions[i]);
  }

  // FPS
  if (millis() - lastFPSUpdate > 1000) {
    currentFPS = frameCount / ((millis() - lastFPSUpdate) / 1000.0);
    Serial.printf("FPS: %.1f | WiFi: %s\n", currentFPS,
                  WiFi.status() == WL_CONNECTED ? "OK" : "OFF");
    frameCount = 0;
    lastFPSUpdate = millis();
  }

  // No loop delay for maximum performance
}
