/*
======================================================================================
STRESS TEST FIRMWARE - ESP32S3
======================================================================================
Companion firmware for stress_test.py

Accepts serial commands to tune parameters at runtime:
  ALPHA:0.5    - Set servo smoothing alpha
  DELAY:1      - Set main loop delay (ms)
  I2C:400      - Set I2C clock speed (kHz)
  PWMFREQ:100  - Set PCA9685 PWM frequency (Hz)
  ACK:ON/OFF   - Enable/disable ACK after each servo packet
  RESET        - Reset packet counters

Reports stats every second:
  STATS|FPS:xx|PKTS:xx|ERRS:xx|LOOP_US:xx

Based on main.cpp production firmware but stripped of WiFi/OTA for raw speed.
======================================================================================
*/

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

// ==================== CONFIGURATION (tunable at runtime) ====================
uint32_t BAUD_RATE = 460800;
uint8_t LOOP_DELAY = 5;       // ms, tunable via DELAY:xx
float SMOOTH_ALPHA = 0.3;     // tunable via ALPHA:xx
uint16_t SERVO_PWM_FREQ = 50; // tunable via PWMFREQ:xx
uint32_t I2C_SPEED = 100000;  // tunable via I2C:xx (in Hz)
bool ACK_MODE = false;        // tunable via ACK:ON/OFF

// ==================== FIXED HARDWARE CONFIG ====================
#define NUM_SERVOS 64
#define PCA9685_ADDR_1 0x40
#define PCA9685_ADDR_2 0x41
#define PCA9685_ADDR_3 0x42
#define PCA9685_ADDR_4 0x43
#define OSC_FREQ 27000000

#define PWM_MIN 250
#define PWM_MAX 470

#define PACKET_SERVO_SIZE 131  // Header(3) + 64*2 bytes
#define PKT_TYPE_SERVO 0x02

// ==================== GLOBALS ====================
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(PCA9685_ADDR_1);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(PCA9685_ADDR_2);
Adafruit_PWMServoDriver pwm3 = Adafruit_PWMServoDriver(PCA9685_ADDR_3);
Adafruit_PWMServoDriver pwm4 = Adafruit_PWMServoDriver(PCA9685_ADDR_4);

float servoPositions[NUM_SERVOS];
float targetPositions[NUM_SERVOS];

uint8_t packetBuffer[PACKET_SERVO_SIZE + 16]; // extra padding for safety
uint16_t packetIndex = 0;

// Stats
unsigned long packetCount = 0;
unsigned long errorCount = 0;
unsigned long lastStatsTime = 0;
unsigned long lastPacketCount = 0;
unsigned long loopCount = 0;
unsigned long loopTimeSum = 0;
unsigned long lastLoopTime = 0;

// Command buffer
char cmdBuffer[64];
uint8_t cmdIndex = 0;

// ==================== SERVO CONTROL ====================
uint16_t angleToPWM(float angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  return map((long)angle, 0, 180, PWM_MIN, PWM_MAX);
}

void setServoAngle(uint8_t id, float angle) {
  if (id >= NUM_SERVOS) return;
  targetPositions[id] = angle;
  servoPositions[id] += SMOOTH_ALPHA * (targetPositions[id] - servoPositions[id]);
  
  uint16_t pwm_value = angleToPWM(servoPositions[id]);
  
  if (id < 16) {
    pwm1.setPWM(id, 0, pwm_value);
  } else if (id < 32) {
    pwm2.setPWM(id - 16, 0, pwm_value);
  } else if (id < 48) {
    pwm3.setPWM(id - 32, 0, pwm_value);
  } else {
    pwm4.setPWM(id - 48, 0, pwm_value);
  }
}

void centerAllServos() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    targetPositions[i] = 90.0;
    servoPositions[i] = 90.0;
    setServoAngle(i, 90.0);
  }
}

void initPCA9685() {
  Wire.begin(18, 19);
  Wire.setClock(I2C_SPEED);
  
  pwm1.begin();
  pwm1.setOscillatorFrequency(OSC_FREQ);
  pwm1.setPWMFreq(SERVO_PWM_FREQ);
  
  pwm2.begin();
  pwm2.setOscillatorFrequency(OSC_FREQ);
  pwm2.setPWMFreq(SERVO_PWM_FREQ);
  
  pwm3.begin();
  pwm3.setOscillatorFrequency(OSC_FREQ);
  pwm3.setPWMFreq(SERVO_PWM_FREQ);
  
  pwm4.begin();
  pwm4.setOscillatorFrequency(OSC_FREQ);
  pwm4.setPWMFreq(SERVO_PWM_FREQ);
}

// ==================== COMMAND PROCESSING ====================
void processCommand(const char* cmd) {
  if (strncmp(cmd, "ALPHA:", 6) == 0) {
    SMOOTH_ALPHA = atof(cmd + 6);
    if (SMOOTH_ALPHA < 0.01) SMOOTH_ALPHA = 0.01;
    if (SMOOTH_ALPHA > 1.0) SMOOTH_ALPHA = 1.0;
    Serial.printf("OK ALPHA=%.2f\n", SMOOTH_ALPHA);
    
  } else if (strncmp(cmd, "DELAY:", 6) == 0) {
    LOOP_DELAY = atoi(cmd + 6);
    if (LOOP_DELAY > 100) LOOP_DELAY = 100;
    Serial.printf("OK DELAY=%dms\n", LOOP_DELAY);
    
  } else if (strncmp(cmd, "I2C:", 4) == 0) {
    uint32_t speed_khz = atoi(cmd + 4);
    I2C_SPEED = speed_khz * 1000;
    Wire.setClock(I2C_SPEED);
    Serial.printf("OK I2C=%lukHz\n", speed_khz);
    
  } else if (strncmp(cmd, "PWMFREQ:", 8) == 0) {
    SERVO_PWM_FREQ = atoi(cmd + 8);
    if (SERVO_PWM_FREQ < 24) SERVO_PWM_FREQ = 24;
    if (SERVO_PWM_FREQ > 1526) SERVO_PWM_FREQ = 1526;
    pwm1.setPWMFreq(SERVO_PWM_FREQ);
    pwm2.setPWMFreq(SERVO_PWM_FREQ);
    pwm3.setPWMFreq(SERVO_PWM_FREQ);
    pwm4.setPWMFreq(SERVO_PWM_FREQ);
    Serial.printf("OK PWMFREQ=%dHz\n", SERVO_PWM_FREQ);
    
  } else if (strcmp(cmd, "ACK:ON") == 0) {
    ACK_MODE = true;
    Serial.println("OK ACK=ON");
    
  } else if (strcmp(cmd, "ACK:OFF") == 0) {
    ACK_MODE = false;
    Serial.println("OK ACK=OFF");
    
  } else if (strcmp(cmd, "RESET") == 0) {
    packetCount = 0;
    errorCount = 0;
    lastPacketCount = 0;
    loopCount = 0;
    loopTimeSum = 0;
    lastStatsTime = millis();
    Serial.println("OK RESET");
    
  } else if (strcmp(cmd, "STATUS") == 0) {
    Serial.printf("STATUS|ALPHA:%.2f|DELAY:%d|I2C:%lukHz|PWMFREQ:%d|ACK:%s\n",
                  SMOOTH_ALPHA, LOOP_DELAY, I2C_SPEED/1000, SERVO_PWM_FREQ,
                  ACK_MODE ? "ON" : "OFF");
  }
}

// ==================== SETUP ====================
void setup() {
  disableCore0WDT();
  
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(1);
  
  Serial.println("\n=== STRESS TEST FIRMWARE ===");
  Serial.println("Commands: ALPHA:x DELAY:x I2C:x PWMFREQ:x ACK:ON/OFF RESET STATUS");
  
  initPCA9685();
  centerAllServos();
  delay(300);
  
  Serial.println("READY - STRESS TEST MODE");
  lastStatsTime = millis();
}

// ==================== LOOP ====================
void loop() {
  unsigned long loopStart = micros();
  
  // --- Process serial data ---
  while (Serial.available() > 0) {
    uint8_t inByte = Serial.read();
    
    // Check for text commands (newline-terminated)
    if (inByte == '\n' || inByte == '\r') {
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0';
        processCommand(cmdBuffer);
        cmdIndex = 0;
      }
      continue;
    }
    
    // Binary packet parsing (same as production)
    if (packetIndex == 0 && inByte == 0xAA) {
      packetBuffer[packetIndex++] = inByte;
    } else if (packetIndex == 1) {
      if (inByte == 0xBB) {
        packetBuffer[packetIndex++] = inByte;
      } else {
        // Could be text command
        if (cmdIndex < sizeof(cmdBuffer) - 1) {
          // Replay the 0xAA as potential text
          cmdBuffer[cmdIndex++] = 0xAA;
          cmdBuffer[cmdIndex++] = inByte;
        }
        packetIndex = 0;
      }
    } else if (packetIndex == 2) {
      if (inByte == PKT_TYPE_SERVO) {
        packetBuffer[packetIndex++] = inByte;
      } else {
        // Not a servo packet, treat as text
        if (cmdIndex < sizeof(cmdBuffer) - 3) {
          cmdBuffer[cmdIndex++] = 0xAA;
          cmdBuffer[cmdIndex++] = 0xBB;
          cmdBuffer[cmdIndex++] = inByte;
        }
        packetIndex = 0;
      }
    } else if (packetIndex > 2) {
      if (packetIndex >= PACKET_SERVO_SIZE) {
        packetIndex = 0;
        errorCount++;
        continue;
      }
      
      packetBuffer[packetIndex++] = inByte;
      
      if (packetIndex >= PACKET_SERVO_SIZE) {
        // Complete servo packet!
        for (uint8_t i = 0; i < NUM_SERVOS; i++) {
          uint16_t value = (packetBuffer[3 + i * 2] << 8) | packetBuffer[4 + i * 2];
          if (value > 1000) value = 1000;
          float angle = map(value, 0, 1000, 0, 180);
          setServoAngle(i, angle);
        }
        
        packetCount++;
        packetIndex = 0;
        
        if (ACK_MODE) {
          Serial.println("ACK");
        }
      }
    } else {
      // Accumulate as text command
      if (cmdIndex < sizeof(cmdBuffer) - 1) {
        cmdBuffer[cmdIndex++] = inByte;
      }
      packetIndex = 0;
    }
  }
  
  // --- Smooth servo updates ---
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, targetPositions[i]);
  }
  
  // --- Stats reporting (every second) ---
  if (millis() - lastStatsTime >= 1000) {
    float fps = (packetCount - lastPacketCount) / ((millis() - lastStatsTime) / 1000.0);
    unsigned long avgLoopUs = loopCount > 0 ? loopTimeSum / loopCount : 0;
    
    Serial.printf("STATS|FPS:%.1f|PKTS:%lu|ERRS:%lu|LOOP_US:%lu\n",
                  fps, packetCount, errorCount, avgLoopUs);
    
    lastPacketCount = packetCount;
    lastStatsTime = millis();
    loopCount = 0;
    loopTimeSum = 0;
  }
  
  // --- Loop timing ---
  unsigned long loopTime = micros() - loopStart;
  loopTimeSum += loopTime;
  loopCount++;
  
  if (LOOP_DELAY > 0) {
    delay(LOOP_DELAY);
  }
}
