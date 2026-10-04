#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <SPI.h>
#include <MFRC522.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

#include "HUSKYLENS.h"
#include "secrets.h"  // Wi-Fi, Telegram and RFID credentials (copy secrets.example.h)

// ============================================================================
// EXACT PIN DEFINITIONS & MASTER HARDWARE CONFIGURATION
// ============================================================================

// --- CREDENTIALS: defined in secrets.h (not committed) ---

// --- CONFIGURED STATION RFID TAG UIDs ---

// --- HUSKYLENS LEARNED APRILTAG IDs ---
const int HUSKY_TAG_EXAM_OFFICE = 1; // Learned Tag ID 1 on HuskyLens
const int HUSKY_TAG_HOD_OFFICE = 2; // Learned Tag ID 2 on HuskyLens

// --- L298N MOTOR DRIVER PINS ---
#define ENA 13
#define IN1 12
#define IN2 14
#define IN3 27
#define IN4 26
#define ENB 25

// --- PERIPHERAL PINS (matches physical wiring) ---
#define RFID_SS_PIN   5   // GPIO 5: RFID SDA
#define RFID_RST_PIN 22 // GPIO 22: RFID RST
#define SOLENOID_PIN 32 // GPIO 32: Relay Lock
#define BUZZER_PIN    4   // GPIO 4: Active Buzzer
#define TRIG_PIN      33 // GPIO 33: Ultrasonic Trig
#define ECHO_PIN      35 // GPIO 35: Ultrasonic Echo

// --- ENCODER HARDWARE INTERRUPT PINS ---
#define ENC_LEFT_A    34 // GPIO 34 (Input Only)
#define ENC_RIGHT_A   36 // GPIO 36 (VP - Input Only)

// --- ENCODER CONSTANTS (1:90 Gearbox x 12 PPR) ---
const float TICKS_PER_REV = 1080.0;

// ============================================================================
// CONTROL LOOP GAINS & CONSTANTS (SMOOTH WIDE-ARC TUNE)
// ============================================================================

// --- BASE SPEED & LIMITS (TARGET RPM DOMAIN) ---
float baseTargetRPM = 75.0;
float minTargetRPM = 40.0; // Higher inner wheel speed floor to widen curve radius
float maxTargetRPM = 110.0;

// --- OUTER PD GAINS (HuskyLens Vision -> Target RPM) ---
float Kp_vision = 0.30;     // Softened to prevent aggressive steering bite
float Kd_vision = 1.20;     // Dampened derivative response

// --- INNER PI GAINS (Encoders -> Dynamic PWM) ---
float Kp_wheel = 1.60;
float Ki_wheel = 0.45;

// --- OBSTACLE THRESHOLD ---
const int OBSTACLE_MAX_CM = 30;

// --- VOLATILE ENCODER COUNTERS ---
volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

// Inner Loop Timing and State
unsigned long lastWheelPIDTime = 0;
const unsigned long WHEEL_PID_INTERVAL = 50;

float currentLeftRPM = 0.0, currentRightRPM = 0.0;
float targetLeftRPM = 0.0, targetRightRPM = 0.0;
float leftErrorIntegral = 0.0, rightErrorIntegral = 0.0;
int leftPWM = 0, rightPWM = 0;

// Outer Loop Timing and State
int error = 0, lastError = 0;
float derivative = 0, pdOutput = 0;
const int SCREEN_CENTER_X = 160;

int lostLineCount = 0;
const int LOST_LINE_THRESHOLD = 10;

// ============================================================================
// SYSTEM STATE MACHINE
// ============================================================================
enum RobotMissionState {
   IDLE_AT_STATION,
   AWAITING_LOAD_SCAN,
   BOX_OPEN_LOADING,
   IN_TRANSIT,
   OBSTACLE_BLOCKED,
   VERIFYING_DESTINATION_TAG,
   ARRIVED_AT_DESTINATION,
   BOX_OPEN_RETRIEVAL
};

enum Station {
   STATION_NONE,
   STATION_HOD_OFFICE,
   STATION_EXAM_OFFICE
};

RobotMissionState currentMission = IDLE_AT_STATION;
Station currentStation = STATION_HOD_OFFICE;
Station targetDestination = STATION_NONE;
String expectedUnlockTag = "";
int expectedAprilTagID = 0;

// Hardware Instances
HUSKYLENS huskylens;
MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);
WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

// Non-blocking Timers
unsigned long lastHuskyRead = 0;
const unsigned long HUSKY_READ_INTERVAL = 30;

unsigned long lastTelegramCheck = 0;
const unsigned long TELEGRAM_INTERVAL = 1000;

unsigned long lastSonarCheck = 0;
const unsigned long SONAR_INTERVAL = 60;

unsigned long lastSirenToneToggle = 0;
bool sirenToneState = false;
unsigned long unlockStartTime = 0;
unsigned long tagVerificationStartTime = 0;
bool isUpdating = false;

// Function Declarations
void handleHuskyPD();
void handleWheelPID();
void handleTagVerification();
void handleRFID();
void handleTelegram();
void handleSonar();
void handleAudioEffects();
long readDistanceCm();
void motorControl(int lPWM, int rPWM);
void stopMotors();

void unlockSolenoid();
void lockSolenoid();

// ============================================================================
// HARDWARE INTERRUPT SERVICE ROUTINES (ISRs)
// ============================================================================
void IRAM_ATTR leftEncoderISR() {
  leftEncoderTicks++;
}

void IRAM_ATTR rightEncoderISR() {
  rightEncoderTicks++;
}

// ============================================================================
// WI-FI & OTA NETWORK
// ============================================================================

bool otaEnabled = false;
unsigned long lastWiFiRetry = 0;
const unsigned long WIFI_RETRY_INTERVAL = 10000;

void setupOTA();
void handleWiFiReconnect();

void setup() {
  Serial.begin(115200);
  delay(500);

    setupOTA();

    // Motor Driver Setup
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);
    stopMotors();

    // Relays, Buzzer & Ultrasonic Setup
    pinMode(SOLENOID_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    lockSolenoid();

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Encoder Hardware Interrupt Setup
    pinMode(ENC_LEFT_A, INPUT);
    pinMode(ENC_RIGHT_A, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderISR, RISING);

    // RFID Setup
    SPI.begin();
    rfid.PCD_Init();
    Serial.println("[SYSTEM] RC522 RFID Ready.");

    // HuskyLens Setup
    Serial2.begin(9600, SERIAL_8N1, 16, 17);
    huskylens.begin(Serial2);
    huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING);

    // Telegram Client Setup
    secured_client.setInsecure();

    Serial.println("[HERMES] Closed-Loop Hardware-Mapped System Ready!");
}

void loop() {
  if (otaEnabled) {
    ArduinoOTA.handle();
  } else {
    handleWiFiReconnect();
  }

    if (isUpdating) return;

  // 1. Telegram Polling
  if (currentMission != IN_TRANSIT && currentMission != OBSTACLE_BLOCKED && currentMission !=
VERIFYING_DESTINATION_TAG) {
    if (millis() - lastTelegramCheck > TELEGRAM_INTERVAL) {
      handleTelegram();

            lastTelegramCheck = millis();
        }
    }

    // 2. Obstacle Detection Scan
    if (currentMission == IN_TRANSIT || currentMission == OBSTACLE_BLOCKED) {
      handleSonar();
    }

    // 3. Cascaded Navigation & Closed-Loop Motor Control
    if (currentMission == IN_TRANSIT) {
      handleHuskyPD();   // Outer Loop: Determines Target RPMs
      handleWheelPID(); // Inner Loop: Real-time Closed-Loop Speed Execution
    } else {
      stopMotors();
    }

    // 4. AprilTag Verification Routine
    if (currentMission == VERIFYING_DESTINATION_TAG) {
      handleTagVerification();
    }

    // 5. RFID Card Authentication
    handleRFID();

    // 6. Audio Engine & Relock Timers
    handleAudioEffects();
}

// ============================================================================
// INNER PID: CLOSED-LOOP WHEEL SPEED REGULATOR (Runs at 20 Hz)
// ============================================================================
void handleWheelPID() {
  if (millis() - lastWheelPIDTime < WHEEL_PID_INTERVAL) return;
  unsigned long dt = millis() - lastWheelPIDTime;
  lastWheelPIDTime = millis();

    // Read atomic snapshot of encoder ticks
    noInterrupts();
    long currentLeftTicks = leftEncoderTicks;
    long currentRightTicks = rightEncoderTicks;
    leftEncoderTicks = 0;
    rightEncoderTicks = 0;
    interrupts();

    // Calculate actual RPM from tick count over dt
    currentLeftRPM = ((float)currentLeftTicks / TICKS_PER_REV) * (60000.0 / (float)dt);
    currentRightRPM = ((float)currentRightTicks / TICKS_PER_REV) * (60000.0 / (float)dt);

    // Left Wheel PI Math
    float leftError = targetLeftRPM - currentLeftRPM;
    leftErrorIntegral += leftError * ((float)dt / 1000.0);
    leftErrorIntegral = constrain(leftErrorIntegral, -60.0, 60.0);

    // Right Wheel PI Math
    float rightError = targetRightRPM - currentRightRPM;
    rightErrorIntegral += rightError * ((float)dt / 1000.0);
    rightErrorIntegral = constrain(rightErrorIntegral, -60.0, 60.0);

    // Compute feed-forward baseline + closed-loop feedback
    if (targetLeftRPM <= 0.0) {
      leftPWM = 0;
      leftErrorIntegral = 0;
    } else {
      leftPWM = (targetLeftRPM * 1.6) + (Kp_wheel * leftError) + (Ki_wheel * leftErrorIntegral);
    }

    if (targetRightRPM <= 0.0) {
      rightPWM = 0;
      rightErrorIntegral = 0;
    } else {
      rightPWM = (targetRightRPM * 1.6) + (Kp_wheel * rightError) + (Ki_wheel * rightErrorIntegral);
    }

    leftPWM = constrain(leftPWM, 0, 245);
    rightPWM = constrain(rightPWM, 0, 245);

    motorControl(leftPWM, rightPWM);
}

// ============================================================================
// OUTER PD: HUSKYLENS LINE TRACKING (Calculates Target Wheel RPMs)
// ============================================================================
void handleHuskyPD() {
  if (millis() - lastHuskyRead < HUSKY_READ_INTERVAL) return;
  lastHuskyRead = millis();

    bool validLineFound = false;

    if (huskylens.request() && huskylens.isLearned() && huskylens.available()) {
      while (huskylens.available()) {
        HUSKYLENSResult result = huskylens.read();

            if (result.command == COMMAND_RETURN_ARROW) {

                // Noise Filter 1: Bottom-Anchor Check
                if (result.yOrigin < 160) continue;

                // Noise Filter 2: Sudden-Jump Deflection Check
                if (abs((int)result.xTarget - (lastError + SCREEN_CENTER_X)) > 100 && lostLineCount == 0) {
                  continue;
                }

                validLineFound = true;
                lostLineCount = 0;

                // 1. Horizontal Tracking Error
                error = result.xTarget - SCREEN_CENTER_X;

                // 2. Derivative Component
                derivative = error - lastError;

                // 3. PD Steering Command (Target RPM offset)
                pdOutput = (Kp_vision * error) + (Kd_vision * derivative);
                lastError = error;

                // 4. Set Independent Target RPMs
                targetLeftRPM = baseTargetRPM + pdOutput;
                targetRightRPM = baseTargetRPM - pdOutput;

                // 5. Constrain within achievable RPM boundaries
                targetLeftRPM = constrain(targetLeftRPM, minTargetRPM, maxTargetRPM);
                targetRightRPM = constrain(targetRightRPM, minTargetRPM, maxTargetRPM);
                break;
            }
        }
    }

    // LINE END DETECTION: Triggers destination tag search
    if (!validLineFound) {
      lostLineCount++;

        if (lostLineCount >= LOST_LINE_THRESHOLD) {
          stopMotors();
          lostLineCount = 0;

            Serial.println("\n[LINE TERMINATED] Switching HuskyLens to Tag Recognition mode...");
            huskylens.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);
            currentMission = VERIFYING_DESTINATION_TAG;
            tagVerificationStartTime = millis();
        }
    }
}

// ============================================================================
// DYNAMIC APRILTAG VERIFICATION ROUTINE
// ============================================================================
void handleTagVerification() {
  if (millis() - lastHuskyRead < HUSKY_READ_INTERVAL) return;
  lastHuskyRead = millis();

    bool targetTagFound = false;

    if (huskylens.request() && huskylens.isLearned() && huskylens.available()) {
      while (huskylens.available()) {
        HUSKYLENSResult result = huskylens.read();

            if (result.command == COMMAND_RETURN_BLOCK && result.ID == expectedAprilTagID) {
              targetTagFound = true;
              break;
            }
        }
    }

    if (targetTagFound) {
      currentMission = ARRIVED_AT_DESTINATION;
      currentStation = targetDestination;

        Serial.printf("[DESTINATION VERIFIED] AprilTag ID %d Confirmed!\n", expectedAprilTagID);

    String destName = (targetDestination == STATION_EXAM_OFFICE) ? "Exam Office" : "HOD / General
Office";
    String alertMsg = "HERMES ARRIVAL ALERT!\n\n";
    alertMsg += "HERMES has verified the visual landmark at the " + destName + " doorway!\n";

     alertMsg += "Please scan your " + destName + " RFID Tag at the robot to unlock the cargo box.";

      bot.sendMessage(CHAT_ID_EXAM, alertMsg, "Markdown");
    }
    else if (millis() - tagVerificationStartTime > 3500) {
      Serial.println("[TAG TIMEOUT] Reverting to Line Tracking mode to attempt recovery...");
      huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING);
      currentMission = IN_TRANSIT;
    }
}

// ============================================================================
// ULTRASONIC OBSTACLE SCANNER (25cm - 30cm)
// ============================================================================
void handleSonar() {
  if (millis() - lastSonarCheck < SONAR_INTERVAL) return;
  lastSonarCheck = millis();

    long dist = readDistanceCm();

    if (dist > 2 && dist <= OBSTACLE_MAX_CM) {
      if (currentMission == IN_TRANSIT) {
        stopMotors();
        currentMission = OBSTACLE_BLOCKED;
        Serial.printf("[OBSTACLE] Distance: %ld cm | Emergency Braking...\n", dist);
      }
    }
    else if (currentMission == OBSTACLE_BLOCKED && (dist > OBSTACLE_MAX_CM || dist == 999)) {
      currentMission = IN_TRANSIT;
      Serial.println("[OBSTACLE CLEARED] Resuming line following.");
    }
}

long readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 25000);
    if (duration == 0) return 999;
    return (duration * 0.034 / 2);
}

// ============================================================================
// TELEGRAM DISPATCH PORTAL
// ============================================================================
void handleTelegram() {
  int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while (numNewMessages) {
      for (int i = 0; i < numNewMessages; i++) {
        String chat_id = String(bot.messages[i].chat_id);
        String text = bot.messages[i].text;

      if (text == "/start" || text == "/menu") {
        String welcome = "PROJECT HERMES DISPATCH PORTAL\n\n";
        welcome += "Current Location: " + String(currentStation == STATION_HOD_OFFICE ? "HOD / General
Office" : "Exam Office") + "\n";
        welcome += "Status: " + String(currentMission == IDLE_AT_STATION ? "Ready for Dispatch" : "Busy
/ In-Mission") + "\n\n";
        welcome += "Select delivery destination:";

          String keyboardJson = "[[\"/send_to_exam\", \"/send_to_hod\"], [\"/status\"]]";
          bot.sendMessageWithReplyKeyboard(chat_id, welcome, "Markdown", keyboardJson, true);
      }
      else if (text == "/status") {
        String stat = "HERMES Live Status:\n";
        stat += "- Mission State: " + String(currentMission) + "\n";
        stat += "- Location: " + String(currentStation == STATION_HOD_OFFICE ? "HOD Office" : "Exam
Office") + "\n";
        bot.sendMessage(chat_id, stat, "Markdown");
      }
      else if (text == "/send_to_exam") {
        if (currentMission != IDLE_AT_STATION) {
          bot.sendMessage(chat_id, "HERMES is currently engaged in a mission!", "Markdown");
        } else {
          targetDestination = STATION_EXAM_OFFICE;
          expectedUnlockTag = TAG_EXAM_OFFICE;
          expectedAprilTagID = HUSKY_TAG_EXAM_OFFICE;
          currentMission = AWAITING_LOAD_SCAN;

          String msg = "DISPATCH INITIATED: Destination -> Exam Office\n\n";
          msg += "Tap the HOD Office RFID Tag (" + TAG_HOD_OFFICE + ") on the robot to unlock the cargo
box and place the document.";
          bot.sendMessage(chat_id, msg, "Markdown");

          }
        }
        else if (text == "/send_to_hod") {
          if (currentMission != IDLE_AT_STATION) {
            bot.sendMessage(chat_id, "HERMES is currently engaged in a mission!", "Markdown");
          } else {
            targetDestination = STATION_HOD_OFFICE;
            expectedUnlockTag = TAG_HOD_OFFICE;
            expectedAprilTagID = HUSKY_TAG_HOD_OFFICE;
            currentMission = AWAITING_LOAD_SCAN;

          String msg = "DISPATCH INITIATED: Destination -> HOD / General Office\n\n";
          msg += "Tap the Exam Office RFID Tag (" + TAG_EXAM_OFFICE + ") on the robot to unlock the
cargo box and place the document.";
          bot.sendMessage(chat_id, msg, "Markdown");
        }
      }
    }
    numNewMessages = bot.getUpdates(bot.last_message_received + 1);
  }
}

// ============================================================================
// RFID AUTHENTICATION & SOLENOID UNLOCK
// ============================================================================
void handleRFID() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return;

    String cardUID = "";
    for (byte i = 0; i < rfid.uid.size; i++) {
      cardUID += String(rfid.uid.uidByte[i] < 0x10 ? "0" : "");
      cardUID += String(rfid.uid.uidByte[i], HEX);
    }
    cardUID.toUpperCase();
    Serial.println("\n[RFID] Scanned UID: " + cardUID);

    // STAGE 1: Sender Loading Document
    if (currentMission == AWAITING_LOAD_SCAN) {
      if (cardUID == TAG_HOD_OFFICE || cardUID == TAG_EXAM_OFFICE) {
        Serial.println("[AUTH] Sender verified. Unlocking Box...");
        unlockSolenoid();
        currentMission = BOX_OPEN_LOADING;
        unlockStartTime = millis();

        tone(BUZZER_PIN, 1800, 100);
        delay(120);
        tone(BUZZER_PIN, 2400, 200);

      bot.sendMessage(CHAT_ID_HOD, "Cargo Box Unlocked! Place document inside. Box will auto-lock in 7
seconds and begin transit.", "Markdown");
    } else {
      tone(BUZZER_PIN, 400, 500);
      Serial.println("[AUTH FAILED] Unrecognized Station Tag.");
    }
  }
  // STAGE 2: Recipient Retrieving Document
  else if (currentMission == ARRIVED_AT_DESTINATION) {
    if (cardUID == expectedUnlockTag) {
      Serial.println("[AUTH SUCCESS] Recipient authenticated! Releasing payload...");
      unlockSolenoid();
      currentMission = BOX_OPEN_RETRIEVAL;
      unlockStartTime = millis();

        tone(BUZZER_PIN, 1800, 100);
        delay(120);
        tone(BUZZER_PIN, 2400, 200);

      bot.sendMessage(CHAT_ID_EXAM, "Document Retrieved Successfully! Box will lock in 7s. HERMES is
docked for the next delivery.", "Markdown");
    } else {
      tone(BUZZER_PIN, 400, 600);
      Serial.println("[ACCESS DENIED] Unauthorized Tag for this delivery!");
    }
  }

    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
}

// ============================================================================
// AUDIO & RELOCK TIMER
// ============================================================================
void handleAudioEffects() {
  unsigned long currentMillis = millis();

    if (currentMission == IN_TRANSIT) {
      if (currentMillis - lastSirenToneToggle >= 300) {

       lastSirenToneToggle = currentMillis;
       sirenToneState = !sirenToneState;
       tone(BUZZER_PIN, sirenToneState ? 950 : 1350);
      }
    }
    else if (currentMission == OBSTACLE_BLOCKED) {
      if (currentMillis - lastSirenToneToggle >= 400) {
        lastSirenToneToggle = currentMillis;
        sirenToneState = !sirenToneState;
        if (sirenToneState) tone(BUZZER_PIN, 800);
        else noTone(BUZZER_PIN);
      }
    }
    else if (currentMission == ARRIVED_AT_DESTINATION) {
      if (currentMillis - lastSirenToneToggle >= 200) {
        lastSirenToneToggle = currentMillis;
        sirenToneState = !sirenToneState;
        if (sirenToneState) tone(BUZZER_PIN, 2200);
        else noTone(BUZZER_PIN);
      }
    }
    else {
      noTone(BUZZER_PIN);
    }

    // Auto-relock box after 7 seconds
    if (currentMission == BOX_OPEN_LOADING && (currentMillis - unlockStartTime >= 7000)) {
      lockSolenoid();
      currentMission = IN_TRANSIT;
      lostLineCount = 0;
      lastError = 0;
      huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING); // Ensure line mode active
      Serial.println("[TRANSIT] Cargo locked. Commencing line navigation...");
    }
    else if (currentMission == BOX_OPEN_RETRIEVAL && (currentMillis - unlockStartTime >= 7000)) {
      lockSolenoid();
      currentMission = IDLE_AT_STATION;
      huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING);
      Serial.println("[IDLE] Mission completed. Standing by for next dispatch.");
    }
}

// ============================================================================
// MOTOR & LOCK ACTUATION HELPERS
// ============================================================================
void motorControl(int leftPWM, int rightPWM) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, leftPWM);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightPWM);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void unlockSolenoid() {
  digitalWrite(SOLENOID_PIN, LOW); // Active-LOW relay
}

void lockSolenoid() {
  digitalWrite(SOLENOID_PIN, HIGH);
}

// ============================================================================
// OTA & BACKGROUND RECONNECT
// ============================================================================
void setupOTA() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

    int timeoutCounter = 0;
    while (WiFi.status() != WL_CONNECTED && timeoutCounter < 30) {
      delay(100);
      timeoutCounter++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      otaEnabled = true;

        ArduinoOTA.setHostname("Hermes-Robot");

        ArduinoOTA.onStart([]() {
          isUpdating = true;
          stopMotors();
          noTone(BUZZER_PIN);
          String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
          Serial.println("\n[OTA] Start updating " + type);
        });

        ArduinoOTA.onEnd([]() {
          isUpdating = false;
          Serial.println("\n[OTA] Update Complete!");
        });

        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
          Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
        });

        ArduinoOTA.onError([](ota_error_t error) {
          isUpdating = false;
          Serial.printf("[OTA] Error[%u]: ", error);
        });

        ArduinoOTA.begin();
        Serial.println("\n[Wi-Fi] Online! IP: " + WiFi.localIP().toString());
    }
}

void handleWiFiReconnect() {
  if (millis() - lastWiFiRetry >= WIFI_RETRY_INTERVAL) {
    lastWiFiRetry = millis();

        if (WiFi.status() == WL_CONNECTED) {
          ArduinoOTA.setHostname("Hermes-Robot");
          ArduinoOTA.begin();
          otaEnabled = true;
          Serial.println("\n[Wi-Fi] Reconnected in Background! OTA is ACTIVE.");
        } else {
          WiFi.begin(ssid, password);
        }
    }
}
