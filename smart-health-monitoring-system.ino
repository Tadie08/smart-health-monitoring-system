#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <MPU6050_light.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>

/************ WIFI CONFIG ************/
const char* WIFI_SSID = "Tadie";
const char* WIFI_PASSWORD = "Tadiwanashe";
const char* PI_IP_ADDRESS = "192.168.1.100"; // Update with your Pi's IP
const int PI_PORT = 5000;
const unsigned long WIFI_RETRY_INTERVAL = 10000;
const unsigned long PI_RECONNECT_INTERVAL = 15000;

/************ OLED CONFIG ************/
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/************ HARDWARE SETUP ************/
#define BUZZER_PIN D8
#define LED_MEDICINE_G D5
#define LED_MEDICINE_B D6
#define BUTTON_PIN D7

MAX30105 particleSensor;
MPU6050 mpu(Wire);
WiFiClient piClient;

/************ NETWORK VARIABLES ************/
unsigned long lastWifiAttempt = 0;
unsigned long lastPiConnectAttempt = 0;
bool wifiConnected = false;
bool piConnected = false;
String piAddress = PI_IP_ADDRESS;

/************ STARTUP STATUS VARIABLES ************/
bool startupPhase = true;
unsigned long startupStartTime = 0;
const unsigned long WIFI_TIMEOUT = 15000;      // 15 seconds
const unsigned long PI_TIMEOUT = 5000;         // 5 seconds
bool wifiStatusDisplayed = false;
bool piStatusDisplayed = false;
String wifiStatusMsg = "WiFi: --";
String piStatusMsg = "Pi: --";

/************ SENSOR VARIABLES ************/
#define SAMPLE_SIZE 50
uint32_t irBuffer[SAMPLE_SIZE];
uint32_t redBuffer[SAMPLE_SIZE];
int32_t spo2 = 0, heartRate = 0;
int8_t validSPO2 = 0, validHeartRate = 0;
int sampleIndex = 0;
bool vitalSignsReady = false;

enum PillType { NONE, MEDICINE_GREEN, MEDICINE_WHITE };
bool fallAlertActive = false;
bool pillAlertActive = false;
PillType currentPillType = NONE;

unsigned long lastPillTime = 0;
unsigned long pillInterval = 60000;
bool nextPillIsGreen = true;

unsigned long lastFallToggle = 0;
unsigned long lastPillToggle = 0;
unsigned long lastMPUUpdate = 0;
bool fallLedState = false;
bool pillLedState = false;
int pillFlashCount = 0;
const int PILL_MAX_FLASHES = 5;
const unsigned long MPU_UPDATE_INTERVAL = 50;

/************ SETUP ************/
void setup() {
  Serial.begin(115200);

  // Initialize hardware
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_MEDICINE_G, OUTPUT);
  pinMode(LED_MEDICINE_B, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(D2, D1);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found");
    while (1);
  }

  Wire.setClock(400000);
  mpu.begin();
  mpu.calcOffsets();

  Wire.setClock(100000);
  if (!particleSensor.begin(Wire)) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("MAX30102 Error");
    display.display();
    while (1);
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeIR(0x0A);

  lastPillTime = millis();
  
  // ** STARTUP CONNECTION PHASE **
  startupStartTime = millis();
  showStartupScreen("Starting...", "");
  
  // Start WiFi connection
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("\n=== STARTUP STATUS ===");
}

/************ MAIN LOOP ************/
void loop() {
  // Handle startup connection phase
  if (startupPhase) {
    handleStartupPhase();
    return; // Stay in startup phase until complete
  }
  
  // Normal operation
  manageWiFiConnection();
  managePiConnection();
  
  checkFallDetection();
  checkPillReminders();
  readVitalSensorsNonBlocking();
  handleAlerts();
  updateDisplay();
}

/************ STARTUP PHASE HANDLER ************/
void handleStartupPhase() {
  unsigned long elapsed = millis() - startupStartTime;
  
  // Check WiFi status (first 15 seconds)
  if (elapsed < WIFI_TIMEOUT && !wifiStatusDisplayed) {
    showStartupScreen("Connecting WiFi...", "");
    
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;
      wifiStatusMsg = "WiFi: OK";
      wifiStatusDisplayed = true;
      Serial.println("✓ WiFi Connected!");
      Serial.print("  IP: ");
      Serial.println(WiFi.localIP());
      delay(1500); // Show success briefly
    }
  }
  
  // If WiFi timeout reached and still not connected
  if (elapsed >= WIFI_TIMEOUT && !wifiStatusDisplayed) {
    wifiStatusMsg = "WiFi: FAIL";
    wifiStatusDisplayed = true;
    Serial.println("✗ WiFi Timeout");
    delay(2000);
  }
  
  // Check Pi connection (after WiFi is resolved, 5 second attempt)
  if (wifiStatusDisplayed && !piStatusDisplayed) {
    unsigned long piElapsed = elapsed - (wifiStatusDisplayed ? WIFI_TIMEOUT : 0);
    
    if (piElapsed < PI_TIMEOUT) {
      showStartupScreen(wifiStatusMsg, "Connecting Pi...");
      
      if (!piClient.connected()) {
        piClient.stop();
        piClient.connect(piAddress.c_str(), PI_PORT);
        delay(500); // Give it a moment
      }
      
      if (piClient.connected()) {
        piConnected = true;
        piStatusMsg = "Pi: OK";
        piStatusDisplayed = true;
        Serial.println("✓ Pi Connected!");
        sendMessageToPi("DEVICE_ONLINE");
        delay(1500);
      }
    }
    
    // If Pi timeout reached
    if (piElapsed >= PI_TIMEOUT && !piStatusDisplayed) {
      piStatusMsg = "Pi: FAIL";
      piStatusDisplayed = true;
      Serial.println("✗ Pi Timeout");
      delay(2000);
    }
  }
  
  // End startup phase and go to normal operation
  if (wifiStatusDisplayed && piStatusDisplayed) {
    startupPhase = false;
    Serial.println("=== SYSTEM RUNNING ===");
    Serial.println("Status: " + wifiStatusMsg + " | " + piStatusMsg);
    display.clearDisplay();
    display.display();
  }
}

void showStartupScreen(String line1, String line2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.println(line1);
  
  if (line2.length() > 0) {
    display.setCursor(0, 12);
    display.println(line2);
  }
  
  // Show spinner animation
  static int spinnerFrame = 0;
  const char* spinner = "|/-\\";
  display.setCursor(120, 24);
  display.print(spinner[spinnerFrame % 4]);
  spinnerFrame++;
  
  display.display();
}

/************ NETWORK MANAGEMENT ************/
void manageWiFiConnection() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiConnected) {
      wifiConnected = true;
      Serial.println("WiFi reconnected");
    }
    return;
  }

  if (wifiConnected) {
    wifiConnected = false;
    piConnected = false;
    piClient.stop();
    Serial.println("WiFi lost");
  }

  if (millis() - lastWifiAttempt >= WIFI_RETRY_INTERVAL) {
    lastWifiAttempt = millis();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

void managePiConnection() {
  if (!wifiConnected) {
    piConnected = false;
    piClient.stop();
    return;
  }

  if (piConnected && piClient.connected()) return;

  if (millis() - lastPiConnectAttempt >= PI_RECONNECT_INTERVAL) {
    lastPiConnectAttempt = millis();
    piClient.stop();
    piConnected = piClient.connect(piAddress.c_str(), PI_PORT);
    if (piConnected) {
      Serial.println("Pi reconnected");
      sendMessageToPi("DEVICE_ONLINE");
    }
  }
}

void sendMessageToPi(String message) {
  if (!piConnected || !piClient.connected()) {
    Serial.print("Drop (offline): ");
    Serial.println(message);
    return;
  }

  message += "\n";
  if (piClient.print(message) == 0) {
    piConnected = false;
    piClient.stop();
  }
}

/************ FALL DETECTION ************/
void checkFallDetection() {
  if (millis() - lastMPUUpdate < MPU_UPDATE_INTERVAL) return;
  lastMPUUpdate = millis();

  mpu.update();
  float a = sqrt(sq(mpu.getAccX()) + sq(mpu.getAccY()) + sq(mpu.getAccZ()));

  if (!fallAlertActive && a > 2.5) {
    fallAlertActive = true;
    lastFallToggle = millis();
    fallLedState = false;
    sendMessageToPi("FALL");
  }
}

/************ PILL REMINDER ************/
void checkPillReminders() {
  if (!pillAlertActive && !fallAlertActive) {
    if (millis() - lastPillTime > pillInterval) {
      pillAlertActive = true;
      currentPillType = nextPillIsGreen ? MEDICINE_GREEN : MEDICINE_WHITE;
      nextPillIsGreen = !nextPillIsGreen;
      pillFlashCount = 0;
      lastPillToggle = millis();
      pillLedState = false;
      
      String pillMsg = "PILL," + String(currentPillType == MEDICINE_GREEN ? "GREEN" : "WHITE");
      sendMessageToPi(pillMsg);
    }
  }
}

/************ ALERT HANDLING ************/
void handleAlerts() {
  if (fallAlertActive) handleFallAlert();
  if (pillAlertActive) handlePillAlert();
}

void handleFallAlert() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    stopFallAlert();
    return;
  }

  if (millis() - lastFallToggle >= 150) {
    lastFallToggle = millis();
    fallLedState = !fallLedState;

    digitalWrite(LED_MEDICINE_G, fallLedState);
    digitalWrite(LED_MEDICINE_B, fallLedState);

    if (fallLedState) tone(BUZZER_PIN, 2000);
    else noTone(BUZZER_PIN);
  }
}

void handlePillAlert() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    stopPillAlert();
    return;
  }

  if (millis() - lastPillToggle >= 2000) {
    lastPillToggle = millis();
    pillLedState = !pillLedState;

    int activeLed = (currentPillType == MEDICINE_GREEN) ? LED_MEDICINE_G : LED_MEDICINE_B;
    int inactiveLed = (currentPillType == MEDICINE_GREEN) ? LED_MEDICINE_B : LED_MEDICINE_G;

    digitalWrite(activeLed, pillLedState);
    digitalWrite(inactiveLed, LOW);

    if (pillLedState) {
      tone(BUZZER_PIN, 1500);
      pillFlashCount++;
    } else {
      noTone(BUZZER_PIN);
    }

    if (pillFlashCount >= PILL_MAX_FLASHES) {
      stopPillAlert();
    }
  }
}

void stopFallAlert() {
  fallAlertActive = false;
  noTone(BUZZER_PIN);
  digitalWrite(LED_MEDICINE_G, LOW);
  digitalWrite(LED_MEDICINE_B, LOW);
  Serial.println("FALL detected");
  sendMessageToPi("FALL_CLEARED");
}

void stopPillAlert() {
  pillAlertActive = false;
  currentPillType = NONE;
  pillFlashCount = 0;
  pillLedState = false;
  noTone(BUZZER_PIN);
  digitalWrite(LED_MEDICINE_G, LOW);
  digitalWrite(LED_MEDICINE_B, LOW);
  lastPillTime = millis();
  Serial.println("PILL taken");
  sendMessageToPi("PILL_TAKEN");
}

/************ VITAL SIGNS ************/
void readVitalSensorsNonBlocking() {
  particleSensor.check();

  if (particleSensor.available()) {
    redBuffer[sampleIndex] = particleSensor.getRed();
    irBuffer[sampleIndex] = particleSensor.getIR();
    particleSensor.nextSample();
    sampleIndex++;

    if (sampleIndex >= SAMPLE_SIZE) {
      maxim_heart_rate_and_oxygen_saturation(irBuffer, SAMPLE_SIZE, redBuffer,
        &spo2, &validSPO2, &heartRate, &validHeartRate);
      sampleIndex = 0;
      vitalSignsReady = validHeartRate && validSPO2;
      
      if (vitalSignsReady) {
        String vitalsMsg = "VITALS," + String(heartRate) + "," + String(spo2);
        sendMessageToPi(vitalsMsg);
      }
    }
  }
}

/************ OLED DISPLAY ************/
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);

  // Connection indicators (top-right)
  display.setCursor(100, 0);
  display.print(wifiConnected ? "W" : "X");
  
  if (piConnected) {
    display.setCursor(108, 0);
    display.print("P");
  }

  // Main content
  if (!fallAlertActive && !pillAlertActive) {
    display.setCursor(0, 0);
    display.print("HR: ");
    if (vitalSignsReady) {
      display.print(heartRate);
      display.print(" bpm");
      display.setCursor(0, 12);
      display.print("SpO2: ");
      display.print(spo2);
      display.print("%");
    } else {
      display.print("--");
      display.setCursor(0, 12);
      display.print("SpO2: --%");
    }
    display.setCursor(0, 24);
    display.print("Stay Healthy");
  }
  else if (fallAlertActive) {
    display.setCursor(0, 0);
    display.println("!!! FALL !!!");
    display.setCursor(0, 12);
    display.println("DETECTED");
    display.setCursor(0, 24);
    display.println("Press Button");
  }
  else if (pillAlertActive) {
    display.setCursor(0, 0);
    display.println("TAKE MEDICINE");
    display.setCursor(0, 12);
    display.println(currentPillType == MEDICINE_GREEN ? "Medicine Avaliable" : "Medicine Not Take?");
  }

  display.display();
}