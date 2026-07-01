#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// -------- WIFI --------
const char* ssid = "WIFI_SSID";
const char* password = "WIFI_PASSWORD";
const char* mqtt_server = "10.176.164.51";

WiFiClient espClient;
PubSubClient client(espClient);

// -------- PINS --------
#define ZMPT_PIN   34
#define ACS_L1_PIN 32
#define ACS_L2_PIN 33

#define RELAY_L1 26
#define RELAY_L2 25

#define LED_GREEN    14
#define LED_FAULT_L1 27
#define LED_FAULT_L2 23
#define BUZZER       18

#define BTN_RESET 13
#define BTN_MODE  19

// -------- OLED (SH1106) --------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// -------- THRESHOLDS --------
const float OC_TRIP  = 1.0;
const float OV_TRIP = 280.0;
const float UV_TRIP = 140.0;
const float MAINS_DEAD = 20.0;

// -------- GLOBAL (Volatile for Dual-Core Safety) --------
volatile bool fault_L1 = false;
volatile bool fault_L2 = false;
volatile bool fault_mains = false;

volatile bool system_tripped = false;

volatile bool reset_active = false;
volatile unsigned long reset_time = 0;

int display_mode = 0;

// -------- MQTT CONTROL --------
volatile bool relay1_cmd = true;
volatile bool relay2_cmd = true;

// -------- MQTT TELEMETRY --------
volatile float tele_V = 0.0;
volatile float tele_I1 = 0.0;
volatile float tele_I2 = 0.0;
volatile float tele_P1 = 0.0;
volatile float tele_P2 = 0.0;

// ENERGY
volatile float energy_kWh = 0;
unsigned long last_energy_time = 0;

// OFFSET
float offset_I1 = 0;
float offset_I2 = 0;

// SMOOTHING
float smoothP = 0;

// FreeRTOS Task Handle
TaskHandle_t NetworkTaskHandle;

// -------- MQTT CALLBACK (Memory Safe) --------
void callback(char* topic, byte* payload, unsigned int length) {
  // Use a static C-string buffer instead of the dynamic String object
  char msg[length + 1];
  memcpy(msg, payload, length);
  msg[length] = '\0';

  Serial.print("MQTT -> ");
  Serial.print(topic);
  Serial.print(" : ");
  Serial.println(msg);

  if (system_tripped) return;

  // Use strcmp for memory-safe comparisons
  if (strcmp(topic, "home/control/relay1") == 0) {
    relay1_cmd = (msg[0] == '1');
  }
  else if (strcmp(topic, "home/control/relay2") == 0) {
    relay2_cmd = (msg[0] == '1');
  }
  else if (strcmp(topic, "home/control/reset") == 0) {
    system_tripped = false;
    fault_L1 = fault_L2 = fault_mains = false;
    reset_active = true;
    reset_time = millis();
  }
}

// -------- MQTT RECONNECT --------
void reconnect() {
  while (!client.connected()) {
    Serial.println("MQTT connecting...");
    if (client.connect("ESP32Client")) {
      Serial.println("MQTT connected");

      client.subscribe("home/control/relay1");
      client.subscribe("home/control/relay2");
      client.subscribe("home/control/reset");
    } else {
      delay(1000); 
    }
  }
}

// -------- FREERTOS NETWORK TASK (Runs on Core 0) --------
void networkTaskCode(void * pvParameters) {
  WiFi.begin(ssid, password);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  unsigned long last_mqtt_publish = 0;
  char pubBuf[20]; // Buffer for safe float-to-string conversion

  for(;;) {
    if (WiFi.status() == WL_CONNECTED) {
      if (!client.connected()) {
        reconnect(); 
      }
      client.loop();

      // --- PUBLISH TELEMETRY SAFELY ---
      if (millis() - last_mqtt_publish > 2000) {
        last_mqtt_publish = millis();
        
        // snprintf converts floats to strings without causing heap fragmentation
        snprintf(pubBuf, sizeof(pubBuf), "%.2f", (float)tele_V);
        client.publish("home/mains/V", pubBuf);

        snprintf(pubBuf, sizeof(pubBuf), "%.2f", (float)tele_I1);
        client.publish("home/load1/I", pubBuf);

        snprintf(pubBuf, sizeof(pubBuf), "%.2f", (float)tele_I2);
        client.publish("home/load2/I", pubBuf);

        snprintf(pubBuf, sizeof(pubBuf), "%.2f", (float)tele_P1);
        client.publish("home/load1/P", pubBuf);

        snprintf(pubBuf, sizeof(pubBuf), "%.2f", (float)tele_P2);
        client.publish("home/load2/P", pubBuf);

        snprintf(pubBuf, sizeof(pubBuf), "%.4f", (float)energy_kWh);
        client.publish("home/energy/total", pubBuf);

        if (system_tripped) {
          if (fault_mains) client.publish("home/fault/alert", "MAINS FAULT");
          else if (fault_L1) client.publish("home/fault/alert", "L1 OVER CURRENT");
          else if (fault_L2) client.publish("home/fault/alert", "L2 OVER CURRENT");
        } else {
          client.publish("home/fault/alert", "OK");
        }
      }

    } else {
      delay(500); 
    }
    delay(10); 
  }
}

// -------- RMS --------
float getTrueRMS(int pin, float factor) {
  uint32_t start = millis();
  uint64_t sum_sq = 0;
  uint32_t sum = 0, samples = 0;

  while (millis() - start < 70) {
    uint32_t val = analogRead(pin);
    sum += val;
    sum_sq += val * val;
    samples++;
  }

  if (samples == 0) return 0;

  float mean = (float)sum / samples;
  float var = ((float)sum_sq / samples) - (mean * mean);
  if (var < 0) var = 0;

  return sqrt(var) * factor;
}

// -------- CALIBRATION --------
void calibrateACS() {
  for (int i = 0; i < 50; i++) {
    offset_I1 += getTrueRMS(ACS_L1_PIN, 0.025);
    offset_I2 += getTrueRMS(ACS_L2_PIN, 0.025);
    delay(20);
  }
  offset_I1 /= 50;
  offset_I2 /= 50;
}

// -------- BUTTON --------
bool buttonPressed(int pin) {
  static uint32_t lastPressTime[2] = {0,0};
  int idx = (pin == BTN_RESET) ? 0 : 1;

  if (digitalRead(pin) == LOW && millis() - lastPressTime[idx] > 250) {
    lastPressTime[idx] = millis();
    return true;
  }
  return false;
}

// -------- SETUP (Runs on Core 1) --------
void setup() {
  Serial.begin(115200);

  pinMode(RELAY_L1, OUTPUT);
  pinMode(RELAY_L2, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_FAULT_L1, OUTPUT);
  pinMode(LED_FAULT_L2, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  pinMode(BTN_RESET, INPUT_PULLUP);
  pinMode(BTN_MODE, INPUT_PULLUP);

  digitalWrite(RELAY_L1, HIGH);
  digitalWrite(RELAY_L2, HIGH);

  Wire.begin(21, 22);

  // OLED INIT
  if (!display.begin(0x3C, true)) {
    Serial.println("SH1106 FAIL");
    while (1);
  }

  // --- INVERTED SPLASH SCREEN ---
  display.clearDisplay();
  display.fillScreen(SH110X_WHITE);     // Light up the background
  display.setTextColor(SH110X_BLACK);   // Set text to black

  display.setTextSize(3);
  display.setCursor(19, 10);            // Centered roughly
  display.println("SHEMS");

  display.setTextSize(1);
  display.setCursor(13, 40);
  display.println("Smart Home Energy");
  display.setCursor(13, 50);
  display.println("Management System");
  
  display.display();
  // ------------------------------

  xTaskCreatePinnedToCore(
    networkTaskCode,   
    "NetworkTask",     
    10000,             
    NULL,              
    1,                 
    &NetworkTaskHandle,
    0);                

  // This will run while the splash screen is visible
  calibrateACS();
  last_energy_time = millis();

  // Revert back to normal text color for the main loop
  display.setTextColor(SH110X_WHITE);
}

// -------- LOOP (Runs on Core 1) --------
void loop() {

  if (buttonPressed(BTN_RESET)) {
    system_tripped = false;
    fault_L1 = fault_L2 = fault_mains = false;
    reset_active = true;
    reset_time = millis();
  }

  if (buttonPressed(BTN_MODE)) {
    display_mode = (display_mode + 1) % 3;
  }

  float V = getTrueRMS(ZMPT_PIN, 0.386);
  float I1 = getTrueRMS(ACS_L1_PIN, 0.025) - offset_I1;
  float I2 = getTrueRMS(ACS_L2_PIN, 0.025) - offset_I2;

  if (I1 < 0.05) I1 = 0;
  if (I2 < 0.05) I2 = 0;
  if (V < 10) V = 0;

  float rawP = V * (I1 + I2);
  smoothP = 0.8 * smoothP + 0.2 * rawP;

  float P1 = V * I1;
  float P2 = V * I2;

  unsigned long now = millis();
  float dt = (now - last_energy_time) / 3600000.0;
  last_energy_time = now;
  energy_kWh += (smoothP / 1000.0) * dt;

  // PASS VALUES TO TELEMETRY GLOBALS
  tele_V = V;
  tele_I1 = I1;
  tele_I2 = I2;
  tele_P1 = P1;
  tele_P2 = P2;

  // FAULT
  // -------- UPDATED FAULT LOGIC --------
  if (!(reset_active && millis() - reset_time < 2000)) {
    // Independent Overcurrent Detection
    if (I1 > OC_TRIP) fault_L1 = true;
    if (I2 > OC_TRIP) fault_L2 = true;
    
    // Mains issues still trip the whole system
    if (V < MAINS_DEAD || V > OV_TRIP || V < UV_TRIP) {
        fault_mains = true;
        system_tripped = true;
    }
  }

  // OUTPUT
  // -------- UPDATED OUTPUT LOGIC --------
  if (system_tripped || fault_mains) {
    // Total Shutdown for Mains Faults
    digitalWrite(RELAY_L1, LOW);
    digitalWrite(RELAY_L2, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_FAULT_L1, HIGH);
    digitalWrite(LED_FAULT_L2, HIGH);
    tone(BUZZER, 2000);
  } else {
    // Independent Control: Relay is ON only if (Commanded by MQTT) AND (No individual fault)
    digitalWrite(RELAY_L1, (relay1_cmd && !fault_L1) ? HIGH : LOW);
    digitalWrite(RELAY_L2, (relay2_cmd && !fault_L2) ? HIGH : LOW);

    // Indicators
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_FAULT_L1, fault_L1 ? HIGH : LOW);
    digitalWrite(LED_FAULT_L2, fault_L2 ? HIGH : LOW);

    // Sound buzzer if ANY individual load is tripped
    if (fault_L1 || fault_L2) {
        tone(BUZZER, 1000); // Lower pitch for individual trip
    } else {
        noTone(BUZZER);
    }
  }

  // -------- OLED DISPLAY --------
  // -------- UPDATED OLED DISPLAY SECTION --------
  display.clearDisplay(); 
  display.setCursor(0,0);

  // Check if ANY fault exists (Mains, L1, or L2)
  if (system_tripped || fault_L1 || fault_L2) {

    display.setTextSize(2);
    display.println("FAULT!");
    display.setTextSize(1);

    if (fault_mains) {
        display.println("MAINS FAILURE");
    } 
    
    // Use individual IFs so both can show if both trip
    if (fault_L1) {
        display.println("L1 Over Current");
    }
    if (fault_L2) {
        display.println("L2 Over Current");
    }

    display.println("\nPress RESET");

  } else {

    display.setTextSize(1);

    if (display_mode == 0) {
      display.setTextSize(1);
      display.println("Mode 1 ------------- \n");
      display.print("Mains: "); display.print(V); display.println(" V\n");
      display.print("Current L1: "); display.print(I1); display.println(" A\n");
      display.print("Current L2: "); display.print(I2); display.println(" A\n");
    }
    else if (display_mode == 1) {
      display.setTextSize(1);
      display.println("Mode 2 -------------\n");
      display.print("Power L1: "); display.print(P1); display.println(" W\n");
      display.print("Power L2: "); display.print(P2); display.println(" W\n");
    }
    else {
      display.setTextSize(1);
      display.println("Mode 3 -------------\n");
      display.println("Energy Consumption: \n");
      display.print(energy_kWh, 4); display.println(" Units (kWh)\n");
    }
  }

  display.display();

  delay(50);
}