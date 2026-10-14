
#define BLYNK_TEMPLATE_ID "TMPL6rEvCPmuQ"
#define BLYNK_TEMPLATE_NAME "Fogging Machine"
#define BLYNK_AUTH_TOKEN "DsRVFkdD6vSW9-nGrH3A8FT9xNg6FY1d"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// ===============================
// WIFI CONFIGURATION
// ===============================
char ssid[] = "mhisyamuddinh@unifi";
char pass[] = "Hisyam94";

// GPIO
#define LED_PIN  0
#define ECHO_PIN 5
#define TRIG_PIN 6

// Tank calibration (adjust for your tank)
const float TANK_EMPTY_CM = 20.0;
const float TANK_FULL_CM = 3.0;
const int LEVEL_SAMPLE_COUNT = 7;
const int LEVEL_MIN_VALID = 4;
const float LEVEL_CLUSTER_CM = 0.6f;
const float LEVEL_DEADBAND_CM = 0.2f;
float lastPublishedDistance = -1.0f;

BlynkTimer timer;

bool sendWaterLevel();
void sendSignalStrength();

// Publish actual ESP32 Wi-Fi RSSI (dBm) to Blynk V2.
void sendSignalStrength() {
  if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
    Blynk.virtualWrite(V2, WiFi.RSSI());
  }
}

// Blynk V0 controls LED on GPIO0
BLYNK_WRITE(V0) {
  int state = param.asInt();

  digitalWrite(LED_PIN, state ? HIGH : LOW);

  Serial.print("LED: ");
  Serial.println(state ? "ON" : "OFF");
}

// Restore LED state when Blynk reconnects
BLYNK_WRITE(V3) {
  if (param.asInt() == 1) {
    // Acknowledge only after publishing V1: 0 = success, -1 = sensor error.
    bool measured = sendWaterLevel();
    Blynk.virtualWrite(V3, measured ? 0 : -1);
  }
}

BLYNK_CONNECTED() {
  Serial.println("Blynk Connected");
  Blynk.syncVirtual(V0);
  sendSignalStrength();

  // Send fresh measurement after reconnect
  timer.setTimeout(1000L, []() {
    // Defined below
    sendWaterLevel();
  });
}

// Read HC-SR04
float readDistanceCM() {
  // Keep separate bursts from interfering, including consecutive refreshes.
  static unsigned long lastTrigger = 0;
  static bool triggered = false;
  unsigned long elapsed = millis() - lastTrigger;
  if (triggered && elapsed < 65UL) delay(65UL - elapsed);
  lastTrigger = millis();
  triggered = true;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration =
      pulseIn(ECHO_PIN, HIGH, 30000UL);

  if (duration == 0) {
    return -1.0f;
  }

  return duration * 0.0343f / 2.0f;
}

float readStableDistanceCM() {
  float samples[LEVEL_SAMPLE_COUNT];
  int count = 0;
  for (int i = 0; i < LEVEL_SAMPLE_COUNT; ++i) {
    float distance = readDistanceCM();
    // Reject missing echoes and reflections beyond the calibrated tank.
    if (distance >= 2.0f && distance <= TANK_EMPTY_CM + 2.0f) {
      int j = count;
      while (j > 0 && samples[j - 1] > distance) {
        samples[j] = samples[j - 1];
        --j;
      }
      samples[j] = distance;
      ++count;
    }
  }
  if (count < LEVEL_MIN_VALID) return -1.0f;
  float median = count % 2 ? samples[count / 2]
      : (samples[count / 2 - 1] + samples[count / 2]) / 2.0f;
  int consistent = 0;
  for (int i = 0; i < count; ++i) {
    if (fabsf(samples[i] - median) <= LEVEL_CLUSTER_CM) ++consistent;
  }
  if (consistent < LEVEL_MIN_VALID) return -1.0f;
  return median;
}

// Send water percentage to Blynk V1
bool sendWaterLevel() {
  float distance = readStableDistanceCM();

  if (distance < 0) {
    Serial.println("HC-SR04: Missing or unstable echoes; keeping previous level");
    return false;
  }
  // Compare against the published value so small real changes accumulate.
  if (lastPublishedDistance >= 0.0f &&
      fabsf(distance - lastPublishedDistance) < LEVEL_DEADBAND_CM) {
    distance = lastPublishedDistance;
  }

  float waterPercent =
      (TANK_EMPTY_CM - distance) * 100.0f /
      (TANK_EMPTY_CM - TANK_FULL_CM);

  waterPercent = constrain(waterPercent, 0.0f, 100.0f);

  Serial.print("Distance: ");
  Serial.print(distance, 1);
  Serial.print(" cm | Water Level: ");
  Serial.print(waterPercent, 1);
  Serial.println("%");

  if (Blynk.connected()) {
    Blynk.virtualWrite(V1, waterPercent);
    lastPublishedDistance = distance;
    Serial.println("Water level sent to Blynk V1");
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(TRIG_PIN, LOW);

  Serial.println("Fogging Machine Starting...");

  delay(2000);
  Serial.println("Starting WiFi...");
  
  WiFi.begin(ssid, pass);
  Blynk.config(BLYNK_AUTH_TOKEN);

  // Send measurement every 60 seconds
  timer.setInterval(60000L, []() { sendWaterLevel(); });
  timer.setInterval(10000L, sendSignalStrength);

  // Send first measurement immediately
  sendWaterLevel();
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!Blynk.connected()) {
      static unsigned long lastAttempt = 0;

      if (millis() - lastAttempt >= 5000) {
        lastAttempt = millis();
        Blynk.connect(1000);
      }
    } else {
      Blynk.run();
    }
  }

  timer.run();
}
