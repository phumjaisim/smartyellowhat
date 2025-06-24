#include <Wire.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

#include "esp_sleep.h"  // Deep sleep control (ESP32 only)

#define crashDetectionThreshold 2.4
bool debugMode = true;

#define SDA_PIN 21
#define SCL_PIN 22
#define BUZZER_PIN 4

#define NEOPIXEL_PIN 5
#define NEOPIXEL_NUM 1

static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;


MPU6050 mpu;
TinyGPSPlus gps;
Adafruit_NeoPixel pixels(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

bool crashDetected = false;
unsigned long crashTime = 0;
const unsigned long buzzerDuration = 2 * 60 * 1000UL;  // 2 minutes in milliseconds

SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(115200);


  // Wake-up log
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("Woke up from deep sleep. Sending ping...");
    // Send ping/log
    Serial.println("Ping: Device is alive.");
    delay(1000);
    goToSleep();  // Go back to sleep
  }

  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("Gyroscope: ON");
    Serial.println("Accelerometer: ON");
  } else {
    Serial.println("Gyroscope: FAILED");
    Serial.println("Accelerometer: FAILED");
    while (1);
  }

  ss.begin(GPSBaud);
  Serial.println("GPS: ON");
  
  pixels.begin();
  Serial.println("NEOPIXEL: ON");
  pixels.clear();

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);  // Ensure buzzer is off initially
}

void loop() {
  pixels.clear(); // Turn all off

  if (crashDetected) {
    if (millis() - crashTime < buzzerDuration) {
      while( true ) {
        digitalWrite(BUZZER_PIN, LOW);
        pixels.setPixelColor(0, pixels.Color(255, 0, 0));
        pixels.show();
        delay(500);
        digitalWrite(BUZZER_PIN, HIGH);
        pixels.setPixelColor(0, pixels.Color(0, 0, 0));
        pixels.show();
        delay(500);
      }

    } else {
      digitalWrite(BUZZER_PIN, LOW);
      Serial.println("Buzzer Off. Going to sleep...");
      delay(1000);
      goToSleep();  // Sleep for 30 minutes
    }
    return;  // Skip rest of loop
  }

  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  float axg = ax / 16384.0;
  float ayg = ay / 16384.0;
  float azg = az / 16384.0;

  float totalG = sqrt(axg * axg + ayg * ayg + azg * azg);
  float impact = abs(totalG - 1.0);  // Remove gravity offset

  if (debugMode) {
    Serial.printf("gF%.2f,", impact);
  }

  if (impact > crashDetectionThreshold) {
    Serial.println(" <-- CRASH DETECTED!");
    crashDetected = true;
    crashTime = millis();
  }

  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();

  Serial.println();
  delay(100);
}

void displayInfo() {
  if (debugMode) {
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    Serial.print(F(","));
  } else {
    Serial.print(F("noLatLong,"));
  }

  if (gps.date.isValid()) {
    if (gps.date.day() < 10) Serial.print('0');
    Serial.print(gps.date.day());
    if (gps.date.month() < 10) Serial.print('0');
    Serial.print(gps.date.month());
    Serial.print(gps.date.year() % 100);
    Serial.print(F(","));
  } else {
    Serial.print(F("noDDMMYY,"));
  }

  if (gps.time.isValid()) {
    int hour = gps.time.hour() + 7;
    int minute = gps.time.minute();
    int second = gps.time.second();
    if (hour >= 24) hour -= 24;
    if (hour < 10) Serial.print('0');
    Serial.print(hour);
    Serial.print(minute);
    Serial.print(second);
    Serial.print(F(","));
  } else {
    Serial.print(F("noHHMMSS,"));
  }

  if (gps.altitude.isValid()) {
    Serial.print(gps.altitude.meters());
  } else {
    Serial.print("noAlt,");
  }

  if (gps.satellites.isValid()) {
    Serial.print(gps.satellites.value());
  } else {
    Serial.print("noSatC");
  }

  pixels.setPixelColor(0, pixels.Color(255, 255, 0)); // Green
  pixels.show();

  Serial.println();
  }
}

void goToSleep() {
  digitalWrite(BUZZER_PIN, LOW);  // Ensure buzzer is off
  Serial.println("Entering deep sleep for 30 minutes...");
  esp_sleep_enable_timer_wakeup(30ULL * 60ULL * 1000000ULL);  // 30 minutes in microseconds
  esp_deep_sleep_start();
}
