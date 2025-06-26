#include <Wire.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>

#include "MAX30105.h"
#include "heartRate.h"
#include "esp_sleep.h"

#define crashDetectionThreshold 2 //G
bool debugMode = true;
const int GPSBaud = 9600;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

#define SDA_PIN 21
#define SCL_PIN 22
#define BUZZER_PIN 4
#define RXPin 16
#define TXPin 17

#define NEOPIXEL_PIN 5
#define NEOPIXEL_NUM 2

const char* ssid = "Nothing Phone (2a)_5714";
const char* password = "45674567";

const char* mqtt_server = "30dfaa13bf1c48e6a0e01501aab4c5ec.s1.eu.hivemq.cloud"; // Public HiveMQ broker
const int mqtt_port = 8883;
const char* mqtt_gforce = "gforce";
const char* mqtt_bpm = "bpm"
const char* mqtt_username = "hivemq.webclient.1750909080438";
const char* mqtt_password = "ZwS&jE0!*2qyRkgG81F.";

MPU6050 mpu;
MAX30105 particleSensor;
TinyGPSPlus gps;
Adafruit_NeoPixel pixels(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

#include <WiFiClientSecure.h>

WiFiClientSecure secureClient;
PubSubClient client(secureClient);


bool crashDetected = false;
unsigned long crashTime = 0;
const unsigned long buzzerDuration = 2 * 60 * 1000UL;  // 2 minutes in milliseconds

SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(115200);

  setupWiFi();
  client.setServer(mqtt_server, mqtt_port);


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
  Serial.println("BUZZER: ON");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("PULSESENSE: FAILED");
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.setPixelColor(1, pixels.Color(0, 0, 255));
    pixels.show();
    while (1);
  }
  Serial.println("PULSESENSE: ON");
  
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED 
  }

void loop() {

  if (!client.connected()) {
  reconnectMQTT();
  }
  client.loop();

  if (millis() > 30000) {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
    pixels.show();
  } else {
    pixels.setPixelColor(0, pixels.Color(255, 255, 0));
    pixels.show();
  }

  if (crashDetected) {
    if (millis() - crashTime < buzzerDuration) {
      while( true ) {
        digitalWrite(BUZZER_PIN, LOW);
        pixels.setPixelColor(0, pixels.Color(255, 0, 0));
        pixels.setPixelColor(1, pixels.Color(0, 0, 0));
        pixels.show();
        delay(500);
        digitalWrite(BUZZER_PIN, HIGH);
        pixels.setPixelColor(0, pixels.Color(0, 0, 0));
        pixels.setPixelColor(1, pixels.Color(0, 255, 0));
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
    Serial.print(millis());
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

  // Pulse Sensor //
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();
    
    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable
      
      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
      beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  if (debugMode) {
    Serial.print("ir");
    Serial.print(irValue);
    Serial.print(", bpm");
    Serial.print(beatsPerMinute);
    Serial.print(", avgbpm");
    Serial.print(beatAvg);
  if (irValue < 50000)
    Serial.print(",noBPM");
  }

  char payload[32];
  dtostrf(impact, 6, 2, payload);  // Convert float to char array
  client.publish(mqtt_gforce, payload);
  
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
    pixels.setPixelColor(0, pixels.Color(255, 92, 0));
    pixels.show();
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

  Serial.println();
  }
}

void goToSleep() {
  digitalWrite(BUZZER_PIN, LOW);  // Ensure buzzer is off
  Serial.println("Entering deep sleep for 30 minutes...");
  esp_sleep_enable_timer_wakeup(30ULL * 60ULL * 1000000ULL);  // 30 minutes in microseconds
  esp_deep_sleep_start();
}

void setupWiFi() {
  delay(10);
  Serial.println("Connecting to WiFi...");
  secureClient.setInsecure(); // Use this ONLY for testing; remove in production
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
