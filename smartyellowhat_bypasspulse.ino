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
const char* hatID = "HAT-002";

#define SDA_PIN 21
#define SCL_PIN 22
#define BUZZER_PIN 4
#define RXPin 16
#define TXPin 17
#define BUTTON_PIN 33

#define NEOPIXEL_PIN 5
#define NEOPIXEL_NUM 2

bool crashDetected = false;
unsigned long crashTime = 0;
const unsigned long buzzerDuration = 2 * 60 * 1000UL;  // 2 minutes in milliseconds

const byte RATE_SIZE = 4; // Change this for averaging window
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

const char* ssid = "Nothing Phone (2a)_5714";
const char* password = "45674567";

const char* mqtt_server = "30dfaa13bf1c48e6a0e01501aab4c5ec.s1.eu.hivemq.cloud"; // Public HiveMQ broker
const int mqtt_port = 8883;
const char* mqtt_gforce = "gforce";
const char* mqtt_bpm = "bpm";
const char* mqtt_username = "hivemq.webclient.1750909080438";
const char* mqtt_password = "ZwS&jE0!*2qyRkgG81F.";

MPU6050 mpu;
MAX30105 particleSensor;
TinyGPSPlus gps;
Adafruit_NeoPixel pixels(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

#include <WiFiClientSecure.h>

WiFiClientSecure secureClient;
PubSubClient client(secureClient);



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

  /*if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
  Serial.println("MAX30105 not found. Check wiring/power.");
  while (1);
  }
  
  particleSensor.setup(); // Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);  // Turn off Green LED
  Serial.println("MAX30105: ON");*/

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);  // Ensure buzzer is off initially
  Serial.println("BUZZER: ON");  

  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Use internal pull-up resistor

  client.setCallback(mqttCallback);

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

  char payload[32];
  dtostrf(impact, 6, 2, payload);  // Convert float to char array
  client.publish(mqtt_gforce, payload);

  /*char bpmPayload[8];
  dtostrf(beatsPerMinute, 4, 1, bpmPayload);  // Convert float BPM to string
  client.publish(mqtt_bpm, bpmPayload);*/

  long irValue = particleSensor.getIR();

// Try to detect a beat, even if IR is low
if (checkForBeat(irValue)) {
  long delta = millis() - lastBeat;
  lastBeat = millis();

  beatsPerMinute = 60 / (delta / 1000.0);

  if (beatsPerMinute < 255 && beatsPerMinute > 20) {
    rates[rateSpot++] = (byte)beatsPerMinute;
    rateSpot %= RATE_SIZE;

    beatAvg = 0;
    for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
    beatAvg /= RATE_SIZE;
  }
}

// Print every loop regardless of detection
if (debugMode) {
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.println(beatAvg);
}

// Publish BPM to MQTT
char bpmPayload[8];
dtostrf(beatAvg, 4, 1, bpmPayload);
client.publish(mqtt_bpm, bpmPayload);

if (digitalRead(BUTTON_PIN) == LOW) {  // Active LOW
  Serial.println("Physical SOS Button Triggered!");
  client.publish("sosbtn", "1");
  crashDetected = true;
  crashTime = millis();
}

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

    char latBuffer[16];
    char lngBuffer[16];
    dtostrf(gps.location.lat(), 9, 6, latBuffer);
    dtostrf(gps.location.lng(), 9, 6, lngBuffer);
  
    client.publish("lat", latBuffer);
    client.publish("long", lngBuffer);
  
  } else {
    Serial.print(F("noLatLong,"));
    pixels.setPixelColor(0, pixels.Color(255, 92, 0));
    pixels.show();
    client.publish("lat", 0);
    client.publish("long", 0);
  }

   if (gps.date.isValid()) {
      char dateBuffer[9];  // DD/MM/YY + null terminator
      snprintf(dateBuffer, sizeof(dateBuffer), "%02d/%02d/%02d",
               gps.date.day(),
               gps.date.month(),
               gps.date.year() % 100);
      Serial.print(dateBuffer);
      Serial.print(F(","));
      client.publish("date", dateBuffer);
    } else {
      Serial.print(F("noDDMMYY,"));
    }

    // Publish time to MQTT in HH:MM:SS format
    if (gps.time.isValid()) {
      int hour = gps.time.hour() + 7;  // timezone adjustment
      if (hour >= 24) hour -= 24;
      char timeBuffer[9];  // HH:MM:SS + null terminator
      snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d",
               hour,
               gps.time.minute(),
               gps.time.second());
      Serial.print(timeBuffer);
      Serial.print(F(","));
      client.publish("time", timeBuffer);
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
      client.subscribe("sosbtn");

      // Publish hat ID to "hatid"
      client.publish("hatid", hatID); 
      Serial.println("Published hat ID to topic 'hatid'");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}


void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  if (String(topic) == "sosbtn") {
    msg.trim();  // Remove any whitespace or newline characters
    if (msg == "1" || msg.equalsIgnoreCase("true")) {
      Serial.println("MQTT SOS Button Pressed!");
      crashDetected = true;
      crashTime = millis();
    }
  }
}
