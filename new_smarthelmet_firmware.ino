#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "MAX30105.h"
#include "heartRate.h"

// === Pin Definitions ===
#define SDA_PIN 21
#define SCL_PIN 22
#define NEOPIXEL_PIN 13
#define NEOPIXEL_NUM 8
#define BUZZER_PIN 4
#define BUTTON_PIN 26
#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

// === Settings ===
#define CRASH_THRESHOLD 2.30// G-force threshold
#define HELMET_ID 1
#define INDICATOR_DURATION 2 * 60 * 1000UL  // 2 minutes

// === WiFi Credentials ===
const char* ssid = "Nothing Phone (2a)_5714";
const char* password = "45674567";

// === MQTT Settings ===
const char* mqtt_server = "ionlypfw.thddns.net";
const int mqtt_port = 2024;
const char* mqtt_topic = "data";
const char* mqtt_username = "smarthelmet";
const char* mqtt_password = "smarthelmet";
char mqttdata[128];

// === BPM Settings ===
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

// === Globals ===
MPU6050 mpu;
Adafruit_NeoPixel pixels(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
WiFiClient espClient;
PubSubClient client(espClient);
MAX30105 particleSensor;  

bool crashDetected = false;
bool debugMode = true;
unsigned long crashTime = 0;
double lastLat = 13.730159732699265, lastLng = 100.77988540128193;

// === WiFi Connection ===
void connectToWiFi() {    
  Serial.print("Connecting to WiFi");
  pixels.setPixelColor(0, pixels.Color(255, 0, 255));
  pixels.setPixelColor(7, pixels.Color(255, 0, 255));
  pixels.show();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");
}

// === MQTT Connection ===
void connectToMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32HelmetClient", mqtt_username, mqtt_password)) {
      Serial.println(" connected");
      client.subscribe("soschannel");  // Resubscribe after reconnect
    } else {
      Serial.printf(" failed, rc=%d. Retrying in 5s...\n", client.state());
      delay(5000);
    }
  }
}

// Global SOS Detection
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (String(topic) == "soschannel") {
    message.trim();
    if (message == "sos" || message == String(HELMET_ID)) {
      Serial.println("SOS or matching HAT_ID received!");
      crashDetected = true;
      crashTime = millis();
    }
  }
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPS_BAUD);

  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected");
  } else {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 connection failed");
    while (1);
  }
  Serial.println("MAX30105 connected");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  pixels.begin();
  pixels.clear();
  pixels.show();
  Serial.println("NeoPixel ready");

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("Buzzer ready");

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  connectToWiFi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  connectToMQTT();
  client.subscribe("soschannel");

}

// === Loop ===
void loop() {

  // === Average BPM Loop ===
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
  
  // === Startup Indicator (Yellow for first 3 seconds) ===
  if (millis() > 3000) {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.setPixelColor(7, pixels.Color(0, 0, 0));
  } else {
    pixels.setPixelColor(0, pixels.Color(255, 255, 0));
    pixels.setPixelColor(7, pixels.Color(255, 255, 0));
  }
  pixels.show();

  // === MQTT Keep-Alive ===
  if (!client.connected()) connectToMQTT();
  client.loop();

  // === Read GPS Data ===
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // === Read Accelerometer Data ===
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float axg = ax / 16384.0;
  float ayg = ay / 16384.0;
  float azg = az / 16384.0;
  float totalG = sqrt(axg * axg + ayg * ayg + azg * azg);
  float impact = abs(totalG - 1.0);
  float acceleration = impact * 9.81;

  // === Update GPS Location ===
  if (gps.location.isUpdated()) {
    lastLat = gps.location.lat();
    lastLng = gps.location.lng();
  }

  // === Debug Output ===
  if (debugMode) {
    Serial.printf("%d,%d,%.2f,%.2f,", HELMET_ID, crashDetected, impact, acceleration);
    if (lastLat != 0.0 && lastLng != 0.0) {
      Serial.printf("%.6f,%.6f,%.2f\n", lastLat, lastLng, (irValue < 50000 ? 0.0 : beatAvg));
    } else {
      Serial.printf("noLat,noLong,%d\n", (irValue < 50000 ? 0 : beatAvg));
    }
  }

  // === Compose MQTT Payload ===
  int sendBeatAvg = (irValue < 50000 ? 0 : beatAvg);

  if (lastLat != 0.0 && lastLng != 0.0) {
    snprintf(mqttdata, sizeof(mqttdata), "%d,%d,%.2f,%.2f,%.6f,%.6f,%d",
            HELMET_ID, crashDetected, impact, acceleration, lastLat, lastLng, sendBeatAvg);
  } else {
    snprintf(mqttdata, sizeof(mqttdata), "%d,%d,%.2f,%.2f,noLat,noLong,%d",
            HELMET_ID, crashDetected, impact, acceleration, sendBeatAvg);
  }

  client.publish(mqtt_topic, mqttdata);

  // === Crash Detection Logic ===
  if (!crashDetected && impact > CRASH_THRESHOLD) {
    Serial.println("CRASH DETECTED!");
    char helmetIdStr[4];
    sprintf(helmetIdStr, "%d", HELMET_ID);  // Convert int to string
    client.publish("soschannel", helmetIdStr);    crashDetected = true;
    crashTime = millis();
  }

  // === Manual SOS Button Press ===
  if (digitalRead(BUTTON_PIN) == LOW) {
    Serial.println("Physical SOS Button Triggered!");
    char helmetIdStr[4];
    sprintf(helmetIdStr, "%d", HELMET_ID);  // Convert int to string
    client.publish("soschannel", helmetIdStr);
    crashDetected = true;
    
    crashTime = millis();
  }

  // === Crash Alert (Blink Red + Buzzer) ===
  static unsigned long lastBlink = 0;
  static bool blinkState = false;

  if (crashDetected) {
    unsigned long startAlert = millis();
    while (millis() - startAlert < 5000) {
      // First phase: LEDs 0–3 red, buzzer OFF
      digitalWrite(BUZZER_PIN, LOW);
      for (int i = 0; i <= 3; i++) pixels.setPixelColor(i, pixels.Color(255, 0, 0));
      for (int i = 4; i <= 7; i++) pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      pixels.show();
      delay(500);

      // Second phase: LEDs 4–7 red, buzzer ON
      digitalWrite(BUZZER_PIN, HIGH);
      for (int i = 0; i <= 3; i++) pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      for (int i = 4; i <= 7; i++) pixels.setPixelColor(i, pixels.Color(255, 0, 0));
      pixels.show();
      delay(500);
    }

    // Turn everything off
    digitalWrite(BUZZER_PIN, LOW);
    pixels.clear();
    pixels.show();
  }
}
