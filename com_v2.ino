#include <TinyGPSPlus.h>
#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <SD.h>
#include "DFRobotDFPlayerMini.h"
#include "HardwareSerial.h"

#define echoPin 4
#define trigPin 33
#define vibrationPin 32
#define gpsSerial Serial2
HardwareSerial mp3Serial(1);
HardwareSerial sim900(0);
const int buttonPin = 27;
const int CS = 5;

File myFile;
TinyGPSPlus gps;
MPU6050 mpu;

long duration, distance;

// Gyro
// Offsets
int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

static const uint8_t PIN_MP3_TX = 14; // Connects to mp3's RX
static const uint8_t PIN_MP3_RX = 13; // Connects to mp3's TX
DFRobotDFPlayerMini player;

// Fall detection flags
bool freefallDetected = false;
bool impactDetected = false;

// SIM
const int SIM_RX = 25;
const int SIM_TX = 26;
const char PHONE[] = "+8801681305504";

// SD CARD
unsigned long previousSDLog = 0;
const unsigned long SD_INTERVAL = 60000;
char formattedData[100];

void setup() {
  Serial.begin(115200);

  // Ultra Sound
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(vibrationPin, OUTPUT);   // LED pin as output

  // GPS
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);

  // MP3
  mp3Serial.begin(9600, SERIAL_8N1, PIN_MP3_RX, PIN_MP3_TX);
  player.begin(mp3Serial);
  player.volume(30);

  // Gyro
  Wire.begin(21, 22);
  mpu.initialize();
  calibrateMPU();

  // SIM
  sim900.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);

  // Button
  pinMode(buttonPin, INPUT_PULLUP);

  // SD CARD
  SD.begin(CS);
}

void loop() {
  // *********************Ultra Sound************************
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration / 58.2;

  if (distance <= 20) {
    digitalWrite(vibrationPin, HIGH);
    delay(600);
    digitalWrite(vibrationPin, LOW);
    delay(200);
  } else if (distance > 20 && distance <= 35) {
    digitalWrite(vibrationPin, HIGH);
    delay(200);
    digitalWrite(vibrationPin, LOW);
    delay(200);
  } else {
    digitalWrite(vibrationPin, LOW);
    delay(200);
  }
  // *********************Ultra Sound************************

  //*******************GPS*********************
  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read())) {}

  if (millis() > 5000 && gps.charsProcessed() < 10) {}
  //*******************GPS*********************

  //********************Gyro****************************
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  float A_total = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

  static float prevA = 1.0;
  static unsigned long startTime = millis();
  if (millis() - startTime < 3000) {
    prevA = A_total;
  }

  if ((prevA - A_total) > 0.5) {
    freefallDetected = true;
  }

  if (freefallDetected && (A_total - prevA) > 0.) {
    impactDetected = true;
  }

  if (impactDetected && abs(gx) < 150 && abs(gy) < 150 && abs(gz) < 150) {
    freefallDetected = false;
    impactDetected = false;

    String sms = "Fall Detected. Location: ";
    sms += String(gps.location.lat(), 6);
    sms += ", ";
    sms += String(gps.location.lng(), 6);
    sendSMS(sms.c_str());
    WriteFile("/log.txt", formattedData);
  }

  prevA = A_total;

  int buttonState = digitalRead(buttonPin);
  if (buttonState == LOW) {
    String sms = "Help me. I am at this location: ";
    sms += String(gps.location.lat(), 6);
    sms += ", ";
    sms += String(gps.location.lng(), 6);
    sendSMS(sms.c_str());
    // player.play(4);
  }
  //********************Gyro****************************

  //*********************SD CARD *****************************
  unsigned long currentMillis = millis();
  if (currentMillis - previousSDLog >= SD_INTERVAL) {
    previousSDLog = currentMillis;

    String sms = "I am at this location: ";
    sms += String(gps.location.lat(), 6);
    sms += ", ";
    sms += String(gps.location.lng(), 6);

    WriteFile("/log.txt", sms.c_str());
  }
  //*********************SD CARD *****************************
}



void calibrateMPU() {
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;

  int samples = 200;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;

    delay(10);
  }

  ax_offset = ax_sum / samples;
  ay_offset = ay_sum / samples;
  az_offset = az_sum / samples;
  gx_offset = gx_sum / samples;
  gy_offset = gy_sum / samples;
  gz_offset = gz_sum / samples;
}

void sendSMS(const char* msg) {
  sim900.print("AT+CMGS=\"");
  sim900.print(PHONE);
  sim900.println("\"");
  delay(1000);

  sim900.print(msg);
  sim900.write(26);
  delay(5000);
}

void WriteFile(const char *path, const char *message) {
  myFile = SD.open(path, FILE_APPEND);
  if (myFile) {
    myFile.println(message);
    myFile.close();
    Serial.println(message);
  } else {
    Serial.println("error opening file");
  }
}