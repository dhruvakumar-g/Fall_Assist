#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <MPU6050.h>

// WiFi credentials
const char* ssid = "BSNL VNR1";
const char* password = "12345678";
const char* serverURL = "https://script.google.com/macros/s/AKfycbz5ynmsRxEHogof83tW5gDuuGmnhCSDOEUJLEe53UXMpbyBZouPGH3MKyofSywQrAPM/exec";  // Paste your Web App URL here

// Sensor and Buzzer
MPU6050 mpu;
#define FALL_THRESHOLD 0.95 // Adjust based on experimentation
const int buzzerPin = 25;  // GPIO pin for buzzer

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  
  // Connect to WiFi
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Failed to connect to MPU6050");
    while (1);
  }
  Serial.println("MPU6050 initialized.");

  // Initialize buzzer
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);  // Ensure buzzer is off initially
}

void loop() {
  // MPU6050 accelerometer and gyroscope data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  // Read accelerometer and gyroscope data
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  // Calculate acceleration magnitude for fall detection
  float acceleration = sqrt(ax * ax + ay * ay + az * az) / 16384.0;
  bool fallDetected = (acceleration < FALL_THRESHOLD);

  // Activate buzzer if fall is detected
  if (fallDetected) {
    digitalWrite(buzzerPin, HIGH);  // Turn on buzzer
    delay(500);                     // Buzzer on for 0.5 seconds
    digitalWrite(buzzerPin, LOW);   // Turn off buzzer
  }

  // Send data every 1 second
  static uint32_t lastMillis = 0;
  if (millis() - lastMillis > 1000) {
    lastMillis = millis();
    
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(serverURL);
      http.addHeader("Content-Type", "application/json");

      // Create JSON payload with accelerometer and gyroscope data in X, Y, Z directions
      String jsonPayload = "{\"fall_detected\":" + String(fallDetected) +
                           ",\"accel_x\":" + String(ax) +
                           ",\"accel_y\":" + String(ay) +
                           ",\"accel_z\":" + String(az) +
                           ",\"gyro_x\":" + String(gx) +
                           ",\"gyro_y\":" + String(gy) +
                           ",\"gyro_z\":" + String(gz) + "}";

      // Print the JSON payload for debugging
      Serial.println("Sending payload: " + jsonPayload);

      // Send data to server
      int httpResponseCode = http.POST(jsonPayload);

      // Check the response
      Serial.print("HTTP Response Code: ");
      Serial.println(httpResponseCode);
      if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println("Response: " + response);
      } else {
        Serial.print("Error on sending POST: ");
        Serial.println(httpResponseCode);
      }
      http.end();
    } else {
      Serial.println("WiFi Disconnected");
    }
  }

  delay(50);
}