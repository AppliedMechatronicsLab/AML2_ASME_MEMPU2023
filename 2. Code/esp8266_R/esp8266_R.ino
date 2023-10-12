/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp8266-nodemcu/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
   version 1.0 
   date: ...
   bios firm: ...
*/

#include <Arduino.h>

#include <Wire.h>
// #include <LiquidCrystal_I2C.h>

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Servo.h>
Servo SG90;

// LiquidCrystal_I2C lcd(0x27, 16, 2);

// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = {0xB4, 0xE6, 0x2D, 0xEE, 0x7C, 0xED};

// Digital pin connected to the DHT sensor


//#define enA 14 //
//#define enB 15
//#define in1 16
//#define in2 17
//#define in3 18
//#define in4 19
// #define relay 21

static const uint8_t servoPin = 5;
// int VolSen = A3;

int motorSpeedA = 0;
int motorSpeedB = 0;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)

// REPLACE WITH THE MAC Address of your receiver
// Define variables to store BME280 readings to be sent
float UReadings;

const long interval = 400;
unsigned long previousMillis = 0;

// Define variables to store incoming readings
float incomingX;
float incomingY;
float incomingK;
int incomingAngle;

// Variable to store if sending data was successful
String success;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message
{
  int x;
  int y;
  int k;
  int angle;

} struct_message;

// Create a struct_message called DHTReadings to hold sensor readings
struct_message incomingReadings;

// Create a struct_message to hold incoming sensor readings

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus)
{
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0)
  {
    Serial.println("Delivery success");
  }
  else
  {
    Serial.println("Delivery fail");
  }
}

// Callback when data is received
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len)
{
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  incomingX = incomingReadings.x;
  incomingY = incomingReadings.y;
  incomingK = incomingReadings.k;
  incomingAngle = incomingReadings.angle;
  SG90.write(random(180)); 
//  Control(incomingX, incomingY, incomingAngle);
}

void getReadings()
{
  UReadings = random(12);
  // UReadings = analogRead(2);
}

void printIncomingReadings()
{
  Serial.println(incomingX);
  Serial.println(incomingY);
  Serial.println(incomingK);
  Serial.println(incomingAngle);
}

//void Control(int x, int y, int angle)
//{
//
//  SG90.write(incomingAngle);
//
//  if (y < 1880)
//  {
//    digitalWrite(in1, HIGH);
//    digitalWrite(in2, LOW);
//    digitalWrite(in3, HIGH);
//    digitalWrite(in4, LOW);
//    motorSpeedA = map(y, 1880, 0, 0, 255);
//    motorSpeedB = map(y, 1880, 0, 0, 255);
//  }
//  else if (y > 1910)
//  {
//    digitalWrite(in1, LOW);
//    digitalWrite(in2, HIGH);
//    digitalWrite(in3, LOW);
//    digitalWrite(in4, HIGH);
//    motorSpeedA = map(y, 1910, 4095, 0, 255);
//    motorSpeedB = map(y, 1910, 4095, 0, 255);
//  }
//  else
//  {
//    motorSpeedA = 0;
//    motorSpeedB = 0;
//  }
//
//  if (x < 1935)
//  {
//    int xMapped = map(x, 1935, 0, 0, 255);
//    motorSpeedA = motorSpeedA - xMapped;
//    motorSpeedB = motorSpeedB + xMapped;
//    if (motorSpeedA < 0)
//    {
//      motorSpeedA = 0;
//    }
//    if (motorSpeedB > 255)
//    {
//      motorSpeedB = 255;
//    }
//  }
//  if (x > 1945)
//  {
//    int xMapped = map(x, 1945, 4095, 0, 255);
//    motorSpeedA = motorSpeedA + xMapped;
//    motorSpeedB = motorSpeedB - xMapped;
//    if (motorSpeedA > 255)
//    {
//      motorSpeedA = 255;
//    }
//    if (motorSpeedB < 0)
//    {
//      motorSpeedB = 0;
//    }
//  }
//  if (motorSpeedA < 70)
//  {
//    motorSpeedA = 0;
//  }
//  if (motorSpeedB < 70)
//  {
//    motorSpeedB = 0;
//  }
//  if (x == 514 && y == 502)
//  {
//    motorSpeedA = 0;
//    motorSpeedB = 0;
//    digitalWrite(in1, LOW);
//    digitalWrite(in2, LOW);
//    digitalWrite(in3, LOW);
//    digitalWrite(in4, LOW);
//  };
//
//  analogWrite(enA, motorSpeedA);
//  analogWrite(enB, motorSpeedB);
//  Serial.println(motorSpeedA);
//  Serial.println(motorSpeedB);
//}

void setup()
{
  // Init Serial Monitor
  Serial.begin(115200);
  pinMode(5, OUTPUT);

    SG90.attach(servoPin);
//    SG90.write(100);
//    delay(1000);
//    SG90.write(0);

  // Init DHT sensor

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != 0)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set ESP-NOW Role
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    // save the last time you updated the DHT values
    previousMillis = currentMillis;

    // Get DHT readings
    getReadings();

    // Set values to send

    // Send message via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *)&UReadings, sizeof(UReadings));

    // Print incoming readings
     printIncomingReadings();
  }

}
