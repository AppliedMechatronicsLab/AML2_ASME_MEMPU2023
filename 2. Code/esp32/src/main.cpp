/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>
#include <Servo.h>
Servo SG90;

#define enA 14 //
#define enB 15
#define in1 16
#define in2 17
#define in3 18
#define in4 19
#define relay 21
int VolSen = A3;

int motorSpeedA = 0;
int motorSpeedB = 0;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)

// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = {0x84, 0xF3, 0xEB, 0xB1, 0xA6, 0xE8};

// Define variables to store BME280 readings to be sent
float UReadings;

const long interval = 600;
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

// Create a struct_message called BME280Readings to hold sensor readings

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0)
  {
    success = "Delivery Success :)";
  }
  else
  {
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);

  incomingX = incomingReadings.x;
  incomingY = incomingReadings.y;
  incomingK = incomingReadings.k;
  incomingAngle = incomingReadings.angle;
  Control(incomingX, incomingY, incomingK);
}

void getReadings()
{
  // UReadings = analogRead();
  UReadings = random(12);
}

void printIncomingReadings()
{
  Serial.println(incomingX);
  Serial.println(incomingY);
  Serial.println(incomingK);
  Serial.println(incomingAngle);
}

void Control(int x, int y, int k)
{
  if (k == 0)
  {
    SG90.write(incomingAngle);

  }

  if (y < 495)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    motorSpeedA = map(y, 495, 0, 0, 255);
    motorSpeedB = map(y, 495, 0, 0, 255);
  }
  else if (y > 505)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    motorSpeedA = map(y, 505, 1023, 0, 255);
    motorSpeedB = map(y, 505, 1023, 0, 255);
  }
  else
  {
    motorSpeedA = 0;
    motorSpeedB = 0;
  }

  if (x < 510)
  {
    int xMapped = map(x, 510, 0, 0, 255);
    motorSpeedA = motorSpeedA - xMapped;
    motorSpeedB = motorSpeedB + xMapped;
    if (motorSpeedA < 0)
    {
      motorSpeedA = 0;
    }
    if (motorSpeedB > 255)
    {
      motorSpeedB = 255;
    }
  }
  if (x > 520)
  {
    int xMapped = map(x, 520, 1023, 0, 255);
    motorSpeedA = motorSpeedA + xMapped;
    motorSpeedB = motorSpeedB - xMapped;
    if (motorSpeedA > 255)
    {
      motorSpeedA = 255;
    }
    if (motorSpeedB < 0)
    {
      motorSpeedB = 0;
    }
  }
  if (motorSpeedA < 70)
  {
    motorSpeedA = 0;
  }
  if (motorSpeedB < 70)
  {
    motorSpeedB = 0;
  }
  if (x == 514 && y == 502)
  {
    motorSpeedA = 0;
    motorSpeedB = 0;
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  };

  analogWrite(enA, motorSpeedA);
  analogWrite(enB, motorSpeedB);
}

void setup()
{
  // Init Serial Monitor
  Serial.begin(115200);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(relay, OUTPUT);
  pinMode(VolSen, INPUT);
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);
  digitalWrite(in3, 0);
  digitalWrite(in4, 0);
  digitalWrite(relay, 0);

  SG90.attach(4);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
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

    getReadings();

    // Set values to send

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&UReadings, sizeof(UReadings));

    if (result == ESP_OK)
    {
      Serial.println("Sent with success");
    }
    else
    {
      Serial.println("Error sending the data");
    }

    printIncomingReadings();
  }
}
