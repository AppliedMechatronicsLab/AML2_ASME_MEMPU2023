/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp8266-nodemcu/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <Arduino.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <ESP8266WiFi.h>
#include <espnow.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = {0xB4, 0xE6, 0x2D, 0xEE, 0x7C, 0xED};

// Digital pin connected to the DHT sensor
#define Xpin 12 // D6
#define Ypin 14 // D5
#define Button 2  //D4


int x, y;
float U;

float incomingU;

const long interval = 5;
unsigned long previousMillis = 0;

boolean check = true;
int k = 0;  // button state
int LstBtSt = 0;
int angle;

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
struct_message Readings;

// Create a struct_message to hold incoming sensor readings
float incomingReadings;

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
  Serial.print("Bytes received: ");
  Serial.println(len);
  incomingU = incomingReadings;
}

void getReadings()
{
  x = analogRead(Xpin);
  y = analogRead(Ypin);
  k = digitalRead(Button);
  if (k != LstBtSt)
  {
    if (k == 1)
    {
      check = !check;
    }
    LstBtSt=k;    
    angle = (check) ? 90 : -90;
  }
}

void printIncomingReadings()
{
  // Serial.print("dien ap: ");
  // Serial.println(incomingU);

}

void setup()
{
  // Init Serial Monitor
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  pinMode(Xpin, INPUT);
  pinMode(Ypin, INPUT);
  pinMode(Button, INPUT);

  lcd.setCursor(4, 0);
  lcd.print("AML-2");
  delay(1000);
  lcd.clear();
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
    Readings.x = x;
    Readings.y = y;
    Readings.k = k;
    Readings.angle = angle;
    Serial.println(x);
    Serial.println(y); 

    // Send message via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *)&Readings, sizeof(Readings));

    // Print incoming readings
    printIncomingReadings();
  }
}
