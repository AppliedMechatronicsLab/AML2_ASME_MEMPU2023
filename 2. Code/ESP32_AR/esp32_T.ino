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

#include <Wire.h>
#include <LiquidCrystal_I2C.h>




#define Xpin 12 // D6
#define Ypin 14 // D5
#define Button 2  //D4

LiquidCrystal_I2C lcd(0x27, 16, 2);

uint8_t broadcastAddress[] = {0xB4, 0xE6, 0x2D, 0xEE, 0x7C, 0xED};

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

// Create a struct_message called BME280Readings to hold sensor readings

// Create a struct_message to hold incoming sensor readings
struct_message Readings;

float incomingReadings;

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
//  Serial.print("Bytes received: ");
//  Serial.println(len);
  incomingU = incomingReadings;

//  Control(incomingX, incomingY, incomingK);
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
  Serial.print("dien ap: ");
  Serial.println(incomingU);
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
    Readings.x = x;
    Readings.y = y;
    Readings.k = k;
    Readings.angle = angle;
    // Serial.println(x);
    // Serial.println(y); 
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&Readings, sizeof(Readings));

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
