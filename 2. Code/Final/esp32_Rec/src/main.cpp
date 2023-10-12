#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>

#include <ESP32Servo.h>
Servo servo;
ESP32PWM pwm;



#define pwmA 25   
#define pwmB 26   

//Các chân rơ le
#define in1 33
#define in2 25
#define in3 26
#define in4 27

#define servoPin 2  //Chân điều khiển servo

#define voltageSensor 34  //Chân đọc giá trị cảm biến điện áp

bool check = true;
int pos;


uint8_t broadcastAddress[] = {0xB8, 0xD6, 0x1A, 0x57, 0x8C, 0x74};; //Địa chỉ ESP32 trên tay điều khiển

// Define variables to store incoming readings
int incomingX;
int incomingY = 1850;
int incomingAngle;

// Variable to store if sending data was successful
String success;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message
{
  int x;
  int y;
  int angle;
} struct_message;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message incomingsReadings;

// Create a struct_message to hold incoming sensor readings
int voltageReadings;

esp_now_peer_info_t peerInfo;

//Hàm điều khiển động cơ từ giá trị của joystick
void Control(int x, int y)
{
  
  if (y > 2300) 
  {
    if (x > 1000 && x < 3000) //Đi tiến thẳng
    {
      digitalWrite(in1, 0);
      digitalWrite(in2, 1);

      digitalWrite(in3, 0);
      digitalWrite(in4, 1);
    }
    if (x < 1000) //Đi tiến và rẽ trái
    {
      digitalWrite(in1, 1);
      digitalWrite(in2, 1);

      digitalWrite(in3, 0);
      digitalWrite(in4, 1);
    }
    if (x > 3000) //Đi tiến và rẽ phải
    {
      digitalWrite(in1, 0);
      digitalWrite(in2, 1);

      digitalWrite(in3, 1);
      digitalWrite(in4, 1);
    }
  }
  else if (y < 1500)
  {
    if (x > 1000 && x < 3000) //Đi lùi thẳng
    {
      digitalWrite(in1, 1);
      digitalWrite(in2, 0);

      digitalWrite(in3, 1);
      digitalWrite(in4, 0);
    }
    if (x < 1000) //Đi lùi và rẽ trái
    {
      digitalWrite(in1, 1);
      digitalWrite(in2, 1);

      digitalWrite(in3, 1);
      digitalWrite(in4, 0);
    }
    if (x > 3000) //Đi lùi và rẽ phải
    {
      digitalWrite(in1, 1);
      digitalWrite(in2, 0);

      digitalWrite(in3, 1);
      digitalWrite(in4, 1);
    }
  }
  else if (y > 1500 && y < 2300)  //Đứng yên
  {
    digitalWrite(in1, 1);
    digitalWrite(in2, 1);

    digitalWrite(in3, 1);
    digitalWrite(in4, 1);
  }
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");    //"Delivery Success" : "Delivery Fail"
  if (status == 0)
  {
    success = "Delivery Success :)";
  }
  else
  {
    success = "Delivery Fail :(";
    Control(0, 0); //Mất kết nối đứng yên
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&incomingsReadings, incomingData, sizeof(incomingsReadings));

  incomingX = incomingsReadings.x;
  incomingY = incomingsReadings.y;
  incomingAngle = incomingsReadings.angle;

  if (incomingAngle != pos) //Đảo góc servo để đóng mở chốt 
  {
    servo.write(incomingAngle);
    pos = incomingAngle;
  }
  Control(incomingX, incomingY);  //Điều khiển động cơ dựa theo dữ liệu nhận được từ joystick
}

void setup()
{
  // Init Serial Monitor
  Serial.begin(9600);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //Cho xe đứng yên
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);
  digitalWrite(in3, 0);
  digitalWrite(in4, 0);

  pinMode(voltageSensor, INPUT);

  servo.attach(servoPin, 500, 2400);

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

//Đọc giá trị cảm biến điện áp
void getReadings()
{
  voltageReadings = analogRead(voltageSensor);
}

void loop()
{
  getReadings();

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&voltageReadings, sizeof(voltageReadings));

  if (result == ESP_OK)
  {
    Serial.println("Sent with success");
  }
  else
  {
    Serial.println("Error sending the data");
  }

  delay(300); //gửi dữ liệu điện áp đi theo chu kì 300ms
}
