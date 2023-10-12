#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define xPin 35
#define yPin 32
#define buttonPin 33

uint8_t broadcastAddress[] = {0xB8, 0xD6, 0x1A, 0x45, 0x49, 0x90};   // Địa chỉ MAC của ESP32 trên Robot

// Các biến để deboucing nút nhấn hạ tải tự động 
boolean check = true;
int lastButtonState = 0;
int connectStatus = 1;

float xReadings;  //Biến đọc giá trị joystick theo trục x
float yReadings;  //Biến đọc giá trị joystick theo trục x
float angle;  //Biến góc Servo

unsigned long timeP;

int enA, enB = 0;

int buttonReadings;

String success;


// Đóng gói dữ liệu vào struct để gửi đi
typedef struct struct_message
{
  int x;
  int y;
  int angle;
} struct_message;


struct_message Readings;


int incomingReadings;
float incomingsVoltage;

esp_now_peer_info_t peerInfo;

// Hàm được gọi khi dữ liệu được gửi
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  status == ESP_NOW_SEND_SUCCESS;

  //Kiểm tra gửi có thành công hay không
  if (status == 0)
  {
    success = "Delivery Success :)";
  }
  else
  {
    success = "Delivery Fail :(";
    lcd.setCursor(1, 1);
    lcd.print("Deli : Fail :(");
  }

 if (connectStatus != status)
 {
    if (status == 0)
    {
      lcd.setCursor(1, 1);
      lcd.print("Deli : Success");
    }
    else
    {
      lcd.setCursor(1, 1);
      lcd.print("Deli : Fail :(");
    }

    connectStatus = status;
 }
}

// Hàm được gọi khi nhận được dữ liệu
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  incomingsVoltage = (float)(incomingReadings * 0.0040293040293040293040293040293); //Tính giá trị điện áp pin từ giá trị đọc được
  
  Serial.println(incomingsVoltage);

  lcd.setCursor(1, 0);
  lcd.print(incomingsVoltage);
  if (incomingsVoltage < 10)
  {
    lcd.setCursor(4, 0);
    lcd.print("   Vol   ");
  }
}

void setup()
{
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  lcd.clear();

  //Cài đặt thiết bị ở chế độ Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Khởi tạo ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    // Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    // Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

}

//Hàm đọc giá trị joystick
void getReadings()
{
  xReadings = analogRead(xPin);
  yReadings = analogRead(yPin);

  buttonReadings = digitalRead(buttonPin);
  if (buttonReadings != lastButtonState)
  {
    if (buttonReadings == 0)
    {
      check = !check;
    }
  }
  lastButtonState = buttonReadings;
  angle = (check) ? 0 : 45;
}

void loop()
{
  getReadings();

  // Set values to send
  Readings.x = xReadings;
  Readings.y = yReadings;
  Readings.angle = angle;
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&Readings, sizeof(Readings));

  if (result == ESP_OK)
  {
    // Serial.println("Sent with success");
  }
  else
  {
    // Serial.println("Error sending the data");
  }

  // delay(10);
}
 