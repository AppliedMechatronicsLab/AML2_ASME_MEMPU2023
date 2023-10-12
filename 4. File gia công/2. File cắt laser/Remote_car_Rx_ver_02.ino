#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CSN_GPIO    8
#define CE_GPIO     7

#define RIGHT_FORWARD     5  // Left side motor forward
#define RIGHT_BACKWARD    4  // Left side motor backward
#define LEFT_FORWARD      3  // Right side motor forward
#define LEFT_BACKWARD     2  // Right side motor backward

// Hardware configuration
RF24 radio(CE_GPIO, CSN_GPIO);                           // Set up nRF24L01 radio on SPI bus plus pins 7 & 8

const byte Address[6] = "00001";
unsigned char Received_Command = '0';

void setup() {
  Serial.begin(9600);
  pinMode(RIGHT_FORWARD,OUTPUT);   //left motors forward
  pinMode(RIGHT_BACKWARD,OUTPUT);   //left motors reverse
  pinMode(LEFT_FORWARD,OUTPUT);   //right motors forward
  pinMode(LEFT_BACKWARD,OUTPUT);   //right motors reverse
  radio.begin();
  radio.openReadingPipe(0, Address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("START");
}

void loop()
{
  if (radio.available())    // If the NRF240L01 module received data
  {  
    delay(10);
    radio.read(&Received_Command, 1);
    Serial.print(Received_Command);
  }

  if(Received_Command == 1)            //move forward(all motors rotate in forward direction)
  {
    digitalWrite(LEFT_FORWARD,HIGH);
    digitalWrite(RIGHT_FORWARD,HIGH);
  }
  else if(Received_Command == 2)      //move reverse (all motors rotate in reverse direction)
  {
    digitalWrite(LEFT_BACKWARD,HIGH);
    digitalWrite(RIGHT_BACKWARD,HIGH);
  }
  else if(Received_Command == 3)      //turn right (left side motors rotate in forward direction, right side motors rotates in backward direction)
  {
    digitalWrite(LEFT_FORWARD,HIGH);
    digitalWrite(RIGHT_BACKWARD,HIGH);
  }
  else if(Received_Command == 4)      //turn left (right side motors rotate in forward direction, left side motors rotates in backward direction)
  {
    digitalWrite(RIGHT_FORWARD,HIGH);
    digitalWrite(LEFT_BACKWARD,HIGH);
  }
  else if(Received_Command == 0)      //STOP (all motors stop)
  {      
    digitalWrite(RIGHT_FORWARD,LOW);
    digitalWrite(RIGHT_BACKWARD,LOW);
    digitalWrite(LEFT_FORWARD,LOW);
    digitalWrite(LEFT_BACKWARD,LOW);
  }
  else                                  //STOP (all motors stop) , If any other command is received.
  {
    digitalWrite(RIGHT_FORWARD,LOW);
    digitalWrite(RIGHT_BACKWARD,LOW);
    digitalWrite(LEFT_FORWARD,LOW);
    digitalWrite(LEFT_BACKWARD,LOW);
  }
}
