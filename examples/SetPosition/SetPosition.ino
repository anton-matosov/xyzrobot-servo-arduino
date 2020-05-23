// This sketch shows how to move a servo back and forth between
// two different position.
//
// Positions are represented as numbers between 0 and 1023.  When
// you set a position, you can also specify the playtime, which
// is how long you want the movement to take, in units of 10 ms.
//
// This sketch only writes data to the servos; it does not
// receive anything.

#include <XYZrobotServo.h>


#define servoSerial Serial1

const uint8_t servoId = 5;

XYZrobotServo servo(servoSerial, servoId);

void setup()
{
  Serial.begin(115200); // console output
  

  // Turn on the serial port and set its baud rate.
  servoSerial.begin(115200);
  servoSerial.setTimeout(20);

  // To receive data, a pull-up is needed on the RX line because
  // the servos do not pull the line high while idle.
  pinMode(DDD2, INPUT_PULLUP);  
}

const uint8_t playtime = 75;

const int Min_Position = 0;
const int Max_Position = 1023;

int pos = 0; //(Max_Position - Min_Position) / 2;

void loop()
{
//  if (pos == Max_Position) {
//    pos = Min_Position;
//    servo.setPosition(pos, 0);
//    delay(1000);
//  } else {
//    pos = Max_Position;
//    servo.setPosition(pos, 0);
//    delay(1000);
//  }


/////////////////////////////////////////////
//  pos = rand() % Max_Position;
//  Serial.print(F("\nSetting pos "));
//  Serial.print(pos);
//  servo.setPosition(pos, 5);
//  delay(50);

/////////////////////////////////////////////
  if (pos > Max_Position) {
    delay(200);
    pos = Min_Position;
    servo.setPosition(pos, 0);
    delay(1000);
  }
  
  Serial.print(F("\nSetting pos "));
  Serial.print(pos);
  servo.setPosition(pos, 1);
  delay(10);

  pos += 1;
}
