// This sketch shows how to set the speed of a servo using open
// loop speed control.
//
// Speeds are represented as numbers between -1023 and 1023.
// Setting the speed to 0 results in abrupt deceleration.
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

void loop()
{
  delay(2500);

  // Move the servo output counter-clockwise for some time,
  // ramping up to the specified speed.
  servo.setSpeed(400);
  delay(2000);

  // Set the speed to 0 to make the servo stop abruptly.
  servo.setSpeed(0);
  delay(1000);

  // Move the servo output clockwise for some time, ramping up to
  // the specified speed.
  servo.setSpeed(-400);
  delay(1000);

  // Set the speed to -1 to make the servo smoothly ramp down to
  // a speed that is effectively zero.
  servo.setSpeed(-1);
}
