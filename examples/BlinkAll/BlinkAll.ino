// This sketch shows how to blink the LEDs on all of the smart
// servos attached to your board.
//
// Note that this sketch will leave your servo in a state where
// the LEDs are controlled by the serial interface instead of
// indicating the servo' status.  To get your servos back to
// normal after running this sketch, you will probably want to
// power cycle them.
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
  
  pinMode(DDD2, INPUT_PULLUP);

  // Turn on the serial port and set its baud rate.
  servoSerial.begin(115200);
  servoSerial.setTimeout(20);
}

void setAllLedsAtThisBaudRate(uint8_t color)
{
  delay(100);

  // Make all the LEDs be user-controlled.
  servo.writeAlarmLedPolicyRam(0b1111);

  // Turn on the specified LEDs.
  servo.writeLedControl(color);
  
  // Make all the LEDs be user-controlled.
//  servo.writeAlarmLedPolicyRam(0b0101);

  // Turn on the specified LEDs.
//  servo.writeLedControl(~color & 0b1111);
}

void setAllLeds(uint8_t color)
{
//  servoSerial.begin(9600);
//  setAllLedsAtThisBaudRate(color);
//
//  servoSerial.begin(19200);
//  setAllLedsAtThisBaudRate(color);
//
//  servoSerial.begin(57600);
//  setAllLedsAtThisBaudRate(color);

//  servoSerial.begin(115200);
  setAllLedsAtThisBaudRate(color);
}

void loop()
{
  delay(2500);

  // Try to make all the LEDs blue.
  setAllLeds(0b0010);

  delay(1000);

  // Try to make all the LEDs green.
  setAllLeds(0b0100);
  
  delay(1000);

  // Try to make all the LEDs white.
  setAllLeds(0b0111);
}
