#include "XYZrobotServo.h"

#include "TcpSerial.h"
#include "UnixSerial.h"

int main() {
  const uint8_t servoId = 5;
//  TcpSerial servoSerial("192.168.1.136", 2022);
  UnixSerial servoSerial("/dev/ttySC0"); // on Dr.QP raspi
  XYZrobotServo servo(servoSerial, servoId);

  servo.torqueOff();

  return 0;
}
