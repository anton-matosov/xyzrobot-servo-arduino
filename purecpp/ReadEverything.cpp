// This example reads all the information from a smart servo and
// prints it to the serial monitor.

#include "XYZrobotServo.h"
#include "UnixSerial.h"

#include <iostream>
#include <thread>
// #define servoSerial Serial1


UnixSerial servoSerial("/dev/ttySC0");

XYZrobotServo servo6(servoSerial, 6);
XYZrobotServo servo12(servoSerial, 12);
XYZrobotServo servo18(servoSerial, 18);

XYZrobotServo servo5(servoSerial, 5);
XYZrobotServo servo11(servoSerial, 11);
XYZrobotServo servo17(servoSerial, 17);

void setup()
{
  // Serial.begin(115200); // console output
  
  // 115200 8N1 => 11520 B/s / 24 frames = 480 Bytes Per Frame / 18 servos = 26 bytes per servo per frame

  // Turn on the serial port and set its baud rate.
  servoSerial.begin(115200);
  // servoSerial.setTimeout(20);

  // To receive data, a pull-up is needed on the RX line because
  // the servos do not pull the line high while idle.
  // pinMode(DDD2, INPUT_PULLUP);  
}

struct CustomStatus48
{
  uint8_t statusError;
  uint8_t statusDetail;
  uint8_t reserved[3];
  uint8_t ledControl;
  uint8_t voltage;
  uint8_t temperature;
  uint8_t currentControlMode;
  uint8_t tick; // 8
  uint16_t reserved2;
  uint16_t jointPosition;
  uint16_t reserved3;
  uint16_t pwmOutputDuty;
  uint16_t busCurrent;
  uint16_t posGoal;
  uint16_t posRef;
  uint16_t speedGoal;
  uint16_t speedRef; // 18 + 8 = 26
} __attribute__((packed));


void readAndPrintStatus(XYZrobotServo & servo)
{
  XYZrobotServoStatus status = servo.readStatus();
  if (servo.getLastError())
  {
    std::cout << "error reading status: " << servo.getLastError() << std::endl;
  }
  else
  {
    std::cout << "status:\n";
    std::cout << "  statusError: 0x";
    std::cout << std::hex << (int)status.statusError << "\n";
    std::cout << "  statusDetail: 0x";
    std::cout << std::hex << (int)status.statusDetail << std::dec << "\n";
    std::cout << "  pwm: ";
    std::cout << (int)status.pwm << "\n";
    std::cout << "  posRef: ";
    std::cout << (int)status.posRef << "\n";
    std::cout << "  position: ";
    std::cout << (int)status.position << "\n";
    std::cout << "  iBus: ";
    std::cout << (int)status.iBus << "\n";
  }
}

void readCustomStatus(XYZrobotServo & servo)
{
  CustomStatus48 status;
  const size_t statusSize = sizeof(CustomStatus48);
  servo.ramRead(48, &status.statusError, statusSize);
  if (servo.getLastError())
  {
    std::cout << "error reading custom status: " << servo.getLastError() << std::endl;
  }
  else
  {
    std::cout << "status:\n";
    std::cout << "  statusError: 0x" << std::hex << (int)status.statusError << "\n";
    std::cout << "  statusDetail: 0x" << std::hex << (int)status.statusDetail << std::dec << "\n";
    std::cout << "  ledControl: " << (int)status.ledControl << "\n";
    std::cout << "  voltage: " << (int)status.voltage / 16 << "V\n";
    std::cout << "  temperature: " << (int)status.temperature << "˚C\n";
    std::cout << "  currentControlMode: " << (int)status.currentControlMode << "\n";
    std::cout << "  tick: " << (int)status.tick * 10 << "ms\n";
    std::cout << "  jointPosition: " << (int)status.jointPosition << "\n";
    std::cout << "  pwmOutputDuty: " << (int)status.pwmOutputDuty << "\n";
    std::cout << "  busCurrent: " << (float)status.busCurrent / 200.f << "A\n";
    std::cout << "  posGoal: " << (int)status.posGoal << "\n";
    std::cout << "  posRef: " << (int)status.posRef << "\n";
    std::cout << "  speedGoal: " << (int)status.speedGoal << "\n";
    std::cout << "  speedRef: " << (int)status.speedRef << "\n";
  }
}

void readAndPrintRAM(XYZrobotServo & servo)
{
  uint8_t ram[80];
  servo.ramRead(0, ram, 30);
  if (!servo.getLastError()) { servo.ramRead(30, ram + 30, 30); }
  if (!servo.getLastError()) { servo.ramRead(60, ram + 60, 20); }
  if (servo.getLastError())
  {
    std::cout << "error reading RAM: ";
    std::cout << servo.getLastError() << "\n";
  }
  else
  {
    std::cout << "RAM:" << "\n";
    std::cout << "  sID: ";
    std::cout << (int)ram[0] << "\n";
    std::cout << "  ACK_Policy: ";
    std::cout << (int)ram[1] << "\n";
    std::cout << "  Alarm_LED_Policy: ";
    std::cout << (int)ram[2] << "\n";
    std::cout << "  Torque_Policy: ";
    std::cout << (int)ram[3] << "\n";
    std::cout << "  SPDctrl_Policy: ";
    std::cout << (int)ram[4] << "\n";
    std::cout << "  Max_Temperature: ";
    std::cout << (int)ram[5] << "\n";
    std::cout << "  Min_Voltage: ";
    std::cout << (int)ram[6] << "\n";
    std::cout << "  Max_Voltage: ";
    std::cout << (int)ram[7] << "\n";
    std::cout << "  Acceleration_Ratio: ";
    std::cout << (int)ram[8] << "\n";
    std::cout << "  Max_Wheel_Ref_Position: ";
    std::cout << (int)ram[12] + (ram[13] << 8) << "\n";
    std::cout << "  Max_PWM: ";
    std::cout << (int)ram[16] + (ram[17] << 8) << "\n";
    std::cout << "  Overload_Threshold: ";
    std::cout << (int)ram[18] + (ram[19] << 8) << "\n";
    std::cout << "  Min_Position: ";
    std::cout << (int)ram[20] + (ram[21] << 8) << "\n";
    std::cout << "  Max_Position: ";
    std::cout << (int)ram[22] + (ram[23] << 8) << "\n";
    std::cout << "  Position_Kp: ";
    std::cout << (int)ram[24] + (ram[25] << 8) << "\n";
    std::cout << "  Position_Kd: ";
    std::cout << (int)ram[26] + (ram[27] << 8) << "\n";
    std::cout << "  Position_Ki: ";
    std::cout << (int)ram[28] + (ram[29] << 8) << "\n";
    std::cout << "  Close_to_Open_Ref_Position: ";
    std::cout << (int)ram[30] + (ram[31] << 8) << "\n";
    std::cout << "  Open_to_Close_Ref_Position: ";
    std::cout << (int)ram[32] + (ram[33] << 8) << "\n";
    std::cout << "  Ramp_Speed: ";
    std::cout << (int)ram[36] + (ram[37] << 8) << "\n";
    std::cout << "  LED_Blink_Period: ";
    std::cout << (int)ram[38] << "\n";
    std::cout << "  Packet_Timeout_Detection_Period: ";
    std::cout << (int)ram[40] << "\n";
    std::cout << "  Overload_Detection_Period: ";
    std::cout << (int)ram[42] << "\n";
    std::cout << "  Inposition_Margin: ";
    std::cout << (int)ram[44] << "\n";
    std::cout << "  Over_Voltage_Detection_Period: ";
    std::cout << (int)ram[45] << "\n";
    std::cout << "  Over_Temperature_Detection_Period: ";
    std::cout << (int)ram[46] << "\n";
    std::cout << "  Calibration_Difference: ";
    std::cout << (int)ram[47] << "\n";
    std::cout << "  Status_Error: ";
    std::cout << (int)ram[48] << "\n";
    std::cout << "  Status_Detail: ";
    std::cout << (int)ram[49] << "\n";
    std::cout << "  LED_Control: ";
    std::cout << (int)ram[53] << "\n";
    std::cout << "  Voltage: ";
    std::cout << (int)ram[54] << "\n";
    std::cout << "  Temperature: ";
    std::cout << (int)ram[55] << "\n";
    std::cout << "  Current_Control_Mode: ";
    std::cout << (int)ram[56] << "\n";
    std::cout << "  Tick: ";
    std::cout << (int)ram[57] << "\n";
    std::cout << "  Joint_Position: ";
    std::cout << (int)ram[60] + (ram[61] << 8) << "\n";
    std::cout << "  PWM_Output_Duty: ";
    std::cout << (int)ram[64] + (ram[65] << 8) << "\n";
    std::cout << "  Bus_Current: ";
    std::cout << (int)ram[66] + (ram[67] << 8) << "\n";
    std::cout << "  Position_Goal: ";
    std::cout << (int)ram[68] + (ram[69] << 8) << "\n";
    std::cout << "  Position_Ref: ";
    std::cout << (int)ram[70] + (ram[71] << 8) << "\n";
    std::cout << "  Omega_Goal: ";
    std::cout << (int)ram[72] + (ram[73] << 8) << "\n";
    std::cout << "  Omega_Ref: ";
    std::cout << (int)ram[74] + (ram[75] << 8) << "\n";
    std::cout << "  Requested_Counts: ";
    std::cout << (int)ram[76] + (ram[77] << 8) << "\n";
    std::cout << "  ACK_Counts: ";
    std::cout << (int)ram[78] + (ram[79] << 8) << "\n";
  }
}

void readAndPrintEEPROM(XYZrobotServo & servo)
{

  uint8_t eeprom[54];
  servo.eepromRead(0, eeprom, 30);
  if (!servo.getLastError()) { servo.eepromRead(30, eeprom + 30, 24); }
  if (servo.getLastError())
  {
    std::cout << "error reading EEPROM: ";
    std::cout << (int)servo.getLastError() << "\n";
  }
  else
  {
    std::cout << "EEPROM:" << "\n";
    std::cout << "  Model_No: ";
    std::cout << (int)eeprom[0] << "\n";
    std::cout << "  Date: ";
    std::cout << (int)eeprom[1];  // Year
    std::cout << '-';
    std::cout << (int)(eeprom[2] & 0xF); // Month
    std::cout << '-';
    std::cout << (int)eeprom[3];  // D << "\n"y
    std::cout << "  Firmware_Version: ";
    std::cout << (int)(eeprom[2] >> 4 & 0xF) << "\n";
    std::cout << "  Baud_Rate: ";
    std::cout << (int)eeprom[5] << "\n";
    std::cout << "  sID: ";
    std::cout << (int)eeprom[6] << "\n";
    std::cout << "  ACK_Policy: ";
    std::cout << (int)eeprom[7] << "\n";
    std::cout << "  Alarm_LED_Policy: ";
    std::cout << (int)eeprom[8] << "\n";
    std::cout << "  Torque_Policy: ";
    std::cout << (int)eeprom[9] << "\n";
    std::cout << "  SPDctrl_Policy: ";
    std::cout << (int)eeprom[10] << "\n";
    std::cout << "  Max_Temperature: ";
    std::cout << (int)eeprom[11] << "\n";
    std::cout << "  Min_Voltage: ";
    std::cout << (int)eeprom[12] << "\n";
    std::cout << "  Max_Voltage: ";
    std::cout << (int)eeprom[13] << "\n";
    std::cout << "  Acceleration_Ratio: ";
    std::cout << (int)eeprom[14] << "\n";
    std::cout << "  Max_Wheel_Ref_Position: ";
    std::cout << (int)eeprom[18] + (eeprom[19] << 8) << "\n";
    std::cout << "  Max_PWM: ";
    std::cout << (int)eeprom[22] + (eeprom[23] << 8) << "\n";
    std::cout << "  Overload_Threshold: ";
    std::cout << (int)eeprom[24] + (eeprom[25] << 8) << "\n";
    std::cout << "  Min_Position: ";
    std::cout << (int)eeprom[26] + (eeprom[27] << 8) << "\n";
    std::cout << "  Max_Position: ";
    std::cout << (int)eeprom[28] + (eeprom[29] << 8) << "\n";
    std::cout << "  Position_Kp: ";
    std::cout << (int)eeprom[30] + (eeprom[31] << 8) << "\n";
    std::cout << "  Position_Kd: ";
    std::cout << (int)eeprom[32] + (eeprom[33] << 8) << "\n";
    std::cout << "  Position_Ki: ";
    std::cout << (int)eeprom[34] + (eeprom[35] << 8) << "\n";
    std::cout << "  Close_to_Open_Ref_Position: ";
    std::cout << (int)eeprom[36] + (eeprom[37] << 8) << "\n";
    std::cout << "  Open_to_Close_Ref_Position: ";
    std::cout << (int)eeprom[38] + (eeprom[39] << 8) << "\n";
    std::cout << "  Ramp_Speed: ";
    std::cout << (int)eeprom[42] + (eeprom[43] << 8) << "\n";
    std::cout << "  LED_Blink_Period: ";
    std::cout << (int)eeprom[44] << "\n";
    std::cout << "  Packet_Timeout_Detection_Period: ";
    std::cout << (int)eeprom[46] << "\n";
    std::cout << "  Overload_Detection_Period: ";
    std::cout << (int)eeprom[48] << "\n";
    std::cout << "  Inposition_Margin: ";
    std::cout << (int)eeprom[50] << "\n";
    std::cout << "  Over_Voltage_Detection_Period: ";
    std::cout << (int)eeprom[51] << "\n";
    std::cout << "  Over_Temperature_Detection_Period: ";
    std::cout << (int)eeprom[52] << "\n";
    std::cout << "  Calibration_Difference: ";
    std::cout << (int)eeprom[53] << "\n";
  }
}

void readEverything(XYZrobotServo & servo)
{
  readAndPrintStatus(servo);
  readAndPrintRAM(servo);
  readAndPrintEEPROM(servo);

  std::cout << "\n";
}

void readLoop(XYZrobotServo& servo)
{
  using namespace std::chrono_literals;

  std::this_thread::sleep_for(2s);
  readEverything(servo);
}


const uint8_t playtime = 75;

const int Min_Position = 400;
const int Max_Position = 600;

int pos = (Max_Position + Min_Position) / 2;

void setPos(XYZrobotServo& servo)
{
  using namespace std::chrono_literals;

/////////////////////////////////////////////
  if (pos > Max_Position) {
    std::this_thread::sleep_for(50ms);
    pos = Min_Position;

    // readCustomStatus(servo);
    std::cout << "1. Setting pos " << std::dec << pos << "\n";
    servo.setPosition(pos, 0);
    
    // readCustomStatus(servo);
    std::this_thread::sleep_for(50ms);
  }
  
  // readCustomStatus(servo);
  std::cout << "2. Setting pos " << std::dec << pos << "\n";
  servo.setPosition(pos, 1);

  // readCustomStatus(servo);
  std::this_thread::sleep_for(10ms);

  pos += 23;
}

void testWrite()
{
  uint8_t byte = rand() % 255;
  std::cout << "write " << (int)byte << "\n";
  servoSerial.write(byte);
}

void testRoundtrip()
{
  using namespace std::chrono_literals;

  std::this_thread::sleep_for(500ms);
  
  uint8_t byte = rand() % 255;
  std::cout << "write " << (int)byte << "\n";
  servoSerial.write(byte);

  std::this_thread::sleep_for(10ms);
  servoSerial.readBytes(&byte, 1); 
  std::cout << "read " << (int)byte << "\n";
}

int main()
{
  setup();

  for (;;)
  {
    // testRoundtrip();
    // testWrite();
    // readLoop();
    setPos(servo6);
    setPos(servo12);
    setPos(servo18);

    setPos(servo5);
    setPos(servo11);
    setPos(servo17);
  }
}