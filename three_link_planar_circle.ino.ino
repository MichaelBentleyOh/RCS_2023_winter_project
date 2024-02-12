#include <Dynamixel2Arduino.h>
#include <math.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)  // When using DynamixelShield
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8);  // DYNAMIXELShield UART RX/TX
#define DXL_SERIAL Serial
#define DEBUG_SERIAL soft_serial
const int DXL_DIR_PIN = 2;      // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE)  // When using DynamixelShield
#define DXL_SERIAL Serial
#define DEBUG_SERIAL SerialUSB
const int DXL_DIR_PIN = 2;  // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO)  // When using DynamixelShield
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL SerialUSB
const int DXL_DIR_PIN = 2;  // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904)  // When using official ROBOTIS board with DXL circuit.
#define DXL_SERIAL Serial3        //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 22;  //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR)  // When using official ROBOTIS board with DXL circuit.
// For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
// Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 84;  // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
//OpenRB does not require the DIR control pin.
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = -1;
#else  // Other boards when using DynamixelShield
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 2;  // DYNAMIXEL Shield DIR PIN
#endif

#define UART_SERIAL Serial3

const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
//This namespace is required to use Control table item names
using namespace ControlTableItem;

const int32_t J0_PROF_ACC = 2, J0_PROF_VEL = 20;
const int32_t J1_PROF_ACC = 2, J1_PROF_VEL = 20;
const int32_t J2_PROF_ACC = 2, J2_PROF_VEL = 20;


const uint8_t DXL_ID_0 = 0;
const uint8_t DXL_ID_1 = 1;
const uint8_t DXL_ID_2 = 2;

int16_t cmd = 0;


float j0_off  = 180.0f;//[deg]
float j1_off  = 180.0f;//[deg]
float j2_off  = 180.0f;//[deg]


long iter_counter   = 0;
uint32_t sampleRate = 10;  // 10Hz interrupt. sample rate in milliseconds, determines how often TC5_Handler is called
int led = 0; //++++++

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(100);
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID_0);
  dxl.ping(DXL_ID_1);
  dxl.ping(DXL_ID_2);

  delay(1000);
  dxl.torqueOff(DXL_ID_0);
  dxl.setOperatingMode(DXL_ID_0, OP_POSITION);
  dxl.torqueOn(DXL_ID_0);
  dxl.torqueOff(DXL_ID_1);
  dxl.setOperatingMode(DXL_ID_1, OP_POSITION);
  dxl.torqueOn(DXL_ID_1);
  dxl.torqueOff(DXL_ID_2);
  dxl.setOperatingMode(DXL_ID_2, OP_POSITION);
  dxl.torqueOn(DXL_ID_2);
  

  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_0, 100);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_0,1000);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_1, 100);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_1, 1000);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_2, 100);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_2, 1000);
  dxl.setGoalPosition(DXL_ID_0, j0_off, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_1, j1_off, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_2, j2_off, UNIT_DEGREE);
  delay(1000);
}

int l1 = 20;  // link1 length
int l2 = 15;  // link2 length
int l3 = 10;
int radius = 4;
int theta = 0;
float phi = 45.f; // const(q1+q2+q3)
float q1_IK;
float q2_IK;
float q3_IK;
float c2;
void three_link_IK(int l1, int l2, int l3, float px, float py, float phi);

void loop() {
    float px = 30 + radius*cos(radians(theta));
    float py = -10 + radius*sin(radians(theta));
    three_link_IK(l1, l2, l3, px, py, phi);
    if( c2 >= 1  c2 != 0.7){
      dxl.setGoalPosition(DXL_ID_0, q1_IK + j0_off, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_1, q2_IK + j1_off, UNIT_DEGREE);
      dxl.setGoalPosition(DXL_ID_2, q3_IK + j2_off, UNIT_DEGREE);
    }
    delay(100);
    theta += 1;
    if (theta >=360) {
      theta-=360;
    }
}

void three_link_IK(int l1, int l2, int l3, float px, float py, float phi) {
  // int phi = q1_IK + q2_IK + q3_IK;
  float fx = px - l3*cos(radians(phi));
  float fy = py - l3*sin(radians(phi));
  c2 = (pow(fx,2)+pow(fy,2) - pow(l1,2) - pow(l2,2)) / (2 * l1 * l2);
  float s2 = sqrt(1 - pow(c2,2));
  float c1 = ((l1+l2*c2)*fx+l2*s2*fy) / (pow(fx,2)+pow(fy,2));
  float s1 = ((l1+l2*c2)*fy-l2*s2*fx) / (pow(fx,2)+pow(fy,2));
  q2_IK = degrees(atan2(s2, c2));
  q1_IK = degrees(atan2(s1, c1));
  q3_IK = phi - q1_IK - q2_IK;

  Serial.println(s2);
  Serial.println(c2);
}
