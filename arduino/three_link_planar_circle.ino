#include <Dynamixel2Arduino.h>
#include <math.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN

#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
//This namespace is required to use Control table item names
using namespace ControlTableItem;   

const int32_t J0_PROF_ACC = 2, J0_PROF_VEL = 20;
const int32_t J1_PROF_ACC = 2, J1_PROF_VEL = 20;


const uint8_t DXL_ID_0 = 0;
const uint8_t DXL_ID_1 = 1;


float j0_off  = 180.0f;//[deg]
float j1_off  = 180.0f;//[deg]


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


  delay(1000);
  dxl.torqueOff(DXL_ID_0);
  dxl.setOperatingMode(DXL_ID_0, OP_POSITION);
  dxl.torqueOn(DXL_ID_0);
  dxl.torqueOff(DXL_ID_1);
  dxl.setOperatingMode(DXL_ID_1, OP_POSITION);
  dxl.torqueOn(DXL_ID_1);

  //dynamixel profile velocity&acceleration
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_0, 20);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_0, 200);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_1, 20);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_1, 200);

  dxl.setGoalPosition(DXL_ID_0, j0_off, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_1, j1_off, UNIT_DEGREE);
  delay(1000);
}


int l1 = 10;  // link1 length
int l2 = 10;  // link2 length
int radius = 12;  //end effector's cricle traj radius
int theta = -90;  //initialize
float q1_IK = 0.0f;
float q2_IK = 0.0f;
void two_link_IK(int radius, int l1, int l2, float px, float py);


void loop() {
    float px = radius * cos(radians(theta));
    float py = radius * sin(radians(theta));
    Serial.println(theta);
    two_link_IK(radius, l1, l2, px, py);
    Serial.println(q1_IK);
    Serial.println(q2_IK);

    // Set position commands to Dynamixel motors
    delay(10);
    dxl.setGoalPosition(DXL_ID_0, q1_IK + j0_off, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_1, q2_IK + j1_off, UNIT_DEGREE);


    theta += 1;
    if (theta >= 90) {
      theta -= 180;
    }

    // if (q1_IK >= 90) {
    //   q1_IK -= 180;
    // }
    // if (q2_IK >= 90) {
    //   q2_IK -= 180;
    // }
}

// Inverse Kinematics
void two_link_IK(int radius, int l1, int l2, float px, float py) {
  int c2 = (radius * radius - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  int s2 = sqrt(1 - c2 * c2);

  q2_IK = degrees(atan2(s2, c2));
  q1_IK = degrees(atan2(py, px) - atan2(l2*sin(radians(q2_IK)), l1+l2*cos(radians(q2_IK))));
}
