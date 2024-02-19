// Note
//The dynamixel only recognizes degrees from 0 to 359.
//Make sure to run the code with the ID set to your configured value for DXL
//In actual modeling, it is important to note that there is a limitation of 180 degrees 
//for the motor's range of motion, so it must be taken into account when modifying the code.
//Trigonometric functions only accept radian values, so be mindful of unit conversion.


#include <Dynamixel2Arduino.h>
#include <math.h>

#if defined(ARDUINO_OpenRB)  // When using OpenRB-150
//OpenRB does not require the DIR control pin.
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = -1;
#else  // Other boards when using DynamixelShield
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 2;  // DYNAMIXEL Shield DIR PIN
#endif

const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
//This namespace is required to use Control table item names
using namespace ControlTableItem;

//Set your DXL_ID
const uint8_t DXL_ID_0 = 0;
const uint8_t DXL_ID_1 = 1;
const uint8_t DXL_ID_2 = 2;

//Set the offset angle of motor.
float j0_off  = 180.0f;//[deg]
float j1_off  = 180.0f;//[deg]
float j2_off  = 180.0f;//[deg]

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
  
  //Set profile of velocity&acceleration
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_0, 20);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_0, 200);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_1, 20);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_1, 200);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_2, 20);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_2, 200);

  //Motor runs at the offset value.
  dxl.setGoalPosition(DXL_ID_0, j0_off, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_1, j1_off, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_2, j2_off, UNIT_DEGREE);
  delay(1000);
}

int l1 = 20;  // Link1 length
int l2 = 20;  // Link2 length
int l3 = 10;  // Link3 length
int radius = 5; // End Effector's radius
int theta = 0;  //Initialize theta
float phi = 70.f; // const(q1+q2+q3)
float q1_IK;  //Initialize q1_IK
float q2_IK;  //Initialize q2_IK
float q3_IK;  //Initialize q3_IK

void three_link_IK(int l1, int l2, int l3, float px, float py, float phi);

void loop() {
    //End Effector's trajectory px, py
    float px = 35 + radius*cos(radians(theta));
    float py = 0 + radius*sin(radians(theta));

    //Inverse Kinematics
    three_link_IK(l1, l2, l3, px, py, phi);

    //Set position commands to Dynamixel motors
    dxl.setGoalPosition(DXL_ID_0, q1_IK + j0_off, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_1, q2_IK + j1_off, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_2, q3_IK + j2_off, UNIT_DEGREE);
    delay(100);

    theta += 1;
    if (theta >=360) {
      theta-=360;
    }
}

//Refer to inverse kinematics theory
void three_link_IK(int l1, int l2, int l3, float px, float py, float phi) {
  float fx = px - l3*cos(radians(phi));
  float fy = py - l3*sin(radians(phi));
  float c2 = (pow(fx,2)+pow(fy,2) - pow(l1,2) - pow(l2,2)) / (2 * l1 * l2);
  float s2 = sqrt(1 - pow(c2,2));
  float c1 = ((l1+l2*c2)*fx+l2*s2*fy) / (pow(fx,2)+pow(fy,2));
  float s1 = ((l1+l2*c2)*fy-l2*s2*fx) / (pow(fx,2)+pow(fy,2));

  q2_IK = degrees(atan2(s2, c2));
  q1_IK = degrees(atan2(s1, c1));
  q3_IK = phi - q1_IK - q2_IK;
}
