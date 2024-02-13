// Note
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

//Set the offset angle of motor.
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

  //Set profile of velocity&acceleration
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_0, 20);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_0, 200);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_1, 20);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_1, 200);

  //Set to return to the motor's offset value at the initial start
  dxl.setGoalPosition(DXL_ID_0, j0_off, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_1, j1_off, UNIT_DEGREE);
  delay(1000);
}

int l1 = 10;  //Link1 length
int l2 = 10;  //Link2 length
int radius = 12;  //End Effector's radius
int theta = 0;  //The angle formed between the end effector and the base
float q1_IK = 0.0f;  //Initialize q1_IK
float q2_IK = 0.0f;  //Initialize q2_IK
void two_link_IK(int radius, int l1, int l2, float px, float py);

void loop() {
    //End Effector's trajectory px, py
    float px = radius * cos(radians(theta));
    float py = radius * sin(radians(theta));

    //Inverse Kinematics
    two_link_IK(radius, l1, l2, px, py);

    // Serial.println(theta);
    // Serial.println(q1_IK);
    // Serial.println(q2_IK);

    //Set position commands to Dynamixel motors
    delay(10);
    dxl.setGoalPosition(DXL_ID_0, q1_IK + j0_off, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_1, q2_IK + j1_off, UNIT_DEGREE);

    //Setting limits on the theta
    theta += 1;
    if (theta >= 90) {
      theta -= 180;
    }
}

//Refer to inverse kinematics theory
void two_link_IK(int radius, int l1, int l2, float px, float py) {
  int c2 = (radius * radius - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  int s2 = sqrt(1 - c2 * c2);

  q2_IK = degrees(atan2(s2, c2));
  q1_IK = degrees(atan2(py, px) - atan2(l2*sin(radians(q2_IK)), l1+l2*cos(radians(q2_IK))));
}
