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


const uint8_t DXL_ID_0 = 0;
const uint8_t DXL_ID_1 = 1;
const uint8_t DXL_ID_2 = 2;

//Please select the ID of the motor you want to adjust, and set it to the desired angle.
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
}


void loop() {
  // put your main code here, to run repeatedly:
  dxl.setGoalPosition(DXL_ID_0,j0_off, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_1,j1_off, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_1,j2_off, UNIT_DEGREE);

}
