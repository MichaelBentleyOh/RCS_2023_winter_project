**주의사항**
```
// Note
//The dynamixel only recognizes degrees from 0 to 359.
//Make sure to run the code with the ID set to your configured value for DXL
//In actual modeling, it is important to note that there is a limitation of 180 degrees
//for the motor's range of motion, so it must be taken into account when modifying the code.
//Trigonometric functions only accept radian values, so be mindful of unit conversion.
```
- dynamixel의 ID는 '*DYNAMIXEL Wizard*'를 통해 본인이 설정해야한다.
- 모델링의 한계로 모터의 가동범위는 180도만 가능하다.(그 이상의 경우 고정부와 충돌)
- 삼각함수의 경우 라디안값만을 입력으로 받는다. 따라서 degree와 radian을 잘 생각하며 변환해야한다.

---
**기본세팅**
```c
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
```
- 보드는 OpenRB-150을 사용했으므로 관련된 코드 이외는 모두 삭제해주었다.
- 다른 보드를 사용할 경우 '*File-Examples-Dynamixel2Arduino*'에서 가져오면 된다.
- OpenRB-150의 보드매니저 설치의 경우 다음 링크를 참고하면 된다.
[OpenRB-150](https://emanual.robotis.com/docs/kr/parts/controller/openrb-150/#%EC%95%84%EB%91%90%EC%9D%B4%EB%85%B8-ide-%EC%84%A4%EC%B9%98)
---
**Dynamixel ID설정**
```c
//Set your DXL_ID
const uint8_t DXL_ID_0 = 0;
const uint8_t DXL_ID_1 = 1;
const uint8_t DXL_ID_2 = 2;
```
- *DYNAMIXEL Wizard*로설정한 Dynamixel ID를 넣어주면 된다.(코드의 경우 각각 0, 1, 2로 설정)
- DXL_ID_0는 Base부의 조인트이며 차례대로 DXL_ID_1, DXL_ID_2로 설정하였다.
---
**offset설정**
```c
//Set the offset angle of motor.
float j0_off  = 180.0f;//[deg]
float j1_off  = 180.0f;//[deg]
float j2_off  = 180.0f;//[deg]
```
- 조인트0(*DXL_ID_0*)과 조인트1(*DXL_ID_1*)의 offset을 설정하였다.
- -90'~+90'까지 움직일 수 있도록 180'를 초기값으로 설정하였다
  - *dynamixel의 경우 0'~359'[degree]까지만 인식*
---
**void setup**
```c
  //Set profile of velocity&acceleration
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_0, 20);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_0, 200);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_1, 20);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_1, 200);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_2, 20);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_2, 200);

  //Set to return to the motor's offset value at the initial start
  dxl.setGoalPosition(DXL_ID_0, j0_off, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_1, j1_off, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_2, j2_off, UNIT_DEGREE);
  delay(1000);
```
- '*File-Examples-Dynamixel2Arduino-basic-profile_velocity_acceleration*'을 참고해 모터의 속도와 가속도를 적절히 설정하였다.
- 처음 코드를 동작시킬 때 설정한 offset값으로 가도록 만들었다.
- delay를 안넣을 경우 값이 정확히 안나오는 경우가 종종 발생하므로 delay를 넣어주어야 한다.
---
**변수설정**
```c
int l1 = 20;  // Link1 length
int l2 = 20;  // Link2 length
int l3 = 10;  // Link3 length
int radius = 5; // End Effector's radius
int theta = 0;  //Initialize theta
float phi = 70.f; // End Effector's angle(const) = q1_IK + q2_IK + q3_IK
float q1_IK;  //Initialize q1_IK
float q2_IK;  //Initialize q2_IK
float q3_IK;  //Initialize q3_IK

void three_link_IK(int l1, int l2, int l3, float px, float py, float phi);
```
- 코드에 필요한 변수들을 선언하였다.
- 코드를 조금 더 깔끔하게 적기위해 함수*three_link_IK*를 선언하였고, 함수는 코드 제일 하단부에 정의하였다.
- c2의 값이 1인 경우, s2의 값이 0이 된다. 이는 c1, s1의 분자에 0이 곱해지는 효과가 나타나므로, 이를 방지하기 위해 phi 값을 70도로 제한.
---
**three_link_IK**
```c
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
```
- 3 link inverse kinematics를 표현한 함수이다.
- q1_IK, q2_IK, q3_IK, phi의 경우 모두 degree로 정의했기에 radian<->degree의 단위변환에 유의해야한다.
---
**Loop**
```c
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
```
- *px, py*는 end effector가 그리는 x,y좌표이고 (35, 0)을 중심으로 radius=5인 원을 그린다.
- *three_link_IK*함수에서 계산한 *q1_IK, q2_IK, q3_IK*를 불러온다.
- dynamixel의 경우 0'~359'[deg]까지만 인식하기에 theta에대한 if문을 작성하였다.
