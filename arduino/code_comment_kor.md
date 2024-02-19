**주의사항**
```
// Note
//Make sure to run the code with the ID set to your configured value for DXL
//In actual modeling, it is important to note that there is a limitation of 180 degrees
//for the motor's range of motion, so it must be taken into account when modifying the code.
//Trigonometric functions only accept radian values, so be mindful of unit conversion.
```
- dynamixel의 ID는 '*DYNAMIXEL Wizard*'를 통해 본인이 설정해야한다. 현재 코드는 0, 1, 2로 설정되어있다.
- 모델링의 한계로 모터의 가동범위는 180도만 가능하다.(그 이상의 경우 고정부와 충돌)
  - 당신이 모델링을 어떻게 만드는 지에 따라 가동범위는 변경해주면 된다.
- 삼각함수의 경우 라디안값만을 입력으로 받는다. 따라서 degree와 radian을 잘 생각하며 변환해야한다.

---
**기본세팅**
```
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
- 다른 보드를 사용할 경우 '*File-Examples-Dynamixel2Arduino*'의 예제로 들어가 가져오면 된다.
- OpenRB-150의 보드매니저 설치의 경우 링크를 참고하면 된다.
[OpenRB-150](https://emanual.robotis.com/docs/kr/parts/controller/openrb-150/#%EC%95%84%EB%91%90%EC%9D%B4%EB%85%B8-ide-%EC%84%A4%EC%B9%98)
---
**Dynamixel ID설정**
```
//Set your DXL_ID
const uint8_t DXL_ID_0 = 0;
const uint8_t DXL_ID_1 = 1;
```

## two_joint_planar_half_circle
> dynamixel 모터 2개를 사용하여 반원을 그리는 코드이다.
