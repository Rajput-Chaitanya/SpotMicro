#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal_I2C.h>
#include <cmath>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
LiquidCrystal_I2C lcd(0x27, 16, 2);  

#define SERVOMIN 150
#define SERVOMAX 600

#define BL_KNEE  0
#define BL_THIGH 2
#define BL_HIP 3
#define BR_KNEE 1
#define BR_THIGH 4
#define BR_HIP 5
#define FL_KNEE 15
#define FL_THIGH 13
#define FL_HIP 11
#define FR_KNEE 14
#define FR_THIGH 12
#define FR_HIP 10

//home cords
float X = 0;
float Y = 40;
float Z = 30;
//basic control
float H = 200; // height of whole robot from ground
//step trajectory
float SL = 100;//steplength
float SH = 50;//stepheight
float speed = 1.0;
int baseDelay = 10;
int baseSteps = 30; 
//offsets from center of body
const float HIP_OX_FL = 120, HIP_OY_FL = -40;
const float HIP_OX_FR = 120, HIP_OY_FR = 40;
const float HIP_OX_BL = -120, HIP_OY_BL = -40;
const float HIP_OX_BR = -120, HIP_OY_BR = 40;

float bezier(float t, float p0, float p1, float p2, float p3) {
  float u = 1 - t;
  return u*u*u*p0 + 3*u*u*t*p1 + 3*u*t*t*p2 + t*t*t*p3;
}

void moveTo(float x,float y,float z, int CH_HIP, int CH_THIGH, int CH_KNEE, bool isRight){
  float L1 = 40.0; //coxa
  float L2 = 120.0; //femur
  float L3 = 130.0;  //tibia

  float C = sqrt(y*y + z*z);
  if (C*C < L1*L1) {
      Serial.println("POSITION UNREACHABLE!!, C^2 < L1^2");
      return;
  }
  float D = sqrt(C*C - L1*L1);
  float Q1 = atan2(y, z) + atan2(D, L1);
  
  float B = sqrt(D*D + x*x);

  if (B < 0.00001){
    Serial.println("POSITION UNREACHABLE!!, value of B is less than 0.00001");
    return;
  }
  if (((B*B - L2*L2 - L3*L3) / (-2*L2*L3))>1 || ((B*B - L2*L2 - L3*L3) / (-2*L2*L3))<-1){
    Serial.println("POSITION UNREACHABLE!!, value inside acos is not between -1 and 1");
    return;
  }
  float Q3 = acos((B*B - L2*L2 - L3*L3) / (-2*L2*L3));

  if ((L3 * sin(Q3) / B)>1 || (L3 * sin(Q3) / B)<-1){
    Serial.println("POSITION UNREACHABLE!!, value inside asin is not between -1 and 1");
    return;
  }
  
  float Q2 = asin(L3 * sin(Q3) / B) + atan2(x, D);

  Q1 = Q1 * 180.0 / PI;
  Q2 = Q2 * 180.0 / PI; 
  Q2 += 90; 
  Q3 = Q3 * 180.0 / PI;

  if (isRight){
    Q1= 180-Q1;
    Q2= 180-Q2;
    Q3= 180-Q3;
  }
/*
  if (Q1>180 || Q1<0){
    Serial.println("POSITION UNREACHABLE!!, Q1 not within 0 and 180");
    return;
  }
  if (Q2>180 || Q2<0){
    Serial.println("POSITION UNREACHABLE!!, Q2 not within 0 and 180");
    return;
  }
  if (Q3>180 || Q3<0){
    Serial.println("POSITION UNREACHABLE!!, Q3 not within 0 and 180");
    return;
  }
  Serial.print("HIP: ");
  Serial.print(Q1);
  Serial.print("  THIGH: ");
  Serial.print(Q2);
  Serial.print("  KNEE: ");
  Serial.println(Q3);
*/
  pca.setPWM(CH_HIP, 0, angleToPulse(Q1));
  pca.setPWM(CH_THIGH, 0, angleToPulse(Q2));
  pca.setPWM(CH_KNEE, 0, angleToPulse(Q3));

}

int angleToPulse(float angle) {
  return SERVOMIN + (angle / 180.0) * (SERVOMAX - SERVOMIN);
}

void setup(){
  Wire.begin();
  pca.begin();
  pca.setPWMFreq(50);
  Serial.begin(115200);
  Serial.println("-------------simple IK model-----------");
}

void loop(){
  
  trotGait();
  trotGait();
  trotGait();
  trotGait();
  trotGait();

  delay(1000);
  walkGaitForward();
  walkGaitForward();
  walkGaitForward();
  walkGaitForward();
  walkGaitForward();
  delay(1000);
  walkGaitBackward();
  walkGaitBackward();
  walkGaitBackward();
  walkGaitBackward();
  walkGaitBackward();
  delay(1000);
  sideWalkGait(true);   // strafe right
  sideWalkGait(true);
  sideWalkGait(true);
  sideWalkGait(true);
  sideWalkGait(true);
  delay(1000);
  sideWalkGait(false);  // strafe left
  sideWalkGait(false);
  sideWalkGait(false);
  sideWalkGait(false);
  sideWalkGait(false);
  delay(1000);
  rotateGait(true);     // rotate clockwise
  rotateGait(true);
  rotateGait(true);
  rotateGait(true);
  rotateGait(true);

  delay(1000);
  rotateGait(false);    // rotate counter-clockwise
  rotateGait(false);
  rotateGait(false);
  rotateGait(false);
  rotateGait(false);
  delay(1000);
  paceGait();
  paceGait();
  paceGait();
  paceGait();
  paceGait();
  delay(1000);
  boundGait();
  boundGait();
  boundGait();
  boundGait();
  boundGait();
  delay(1000);
}



/* SINE WAVE STEP

void loop(){

  for (float X = -SL/2; X<= SL/2; X+=1){
    moveTo(X, Y, H, BR_HIP, BR_THIGH, BR_KNEE, 1);
    delay(10);
  }

  for (float X = SL/2; X>= -SL/2; X = X-1){
    float Z = H - SH * sin(PI * (X + SL/2) / SL);
    moveTo(X, Y, Z, BR_HIP, BR_THIGH, BR_KNEE, 1);
    delay(10);
  }

}*/

void getGaitTiming(int &steps, int &stepDelay) {
  stepDelay = (int)(baseDelay / speed);
  steps = (int)(baseSteps * speed);
  if (stepDelay < 1) stepDelay = 1;
  if (steps < 5) steps = 5;
}
void moveAllLegs(float x, float y, float z) {
  moveTo(x, y, z, BR_HIP, BR_THIGH, BR_KNEE, 1);
  moveTo(x, y, z, BL_HIP, BL_THIGH, BL_KNEE, 0);
  moveTo(x, y, z, FR_HIP, FR_THIGH, FR_KNEE, 1);
  moveTo(x, y, z, FL_HIP, FL_THIGH, FL_KNEE, 0);
}
void legSwingBezier(int CH_HIP, int CH_THIGH, int CH_KNEE, bool isRight,
                     float xStart, float xEnd, float liftMultiplier) {
  int steps, stepDelay;
  getGaitTiming(steps, stepDelay);

  float xP0 = xStart, xP1 = xStart*0.5 + xEnd*0.5 - (xEnd-xStart)*0.25,
        xP2 = xStart*0.5 + xEnd*0.5 + (xEnd-xStart)*0.25, xP3 = xEnd;
  float zP0 = H, zP1 = H - SH*liftMultiplier, zP2 = H - SH*liftMultiplier, zP3 = H;

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    float sx = bezier(t, xP0, xP1, xP2, xP3);
    float sz = bezier(t, zP0, zP1, zP2, zP3);
    moveTo(sx, Y, sz, CH_HIP, CH_THIGH, CH_KNEE, isRight);
    delay(stepDelay);
  }
}
void walkGaitForward() {
  int steps, stepDelay;
  getGaitTiming(steps, stepDelay);

  struct LegRef { int hip, thigh, knee; bool isRight; };
  LegRef legs[4] = {
    {FL_HIP, FL_THIGH, FL_KNEE, 0},
    {BR_HIP, BR_THIGH, BR_KNEE, 1},
    {FR_HIP, FR_THIGH, FR_KNEE, 1},
    {BL_HIP, BL_THIGH, BL_KNEE, 0}
  };

  for (int leg = 0; leg < 4; leg++) {
    legSwingBezier(legs[leg].hip, legs[leg].thigh, legs[leg].knee, legs[leg].isRight,
                   -SL/2, SL/2, 1.3);
   
    for (int i = 0; i <= steps/4; i++) {
      float t = (float)i / (steps/4);
      float sx = SL/2 - t * (SL/4); 
      for (int other = 0; other < 4; other++) {
        if (other == leg) continue;
        moveTo(sx, Y, H, legs[other].hip, legs[other].thigh, legs[other].knee, legs[other].isRight);
      }
      delay(stepDelay);
    }
  }
}
void walkGaitBackward() {
  int steps, stepDelay;
  getGaitTiming(steps, stepDelay);

  struct LegRef { int hip, thigh, knee; bool isRight; };
  LegRef legs[4] = {
    {FL_HIP, FL_THIGH, FL_KNEE, 0},
    {BR_HIP, BR_THIGH, BR_KNEE, 1},
    {FR_HIP, FR_THIGH, FR_KNEE, 1},
    {BL_HIP, BL_THIGH, BL_KNEE, 0}
  };

  for (int leg = 0; leg < 4; leg++) {
    legSwingBezier(legs[leg].hip, legs[leg].thigh, legs[leg].knee, legs[leg].isRight,
                   SL/2, -SL/2, 1.3);

    for (int i = 0; i <= steps/4; i++) {
      float t = (float)i / (steps/4);
      float sx = -SL/2 + t * (SL/4); 
      for (int other = 0; other < 4; other++) {
        if (other == leg) continue;
        moveTo(sx, Y, H, legs[other].hip, legs[other].thigh, legs[other].knee, legs[other].isRight);
      }
      delay(stepDelay);
    }
  }
}
void sideWalkGait(bool strafeRight) {
  int steps, stepDelay;
  getGaitTiming(steps, stepDelay);

  float sign = strafeRight ? 1.0 : -1.0;
  float yP0 = 40 * sign, yP1 = 20*sign, yP2 = -20*sign, yP3 = -40*sign;

  float zP0 = H, zP1 = H - SH*1.3, zP2 = H - SH*1.3, zP3 = H;
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    float sy = bezier(t, yP0, yP1, yP2, yP3);
    float sz = bezier(t, zP0, zP1, zP2, zP3);
    moveAllLegs(X, sy, sz);
    delay(stepDelay);
  }

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    float sy = -40*sign + t * (80*sign);
    moveAllLegs(X, sy, H);
    delay(stepDelay);
  }
}
void rotateGait(bool isCW) {
  int steps, stepDelay;
  getGaitTiming(steps, stepDelay);

  float dir = isCW ? -1.0 : 1.0; // flip rotation direction

  struct LegInfo { int hip, thigh, knee; bool isRight; float ox, oy; };
  LegInfo legs[4] = {
    {FL_HIP, FL_THIGH, FL_KNEE, 0, HIP_OX_FL, HIP_OY_FL},
    {FR_HIP, FR_THIGH, FR_KNEE, 1, HIP_OX_FR, HIP_OY_FR},
    {BL_HIP, BL_THIGH, BL_KNEE, 0, HIP_OX_BL, HIP_OY_BL},
    {BR_HIP, BR_THIGH, BR_KNEE, 1, HIP_OX_BR, HIP_OY_BR}
  };

  float rotAmount = 20.0; 

  float tx[4], ty[4];
  for (int i = 0; i < 4; i++) {
    float r = sqrt(legs[i].ox*legs[i].ox + legs[i].oy*legs[i].oy);
    tx[i] = -legs[i].oy / r * dir;
    ty[i] =  legs[i].ox / r * dir;
  }

  int pairA[2] = {0, 3}; // FL, BR
  int pairB[2] = {1, 2}; // FR, BL

  for (int cycle = 0; cycle < 2; cycle++) {
    int *swingPair = (cycle == 0) ? pairA : pairB;
    int *stancePair = (cycle == 0) ? pairB : pairA;

    for (int i = 0; i <= steps; i++) {
      float t = (float)i / steps;
      float lift = H - SH * 1.3 * sin(PI * t); 
      for (int p = 0; p < 2; p++) {
        int idx = swingPair[p];
        float sx = X + tx[idx] * rotAmount * (t - 0.5) * 2; 
        float sy = Y + ty[idx] * rotAmount * (t - 0.5) * 2;
        moveTo(sx, sy, lift, legs[idx].hip, legs[idx].thigh, legs[idx].knee, legs[idx].isRight);
      }

      for (int p = 0; p < 2; p++) {
        int idx = stancePair[p];
        float sx = X - tx[idx] * rotAmount * (t - 0.5) * 2;
        float sy = Y - ty[idx] * rotAmount * (t - 0.5) * 2;
        moveTo(sx, sy, H, legs[idx].hip, legs[idx].thigh, legs[idx].knee, legs[idx].isRight);
      }
      delay(stepDelay);
    }
  }
}
void trotGait() {
  int steps, stepDelay;
  getGaitTiming(steps, stepDelay);

  struct LegInfo { int hip, thigh, knee; bool isRight; };
  LegInfo FL = {FL_HIP, FL_THIGH, FL_KNEE, 0};
  LegInfo FR = {FR_HIP, FR_THIGH, FR_KNEE, 1};
  LegInfo BL = {BL_HIP, BL_THIGH, BL_KNEE, 0};
  LegInfo BR = {BR_HIP, BR_THIGH, BR_KNEE, 1};

  float xP0 = -SL/2, xP1 = -SL/4, xP2 = SL/4, xP3 = SL/2;
  float zP0 = H, zP1 = H - SH*1.3, zP2 = H - SH*1.3, zP3 = H;

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    float swingX = bezier(t, xP0, xP1, xP2, xP3);
    float swingZ = bezier(t, zP0, zP1, zP2, zP3);
    float stanceX = SL/2 - t * SL;

    moveTo(swingX, Y, swingZ, FL.hip, FL.thigh, FL.knee, FL.isRight);
    moveTo(swingX, Y, swingZ, BR.hip, BR.thigh, BR.knee, BR.isRight);
    moveTo(stanceX, Y, H, FR.hip, FR.thigh, FR.knee, FR.isRight);
    moveTo(stanceX, Y, H, BL.hip, BL.thigh, BL.knee, BL.isRight);
    delay(stepDelay);
  }

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    float swingX = bezier(t, xP0, xP1, xP2, xP3);
    float swingZ = bezier(t, zP0, zP1, zP2, zP3);
    float stanceX = SL/2 - t * SL;

    moveTo(swingX, Y, swingZ, FR.hip, FR.thigh, FR.knee, FR.isRight);
    moveTo(swingX, Y, swingZ, BL.hip, BL.thigh, BL.knee, BL.isRight);
    moveTo(stanceX, Y, H, FL.hip, FL.thigh, FL.knee, FL.isRight);
    moveTo(stanceX, Y, H, BR.hip, BR.thigh, BR.knee, BR.isRight);
    delay(stepDelay);
  }
}
void paceGait() {
  int steps, stepDelay;
  getGaitTiming(steps, stepDelay);

  struct LegInfo { int hip, thigh, knee; bool isRight; };
  LegInfo FL = {FL_HIP, FL_THIGH, FL_KNEE, 0};
  LegInfo FR = {FR_HIP, FR_THIGH, FR_KNEE, 1};
  LegInfo BL = {BL_HIP, BL_THIGH, BL_KNEE, 0};
  LegInfo BR = {BR_HIP, BR_THIGH, BR_KNEE, 1};

  float xP0 = -SL/2, xP1 = -SL/4, xP2 = SL/4, xP3 = SL/2;
  float zP0 = H, zP1 = H - SH*1.3, zP2 = H - SH*1.3, zP3 = H;

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    float swingX = bezier(t, xP0, xP1, xP2, xP3);
    float swingZ = bezier(t, zP0, zP1, zP2, zP3);
    float stanceX = SL/2 - t * SL;

    moveTo(swingX, Y, swingZ, FL.hip, FL.thigh, FL.knee, FL.isRight);
    moveTo(swingX, Y, swingZ, BL.hip, BL.thigh, BL.knee, BL.isRight);
    moveTo(stanceX, Y, H, FR.hip, FR.thigh, FR.knee, FR.isRight);
    moveTo(stanceX, Y, H, BR.hip, BR.thigh, BR.knee, BR.isRight);
    delay(stepDelay);
  }

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    float swingX = bezier(t, xP0, xP1, xP2, xP3);
    float swingZ = bezier(t, zP0, zP1, zP2, zP3);
    float stanceX = SL/2 - t * SL;

    moveTo(swingX, Y, swingZ, FR.hip, FR.thigh, FR.knee, FR.isRight);
    moveTo(swingX, Y, swingZ, BR.hip, BR.thigh, BR.knee, BR.isRight);
    moveTo(stanceX, Y, H, FL.hip, FL.thigh, FL.knee, FL.isRight);
    moveTo(stanceX, Y, H, BL.hip, BL.thigh, BL.knee, BL.isRight);
    delay(stepDelay);
  }
}
void boundGait() {
  int steps, stepDelay;
  getGaitTiming(steps, stepDelay);

  struct LegInfo { int hip, thigh, knee; bool isRight; };
  LegInfo FL = {FL_HIP, FL_THIGH, FL_KNEE, 0};
  LegInfo FR = {FR_HIP, FR_THIGH, FR_KNEE, 1};
  LegInfo BL = {BL_HIP, BL_THIGH, BL_KNEE, 0};
  LegInfo BR = {BR_HIP, BR_THIGH, BR_KNEE, 1};

  float xP0 = -SL/2, xP1 = -SL/4, xP2 = SL/4, xP3 = SL/2;
  float zP0 = H, zP1 = H - SH*1.3, zP2 = H - SH*1.3, zP3 = H;

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    float swingX = bezier(t, xP0, xP1, xP2, xP3);
    float swingZ = bezier(t, zP0, zP1, zP2, zP3);
    float stanceX = SL/2 - t * SL;

    moveTo(swingX, Y, swingZ, FL.hip, FL.thigh, FL.knee, FL.isRight);
    moveTo(swingX, Y, swingZ, FR.hip, FR.thigh, FR.knee, FR.isRight);
    moveTo(stanceX, Y, H, BL.hip, BL.thigh, BL.knee, BL.isRight);
    moveTo(stanceX, Y, H, BR.hip, BR.thigh, BR.knee, BR.isRight);
    delay(stepDelay);
  }

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    float swingX = bezier(t, xP0, xP1, xP2, xP3);
    float swingZ = bezier(t, zP0, zP1, zP2, zP3);
    float stanceX = SL/2 - t * SL;

    moveTo(swingX, Y, swingZ, BL.hip, BL.thigh, BL.knee, BL.isRight);
    moveTo(swingX, Y, swingZ, BR.hip, BR.thigh, BR.knee, BR.isRight);
    moveTo(stanceX, Y, H, FL.hip, FL.thigh, FL.knee, FL.isRight);
    moveTo(stanceX, Y, H, FR.hip, FR.thigh, FR.knee, FR.isRight);
    delay(stepDelay);
  }
}