// =================================================================
// STANDALONE POSTURE MODE TEST - Roll/Pitch/Yaw ONLY
// =================================================================
// No RX/TX, no gaits, no walking. This isolates the body-tilt IK
// (posture mode) so you can verify roll, pitch, and yaw behave
// correctly on hardware before trusting them inside the full
// RC-controlled sketch.
//
// Behavior: automatically sweeps ROLL from -max to +max and back,
// pauses, then does the same for PITCH, then YAW, then repeats the
// whole sequence. Heave stays at 0 throughout (not being tested here).
//
// WHAT TO CHECK ON HARDWARE:
//   - ROLL sweep: body should tilt side-to-side, feet stay planted
//     (don't visibly slide), legs on the "down" side extend more.
//   - PITCH sweep: body should tilt nose-down/nose-up, front/back
//     legs extend oppositely.
//   - YAW sweep: body should twist left/right while staying level
//     and at the same height (Z should NOT change during yaw-only).
//   - If a leg visibly snaps, jitters, or the robot leans wrong,
//     check Serial Monitor for "UNREACHABLE" messages from moveTo().
//
// This is the EXACT verified math from the full combined sketch's
// posture mode - only the input source changed (auto-sweep instead
// of RC sticks).
// =================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal_I2C.h>
#include <cmath>

struct LegInfo {
  int hip, thigh, knee;
  bool isRight;
  float ox, oy;
  int idx;
};

struct Vec3 {
  float x, y, z;
};

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

const float HIP_OX_FL = 120, HIP_OY_FL = -40;
const float HIP_OX_FR = 120, HIP_OY_FR = 40;
const float HIP_OX_BL = -120, HIP_OY_BL = -40;
const float HIP_OX_BR = -120, HIP_OY_BR = 40;

LegInfo legFL = {FL_HIP, FL_THIGH, FL_KNEE, 0, HIP_OX_FL, HIP_OY_FL, 0};
LegInfo legFR = {FR_HIP, FR_THIGH, FR_KNEE, 1, HIP_OX_FR, HIP_OY_FR, 1};
LegInfo legBL = {BL_HIP, BL_THIGH, BL_KNEE, 0, HIP_OX_BL, HIP_OY_BL, 2};
LegInfo legBR = {BR_HIP, BR_THIGH, BR_KNEE, 1, HIP_OX_BR, HIP_OY_BR, 3};

float curX[4] = {0, 0, 0, 0};
float curY[4] = {40, 40, 40, 40};
float curZ[4] = {30, 30, 30, 30};

int angleToPulse(float angle);

void moveTo(float x, float y, float z, int CH_HIP, int CH_THIGH, int CH_KNEE, bool isRight){
  float L1 = 40.0;
  float L2 = 120.0;
  float L3 = 130.0;

  float C = sqrt(y*y + z*z);
  if (C*C < L1*L1) {
      Serial.println("UNREACHABLE: C^2 < L1^2");
      return;
  }
  float D = sqrt(C*C - L1*L1);
  float Q1 = atan2(y, z) + atan2(D, L1);

  float B = sqrt(D*D + x*x);
  if (B < 0.00001){
    Serial.println("UNREACHABLE: B < 0.00001");
    return;
  }
  float acosArg = (B*B - L2*L2 - L3*L3) / (-2*L2*L3);
  if (acosArg > 1 || acosArg < -1){
    Serial.println("UNREACHABLE: acos arg out of range");
    return;
  }
  float Q3 = acos(acosArg);

  float asinArg = L3 * sin(Q3) / B;
  if (asinArg > 1 || asinArg < -1){
    Serial.println("UNREACHABLE: asin arg out of range");
    return;
  }
  float Q2 = asin(asinArg) + atan2(x, D);

  Q1 = Q1 * 180.0 / PI;
  Q2 = Q2 * 180.0 / PI;
  Q2 += 90;
  Q3 = Q3 * 180.0 / PI;

  if (isRight){
    Q1 = 180 - Q1;
    Q2 = 180 - Q2;
    Q3 = 180 - Q3;
  }

  if (Q1 > 180 || Q1 < 0){
    Serial.println("UNREACHABLE: Q1 out of 0-180");
    return;
  }
  if (Q2 > 180 || Q2 < 0){
    Serial.println("UNREACHABLE: Q2 out of 0-180");
    return;
  }
  if (Q3 > 180 || Q3 < 0){
    Serial.println("UNREACHABLE: Q3 out of 0-180");
    return;
  }

  pca.setPWM(CH_HIP, 0, angleToPulse(Q1));
  pca.setPWM(CH_THIGH, 0, angleToPulse(Q2));
  pca.setPWM(CH_KNEE, 0, angleToPulse(Q3));
}

int angleToPulse(float angle) {
  angle = constrain(angle, 0, 180);
  return SERVOMIN + (int)((angle / 180.0) * (SERVOMAX - SERVOMIN));
}

void moveLeg(LegInfo &leg, float x, float y, float z) {
  moveTo(x, y, z, leg.hip, leg.thigh, leg.knee, leg.isRight);
  curX[leg.idx] = x;
  curY[leg.idx] = y;
  curZ[leg.idx] = z;
}

void moveAllLegs(float x, float y, float z) {
  moveLeg(legFL, x, y, z);
  moveLeg(legFR, x, y, z);
  moveLeg(legBL, x, y, z);
  moveLeg(legBR, x, y, z);
}

// =================================================================
// POSTURE MODE MATH - exact copy from the verified combined sketch
// =================================================================

const float POSTURE_STAND_Z = 200.0;
const float POSTURE_HOME_Y = 40.0;

const float MAX_ROLL_RAD = 0.15;
const float MAX_PITCH_RAD = 0.15;
const float MAX_YAW_RAD = 0.20;

Vec3 rotateVec(Vec3 v, float roll, float pitch, float yaw) {
  float cr = cos(roll), sr = sin(roll);
  float cp = cos(pitch), sp = sin(pitch);
  float cy = cos(yaw), sy = sin(yaw);

  float x1 = v.x, y1 = cr*v.y - sr*v.z, z1 = sr*v.y + cr*v.z;
  float x2 = cp*x1 + sp*z1, y2 = y1, z2 = -sp*x1 + cp*z1;
  float x3 = cy*x2 - sy*y2, y3 = sy*x2 + cy*y2, z3 = z2;

  Vec3 result = {x3, y3, z3};
  return result;
}

Vec3 rotateVecInverse(Vec3 v, float roll, float pitch, float yaw) {
  float cr = cos(roll), sr = sin(roll);
  float cp = cos(pitch), sp = sin(pitch);
  float cy = cos(yaw), sy = sin(yaw);

  float x1 = cy*v.x + sy*v.y, y1 = -sy*v.x + cy*v.y, z1 = v.z;
  float x2 = cp*x1 - sp*z1, y2 = y1, z2 = sp*x1 + cp*z1;
  float x3 = x2, y3 = cr*y2 + sr*z2, z3 = -sr*y2 + cr*z2;

  Vec3 result = {x3, y3, z3};
  return result;
}

void applyPostureTilt(float roll, float pitch, float yaw, float heave) {
  LegInfo* legs[4] = {&legFL, &legFR, &legBL, &legBR};

  for (int i = 0; i < 4; i++) {
    Vec3 hipOffset = {legs[i]->ox, legs[i]->oy, 0};
    Vec3 standingLocal = {0, POSTURE_HOME_Y, POSTURE_STAND_Z};
    Vec3 footWorld = {hipOffset.x + standingLocal.x,
                      hipOffset.y + standingLocal.y,
                      hipOffset.z + standingLocal.z};

    Vec3 hipRotated = rotateVec(hipOffset, roll, pitch, yaw);
    Vec3 hipNew = {hipRotated.x, hipRotated.y, hipRotated.z + heave};

    Vec3 vecWorld = {footWorld.x - hipNew.x, footWorld.y - hipNew.y, footWorld.z - hipNew.z};
    Vec3 vecLocal = rotateVecInverse(vecWorld, roll, pitch, yaw);

    moveLeg(*legs[i], vecLocal.x, vecLocal.y, vecLocal.z);
  }
}

// =================================================================
// AUTO-SWEEP TEST LOGIC
// =================================================================

#define AXIS_ROLL 0
#define AXIS_PITCH 1
#define AXIS_YAW 2

int currentAxis = AXIS_ROLL;
float sweepT = 0;
const float SWEEP_SPEED = 0.005;
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL_MS = 20;

void updateSweepDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  if (currentAxis == AXIS_ROLL) lcd.print("Testing: ROLL");
  else if (currentAxis == AXIS_PITCH) lcd.print("Testing: PITCH");
  else lcd.print("Testing: YAW");
}

void setup() {
  Wire.begin();
  pca.begin();
  pca.setPWMFreq(50);
  Serial.begin(115200);
  Serial.println("--- Standalone Posture Test: Roll/Pitch/Yaw ---");

  lcd.init();
  lcd.backlight();
  updateSweepDisplay();

  moveAllLegs(0, POSTURE_HOME_Y, POSTURE_STAND_Z);
  delay(1000);
}

void loop() {
  unsigned long now = millis();
  if (now - lastUpdateTime < UPDATE_INTERVAL_MS) return;
  lastUpdateTime = now;

  float sweepVal = sin(2 * PI * sweepT - PI / 2);

  float roll = 0, pitch = 0, yaw = 0;
  if (currentAxis == AXIS_ROLL) {
    roll = sweepVal * MAX_ROLL_RAD;
  } else if (currentAxis == AXIS_PITCH) {
    pitch = sweepVal * MAX_PITCH_RAD;
  } else {
    yaw = sweepVal * MAX_YAW_RAD;
  }

  applyPostureTilt(roll, pitch, yaw, 0);

  Serial.print("axis=");
  Serial.print(currentAxis);
  Serial.print(" roll=");
  Serial.print(roll * 180.0 / PI);
  Serial.print(" pitch=");
  Serial.print(pitch * 180.0 / PI);
  Serial.print(" yaw=");
  Serial.println(yaw * 180.0 / PI);

  sweepT += SWEEP_SPEED;
  if (sweepT > 1.0) {
    sweepT = 0;
    currentAxis++;
    if (currentAxis > AXIS_YAW) {
      currentAxis = AXIS_ROLL;
    }
    updateSweepDisplay();
    applyPostureTilt(0, 0, 0, 0);
    delay(1000);
  }
}
