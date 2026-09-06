// =================================================================
// SPOTMICRO QUADRUPED - FULL COMBINED SKETCH
// =================================================================
// Assembled from all stages: IK core, non-blocking gaits (trot/pace/
// bound/walk/side/rotate), blended-input trot, IBUS RC receiver,
// robot state machine (sleep/stand/estop), gait selection, RC
// dispatcher (locomotion), posture/attitude mode with body-tilt IK,
// Ch7 locomotion<->posture mode toggle, and Ch8 MPU6050 self-balancing.
//
// HARDWARE SETUP REQUIRED BEFORE USE:
//   1. Set IBUS_SERIAL / IBUS_RX_PIN below to match your wiring.
//   2. Confirm LiquidCrystal_I2C library has .init() (some forks use
//      .begin() instead) - swap in setup() if needed.
//   3. Verify MPU6050 mounting orientation against measuredRoll/
//      measuredPitch by tilting the robot by hand (see stage5 notes).
//   4. Test each gait individually at low speed before trusting the
//      full RC-driven system - see testing order notes at the bottom.
//
// CHANNEL MAP (0-indexed internally, Ch1-10 as labeled on your TX):
//   Ch1 (idx 0) - Right stick horizontal - Strafe (locomotion) / Yaw (posture)
//   Ch2 (idx 1) - Right stick vertical   - Walk fwd/back (locomotion) / Pitch (posture)
//   Ch3 (idx 2) - Left stick vertical    - Body height (locomotion) / Heave (posture)
//   Ch4 (idx 3) - Left stick horizontal  - Turn (locomotion) / Roll (posture)
//   Ch5 (idx 4) - SWB 3-position         - Gait select (Trot/Pace/Bound)
//   Ch6 (idx 5) - SWC 3-position         - Sleep/Stand/E-Stop
//   Ch7 (idx 6) - SWA 2-position         - Locomotion mode <-> Posture mode
//   Ch8 (idx 7) - SWD 2-position         - Self-balancing on/off (locomotion only)
// =================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <cmath>

struct LegInfo {
  int hip, thigh, knee;
  bool isRight;
  float ox, oy;
  int idx;
};

struct GaitRunner {
  bool active = false;
  int phase = 1;
  int stepIndex = 0;
  int totalSteps = 32;
  unsigned long lastStepTime = 0;
  unsigned long stepDelayMs = 10;
};
struct WalkRunner {
  bool active = false;
  int currentLeg = 0;
  int stepIndex = 0;
  int totalSteps = 32;
  unsigned long lastStepTime = 0;
  unsigned long stepDelayMs = 10;
  float legX[4];
  float dir = 1.0;
  float xP0, xP1, xP2, xP3;
};
struct SideRunner {
  bool active = false;
  int phase = 0;
  int stepIndex = 0;
  int totalSteps = 32;
  unsigned long lastStepTime = 0;
  unsigned long stepDelayMs = 10;
  float sign = 1.0;
};
struct RotateRunner {
  bool active = false;
  int cycle = 0;
  int stepIndex = 0;
  int totalSteps = 32;
  unsigned long lastStepTime = 0;
  unsigned long stepDelayMs = 10;
  float dir = 1.0;
  float tx[4], ty[4];
};
struct BlendedTrotRunner {
  bool active = false;
  int phase = 1;
  int stepIndex = 0;
  int totalSteps = 32;
  unsigned long lastStepTime = 0;
  unsigned long stepDelayMs = 10;

  float walkStick = 0;
  float strafeStick = 0;
  float turnStick = 0;

  float tx[4], ty[4];
};
struct GoToZRunner {
  bool active = false;
  int stepIndex = 0;
  int totalSteps = 32;
  unsigned long lastStepTime = 0;
  unsigned long stepDelayMs = 10;
  float startZ[4];
  float targetZ;
};
struct Vec3 {
  float x, y, z;
};

// Forward declaration: moveLeg() (defined below) calls balanceCorrectionZ(),
// which is defined much later (Stage 5) - this declaration lets the compiler
// resolve the call without needing balanceCorrectionZ()'s full body yet.
float balanceCorrectionZ(LegInfo &leg);

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_MPU6050 mpu;

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

float X = 0;
float Y = 40;
float Z = 30;
float H = 200;
float SL = 100;
float SH = 50;
float speed = 1.0;
int baseDelay = 10;
int baseSteps = 32;
const float FWD = -1.0;

const float HIP_OX_FL = 120, HIP_OY_FL = -40;
const float HIP_OX_FR = 120, HIP_OY_FR = 40;
const float HIP_OX_BL = -120, HIP_OY_BL = -40;
const float HIP_OX_BR = -120, HIP_OY_BR = 40;

int angleToPulse(float angle);

float curX[4] = {0, 0, 0, 0};
float curY[4] = {40, 40, 40, 40};
float curZ[4] = {30, 30, 30, 30};

float bezier(float t, float p0, float p1, float p2, float p3) {
  float u = 1 - t;
  return u*u*u*p0 + 3*u*u*t*p1 + 3*u*t*t*p2 + t*t*t*p3;
}

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

void getGaitTiming(int &steps, int &stepDelay) {
  steps = (int)(baseSteps * speed);
  stepDelay = (int)(baseDelay / speed);
  if (steps < 8) steps = 8;
  if (stepDelay < 1) stepDelay = 1;
}

LegInfo legFL = {FL_HIP, FL_THIGH, FL_KNEE, 0, HIP_OX_FL, HIP_OY_FL, 0};
LegInfo legFR = {FR_HIP, FR_THIGH, FR_KNEE, 1, HIP_OX_FR, HIP_OY_FR, 1};
LegInfo legBL = {BL_HIP, BL_THIGH, BL_KNEE, 0, HIP_OX_BL, HIP_OY_BL, 2};
LegInfo legBR = {BR_HIP, BR_THIGH, BR_KNEE, 1, HIP_OX_BR, HIP_OY_BR, 3};

void moveLeg(LegInfo &leg, float x, float y, float z) {
  float z_corrected = z + balanceCorrectionZ(leg);
  moveTo(x, y, z_corrected, leg.hip, leg.thigh, leg.knee, leg.isRight);
  curX[leg.idx] = x;
  curY[leg.idx] = y;
  curZ[leg.idx] = z_corrected;
}

void moveAllLegs(float x, float y, float z) {
  moveLeg(legFL, x, y, z);
  moveLeg(legFR, x, y, z);
  moveLeg(legBL, x, y, z);
  moveLeg(legBR, x, y, z);
}

void showGait(const char* name) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Gait:");
  lcd.setCursor(0, 1);
  lcd.print(name);
}



// =================================================================
// STAGE 1: IBUS RECEIVER PARSER (FS-iA6B, 10 channels, single wire)
// =================================================================
// Protocol: 32-byte packets over UART @ 115200 baud
//   byte0 = 0x20 (packet length)
//   byte1 = 0x40 (command: channel data)
//   bytes 2-29 = 14 channels x 2 bytes, little-endian, ~1000-2000us
//   bytes 30-31 = checksum (0xFFFF - sum of bytes 0-29), little-endian
// =================================================================

#define IBUS_SERIAL Serial2   // CHANGE THIS to whichever port your iBUS wire is on
#define IBUS_BAUD 115200
#define IBUS_RX_PIN 16        // CHANGE THIS to your actual RX pin (TX pin unused, iBUS is RX-only into ESP32)
#define IBUS_NUM_CHANNELS 10

uint16_t ibusChannels[IBUS_NUM_CHANNELS];
bool ibusFrameValid = false;

void ibusInit() {
  IBUS_SERIAL.begin(IBUS_BAUD, SERIAL_8N1, IBUS_RX_PIN, -1);
  for (int i = 0; i < IBUS_NUM_CHANNELS; i++) {
    ibusChannels[i] = 1500;
  }
}

void ibusRead() {
  static uint8_t buf[32];
  static uint8_t idx = 0;
  static unsigned long lastByteTime = 0;

  unsigned long now = millis();
  if (idx > 0 && (now - lastByteTime) > 5) {
    idx = 0;
  }

  while (IBUS_SERIAL.available()) {
    uint8_t b = IBUS_SERIAL.read();
    lastByteTime = millis();

    if (idx == 0 && b != 0x20) {
      continue;
    }
    if (idx == 1 && b != 0x40) {
      idx = 0;
      continue;
    }

    buf[idx++] = b;

    if (idx >= 32) {
      uint16_t checksum = 0xFFFF;
      for (int i = 0; i < 30; i++) {
        checksum -= buf[i];
      }
      uint16_t recvChecksum = buf[30] | (buf[31] << 8);

      if (checksum == recvChecksum) {
        for (int ch = 0; ch < IBUS_NUM_CHANNELS; ch++) {
          ibusChannels[ch] = buf[2 + ch*2] | (buf[3 + ch*2] << 8);
        }
        ibusFrameValid = true;
      }
      idx = 0;
    }
  }
}

uint16_t ibusGetChannel(int channelIndex) {
  if (channelIndex < 0 || channelIndex >= IBUS_NUM_CHANNELS) return 1500;
  return ibusChannels[channelIndex];
}

float ibusGetNormalized(int channelIndex) {
  uint16_t raw = ibusGetChannel(channelIndex);
  float norm = (raw - 1500) / 500.0;
  return constrain(norm, -1.0, 1.0);
}

int ibusGetSwitchPosition(int channelIndex, int numPositions) {
  uint16_t raw = ibusGetChannel(channelIndex);
  if (numPositions == 2) {
    return (raw < 1500) ? 0 : 1;
  } else if (numPositions == 3) {
    if (raw < 1300) return 0;
    if (raw > 1700) return 2;
    return 1;
  }
  return 0;
}


// =================================================================
// STICK-PROPORTIONAL SCALING (replaces VRA/VRB knob-based scaling)
// =================================================================
// Each locomotion direction (walk, strafe, turn) computes its OWN
// stride length / speed from how far ITS OWN stick is pushed -
// no shared/combined scaling between directions, and no VRA/VRB
// knob involvement at all.
// =================================================================

const float MAX_SL = 100.0;
const float MAX_SH = 50.0;
const float MAX_SPEED = 1.0;
const float MIN_SPEED = 0.3;
const float STICK_DEADZONE = 0.08;

const float MAX_ROT_AMOUNT = 25.0;

bool stickActive(float stickNorm) {
  return fabs(stickNorm) > STICK_DEADZONE;
}

void applyStickScaling(float stickNorm) {
  float mag = fabs(stickNorm);
  SL = MAX_SL * mag;
  SH = MAX_SH * mag;
  speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * mag;
}

float rotAmountFromStick(float stickNorm) {
  float mag = fabs(stickNorm);
  return MAX_ROT_AMOUNT * mag;
}


// =================================================================
// STAGE (non-blocking rewrite, part 1): trotGait, paceGait, boundGait, goHome
// =================================================================
// These replace the BLOCKING versions of the same names from your main
// sketch. Call the matching *_Step() function ONCE per loop() iteration;
// it internally checks timing via millis() and does nothing if it's not
// time for the next step yet. Each gait must be "started" once via its
// *_Start() function before stepping it.
// =================================================================


GaitRunner trotRunner;
GaitRunner paceRunner;
GaitRunner boundRunner;
GaitRunner homeRunner;

void gaitRunnerStart(GaitRunner &r) {
  int steps, stepDelay;
  getGaitTiming(steps, stepDelay);
  r.active = true;
  r.phase = 1;
  r.stepIndex = 0;
  r.totalSteps = steps;
  r.stepDelayMs = stepDelay;
  r.lastStepTime = millis();
}

bool gaitRunnerReady(GaitRunner &r) {
  if (!r.active) return false;
  unsigned long now = millis();
  if (now - r.lastStepTime < r.stepDelayMs) return false;
  r.lastStepTime = now;
  return true;
}

bool gaitRunnerAdvance(GaitRunner &r, int numPhases) {
  r.stepIndex++;
  bool cycleComplete = false;
  if (r.stepIndex > r.totalSteps) {
    r.stepIndex = 0;
    r.phase++;
    if (r.phase > numPhases) {
      r.phase = 1;
      cycleComplete = true;
    }
  }
  return cycleComplete;
}

void trotGaitStart() {
  gaitRunnerStart(trotRunner);
}

bool trotCycleJustCompleted = false;

bool trotGaitStep() {
  trotCycleJustCompleted = false;
  if (!gaitRunnerReady(trotRunner)) return false;

  float t = (float)trotRunner.stepIndex / trotRunner.totalSteps;
  float xP0 = -SL/2*FWD, xP1 = -SL/4*FWD, xP2 = SL/4*FWD, xP3 = SL/2*FWD;
  float zP0 = H, zP1 = H - SH*1.3, zP2 = H - SH*1.3, zP3 = H;
  float swingX = bezier(t, xP0, xP1, xP2, xP3);
  float swingZ = bezier(t, zP0, zP1, zP2, zP3);
  float stanceX = SL/2*FWD - t * SL*FWD;

  if (trotRunner.phase == 1) {
    moveLeg(legFL, swingX, Y, swingZ);
    moveLeg(legBR, swingX, Y, swingZ);
    moveLeg(legFR, stanceX, Y, H);
    moveLeg(legBL, stanceX, Y, H);
  } else {
    moveLeg(legFR, swingX, Y, swingZ);
    moveLeg(legBL, swingX, Y, swingZ);
    moveLeg(legFL, stanceX, Y, H);
    moveLeg(legBR, stanceX, Y, H);
  }

  trotCycleJustCompleted = gaitRunnerAdvance(trotRunner, 2);
  return true;
}

void paceGaitStart() {
  gaitRunnerStart(paceRunner);
}

bool paceCycleJustCompleted = false;

bool paceGaitStep() {
  paceCycleJustCompleted = false;
  if (!gaitRunnerReady(paceRunner)) return false;

  float t = (float)paceRunner.stepIndex / paceRunner.totalSteps;
  float xP0 = -SL/2*FWD, xP1 = -SL/4*FWD, xP2 = SL/4*FWD, xP3 = SL/2*FWD;
  float zP0 = H, zP1 = H - SH*1.3, zP2 = H - SH*1.3, zP3 = H;
  float swingX = bezier(t, xP0, xP1, xP2, xP3);
  float swingZ = bezier(t, zP0, zP1, zP2, zP3);
  float stanceX = SL/2*FWD - t * SL*FWD;

  if (paceRunner.phase == 1) {
    moveLeg(legFL, swingX, Y, swingZ);
    moveLeg(legBL, swingX, Y, swingZ);
    moveLeg(legFR, stanceX, Y, H);
    moveLeg(legBR, stanceX, Y, H);
  } else {
    moveLeg(legFR, swingX, Y, swingZ);
    moveLeg(legBR, swingX, Y, swingZ);
    moveLeg(legFL, stanceX, Y, H);
    moveLeg(legBL, stanceX, Y, H);
  }

  paceCycleJustCompleted = gaitRunnerAdvance(paceRunner, 2);
  return true;
}

void boundGaitStart() {
  gaitRunnerStart(boundRunner);
}

bool boundCycleJustCompleted = false;

bool boundGaitStep() {
  boundCycleJustCompleted = false;
  if (!gaitRunnerReady(boundRunner)) return false;

  float t = (float)boundRunner.stepIndex / boundRunner.totalSteps;
  float xP0 = -SL/2*FWD, xP1 = -SL/4*FWD, xP2 = SL/4*FWD, xP3 = SL/2*FWD;
  float zP0 = H, zP1 = H - SH*1.3, zP2 = H - SH*1.3, zP3 = H;
  float swingX = bezier(t, xP0, xP1, xP2, xP3);
  float swingZ = bezier(t, zP0, zP1, zP2, zP3);
  float stanceX = SL/2*FWD - t * SL*FWD;

  if (boundRunner.phase == 1) {
    moveLeg(legFL, swingX, Y, swingZ);
    moveLeg(legFR, swingX, Y, swingZ);
    moveLeg(legBL, stanceX, Y, H);
    moveLeg(legBR, stanceX, Y, H);
  } else {
    moveLeg(legBL, swingX, Y, swingZ);
    moveLeg(legBR, swingX, Y, swingZ);
    moveLeg(legFL, stanceX, Y, H);
    moveLeg(legFR, stanceX, Y, H);
  }

  boundCycleJustCompleted = gaitRunnerAdvance(boundRunner, 2);
  return true;
}

float homeStartX[4], homeStartY[4], homeStartZ[4];
float homeTargetX, homeTargetY, homeTargetZ;

void goHomeStart(float targetX, float targetY, float targetZ) {
  int steps, stepDelay;
  getGaitTiming(steps, stepDelay);
  homeRunner.active = true;
  homeRunner.stepIndex = 0;
  homeRunner.totalSteps = steps;
  homeRunner.stepDelayMs = stepDelay;
  homeRunner.lastStepTime = millis();

  for (int i = 0; i < 4; i++) {
    homeStartX[i] = curX[i];
    homeStartY[i] = curY[i];
    homeStartZ[i] = curZ[i];
  }
  homeTargetX = targetX;
  homeTargetY = targetY;
  homeTargetZ = targetZ;
}

bool goHomeStep() {
  if (!gaitRunnerReady(homeRunner)) return false;

  float t = (float)homeRunner.stepIndex / homeRunner.totalSteps;
  LegInfo* legs[4] = {&legFL, &legFR, &legBL, &legBR};
  for (int L = 0; L < 4; L++) {
    float sx = homeStartX[L] + (homeTargetX - homeStartX[L]) * t;
    float sy = homeStartY[L] + (homeTargetY - homeStartY[L]) * t;
    float sz = homeStartZ[L] + (homeTargetZ - homeStartZ[L]) * t;
    moveLeg(*legs[L], sx, sy, sz);
  }

  homeRunner.stepIndex++;
  if (homeRunner.stepIndex > homeRunner.totalSteps) {
    homeRunner.active = false;
    return false;
  }
  return true;
}

bool goHomeIsDone() {
  return !homeRunner.active;
}


// =================================================================
// STAGE (non-blocking rewrite, part 2): walkGaitForward/Backward,
// sideWalkGait, rotateGait
// =================================================================
// Same pattern as part 1: call *_Start() once, then *_Step() every
// loop() iteration. Requires stage_nb1_trot_pace_bound.ino's
// GaitRunner struct/helpers, and the main sketch's
// moveLeg/moveAllLegs/getGaitTiming/bezier/leg objects.
// =================================================================

WalkRunner walkRunner;

bool gaitRunnerReady2(WalkRunner &r) {
  if (!r.active) return false;
  unsigned long now = millis();
  if (now - r.lastStepTime < r.stepDelayMs) return false;
  r.lastStepTime = now;
  return true;
}

SideRunner sideRunner;

bool gaitRunnerReady3(SideRunner &r) {
  if (!r.active) return false;
  unsigned long now = millis();
  if (now - r.lastStepTime < r.stepDelayMs) return false;
  r.lastStepTime = now;
  return true;
}

RotateRunner rotateRunner;
float ROT_AMOUNT = 25.0;

bool gaitRunnerReady4(RotateRunner &r) {
  if (!r.active) return false;
  unsigned long now = millis();
  if (now - r.lastStepTime < r.stepDelayMs) return false;
  r.lastStepTime = now;
  return true;
}

void walkComputeLegParams(WalkRunner &r) {
  r.xP0 = r.legX[r.currentLeg];
  r.xP3 = r.xP0 + SL * r.dir;
  r.xP1 = r.xP0 + (r.xP3 - r.xP0) * 0.25;
  r.xP2 = r.xP0 + (r.xP3 - r.xP0) * 0.75;
}

void walkGaitStart(bool forward) {
  int steps, stepDelay;
  getGaitTiming(steps, stepDelay);

  walkRunner.active = true;
  walkRunner.currentLeg = 0;
  walkRunner.stepIndex = 0;
  walkRunner.totalSteps = steps;
  walkRunner.stepDelayMs = stepDelay;
  walkRunner.lastStepTime = millis();
  walkRunner.dir = (forward ? 1.0 : -1.0) * FWD;

  for (int i = 0; i < 4; i++) {
    walkRunner.legX[i] = -SL/2 * walkRunner.dir;
  }
  walkComputeLegParams(walkRunner);
}

bool walkGaitStep() {
  if (!gaitRunnerReady2(walkRunner)) return false;

  LegInfo* legs[4] = {&legFL, &legBR, &legFR, &legBL};

  float t = (float)walkRunner.stepIndex / walkRunner.totalSteps;
  float zP0 = H, zP1 = H - SH*1.3, zP2 = H - SH*1.3, zP3 = H;
  float sx = bezier(t, walkRunner.xP0, walkRunner.xP1, walkRunner.xP2, walkRunner.xP3);
  float sz = bezier(t, zP0, zP1, zP2, zP3);
  moveLeg(*legs[walkRunner.currentLeg], sx, Y, sz);

  float stanceShift = t * (SL/4) * walkRunner.dir;
  for (int other = 0; other < 4; other++) {
    if (other == walkRunner.currentLeg) continue;
    float sxOther = walkRunner.legX[other] - stanceShift;
    moveLeg(*legs[other], sxOther, Y, H);
  }

  walkRunner.stepIndex++;
  if (walkRunner.stepIndex > walkRunner.totalSteps) {
    walkRunner.legX[walkRunner.currentLeg] = walkRunner.xP3;
    for (int other = 0; other < 4; other++) {
      if (other == walkRunner.currentLeg) continue;
      walkRunner.legX[other] -= (SL/4) * walkRunner.dir;
    }
    walkRunner.currentLeg++;
    walkRunner.stepIndex = 0;
    if (walkRunner.currentLeg >= 4) {
      walkRunner.active = false;
      return false;
    }
    walkComputeLegParams(walkRunner);
  }
  return true;
}

void sideWalkStart(bool strafeRight) {
  int steps, stepDelay;
  getGaitTiming(steps, stepDelay);
  sideRunner.active = true;
  sideRunner.phase = 0;
  sideRunner.stepIndex = 0;
  sideRunner.totalSteps = steps;
  sideRunner.stepDelayMs = stepDelay;
  sideRunner.lastStepTime = millis();
  sideRunner.sign = strafeRight ? 1.0 : -1.0;
}

bool sideWalkStep() {
  if (!gaitRunnerReady3(sideRunner)) return false;

  float t = (float)sideRunner.stepIndex / sideRunner.totalSteps;
  float sign = sideRunner.sign;

  if (sideRunner.phase == 0) {
    float yP0 = Y - (SL/2)*sign, yP1 = Y - (SL/4)*sign, yP2 = Y + (SL/4)*sign, yP3 = Y + (SL/2)*sign;
    float zP0 = H, zP1 = H - SH*1.3, zP2 = H - SH*1.3, zP3 = H;
    float sy = bezier(t, yP0, yP1, yP2, yP3);
    float sz = bezier(t, zP0, zP1, zP2, zP3);
    moveAllLegs(X, sy, sz);
  } else {
    float sy = (Y + (SL/2)*sign) - t * SL * sign;
    moveAllLegs(X, sy, H);
  }

  sideRunner.stepIndex++;
  if (sideRunner.stepIndex > sideRunner.totalSteps) {
    sideRunner.stepIndex = 0;
    sideRunner.phase = 1 - sideRunner.phase;
  }
  return true;
}

void rotateGaitStart(bool isCW) {
  int steps, stepDelay;
  getGaitTiming(steps, stepDelay);
  rotateRunner.active = true;
  rotateRunner.cycle = 0;
  rotateRunner.stepIndex = 0;
  rotateRunner.totalSteps = steps;
  rotateRunner.stepDelayMs = stepDelay;
  rotateRunner.lastStepTime = millis();
  rotateRunner.dir = isCW ? -1.0 : 1.0;

  LegInfo* legs[4] = {&legFL, &legFR, &legBL, &legBR};
  for (int i = 0; i < 4; i++) {
    float r = sqrt(legs[i]->ox*legs[i]->ox + legs[i]->oy*legs[i]->oy);
    rotateRunner.tx[i] = -legs[i]->oy / r * rotateRunner.dir;
    rotateRunner.ty[i] =  legs[i]->ox / r * rotateRunner.dir;
  }
}

bool rotateGaitStep() {
  if (!gaitRunnerReady4(rotateRunner)) return false;

  LegInfo* legs[4] = {&legFL, &legFR, &legBL, &legBR};
  int pairA[2] = {0, 3};
  int pairB[2] = {1, 2};
  int* swingPair = (rotateRunner.cycle == 0) ? pairA : pairB;
  int* stancePair = (rotateRunner.cycle == 0) ? pairB : pairA;

  float t = (float)rotateRunner.stepIndex / rotateRunner.totalSteps;
  float lift = H - SH * 1.3 * sin(PI * t);
  float swingOffset = (t - 0.5) * 2 * ROT_AMOUNT;
  float stanceOffset = -(t - 0.5) * 2 * ROT_AMOUNT;

  for (int p = 0; p < 2; p++) {
    int idx = swingPair[p];
    float sx = X + rotateRunner.tx[idx] * swingOffset;
    float sy = Y + rotateRunner.ty[idx] * swingOffset;
    moveLeg(*legs[idx], sx, sy, lift);
  }
  for (int p = 0; p < 2; p++) {
    int idx = stancePair[p];
    float sx = X + rotateRunner.tx[idx] * stanceOffset;
    float sy = Y + rotateRunner.ty[idx] * stanceOffset;
    moveLeg(*legs[idx], sx, sy, H);
  }

  rotateRunner.stepIndex++;
  if (rotateRunner.stepIndex > rotateRunner.totalSteps) {
    rotateRunner.stepIndex = 0;
    rotateRunner.cycle = 1 - rotateRunner.cycle;
  }
  return true;
}


// =================================================================
// STAGE 3: BLENDED-INPUT TROT ENGINE
// =================================================================
// Combines walk (Ch2), strafe (Ch1), and turn (Ch4) into ONE trot
// gait, each direction weighted by its own stick's deflection.
// Directions mix additively per-leg, so e.g. "strafe left 0.6 +
// rotate right 0.3" produces one combined motion, not two separate
// gaits fighting for the same legs.
//
// Requires: LegInfo struct/leg objects, moveLeg(), getGaitTiming(),
// bezier(), FWD, H, SH, curX/curY/curZ, and the tangent vectors
// used by rotateGaitStart() (recomputed locally here so this file
// has no hidden dependency on rotateRunner's internal state).
// =================================================================


BlendedTrotRunner blendedTrot;

const float BT_MAX_SL = 100.0;
const float BT_MAX_ROT = 25.0;
const float BT_MAX_SH = 50.0;

void blendedTrotComputeTangents() {
  LegInfo* legs[4] = {&legFL, &legFR, &legBL, &legBR};
  for (int i = 0; i < 4; i++) {
    float r = sqrt(legs[i]->ox*legs[i]->ox + legs[i]->oy*legs[i]->oy);
    blendedTrot.tx[i] = -legs[i]->oy / r;
    blendedTrot.ty[i] =  legs[i]->ox / r;
  }
}

void blendedTrotStart() {
  int steps, stepDelay;
  getGaitTiming(steps, stepDelay);
  blendedTrot.active = true;
  blendedTrot.phase = 1;
  blendedTrot.stepIndex = 0;
  blendedTrot.totalSteps = steps;
  blendedTrot.stepDelayMs = stepDelay;
  blendedTrot.lastStepTime = millis();
  blendedTrotComputeTangents();
}

void blendedTrotSetInputs(float walkStick, float strafeStick, float turnStick) {
  blendedTrot.walkStick = walkStick;
  blendedTrot.strafeStick = strafeStick;
  blendedTrot.turnStick = turnStick;
}

bool blendedTrotActive() {
  return fabs(blendedTrot.walkStick) > 0.01 ||
         fabs(blendedTrot.strafeStick) > 0.01 ||
         fabs(blendedTrot.turnStick) > 0.01;
}

bool blendedTrotReady() {
  if (!blendedTrot.active) return false;
  unsigned long now = millis();
  if (now - blendedTrot.lastStepTime < blendedTrot.stepDelayMs) return false;
  blendedTrot.lastStepTime = now;
  return true;
}

bool blendedTrotCycleJustCompleted = false;

bool blendedTrotStep() {
  blendedTrotCycleJustCompleted = false;
  if (!blendedTrotReady()) return false;

  float walk_amt = BT_MAX_SL * blendedTrot.walkStick * FWD;
  float strafe_amt = BT_MAX_SL * blendedTrot.strafeStick;
  float turn_amt = BT_MAX_ROT * blendedTrot.turnStick;

  float liftMag = fmax(fabs(blendedTrot.walkStick), fmax(fabs(blendedTrot.strafeStick), fabs(blendedTrot.turnStick)));
  float shAmount = BT_MAX_SH * liftMag;

  LegInfo* legs[4] = {&legFL, &legFR, &legBL, &legBR};
  int swingPair[2], stancePair[2];
  if (blendedTrot.phase == 1) {
    swingPair[0] = 0; swingPair[1] = 3;
    stancePair[0] = 1; stancePair[1] = 2;
  } else {
    swingPair[0] = 1; swingPair[1] = 2;
    stancePair[0] = 0; stancePair[1] = 3;
  }

  float t = (float)blendedTrot.stepIndex / blendedTrot.totalSteps;
  float zP0 = H, zP1 = H - shAmount*1.3, zP2 = H - shAmount*1.3, zP3 = H;
  float swingZ = bezier(t, zP0, zP1, zP2, zP3);

  for (int p = 0; p < 2; p++) {
    int idx = swingPair[p];
    float dx = walk_amt + turn_amt * blendedTrot.tx[idx];
    float dy = strafe_amt + turn_amt * blendedTrot.ty[idx];

    float xP0 = -dx/2, xP3 = dx/2;
    float xP1 = xP0 + (xP3-xP0)*0.25, xP2 = xP0 + (xP3-xP0)*0.75;
    float yP0 = -dy/2, yP3 = dy/2;
    float yP1 = yP0 + (yP3-yP0)*0.25, yP2 = yP0 + (yP3-yP0)*0.75;

    float sx = bezier(t, xP0, xP1, xP2, xP3);
    float sy = bezier(t, yP0, yP1, yP2, yP3);
    moveLeg(*legs[idx], sx, Y + sy, swingZ);
  }

  for (int p = 0; p < 2; p++) {
    int idx = stancePair[p];
    float dx = walk_amt + turn_amt * blendedTrot.tx[idx];
    float dy = strafe_amt + turn_amt * blendedTrot.ty[idx];

    float stanceX = dx/2 - t * dx;
    float stanceY = dy/2 - t * dy;
    moveLeg(*legs[idx], stanceX, Y + stanceY, H);
  }

  blendedTrot.stepIndex++;
  if (blendedTrot.stepIndex > blendedTrot.totalSteps) {
    blendedTrot.stepIndex = 0;
    if (blendedTrot.phase == 2) {
      blendedTrot.phase = 1;
      blendedTrotCycleJustCompleted = true;
    } else {
      blendedTrot.phase = 2;
    }
  }
  return true;
}


// =================================================================
// STAGE 2 (final): ROBOT STATE MACHINE (Ch6) + GAIT SELECTION (Ch5)
// =================================================================
// goToZ is now NON-BLOCKING (converted during final integration for
// consistency with the rest of the non-blocking control system - a
// blocking sleep/stand/estop transition would otherwise stall RC
// input at exactly the moments responsiveness matters most).
//
// runSelectedGait() from the earlier draft has been REMOVED - it
// called the old blocking trotGait/paceGait/boundGait, which no
// longer exist. Actual gait dispatch now happens in
// stage3b_rc_dispatch.ino's updateLocomotion(), which uses the
// non-blocking gait engines.
// =================================================================

#define STATE_SLEEP 0
#define STATE_STAND 1
#define STATE_ESTOP 2

#define GAIT_TROT 0
#define GAIT_PACE 1
#define GAIT_BOUND 2

int robotState = STATE_SLEEP;
int selectedGait = GAIT_TROT;
int pendingGait = GAIT_TROT;
bool estopLatched = false;

const float SIT_Z = 30.0;
const float STAND_Z = 200.0;

GoToZRunner goToZRunner;

void goToZStart(float targetZ) {
  int steps, stepDelay;
  getGaitTiming(steps, stepDelay);
  goToZRunner.active = true;
  goToZRunner.stepIndex = 0;
  goToZRunner.totalSteps = steps;
  goToZRunner.stepDelayMs = stepDelay;
  goToZRunner.lastStepTime = millis();
  for (int i = 0; i < 4; i++) {
    goToZRunner.startZ[i] = curZ[i];
  }
  goToZRunner.targetZ = targetZ;
}

bool goToZStep() {
  if (!goToZRunner.active) return false;
  unsigned long now = millis();
  if (now - goToZRunner.lastStepTime < goToZRunner.stepDelayMs) return true;
  goToZRunner.lastStepTime = now;

  float t = (float)goToZRunner.stepIndex / goToZRunner.totalSteps;
  LegInfo* legs[4] = {&legFL, &legFR, &legBL, &legBR};
  for (int L = 0; L < 4; L++) {
    float sz = goToZRunner.startZ[L] + (goToZRunner.targetZ - goToZRunner.startZ[L]) * t;
    moveLeg(*legs[L], curX[L], curY[L], sz);
  }

  goToZRunner.stepIndex++;
  if (goToZRunner.stepIndex > goToZRunner.totalSteps) {
    goToZRunner.active = false;
    return false;
  }
  return true;
}

bool goToZIsDone() {
  return !goToZRunner.active;
}

void updateRobotState() {
  int swC = ibusGetSwitchPosition(5, 3);

  if (swC == 2) {
    if (robotState != STATE_ESTOP) {
      showGait("E-STOP");
      goToZStart(SIT_Z);
      robotState = STATE_ESTOP;
      estopLatched = true;
    }
    goToZStep();
    return;
  }

  if (estopLatched) {
    if (swC != 2) {
      estopLatched = false;
    } else {
      goToZStep();
      return;
    }
  }

  if (swC == 0) {
    if (robotState != STATE_SLEEP) {
      showGait("Sleeping");
      goToZStart(SIT_Z);
      robotState = STATE_SLEEP;
    }
  } else if (swC == 1) {
    if (robotState != STATE_STAND) {
      showGait("Standing");
      goToZStart(STAND_Z);
      robotState = STATE_STAND;
    }
  }
  goToZStep();
}

void updateGaitSelection() {
  int swB = ibusGetSwitchPosition(4, 3);
  pendingGait = swB;
}


// =================================================================
// STAGE 3b: RC CHANNEL DISPATCHER
// =================================================================
// Ties together: IBUS reads (Stage 1), state machine + gait select
// (Stage 2), stick scaling (stick_scaling.ino), non-blocking gaits
// (stage_nb1/nb2), and the blended trot engine (stage3_blended_trot).
//
// GAIT SWITCH BEHAVIOR: switching Ch5 (Trot/Pace/Bound) while a gait
// is actively running does NOT interrupt it mid-swing. The currently
// running gait keeps stepping (using selectedGait) until it completes
// a full 2-phase cycle (all 4 feet back in a symmetric grounded
// position), THEN the new gait (pendingGait) takes over. If no gait
// is currently active (robot idle/no stick input), the switch applies
// immediately since there's nothing mid-motion to protect.
//
// NOTE per your instruction: only TROT uses blended (Ch1+Ch2+Ch4)
// motion. Pace and Bound remain static/forward-back-only, driven
// by Ch2 alone - Ch1 (strafe) and Ch4 (turn) are ignored while
// Pace or Bound is selected.
// =================================================================

const float MIN_STAND_H = 100.0;
const float MAX_STAND_H = 220.0;

bool paceOrBoundStarted = false;

void updateHeightControl() {
  float heightStick = ibusGetNormalized(2);
  float targetH = MIN_STAND_H + (heightStick + 1.0) * 0.5 * (MAX_STAND_H - MIN_STAND_H);
  H = targetH;
}

bool anyGaitCurrentlyActive() {
  return blendedTrot.active || paceOrBoundStarted;
}

void applyPendingGaitIfSafe() {
  if (selectedGait == pendingGait) return;

  bool switchIsSafe = false;

  if (!anyGaitCurrentlyActive()) {
    switchIsSafe = true;
  } else if (selectedGait == GAIT_TROT && blendedTrotCycleJustCompleted) {
    switchIsSafe = true;
  } else if (selectedGait == GAIT_PACE && paceCycleJustCompleted) {
    switchIsSafe = true;
  } else if (selectedGait == GAIT_BOUND && boundCycleJustCompleted) {
    switchIsSafe = true;
  }

  if (switchIsSafe) {
    selectedGait = pendingGait;
    blendedTrot.active = false;
    paceOrBoundStarted = false;
  }
}

void updateLocomotion() {
  updateHeightControl();

  if (robotState != STATE_STAND) {
    blendedTrot.active = false;
    paceOrBoundStarted = false;
    selectedGait = pendingGait;
    return;
  }

  float strafeStick = ibusGetNormalized(0);
  float walkStick = ibusGetNormalized(1);
  float turnStick = ibusGetNormalized(3);

  bool anyInput = stickActive(strafeStick) || stickActive(walkStick) || stickActive(turnStick);

  if (selectedGait == GAIT_TROT) {
    if (!anyInput) {
      blendedTrot.active = false;
      applyPendingGaitIfSafe();
      return;
    }
    if (!blendedTrot.active) {
      blendedTrotStart();
    }
    blendedTrotSetInputs(walkStick, strafeStick, turnStick);
    blendedTrotStep();
    applyPendingGaitIfSafe();

  } else if (selectedGait == GAIT_PACE) {
    if (!stickActive(walkStick)) {
      paceOrBoundStarted = false;
      applyPendingGaitIfSafe();
      return;
    }
    applyStickScaling(walkStick);
    if (!paceOrBoundStarted) {
      paceGaitStart();
      paceOrBoundStarted = true;
    }
    paceGaitStep();
    applyPendingGaitIfSafe();

  } else if (selectedGait == GAIT_BOUND) {
    if (!stickActive(walkStick)) {
      paceOrBoundStarted = false;
      applyPendingGaitIfSafe();
      return;
    }
    applyStickScaling(walkStick);
    if (!paceOrBoundStarted) {
      boundGaitStart();
      paceOrBoundStarted = true;
    }
    boundGaitStep();
    applyPendingGaitIfSafe();
  }
}

// NOTE: applyStickScaling() runs every loop() iteration for Pace/Bound
// (not just once at Start()), so SL/SH/speed update continuously as the
// stick moves - matching how blendedTrot already behaves. Confirmed
// acceptable: small (~<15mm) position discontinuities are possible if
// the stick moves fast mid-swing, per explicit approval - the gait's
// totalSteps/stepDelayMs (set once at Start()) are NOT re-read mid-cycle,
// only the bezier amplitude (SL/SH) changes, which is what causes the
// (accepted) minor wobble rather than a timing-index corruption.


// =================================================================
// STAGE 4: POSTURE / ATTITUDE MODE (Ch7 mode toggle)
// =================================================================
// Feet stay planted at fixed world positions while the BODY rolls,
// pitches, yaws, and heaves (rises/sinks) in place. This is real
// body-tilt IK, not a locomotion gait - no bezier trajectories,
// legs are simply repositioned to their computed target every step.
//
// Convention (matches your existing tested code):
//   - Leg-space Z is DOWNWARD-POSITIVE (confirmed via StandAndSit:
//     larger Z = leg more extended = standing taller)
//   - Body frame shares this Z convention (Z+ = down)
//   - Roll = tilt around body's forward (X) axis
//   - Pitch = tilt around body's side (Y) axis (positive = nose down)
//   - Yaw = rotate around vertical (Z) axis
//   - Heave = vertical body shift. heaveStick maps to pilot intuition
//     (stick up = heaveStick>0 = body rises = stands TALLER). Internally,
//     "rises" means the hip moves further from the fixed foot, i.e. LARGER
//     leg-local Z (matches the confirmed Z convention above), which is why
//     heave is negated before being added to hipRotated.z in the code below.
// =================================================================

const float POSTURE_STAND_Z = 200.0;
const float POSTURE_HOME_Y = 40.0;

const float MAX_ROLL_RAD = 0.15;   // ~8.6 degrees - reduced from 15deg so combined
const float MAX_PITCH_RAD = 0.15;  // ~8.6 degrees   roll+pitch+yaw+heave at full
const float MAX_YAW_RAD = 0.20;    // ~11.5 degrees  stick deflection stays IK-reachable
const float MAX_HEAVE = 20.0;      // mm, reduced from 40mm for the same reason


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

void updatePostureMode() {
  float rollStick = ibusGetNormalized(3);
  float pitchStick = ibusGetNormalized(1);
  float yawStick = ibusGetNormalized(0);
  float heaveStick = ibusGetNormalized(2);

  float roll = rollStick * MAX_ROLL_RAD;
  float pitch = pitchStick * MAX_PITCH_RAD;
  float yaw = yawStick * MAX_YAW_RAD;
  float heave = -heaveStick * MAX_HEAVE;

  applyPostureTilt(roll, pitch, yaw, heave);
}


// =================================================================
// STAGE 4b: CH7 CONTROL MODE TOGGLE (Locomotion <-> Posture)
// =================================================================
// Switching FROM Posture mode is always immediate/safe - posture mode
// holds a static tilted pose, there is no mid-swing state to protect.
//
// Switching FROM Locomotion mode (while a gait is actively stepping)
// waits for that gait to go idle (cycle complete + stick released, or
// stick simply not pushed), THEN runs a brief goHome-style transition
// to Posture mode's neutral stance (X=0, Y=POSTURE_HOME_Y,
// Z=POSTURE_STAND_Z) before Posture mode's continuous stick-driven
// tilting takes over - this avoids a snap, since a gait's active
// stance (legs at +-SL/2) does not match Posture's neutral stance
// (legs at X=0) whenever stride length is nonzero.
// =================================================================

#define MODE_LOCOMOTION 0
#define MODE_POSTURE 1

int controlMode = MODE_LOCOMOTION;
int pendingControlMode = MODE_LOCOMOTION;
bool transitioningToPosture = false;

void updateControlModeSelection() {
  int swA = ibusGetSwitchPosition(6, 2);
  pendingControlMode = swA;
}

void updateControlMode() {
  if (robotState != STATE_STAND) {
    return;
  }

  if (controlMode == MODE_POSTURE) {
    if (pendingControlMode == MODE_LOCOMOTION) {
      controlMode = MODE_LOCOMOTION;
      transitioningToPosture = false;
      return;
    }
    updatePostureMode();
    return;
  }

  if (transitioningToPosture) {
    goHomeStep();
    if (goHomeIsDone()) {
      controlMode = MODE_POSTURE;
      transitioningToPosture = false;
    }
    return;
  }

  if (pendingControlMode == MODE_POSTURE && !anyGaitCurrentlyActive()) {
    goHomeStart(0, POSTURE_HOME_Y, POSTURE_STAND_Z);
    transitioningToPosture = true;
    goHomeStep();
    return;
  }

  updateLocomotion();
}


// =================================================================
// STAGE 5: CH8 SELF-BALANCING (MPU6050, LOCOMOTION MODE ONLY)
// =================================================================
// Per your instruction: balancing applies ONLY during Locomotion mode
// (walking gaits). Posture mode ignores Ch8 entirely - balancing is
// simply not available there.
//
// MOUNTING ORIENTATION WARNING: the roll/pitch formula below assumes
// the MPU6050 is mounted flat with its OWN X-axis aligned to the
// robot's forward direction and its OWN Z-axis aligned to the robot's
// down direction. If your physical mounting differs (e.g. rotated
// 90 degrees, upside down, etc.), the axes in the formula below need
// to be swapped/negated to match. TEST THIS ON HARDWARE: tilt the
// robot by hand and confirm computedRoll/computedPitch change in the
// direction you'd expect before trusting the balance correction.
//
// Requires: Adafruit_MPU6050 and Adafruit_Sensor libraries installed,
// #include <Adafruit_MPU6050.h> and <Adafruit_Sensor.h> in main sketch.
// =================================================================

// mpu object is declared once, near the top of this combined file,
// alongside pca and lcd.
bool mpuReady = false;

float measuredRoll = 0;
float measuredPitch = 0;

const float BALANCE_GAIN = 1.5;
const float BALANCE_MAX_CORRECTION = 25.0;
const float BALANCE_FILTER_ALPHA = 0.15;

int balancingEnabled = 0;

void mpuInit() {
  mpuReady = mpu.begin();
  if (mpuReady) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
}

void updateBalanceSensing() {
  if (!mpuReady) return;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float rawRoll = atan2(a.acceleration.y, a.acceleration.z);
  float rawPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z));

  measuredRoll = measuredRoll + BALANCE_FILTER_ALPHA * (rawRoll - measuredRoll);
  measuredPitch = measuredPitch + BALANCE_FILTER_ALPHA * (rawPitch - measuredPitch);
}

void updateBalanceModeSelection() {
  balancingEnabled = ibusGetSwitchPosition(7, 2);
}

float balanceCorrectionZ(LegInfo &leg) {
  if (!balancingEnabled || !mpuReady) return 0;
  if (controlMode != MODE_LOCOMOTION) return 0;

  float sr = sin(measuredRoll);
  float cp = cos(measuredPitch), sp = sin(measuredPitch);
  float hipRotatedZ = -leg.ox * sp + leg.oy * sr * cp;

  float correction = -hipRotatedZ * BALANCE_GAIN;
  correction = constrain(correction, -BALANCE_MAX_CORRECTION, BALANCE_MAX_CORRECTION);
  return correction;
}


// =================================================================
// SETUP AND MAIN LOOP
// =================================================================

void setup() {
  Wire.begin();
  pca.begin();
  pca.setPWMFreq(50);
  Serial.begin(115200);
  Serial.println("-------------SpotMicro RC control-----------");

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("SpotMicro Ready");

  ibusInit();
  mpuInit();

  moveAllLegs(X, Y, Z);
  delay(500);
}

void loop() {
  ibusRead();

  updateRobotState();
  updateGaitSelection();
  updateControlModeSelection();
  updateBalanceModeSelection();
  updateBalanceSensing();

  if (robotState == STATE_STAND) {
    updateControlMode();
  }
}
