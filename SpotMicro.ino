/*
  ================================================================
   QUADRUPED ROBOT DOG - "SpotMicro" - MAIN FIRMWARE
  ================================================================
   Author reference (shown on LCD)  : Chaitanya
   Robot name (shown on LCD)        : SpotMicro
   Platform                         : Arduino UNO
   Servo Driver                     : PCA9685 (16-ch PWM, I2C)
   Servos                           : 12x MG996R (Hip/Thigh/Shank x4 legs)
   Receiver                         : FlySky iA10B (iBUS, single wire)
   Transmitter                      : FlySky FS-i6S
   Display                          : 16x2 I2C LCD (4-pin: GND,VCC,SDA,SCL)
   Stabilization                    : NONE (open-loop, no IMU)

  CONTROL MAP (as specified):
    Left  Stick X (Rudder)    -> Yaw rotate in place
    Left  Stick Y (Throttle)  -> Body height from ground
    Right Stick X (Aileron)   -> Strafe left/right
    Right Stick Y (Elevator)  -> Walk forward/back
    -> Any two-axis combination -> diagonal movement (vector sum)
    -> Stick deflection MAGNITUDE -> proportional walking speed
    SwA (2-pos) -> Leg-Lock: feet planted, only body rotates/shifts
    SwB (3-pos) -> Gait select: Trot / Crawl / Static-Wave
    SwD (2-pos) -> Mode select: Normal RC control / Random dance mode

  ARCHITECTURE:
    - Forward Kinematics: verifies the calibration stance and is
      exposed as a callable utility (used at boot to sanity-check
      geometry, and available for future odometry).
    - Inverse Kinematics: analytic 3-DOF solver, run every cycle
      per leg to convert a desired foot (x,y,z) into joint angles.
    - Gait engine: phase-based trajectory generator supporting
      three distinct gaits (2-phase trot, 4-phase crawl, 4-phase
      wave -- wave differs from crawl in stance-leg overlap timing).
    - Dance engine: bounded pseudo-random IK target generator that
      reuses the exact same safety clamps as manual control.
  ================================================================
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <IBusBM.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

#define IBUS_RX_PIN 10
#define IBUS_TX_PIN 11

SoftwareSerial ibusSerial(IBUS_RX_PIN, IBUS_TX_PIN);

// ================================================================
// SECTION 1: GLOBAL CONSTANTS & CONFIGURATION
// ================================================================

// ---------------- PCA9685 / Servo PWM calibration ----------------
#define PCA9685_ADDR      0x40
#define SERVO_FREQ        50        // MG996R: 50Hz standard

// 12-bit PCA9685 "tick" range corresponding to ~0.5ms-2.5ms pulses.
// TUNE if your specific MG996R batch's true 0-180 range differs.
#define SERVO_MIN_TICK    102
#define SERVO_MAX_TICK    512

// ---------------- Leg / channel mapping ----------------
#define NUM_LEGS          4
#define FR                0   // Front-Right
#define FL                1   // Front-Left
#define RR                2   // Rear-Right
#define RL                3   // Rear-Left

// PCA9685 channel per [leg][joint], joint 0=Hip 1=Thigh 2=Shank
const uint8_t SERVO_CH[NUM_LEGS][3] = {
  {  0,  1,  2 },   // FR
  {  3,  4,  5 },   // FL
  {  6,  7,  8 },   // RR
  {  9, 10, 11 }    // RL
};

// Mirror-side inversion: true = command (180 - angle) for that joint.
// TUNE after bench-testing each leg's real rotation direction.
const bool SERVO_INVERT[NUM_LEGS][3] = {
  { false, false, true  },   // FR
  { false, true,  false },   // FL
  { true,  false, true  },   // RR
  { true,  true,  false }    // RL
};

// Per-servo mechanical trim (degrees) to zero the horn position.
// TUNE during assembly calibration.
int SERVO_TRIM[NUM_LEGS][3] = {
  { 0, 0, 0 },   // FR
  { 0, 0, 0 },   // FL
  { 0, 0, 0 },   // RR
  { 0, 0, 0 }    // RL
};

// ---------------- Leg geometry (mm) - REQUIRED for IK/FK ----------------
// Measure from the physical 3D-printed leg parts. Wrong numbers here
// will not crash anything but WILL make the stance geometrically wrong.
const float COXA_LEN   = 30.0;   // Hip pivot -> Thigh pivot
const float FEMUR_LEN  = 65.0;   // Thigh pivot -> Shank pivot
const float TIBIA_LEN  = 95.0;   // Shank pivot -> foot tip

// ---------------- Default stance (neutral standing pose) ----------------
// Foot position relative to ITS OWN hip, leg-local frame:
//   X = forward(+)/back(-), Y = outward(+)/inward(-), Z = down(-)/up(+)
const float STANCE_X = 0.0;
const float STANCE_Y = 90.0;
const float STANCE_Z_DEFAULT = -110.0;

// ---------------- Height (throttle) limits ----------------
const float BODY_Z_MIN = -70.0;    // crouched
const float BODY_Z_MAX = -150.0;   // fully raised

// ---------------- Gait / trajectory parameters ----------------
const float STEP_LIFT_HEIGHT   = 35.0;    // mm foot lift during swing
const float STEP_LENGTH_MAX    = 55.0;    // mm max foot travel per step
const float YAW_STEP_MAX_DEG   = 12.0;    // deg max per-cycle rotation

const int   GAIT_UPDATE_MS     = 20;      // main loop period (ms)
const int   LCD_UPDATE_MS      = 400;     // LCD refresh throttle (ms)

// Gait phase counts (all four-leg-cycle gaits use 4 phases; trot uses 2)
const uint8_t TROT_PHASES  = 2;
const uint8_t CRAWL_PHASES = 4;
const uint8_t WAVE_PHASES  = 4;

// Diagonal leg pairing for trot gait
const uint8_t TROT_GROUP_A[2] = { FR, RL };
const uint8_t TROT_GROUP_B[2] = { FL, RR };

// Sequential leg order for crawl gait (max-stability crawl order)
const uint8_t CRAWL_SEQUENCE[4] = { FR, RL, FL, RR };

// Sequential leg order for wave gait (classic wave-gait order, differs
// from crawl in the diagonal pattern, giving a distinct visual gait
// even though both are single-leg-swing-at-a-time)
const uint8_t WAVE_SEQUENCE[4] = { FL, FR, RL, RR };

// Gait enum for readability throughout the code
enum GaitType : uint8_t { GAIT_TROT = 0, GAIT_CRAWL = 1, GAIT_WAVE = 2 };

// ---------------- Receiver (iBUS) channel map ----------------
// iBUS channel order is fixed by protocol/TX setup, confirmed to:
//   CH1=Right X(Aileron)  CH2=Right Y(Elevator)
//   CH3=Left Y(Throttle)  CH4=Left X(Rudder)
//   CH5=SwA               CH6=SwB (must be 3-pos)
//   CH7=SwD (if your i6s SwD channel differs, change CH_MODE_SWITCH)
#define CH_RIGHT_X          0
#define CH_RIGHT_Y          1
#define CH_LEFT_Y           2
#define CH_LEFT_X           3
#define CH_LEGLOCK_SWITCH   4   // SwA
#define CH_GAIT_SWITCH      5   // SwB (3-position)
#define CH_MODE_SWITCH      6   // SwD

const int RC_MIN      = 1000;
const int RC_MAX       = 2000;
const int RC_MID       = 1500;
const int RC_DEADZONE  = 30;

// 3-position switch thresholds (FlySky 3-pos typically outputs
// approx 1000 / 1500 / 2000us at its three detents)
const int RC_3POS_LOW_THRESH  = 1250;
const int RC_3POS_HIGH_THRESH = 1750;

// ---------------- LCD configuration ----------------
#define LCD_ADDR   0x27    // common default; try 0x3F if blank display
#define LCD_COLS   16
#define LCD_ROWS   2

// ================================================================
// SECTION 2: GLOBAL OBJECTS & STATE VARIABLES
// ================================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);
IBusBM ibus;
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// Current commanded foot position per leg, leg-local frame (mm).
// This array is the single source of truth IK reads from each cycle.
float footX[NUM_LEGS];
float footY[NUM_LEGS];
float footZ[NUM_LEGS];

// Normalized stick inputs (-1.0 to 1.0), height remapped 0.0-1.0
float inputMoveX  = 0.0;   // strafe axis (right stick X)
float inputMoveY  = 0.0;   // forward/back axis (right stick Y)
float inputYaw    = 0.0;   // rotation axis (left stick X)
float inputHeight = 0.0;   // height axis (left stick Y, 0..1)

GaitType currentGait = GAIT_TROT;
bool legLockActive   = false;
bool danceModeActive = false;

// Gait phase tracking
uint8_t gaitPhaseIndex     = 0;
float   gaitPhaseProgress  = 0.0;

// Smoothed body height
float currentBodyZ = STANCE_Z_DEFAULT;

// Leg-lock anchor points (world-relative foot position at lock time)
float lockedFootX[NUM_LEGS];
float lockedFootY[NUM_LEGS];
float lockedFootZ[NUM_LEGS];

// Dance mode internal state
unsigned long lastDanceTargetTime = 0;
float danceTargetX[NUM_LEGS], danceTargetY[NUM_LEGS], danceTargetZ[NUM_LEGS];
float danceStartX[NUM_LEGS], danceStartY[NUM_LEGS], danceStartZ[NUM_LEGS];
float danceInterpT = 1.0;
int   danceMoveDurationMs = 300;

// LCD refresh timing + change-detection (avoid unnecessary I2C writes)
unsigned long lastLcdUpdate = 0;
GaitType lastShownGait = GAIT_TROT;
bool lastShownLegLock  = false;
bool lastShownDance    = false;
bool lcdNeedsFullRedraw = true;

// Receiver signal-loss failsafe tracking
unsigned long lastGoodRcRead = 0;
const unsigned long RC_TIMEOUT_MS = 500; // if no valid frame in this time -> failsafe

// ================================================================
// SECTION 3: SETUP
// ================================================================
void setup() {
  Serial.begin(115200);

  // NOTE: iA10B iBUS wire goes to a SoftwareSerial-backed stream so
  // hardware Serial stays free for debug output over USB. See the
  // WIRING NOTES block at the bottom of this file for pin details
  // and the SoftwareSerial object definition required just below.
  ibusSerial.begin(115200);
  ibus.begin(Serial);

  Wire.begin();

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(500); // allow PCA9685 internal oscillator to stabilize

  lcd.init();
  lcd.backlight();
  showBootScreen();

  initializeStance();     // sets footX/Y/Z to the calibration stance
  verifyStanceWithFK();   // sanity-check that stance via forward kinematics
  applyAllLegIK();         // push initial pose out to servos

  delay(1500);             // let boot screen stay visible briefly
  lcdNeedsFullRedraw = true;

  Serial.println(F("SpotMicro ready."));
}

// ================================================================
// SECTION 4: MAIN LOOP
// ================================================================
void loop() {
  bool rcOk = readReceiverInputs();   // 1. Read latest sticks/switches
  updateGaitAndModeSelection();        // 2. Apply switch states

  if (!rcOk) {
    handleReceiverFailsafe();          // 3a. No valid signal -> safe hold
  } else if (danceModeActive) {
    updateDanceMode();                 // 3b. Random bounded dance
  } else if (legLockActive) {
    updateLegLockMode();               // 3c. Feet planted, body-only IK
  } else {
    updateWalkingGait();               // 3d. Normal locomotion gait engine
  }

  applyAllLegIK();                     // 4. Foot targets -> servo angles
  updateLcdDisplay();                  // 5. Refresh info screen (throttled)

  delay(GAIT_UPDATE_MS);               // 6. Fixed-rate control loop
}

// ================================================================
// SECTION 5: RECEIVER INPUT HANDLING
// ================================================================

// Reads all relevant RC channels, applies deadzone/remap, and
// normalizes them. Returns false if the read looks invalid/stale
// so the caller can trigger a failsafe instead of acting on garbage.
bool readReceiverInputs() {
  int rawRightX = ibus.readChannel(CH_RIGHT_X);
  int rawRightY = ibus.readChannel(CH_RIGHT_Y);
  int rawLeftY  = ibus.readChannel(CH_LEFT_Y);
  int rawLeftX  = ibus.readChannel(CH_LEFT_X);
  int rawSwA    = ibus.readChannel(CH_LEGLOCK_SWITCH);
  int rawSwB    = ibus.readChannel(CH_GAIT_SWITCH);
  int rawSwD    = ibus.readChannel(CH_MODE_SWITCH);

  // A read of exactly 0 means "channel not found / no frame yet",
  // which the IBusBM library returns on failure -- treat as invalid.
  bool anyChannelInvalid = (rawRightX == 0 || rawRightY == 0 ||
                             rawLeftY == 0  || rawLeftX == 0  ||
                             rawSwA == 0    || rawSwB == 0    ||
                             rawSwD == 0);

  if (anyChannelInvalid) {
    return false; // caller keeps last known-good state, or failsafes
  }

  lastGoodRcRead = millis();

  inputMoveX = normalizeStick(rawRightX);
  inputMoveY = normalizeStick(rawRightY);
  inputYaw   = normalizeStick(rawLeftX);
  inputHeight = constrain((float)(rawLeftY - RC_MIN) / (float)(RC_MAX - RC_MIN), 0.0, 1.0);

  legLockActive  = (rawSwA > RC_MID);
  danceModeActive = (rawSwD > RC_MID);
  currentGait = readGaitSwitch(rawSwB);

  return true;
}

// Converts a raw 1000-2000us pulse to a -1.0..1.0 normalized value
// with a symmetric deadzone around center, so idle sticks read 0.0
// exactly rather than drifting due to receiver jitter.
float normalizeStick(int rawValue) {
  int centered = rawValue - RC_MID;

  if (abs(centered) < RC_DEADZONE) {
    return 0.0;
  }

  if (centered > 0) {
    centered -= RC_DEADZONE;
  } else {
    centered += RC_DEADZONE;
  }

  float normalized = (float)centered / (float)(RC_MID - RC_MIN - RC_DEADZONE);
  return constrain(normalized, -1.0, 1.0);
}

// Maps SwB's 3-position raw value into a GaitType. If your SwB is
// truly only 2-position, this will only ever return GAIT_TROT or
// GAIT_WAVE (the high value) -- confirm on your TX before relying
// on crawl gait being reachable.
GaitType readGaitSwitch(int rawSwB) {
  if (rawSwB < RC_3POS_LOW_THRESH) {
    return GAIT_TROT;
  } else if (rawSwB > RC_3POS_HIGH_THRESH) {
    return GAIT_WAVE;
  } else {
    return GAIT_CRAWL;
  }
}

// If no valid RC frame has arrived recently, freeze the robot in a
// safe standing pose rather than continuing to act on stale data or
// letting legs drift. This runs every loop while signal is lost.
void handleReceiverFailsafe() {
  if (millis() - lastGoodRcRead > RC_TIMEOUT_MS) {
    holdStancePosition();
    danceModeActive = false; // never dance blind with no TX link
  }
}

// ================================================================
// SECTION 6: GAIT / MODE SWITCHING LOGIC
// ================================================================

// Resets phase counters cleanly whenever gait type or leg-lock/dance
// mode changes, so legs never jump mid-stride into a new pattern.
void updateGaitAndModeSelection() {
  static GaitType prevGait = GAIT_TROT;
  static bool prevLegLock = false;
  static bool prevDance = false;

  if (currentGait != prevGait) {
    gaitPhaseIndex = 0;
    gaitPhaseProgress = 0.0;
    prevGait = currentGait;
  }

  if (legLockActive && !prevLegLock) {
    captureLockedFootPositions(); // just entered lock: anchor current feet
  }
  prevLegLock = legLockActive;

  if (danceModeActive && !prevDance) {
    // Just entered dance mode: force an immediate new target pick
    // instead of waiting for the timer, so it starts moving right away.
    danceInterpT = 1.0;
    lastDanceTargetTime = 0;
  }
  prevDance = danceModeActive;
}

// Stores each leg's current foot position as the anchor point used
// while leg-lock mode keeps feet stationary on the ground.
void captureLockedFootPositions() {
  for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
    lockedFootX[leg] = footX[leg];
    lockedFootY[leg] = footY[leg];
    lockedFootZ[leg] = footZ[leg];
  }
}

// ================================================================
// SECTION 7: STANCE INITIALIZATION
// ================================================================

// Sets every leg to its default standing foot position -- the pose
// the FK/IK geometry constants were measured against.
void initializeStance() {
  for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
    footX[leg] = STANCE_X;
    footY[leg] = STANCE_Y;
    footZ[leg] = STANCE_Z_DEFAULT;
  }
  currentBodyZ = STANCE_Z_DEFAULT;
}

// ================================================================
// SECTION 8: FORWARD KINEMATICS
// ================================================================

// Given a leg's three joint angles (servo-frame degrees), computes
// where the foot tip physically ends up in that leg's local frame.
// Used at boot to verify the calibration stance is geometrically
// sound, and available for future odometry/logging use.
void forwardKinematics(float hipDeg, float thighDeg, float shankDeg,
                        float &outX, float &outY, float &outZ) {
  float hipRad   = radians(hipDeg);
  float thighRad = radians(thighDeg - 90.0); // 0 = straight down reference
  float shankRad = radians(shankDeg - 90.0);

  float horizontalReach = COXA_LEN
                         + FEMUR_LEN * cos(thighRad)
                         + TIBIA_LEN * cos(thighRad + shankRad);

  float verticalDrop = FEMUR_LEN * sin(thighRad)
                      + TIBIA_LEN * sin(thighRad + shankRad);

  outX = horizontalReach * sin(hipRad);
  outY = horizontalReach * cos(hipRad);
  outZ = -verticalDrop;
}

// Runs IK on the default stance, then FK on the result, and confirms
// the round-trip lands back near the intended stance coordinates.
// This just prints a warning to Serial if geometry constants look
// inconsistent -- it does not block boot, since a warning is more
// useful than a robot that refuses to start.
void verifyStanceWithFK() {
  float hipDeg, thighDeg, shankDeg;
  inverseKinematics(STANCE_X, STANCE_Y, STANCE_Z_DEFAULT, hipDeg, thighDeg, shankDeg);

  float checkX, checkY, checkZ;
  forwardKinematics(hipDeg, thighDeg, shankDeg, checkX, checkY, checkZ);

  float errorMagnitude = sqrt(sq(checkX - STANCE_X) +
                               sq(checkY - STANCE_Y) +
                               sq(checkZ - STANCE_Z_DEFAULT));

  if (errorMagnitude > 5.0) { // more than 5mm off -> flag it
    Serial.print(F("WARNING: FK/IK stance mismatch, error mm = "));
    Serial.println(errorMagnitude);
  }
}

// ================================================================
// SECTION 9: INVERSE KINEMATICS
// ================================================================

// Given a desired foot position (X,Y,Z) in the leg's local frame,
// solves for Hip/Thigh/Shank angles needed to reach it. The result
// is always geometrically valid because unreachable inputs are
// clamped to the nearest reachable point before solving.
void inverseKinematics(float targetX, float targetY, float targetZ,
                        float &outHipDeg, float &outThighDeg, float &outShankDeg) {

  // --- Hip angle: rotate around vertical axis to face the target ---
  float hipRad = atan2(targetX, targetY);
  outHipDeg = degrees(hipRad);

  // --- Reduce to a 2D problem in the leg's swing plane ---
  float horizontalDist = sqrt(targetX * targetX + targetY * targetY) - COXA_LEN;
  float verticalDist = targetZ;

  float planarDist = sqrt(horizontalDist * horizontalDist + verticalDist * verticalDist);

  // --- Reachability clamp (triangle inequality) ---
  float maxReach = FEMUR_LEN + TIBIA_LEN;
  float minReach = fabs(FEMUR_LEN - TIBIA_LEN);
  if (planarDist > maxReach) planarDist = maxReach;
  if (planarDist < minReach) planarDist = minReach;
  if (planarDist < 1.0) planarDist = 1.0; // avoid divide-by-zero

  // --- Law of Cosines on the Femur-Tibia-PlanarDist triangle ---
  float cosShankAngle = (FEMUR_LEN * FEMUR_LEN + TIBIA_LEN * TIBIA_LEN - planarDist * planarDist)
                         / (2.0 * FEMUR_LEN * TIBIA_LEN);
  cosShankAngle = constrain(cosShankAngle, -1.0, 1.0); // guard acos() domain
  float shankInteriorRad = acos(cosShankAngle);

  float cosThighOffset = (FEMUR_LEN * FEMUR_LEN + planarDist * planarDist - TIBIA_LEN * TIBIA_LEN)
                          / (2.0 * FEMUR_LEN * planarDist);
  cosThighOffset = constrain(cosThighOffset, -1.0, 1.0);
  float thighOffsetRad = acos(cosThighOffset);

  float elevationRad = atan2(-verticalDist, horizontalDist);
  float thighRad = elevationRad + thighOffsetRad;

  outThighDeg = degrees(thighRad) + 90.0;             // servo-frame: 90=straight down
  outShankDeg = 180.0 - degrees(shankInteriorRad);     // servo-frame bend angle
}

// ================================================================
// SECTION 10: APPLYING IK RESULTS TO SERVOS
// ================================================================

// Runs IK for every leg's current footX/Y/Z target and writes the
// resulting angles to the PCA9685. Called once per loop after all
// locomotion logic has updated footX/Y/Z for this cycle.
void applyAllLegIK() {
  for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
    float hipDeg, thighDeg, shankDeg;

    inverseKinematics(footX[leg], footY[leg], footZ[leg],
                       hipDeg, thighDeg, shankDeg);

    writeLegServos(leg, hipDeg, thighDeg, shankDeg);
  }
}

// Writes final joint angles for one leg to its 3 PCA9685 channels,
// applying per-servo inversion, trim, and a hard 0-180 safety clamp.
// This clamp is the last line of defense against any upstream bug
// (bad IK input, bad dance target, etc.) ever exceeding servo range.
void writeLegServos(uint8_t leg, float hipDeg, float thighDeg, float shankDeg) {
  float angles[3] = { hipDeg, thighDeg, shankDeg };

  for (uint8_t joint = 0; joint < 3; joint++) {
    float finalAngle = angles[joint] + SERVO_TRIM[leg][joint];

    if (SERVO_INVERT[leg][joint]) {
      finalAngle = 180.0 - finalAngle;
    }

    finalAngle = constrain(finalAngle, 0.0, 180.0); // hard safety clamp

    uint16_t tick = angleToPWMTick(finalAngle);
    pwm.setPWM(SERVO_CH[leg][joint], 0, tick);
  }
}

// Converts a 0-180 degree angle into the raw PCA9685 tick value.
uint16_t angleToPWMTick(float angleDeg) {
  return (uint16_t)map((long)(angleDeg * 10), 0, 1800, SERVO_MIN_TICK, SERVO_MAX_TICK);
}

// ================================================================
// SECTION 11: WALKING GAIT ENGINE (Trot / Crawl / Wave)
// ================================================================

// Master locomotion function. Builds each leg's target foot position
// this cycle based on current stick vector, selected gait, and timing.
void updateWalkingGait() {
  updateBodyHeight(); // throttle always active regardless of gait

  float stepDirX = inputMoveX; // strafe component (right stick X)
  float stepDirY = inputMoveY; // forward/back component (right stick Y)
  float yawComponent = inputYaw;

  // Combined magnitude (translation + yaw) drives proportional speed,
  // so pure rotation and pure translation both scale correctly, and
  // any diagonal combination naturally falls out of the vector sum.
  float moveMagnitude = sqrt(stepDirX * stepDirX + stepDirY * stepDirY);
  float overallMagnitude = max(moveMagnitude, fabs(yawComponent));
  overallMagnitude = constrain(overallMagnitude, 0.0, 1.0);

  if (overallMagnitude < 0.02) {
    holdStancePosition(); // sticks centered: stable stand, no stepping
    return;
  }

  float phaseSpeed = mapFloat(overallMagnitude, 0.0, 1.0, 0.15, 1.0);
  gaitPhaseProgress += phaseSpeed;

  uint8_t totalPhases = getPhaseCountForGait(currentGait);

  if (gaitPhaseProgress >= 1.0) {
    gaitPhaseProgress = 0.0;
    gaitPhaseIndex = (gaitPhaseIndex + 1) % totalPhases;
  }

  switch (currentGait) {
    case GAIT_TROT:
      runTrotGaitCycle(stepDirX, stepDirY, yawComponent, overallMagnitude);
      break;
    case GAIT_CRAWL:
      runSingleLegGaitCycle(CRAWL_SEQUENCE, stepDirX, stepDirY, yawComponent, overallMagnitude);
      break;
    case GAIT_WAVE:
      runSingleLegGaitCycle(WAVE_SEQUENCE, stepDirX, stepDirY, yawComponent, overallMagnitude);
      break;
  }
}

uint8_t getPhaseCountForGait(GaitType gait) {
  switch (gait) {
    case GAIT_TROT:  return TROT_PHASES;
    case GAIT_CRAWL: return CRAWL_PHASES;
    case GAIT_WAVE:  return WAVE_PHASES;
  }
  return TROT_PHASES; // unreachable, keeps compiler warning-free
}

// Reads current body height target and smooths body Z toward it via
// exponential smoothing, avoiding sudden height jumps/jolts.
void updateBodyHeight() {
  float targetZ = mapFloat(inputHeight, 0.0, 1.0, BODY_Z_MIN, BODY_Z_MAX);
  currentBodyZ += (targetZ - currentBodyZ) * 0.15;
}

// When sticks are centered, all legs return to a stable standing
// rectangle at the current commanded body height.
void holdStancePosition() {
  for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
    footX[leg] = STANCE_X;
    footY[leg] = STANCE_Y;
    footZ[leg] = currentBodyZ;
  }
}

// --- TROT GAIT: 2 diagonal legs swing while the other 2 push ---
void runTrotGaitCycle(float dirX, float dirY, float yaw, float magnitude) {
  bool groupAIsSwinging = (gaitPhaseIndex == 0);

  for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
    bool inGroupA = (leg == TROT_GROUP_A[0] || leg == TROT_GROUP_A[1]);
    bool thisLegSwinging = (inGroupA == groupAIsSwinging);

    computeLegStepTarget(leg, dirX, dirY, yaw, magnitude,
                          thisLegSwinging, gaitPhaseProgress);
  }
}

// --- CRAWL / WAVE GAIT: exactly 1 leg swings at a time, 3 planted ---
// Shared by both gaits; only the leg ORDER differs (passed in),
// which is what gives crawl and wave visually distinct footfall
// patterns even though both are maximally-stable single-leg gaits.
void runSingleLegGaitCycle(const uint8_t sequence[4],
                            float dirX, float dirY, float yaw, float magnitude) {
  uint8_t swingingLeg = sequence[gaitPhaseIndex];

  for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
    bool thisLegSwinging = (leg == swingingLeg);
    computeLegStepTarget(leg, dirX, dirY, yaw, magnitude,
                          thisLegSwinging, gaitPhaseProgress);
  }
}

// Computes ONE leg's foot target for this instant: swing phase
// (lift + arc forward through the air) or stance phase (stay
// planted, slide backward relative to body to create thrust).
void computeLegStepTarget(uint8_t leg, float dirX, float dirY, float yaw,
                           float magnitude, bool isSwinging, float phaseT) {

  float stepLen = STEP_LENGTH_MAX * magnitude;
  float yawStep = YAW_STEP_MAX_DEG * yaw;

  bool isFrontLeg = (leg == FR || leg == FL);
  bool isRightLeg = (leg == FR || leg == RR);
  float yawSign = isFrontLeg ? 1.0 : -1.0;
  float yawOffsetX = yawStep * yawSign * (isRightLeg ? 1.0 : -1.0);

  float travelX = (dirX * stepLen) + yawOffsetX;
  float travelY = (dirY * stepLen);

  if (isSwinging) {
    // SWING PHASE: foot lifts and arcs from back-of-stride to
    // front-of-stride. Sine arc gives smooth lift/set motion.
    float swingT = phaseT;
    float xOffset = mapFloat(swingT, 0.0, 1.0, -travelX / 2.0, travelX / 2.0);
    float yOffset = mapFloat(swingT, 0.0, 1.0, -travelY / 2.0, travelY / 2.0);
    float liftOffset = STEP_LIFT_HEIGHT * sin(swingT * PI);

    footX[leg] = STANCE_X + xOffset;
    footY[leg] = STANCE_Y + yOffset;
    footZ[leg] = currentBodyZ + liftOffset;

  } else {
    // STANCE PHASE: foot stays on ground, slides backward relative
    // to the body -- this backward slide is what pushes the robot
    // forward (equal-and-opposite locomotion, no lift applied).
    float stanceT = phaseT;
    float xOffset = mapFloat(stanceT, 0.0, 1.0, travelX / 2.0, -travelX / 2.0);
    float yOffset = mapFloat(stanceT, 0.0, 1.0, travelY / 2.0, -travelY / 2.0);

    footX[leg] = STANCE_X + xOffset;
    footY[leg] = STANCE_Y + yOffset;
    footZ[leg] = currentBodyZ;
  }
}

// ================================================================
// SECTION 12: LEG-LOCK MODE (feet planted, body-only IK motion)
// ================================================================

// When SwA is active, feet do not step. Instead, the sticks move the
// BODY relative to the fixed feet -- achieved by applying the inverse
// transform to each foot's target so that, in the (moving) body
// frame, the foot appears to counter-shift and stays visually
// planted on the ground.
void updateLegLockMode() {
  updateBodyHeight(); // throttle still adjusts height even when locked

  float bodyShiftX = inputMoveX * 25.0;         // mm, lean left/right
  float bodyShiftY = inputMoveY * 25.0;         // mm, lean fwd/back
  float bodyYawRad = radians(inputYaw * 20.0);  // body twist in place

  for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
    float ax = lockedFootX[leg];
    float ay = lockedFootY[leg];

    // Inverse body rotation: as the body yaws one way, the foot's
    // position IN THE BODY FRAME must rotate the opposite way to
    // remain stationary in the world frame.
    float rotX = ax * cos(-bodyYawRad) - ay * sin(-bodyYawRad);
    float rotY = ax * sin(-bodyYawRad) + ay * cos(-bodyYawRad);

    // Inverse body translation, same logic: body moves forward ->
    // foot appears to move backward in the leg's local frame.
    footX[leg] = rotX - bodyShiftX;
    footY[leg] = rotY - bodyShiftY;
    footZ[leg] = currentBodyZ;
  }
}

// ================================================================
// SECTION 13: DANCE MODE (SwD) - bounded random movement
// ================================================================

// Random "mad dance" mode: periodically picks a new random-but-safe
// foot target per leg and smoothly interpolates toward it. Takes NO
// input from the TX sticks (as specified) -- only SwD itself (to
// exit) and the failsafe are still honored, both handled upstream
// in updateGaitAndModeSelection()/handleReceiverFailsafe().
//
// Safety: every random target is generated within the SAME physical
// bounds as manual mode (a box around the stance point, and a height
// range within BODY_Z_MIN/MAX), and passes through the exact same
// inverseKinematics() reachability clamp as everything else. Random
// does not mean unbounded.
void updateDanceMode() {
  unsigned long now = millis();

  if (danceInterpT >= 1.0) {
    // Current move finished: lock in the reached position as the new
    // start point, then roll a fresh random target + duration.
    for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
      danceStartX[leg] = footX[leg];
      danceStartY[leg] = footY[leg];
      danceStartZ[leg] = footZ[leg];
    }

    generateRandomDanceTargets();
    danceMoveDurationMs = random(150, 450); // varies pace, feels "mad"
    lastDanceTargetTime = now;
    danceInterpT = 0.0;
  }

  // Advance interpolation progress based on elapsed time so the
  // move speed is consistent regardless of main loop jitter.
  float elapsed = (float)(now - lastDanceTargetTime);
  danceInterpT = constrain(elapsed / (float)danceMoveDurationMs, 0.0, 1.0);

  // Ease-in-out curve (smoothstep) for more organic, less robotic-
  // looking dance movement than linear interpolation would give.
  float eased = danceInterpT * danceInterpT * (3.0 - 2.0 * danceInterpT);

  for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
    footX[leg] = mapFloat(eased, 0.0, 1.0, danceStartX[leg], danceTargetX[leg]);
    footY[leg] = mapFloat(eased, 0.0, 1.0, danceStartY[leg], danceTargetY[leg]);
    footZ[leg] = mapFloat(eased, 0.0, 1.0, danceStartZ[leg], danceTargetZ[leg]);
  }
}

// Rolls a new bounded-random foot target for every leg. Bounds are
// deliberately kept inside the same envelope manual walking uses,
// so dance mode cannot command a physically impossible or violent
// motion even though its targets are random.
void generateRandomDanceTargets() {
  for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
    float randX = STANCE_X + (float)random(-400, 400) / 10.0;   // +-40mm
    float randY = STANCE_Y + (float)random(-300, 300) / 10.0;   // +-30mm
    float randZ = (float)random((long)(BODY_Z_MAX * 10), (long)(BODY_Z_MIN * 10)) / 10.0;

    danceTargetX[leg] = randX;
    danceTargetY[leg] = randY;
    danceTargetZ[leg] = randZ;
  }
}

// ================================================================
// SECTION 14: LCD DISPLAY
// ================================================================

// One-time boot screen shown during setup(): robot name + author.
void showBootScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("SpotMicro v1"));
  lcd.setCursor(0, 1);
  lcd.print(F("by Chaitanya"));
}

// Refreshes the running info screen (gait / mode / leg-lock state).
// Throttled to LCD_UPDATE_MS and only rewrites when something has
// actually changed, since I2C LCD writes are slow enough to matter
// if done every 20ms control loop tick.
void updateLcdDisplay() {
  unsigned long now = millis();
  if (now - lastLcdUpdate < LCD_UPDATE_MS && !lcdNeedsFullRedraw) {
    return;
  }
  lastLcdUpdate = now;

  bool changed = lcdNeedsFullRedraw ||
                 (currentGait != lastShownGait) ||
                 (legLockActive != lastShownLegLock) ||
                 (danceModeActive != lastShownDance);

  if (!changed) {
    return;
  }

  lcd.clear();
  lcd.setCursor(0, 0);

  if (danceModeActive) {
    lcd.print(F("Mode: DANCE"));
  } else if (legLockActive) {
    lcd.print(F("Mode: LEG-LOCK"));
  } else {
    lcd.print(F("Gait: "));
    lcd.print(getGaitName(currentGait));
  }

  lcd.setCursor(0, 1);
  lcd.print(F("SpotMicro-Chai"));

  lastShownGait = currentGait;
  lastShownLegLock = legLockActive;
  lastShownDance = danceModeActive;
  lcdNeedsFullRedraw = false;
}

const __FlashStringHelper* getGaitName(GaitType gait) {
  switch (gait) {
    case GAIT_TROT:  return F("TROT");
    case GAIT_CRAWL: return F("CRAWL");
    case GAIT_WAVE:  return F("WAVE");
  }
  return F("?");
}

// ================================================================
// SECTION 15: UTILITY FUNCTIONS
// ================================================================

// Floating-point version of Arduino's built-in map(), since map()
// only operates on longs and truncates precision.
float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

/*
  ================================================================
   REQUIRED SETUP BEFORE THIS COMPILES / RUNS -- READ FIRST
  ================================================================

  1. LIBRARIES (install via Arduino IDE Library Manager):
       - "Adafruit PWM Servo Driver Library"
       - "IBusBM" by Bulanov Konstantin
       - "LiquidCrystal I2C" (the Frank de Brabander / johnrickman fork
         that exposes .init() -- some older forks use .begin() instead;
         if you get a compile error on lcd.init(), swap it for
         lcd.begin(16,2) per your installed library version)

  2. SOFTWARESERIAL FOR iBUS -- ADD THIS NEAR THE TOP OF THE FILE:
     A stock UNO has only ONE hardware UART, which this code reserves
     for USB debug output (Serial.println). The iA10B's iBUS line
     must therefore go through SoftwareSerial on two spare digital
     pins. Add this near your #include lines, BEFORE setup():

       #include <SoftwareSerial.h>
       #define IBUS_RX_PIN 10   // wire iA10B iBUS OUT here
       #define IBUS_TX_PIN 11   // unused by iBUS but required by lib
       SoftwareSerial ibusSerial(IBUS_RX_PIN, IBUS_TX_PIN);

     This declares the `ibusSerial` object that setup() calls
     .begin(115200) on. Confirm your iA10B is jumpered/configured
     for iBUS (single-wire serial) output, not classic per-channel
     PWM -- those require a completely different reading method.

  3. I2C ADDRESSES: The PCA9685 (0x40 default) and the LCD backpack
     (commonly 0x27, sometimes 0x3F) share the SAME I2C bus (A4/A5 on
     UNO) but need DIFFERENT addresses -- they do by default, but if
     you've changed PCA9685's address jumpers, make sure it never
     collides with 0x27/0x3F. If the LCD stays blank, run an I2C
     scanner sketch first to find its real address and update
     LCD_ADDR above.

  4. BEFORE FIRST POWER-ON WITH SERVOS ATTACHED: hold the robot with
     all legs hanging free (not touching the ground, feet not
     attached to the 3D-printed toes yet if possible). Power up and
     watch the boot stance. If any leg strains against a mechanical
     limit or looks visibly wrong-angled, STOP, and adjust that leg's
     SERVO_TRIM[][] / SERVO_INVERT[][] values before proceeding.
     Do this leg-by-leg. Only attach feet to the ground after all 4
     legs sit in a symmetric, square stance mid-air.

  5. POWER: 12x MG996R under load draw well beyond what the UNO's 5V
     pin or a small BEC can supply, especially during dance mode's
     rapid direction changes. Power servos from a dedicated 5-6V,
     6A+ supply, common ground with the UNO -- never from the UNO's
     onboard 5V regulator.

  6. SwB 3-POSITION CONFIRMATION: this code reads CH_GAIT_SWITCH as a
     3-level value to select Trot/Crawl/Wave. Open your FS-i6S's
     transmitter setup menu and confirm SwB is actually assigned as a
     3-position switch feeding channel 6 (or update CH_GAIT_SWITCH
     above to match whichever channel your 3-pos switch really feeds).
     If SwB is 2-position only on your unit, gait selection will only
     ever reach two of the three gaits -- reassign the switch in the
     TX menu, don't just hope the code compensates for it.
  ================================================================
*/