#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050_light.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define IBUS_RX 16

HardwareSerial IBUS(2);
uint8_t frame[64];
uint8_t framePos = 0;
uint8_t frameLength = 0;
uint16_t channels[10];
uint32_t lastFrameMs = 0;

// ---- Types first: Arduino auto-prototypes need them before any function ----
enum RobotCommand : uint8_t {
  CMD_STOP, CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT,
  CMD_STRAFE_LEFT, CMD_STRAFE_RIGHT, CMD_TROT, CMD_PACE, CMD_CRAWL,
  CMD_BOUND, CMD_STAND, CMD_PRY_TEST, CMD_PRY_SET,
  CMD_HANDSHAKE
};
struct LegInfo { uint8_t hip, thigh, knee; bool isRight; float ox, oy; };
struct BleCmd  { const char *keys; RobotCommand cmd; const char *name; };

// Redraws only when the text changes (keeps I2C traffic low)
void lcdPrint(const char *a, const char *b) {
  static char la[17] = "", lb[17] = "";
  if (!strcmp(a, la) && !strcmp(b, lb)) return;
  strncpy(la, a, 16); strncpy(lb, b, 16);
  char buf[17];
  lcd.setCursor(0, 0); snprintf(buf, 17, "%-16s", a); lcd.print(buf);
  lcd.setCursor(0, 1); snprintf(buf, 17, "%-16s", b); lcd.print(buf);
}

// ---- Hardware ----
Adafruit_PWMServoDriver pca(0x40);
MPU6050 mpu(Wire);
const int SERVOMIN = 150, SERVOMAX = 600;

// leg order everywhere: 0=FL 1=FR 2=BL 3=BR
LegInfo legFL = {11, 13, 15, false,  120, -40};
LegInfo legFR = {10, 12, 14, true,   120,  40};
LegInfo legBL = { 3,  2,  0, false, -120, -40};
LegInfo legBR = { 5,  4,  1, true,  -120,  40};
LegInfo *const legs[4] = {&legFL, &legFR, &legBL, &legBR};

float SERVO_OFFSET[16] = {-2.00, -10.00, 2.00, -2.00, -3.00, -3.00, 0.00, 0.00, 0.00, 0.00, -8.00, -1.00, 5.00, 3.00, 18.00, -18.00};


// ---- BLE ----
#define SVC_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_UUID  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define TX_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
BLECharacteristic *bleTx = NULL;
volatile bool bleConnected = false;

// ---- Robot state ----
volatile RobotCommand robotCommand = CMD_STOP;
volatile float pryP = 0, pryR = 0, pryY = 0;
float X = 0, Y = 40, H = 150;          // home pose
float SL = 40, SH = 40;                // step length / height
float StrafeHeight = 40, StrafeLength = 40;
volatile float speed = 3.5;
const int baseDelay = 5, baseSteps = 10;
volatile bool handshakeActive = false;

void setRobotCommand(RobotCommand c) { robotCommand = c; }

float bezier(float t, float p0, float p1, float p2, float p3) {
  float u = 1 - t;
  return u*u*u*p0 + 3*u*u*t*p1 + 3*u*t*t*p2 + t*t*t*p3;
}

int angleToPulse(float angle, int ch) {
  angle = constrain(angle + SERVO_OFFSET[ch], 0.0f, 180.0f);
  return SERVOMIN + (int)(angle / 180.0f * (SERVOMAX - SERVOMIN));
}

// ---- Inverse kinematics (x fore-aft, y sideways, z down) ----
void moveTo(const LegInfo &leg, float x, float y, float z) {
  const float L1 = 40, L2 = 120, L3 = 130;
  float c2 = y*y + z*z;
  if (c2 < L1*L1) { Serial.println("UNREACHABLE: C^2 < L1^2"); return; }
  float D = sqrt(c2 - L1*L1);
  float B = sqrt(D*D + x*x);
  if (B < 1e-5f) { Serial.println("UNREACHABLE: B ~ 0"); return; }
  float kc = (B*B - L2*L2 - L3*L3) / (-2*L2*L3);
  if (kc > 1 || kc < -1) { Serial.println("UNREACHABLE: acos range"); return; }
  float Q3 = acos(kc);
  float sv = L3 * sin(Q3) / B;
  if (sv > 1 || sv < -1) { Serial.println("UNREACHABLE: asin range"); return; }
  float Q1 = degrees(atan2(y, z) + atan2(D, L1));
  float Q2 = degrees(asin(sv) + atan2(x, D)) + 90;
  Q3 = degrees(Q3);
  if (leg.isRight) { Q1 = 180 - Q1; Q2 = 180 - Q2; Q3 = 180 - Q3; }
  pca.setPWM(leg.hip,   0, angleToPulse(Q1, leg.hip));
  pca.setPWM(leg.thigh, 0, angleToPulse(Q2, leg.thigh));
  pca.setPWM(leg.knee,  0, angleToPulse(Q3, leg.knee));
}

void moveAllLegs(float x, float y, float z) {
  for (int i = 0; i < 4; i++) moveTo(*legs[i], x, y, z);
}

// ---- Gait helpers ----
void getGaitTiming(int &steps, int &stepDelay) {
  stepDelay = max(1, (int)(baseDelay / speed));
  steps = max(5, (int)(baseSteps * speed));
}
float swingX(float t, float e) { return bezier(t, -e, -e/2, e/2, e); }
float swingZ(float t) { return bezier(t, H, H - SH*1.3, H - SH*1.3, H); }

// ---- Pitch / Roll / Yaw posing ----
void rollYZ(float z, float r, bool left, float &y, float &zz) {
  float h = left ? z - 60*sin(r) : z + 60*sin(r);
  float L = sqrt(1600 + h*h);
  float a = left ? atan2(40, h) + r : r;
  y = L * sin(a);
  zz = L * cos(a);
}

void applyPRY(float pitch, float roll, float yaw, float height){
  float p = radians(pitch), r = radians(roll), w = radians(yaw);
  float fz = height + 95*sin(p), bz = height - 95*sin(p);
  float fx = sin(p)*fz, bx = sin(p)*bz;
  //front left roll
  float FL_L = sqrt((40*40)+((fz-(60*sin(r)))*(fz-(60*sin(r)))));
  float FL_Q = r + atan2(40, fz - (60*sin(r)));
  float FL_Y = (sin(FL_Q)) * FL_L;
  float FL_Z = (cos(FL_Q)) * FL_L;
  //front right roll
  float FR_L = sqrt((40*40)+((fz+(60*sin(r)))*(fz+(60*sin(r)))));
  float FR_Q = r - atan2(40, fz + (60*sin(r)));
  float FR_Y = (sin(FR_Q)) * FR_L;
  float FR_Z = (cos(FR_Q)) * FR_L;
  //back left roll
  float BL_L = sqrt((40*40)+((bz-(60*sin(r)))*(bz-(60*sin(r)))));
  float BL_Q = r + atan2(40, bz - (60*sin(r)));
  float BL_Y = (sin(BL_Q)) * BL_L;
  float BL_Z = (cos(BL_Q)) * BL_L;
  //back right roll
  float BR_L = sqrt((40*40)+((bz+(60*sin(r)))*(bz+(60*sin(r)))));
  float BR_Q = r - atan2(40, bz + (60*sin(r)));
  float BR_Y = (sin(BR_Q)) * BR_L;
  float BR_Z = (cos(BR_Q)) * BR_L; 

  const float R = sqrt(95*95 + 100*100), A = radians(43.53), B = radians(46.47);
  //yaw
  float FLx = ((cos(w+B))*R)-95;
  float FLy = 60-((sin(w+B))*R);
  float FRx = ((sin(w+A))*R) -95;
  float FRy = ((cos(w+A))*R) -60;
  float BLx = 95 - ((sin(w+A))*R);
  float BLy = 60 - ((cos(w+A))*R);
  float BRx = 95 - ((cos(w+B))*R);
  float BRy = ((sin(w+B))*R) -60;
  

  moveTo(legFL, fx + FLx, FL_Y + FLy +40, FL_Z);
  moveTo(legFR, fx + FRx, FR_Y + FRy +40, FR_Z );
  moveTo(legBL, bx + BLx, BL_Y + BLy +40, BL_Z );
  moveTo(legBR, bx + BRx, BR_Y + BRy +40, BR_Z );


}

void testPRY(float height) {
  static const float poses[][4] = {   // pitch, roll, yaw, hold_ms
    {20, 0, 0, 0}, {-20, 0, 0, 0}, {0, 0, 0, 0}, {0, 20, 0, 0}, {0, -20, 0, 0},
    {0, 0, 0, 0}, {0, 0, 20, 0}, {0, 0, -20, 0}, {0, 0, 0, 2000}, {20, 0, 0, 0},
    {20, 20, 0, 0}, {20, 20, 20, 0}, {-20, 20, 20, 0}, {-20, -20, 0, 0},
    {-20, -20, -20, 0}, {0, -20, -20, 0}, {0, 0, -20, 0}, {0, 0, 0, 1000}
  };
  static float cur[3] = {0, 0, 0};
  const int steps = 25;                       // 500 ms per move, 20 ms per step
  for (unsigned k = 0; k < sizeof(poses) / sizeof(poses[0]); k++) {
    float s[3] = {cur[0], cur[1], cur[2]};
    for (int i = 1; i <= steps; i++) {
      float t = (float)i / steps;
      t = t * t * (3 - 2 * t);                // smoothstep
      applyPRY(s[0] + (poses[k][0] - s[0]) * t,
               s[1] + (poses[k][1] - s[1]) * t,
               s[2] + (poses[k][2] - s[2]) * t,
               height);
      delay(20);
    }
    for (int j = 0; j < 3; j++) cur[j] = poses[k][j];
    if (poses[k][3] > 0) delay((unsigned long)poses[k][3]);
  }
}

void trot(float SH, float SL, float height){
  static const float XF[6][4]={{.5,-.5,-.5,.5},{.1667,-.1667,-.1667,.1667},{-.1667,.1667,.1667,-.1667},
                               {-.5,.5,.5,-.5},{-.1667,.1667,.1667,-.1667},{.1667,-.1667,-.1667,.1667}};
  static const byte LF[6][4]={{0,0,0,0},{0,1,1,0},{0,1,1,0},{0,0,0,0},{1,0,0,1},{1,0,0,1}};
  static const unsigned STAGE_MS=80;          // time per stage; lower = faster
  static byte st=0;
  static unsigned long t0=0,tl=0;
  static float fSH,fSL,fH;
  unsigned long now=millis();

  if(now-tl>300){ fSH=SH; fSL=SL; fH=height; t0=now; st=0; }  // restart after a pause
  if(now-tl<10) return;                                        // ~100 Hz update limit
  tl=now;

  fSH+=(SH-fSH)*0.15f; fSL+=(SL-fSL)*0.15f; fH+=(height-fH)*0.15f;  // smooth stick changes

  while(now-t0>=STAGE_MS){ t0+=STAGE_MS; st=(st+1)%6; }
  float t=(now-t0)/(float)STAGE_MS;
  float s=t*t*(3-2*t);                         // smoothstep for lifting
  byte n=(st+1)%6;

  float x[4],z[4];
  for(byte i=0;i<4;i++){
    x[i]=fSL*(XF[st][i]+(XF[n][i]-XF[st][i])*t);
    z[i]=fH-fSH*(LF[st][i]+(LF[n][i]-LF[st][i])*s);
  }
  moveTo(legFL,x[0],Y,z[0]);
  moveTo(legFR,x[1],Y,z[1]);
  moveTo(legBL,x[2],Y,z[2]);
  moveTo(legBR,x[3],Y,z[3]);
}

void strafe(float SH, float SL, float height){
  static const float XF[6][4]={{.5,-.5,-.5,.5},{.1667,-.1667,-.1667,.1667},{-.1667,.1667,.1667,-.1667},
                               {-.5,.5,.5,-.5},{-.1667,.1667,.1667,-.1667},{.1667,-.1667,-.1667,.1667}};
  static const byte LF[6][4]={{0,0,0,0},{0,1,1,0},{0,1,1,0},{0,0,0,0},{1,0,0,1},{1,0,0,1}};
  static const float SD[4]={1,-1,1,-1};        // side sign: FL,FR,BL,BR
  static const unsigned STAGE_MS=80;
  static byte st=0;
  static unsigned long t0=0,tl=0;
  static float fSH,fSL,fH;
  unsigned long now=millis();

  if(now-tl>300){ fSH=SH; fSL=SL; fH=height; t0=now; st=0; }
  if(now-tl<10) return;
  tl=now;

  fSH+=(SH-fSH)*0.15f; fSL+=(SL-fSL)*0.15f; fH+=(height-fH)*0.15f;

  while(now-t0>=STAGE_MS){ t0+=STAGE_MS; st=(st+1)%6; }
  float t=(now-t0)/(float)STAGE_MS;
  float s=t*t*(3-2*t);
  byte n=(st+1)%6;

  float y[4],z[4];
  for(byte i=0;i<4;i++){
    y[i]=Y+SD[i]*fSL*(XF[st][i]+(XF[n][i]-XF[st][i])*t);
    z[i]=fH-fSH*(LF[st][i]+(LF[n][i]-LF[st][i])*s);
  }
  moveTo(legFL,0,y[0],z[0]);
  moveTo(legFR,0,y[1],z[1]);
  moveTo(legBL,0,y[2],z[2]);
  moveTo(legBR,0,y[3],z[3]);
}

const float SIDE_R = -1;     // set from the test above (+1 or -1)

bool reachable(float x, float y, float z){
  float c2 = y*y + z*z;
  if (c2 < 1600.0f) return false;
  float D = sqrt(c2 - 1600.0f), B = sqrt(D*D + x*x);
  return B > 12.0f && B < 248.0f;              // |L2-L3| < B < L2+L3
}

void moveSafe(int i, float x, float y, float z){
  for (int k = 0; k < 8 && !reachable(x, y, z); k++){   // pull toward neutral if unreachable
    x *= 0.8f;
    y = Y + (y - Y) * 0.8f;
  }
  if (reachable(x, y, z)) moveTo(*legs[i], x, y, z);
}

void omniTrot(float SH, float vx, float vy, float wz, float height){
  static const float TX[4]  = { 0.316f,-0.316f, 0.316f,-0.316f};  // rotation direction per leg
  static const float TY[4]  = { 0.949f, 0.949f,-0.949f,-0.949f};
  static const float PH[4]  = { 0.0f, 0.5f, 0.5f, 0.0f};          // FL+BR together, FR+BL opposite
  static const float SIDE[4]= { 1, SIDE_R, 1, SIDE_R};
  const float CYCLE_MS = 500.0f;                                   // gait speed: lower = faster
  const float DUTY = 0.6f;                                         // fraction of cycle on the ground
  static float phase = 0, fSH = 0, fX = 0, fY = 0, fW = 0, fH = 150;
  static unsigned long tl = 0;
  unsigned long now = millis();

  if (now - tl > 300) { fX = fY = fW = 0; fSH = SH; fH = height; phase = 0; tl = now; return; }
  unsigned long dt = now - tl;
  if (dt < 10) return;
  tl = now;

  phase += dt / CYCLE_MS;  if (phase >= 1) phase -= 1;
  fSH += (SH - fSH) * 0.15f; fH += (height - fH) * 0.15f;
  fX += (vx - fX) * 0.15f; fY += (vy - fY) * 0.15f; fW += (wz - fW) * 0.15f;

  float amp  = max(max(fabs(fX), fabs(fY)), fabs(fW));
  float lift = fSH * constrain(amp / 15.0f, 0.0f, 1.0f);

  for (int i = 0; i < 4; i++){
    float ph = phase + PH[i];  if (ph >= 1) ph -= 1;
    float p, up;
    if (ph < DUTY){                                  // stance: foot slides back at constant speed
      p  = 0.5f - ph / DUTY;
      up = 0;
    } else {                                         // swing: foot lifts and returns forward
      float u = (ph - DUTY) / (1 - DUTY);
      p  = -0.5f + (u - sin(2*PI*u) / (2*PI));       // zero speed at both ends
      up = sin(PI * u);
    }
    float lat = fY + fW * TY[i];
    moveSafe(i, (fX + fW * TX[i]) * p,
                Y + SIDE[i] * lat * p,
                fH - lift * up);
  }
}


// Complete dog-like handshake sequence.
// IMPORTANT: this function is intentionally blocking.
void doHandshake() {

  handshakeActive = true;
  lcdPrint("HANDSHAKE", "Shaking hand...");
  delay(200);
  applyPRY(0,0,0, 150);
  delay(400);
  Serial.println("HANDSHAKE START");
  applyPRY(25, 15, 0, 150);
  delay(300);
  for (int i =0; i<=200; i++){
    moveTo(legFR, 84 + i/2, 36, 209 - i);
    delay(5);
  }
  delay(200);
  for (int i = 0; i<=50; i++){
    moveTo(legFR, 184, 36, 9 + i );
    delay(2);
  }
  delay(500);
  for (int i = 0; i<=50; i++){
    moveTo(legFR, 184, 36, 59- i);
    delay(2);
  }
  delay(300);
  for (int i =0; i<=200; i++){
    moveTo(legFR, 184 - i/2, 36, 9 +i);
    delay(5);
  }
  delay(300);
  applyPRY(0, 0, 0, 150);
  delay(200);

  lcdPrint("HANDSHAKE", "Done");
  handshakeActive = false;
  robotCommand = CMD_STAND;
}

// ---- Gaits ----
// Creep walk: FL, BR, FR, BL. d=+1 forward, d=-1 backward.
void walkGait(float d) {
  int steps, sd; getGaitTiming(steps, sd);
  static const uint8_t order[4] = {0, 3, 1, 2};
  float e = d * SL / 2;
  int n = steps / 4;
  for (int k = 0; k < 4; k++) {
    int s = order[k];
    for (int i = 0; i <= steps; i++) {
      float t = (float)i / steps;
      moveTo(*legs[s], swingX(t, e), Y, swingZ(t));
      delay(sd);
    }
    for (int i = 0; i <= n; i++) {
      float sx = e - (float)i / n * e / 2;
      for (int j = 0; j < 4; j++) if (j != s) moveTo(*legs[j], sx, Y, H);
      delay(sd);
    }
    delay(d > 0 ? 500 : 100);
  }
}

// Two-phase gait: legs in maskA swing first, then the rest. Bit j = leg j.
void pairGait(uint8_t maskA, float dir) {
  int steps, sd; getGaitTiming(steps, sd);
  float e = dir * SL / 2;
  for (int ph = 0; ph < 2; ph++) {
    uint8_t mask = ph ? (~maskA & 15) : maskA;
    for (int i = 0; i <= steps; i++) {
      float t = (float)i / steps;
      float sx = swingX(t, e), sz = swingZ(t), tx = e - 2*e*t;
      for (int j = 0; j < 4; j++) {
        if ((mask >> j) & 1) moveTo(*legs[j], sx, Y, sz);
        else                 moveTo(*legs[j], tx, Y, H);
      }
      delay(sd);
    }
  }
}
void trotGait()  { pairGait(0b1001, -1); }   // FL+BR, then FR+BL
void paceGait()  { pairGait(0b0101,  1); }   // FL+BL, then FR+BR
void boundGait() { pairGait(0b0011,  1); }   // FL+FR, then BL+BR

// 4-beat crawl: one leg swings (FL, BR, FR, BL) while three stay down.
void crawlGait() {
  int steps, sd; getGaitTiming(steps, sd);
  static const uint8_t order[4] = {0, 3, 1, 2};
  float a = SL / 2, b = -SL / 2;
  for (int k = 0; k < 4; k++) {
    for (int i = 0; i <= steps; i++) {
      float t = (float)i / steps;
      float sx = bezier(t, a, a + (b - a)*0.33, a + (b - a)*0.66, b);
      float sz = H - SH*1.3 * 4*t*(1 - t);
      float tx = b + t * SL;
      for (int j = 0; j < 4; j++) {
        if (j == order[k]) moveTo(*legs[j], sx, Y, sz);
        else               moveTo(*legs[j], tx, Y, H);
      }
      delay(sd);
    }
  }
}

void sideWalkGait(bool right) {
  int steps, sd; getGaitTiming(steps, sd);
  float s = right ? 1 : -1;
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    moveAllLegs(X, bezier(t, 40*s, 20*s, -20*s, -40*s), swingZ(t));
    delay(sd);
  }
  for (int i = 0; i <= steps; i++) {
    moveAllLegs(X, -40*s + (float)i / steps * 80*s, H);
    delay(sd);
  }
}

void rotateGait(bool cw) {
  int steps, sd; getGaitTiming(steps, sd);
  float dir = cw ? -1 : 1, rot = 20, tx[4], ty[4];
  for (int i = 0; i < 4; i++) {
    float r = sqrt(legs[i]->ox * legs[i]->ox + legs[i]->oy * legs[i]->oy);
    tx[i] = -legs[i]->oy / r * dir;
    ty[i] =  legs[i]->ox / r * dir;
  }
  for (int c = 0; c < 2; c++) {
    uint8_t mask = c ? 0b0110 : 0b1001;      // swing pair: FL+BR, then FR+BL
    for (int i = 0; i <= steps; i++) {
      float t = (float)i / steps;
      float lift = H - SH*1.3 * sin(PI * t);
      float k = rot * (t - 0.5) * 2;
      for (int j = 0; j < 4; j++) {
        if ((mask >> j) & 1) moveTo(*legs[j], X + tx[j]*k, Y + ty[j]*k, lift);
        else                 moveTo(*legs[j], X - tx[j]*k, Y - ty[j]*k, H);
      }
      delay(sd);
    }
  }
}

// ---- BLE ----
void sendBLE(const String &m) {
  if (bleConnected && bleTx) { bleTx->setValue(m.c_str()); bleTx->notify(); }
}

const BleCmd bleCmds[] = {

  {"|F|FORWARD|",          CMD_FORWARD,      "FORWARD"},
  {"|B|BACK|BACKWARD|",    CMD_BACKWARD,     "BACKWARD"},
  {"|L|LEFT|",             CMD_LEFT,         "LEFT"},
  {"|R|RIGHT|",            CMD_RIGHT,        "RIGHT"},
  {"|Q|SL|STRAFELEFT|",    CMD_STRAFE_LEFT,  "STRAFE LEFT"},
  {"|E|SR|STRAFERIGHT|",   CMD_STRAFE_RIGHT, "STRAFE RIGHT"},
  {"|T|TROT|",             CMD_TROT,         "TROT"},
  {"|P|PACE|",             CMD_PACE,         "PACE"},
  {"|C|CRAWL|",            CMD_CRAWL,        "CRAWL"},
  {"|D|BOUND|",            CMD_BOUND,        "BOUND"},
  {"|S|STOP|",             CMD_STOP,         "STOP"},
  {"|H|HOME|STAND|",       CMD_STAND,        "STAND"},
  {"|Y|TEST|PRY|",         CMD_PRY_TEST,     "PRY TEST"},

  // NEW
  {"|K|SHAKE|HANDSHAKE|",  CMD_HANDSHAKE,    "HANDSHAKE"}
};

void bleHandleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (!cmd.length()) return;

  String key = "|" + cmd + "|";
  for (const BleCmd &b : bleCmds) {
    if (strstr(b.keys, key.c_str())) {
      setRobotCommand(b.cmd);
      sendBLE(String(b.name) + "\n");
      return;
    }
  }

  if (cmd.startsWith("SPEED")) {                 // SPEED 0.5 .. 6.0
    float v = cmd.substring(5).toFloat();
    if (v >= 0.5f && v <= 6.0f) { speed = v; sendBLE("SPEED " + String(v, 2) + "\n"); }
    else sendBLE("SPEED RANGE 0.5-6.0\n");
    return;
  }

  if (cmd.startsWith("PRY,")) {                  // PRY,pitch,roll,yaw
    String d = cmd.substring(4);
    int p1 = d.indexOf(','), p2 = d.indexOf(',', p1 + 1);
    if (p1 > 0 && p2 > p1) {
      pryP = constrain(d.substring(0, p1).toFloat(), -25.0f, 25.0f);
      pryR = constrain(d.substring(p1 + 1, p2).toFloat(), -25.0f, 25.0f);
      pryY = constrain(d.substring(p2 + 1).toFloat(), -25.0f, 25.0f);
      setRobotCommand(CMD_PRY_SET);
      sendBLE("PRY OK\n");
    } else sendBLE("USE PRY,pitch,roll,yaw\n");
    return;
  }

  // Legacy axis commands: W/S = fwd/back, T = turn, H = strafe (e.g. W50, T-40)
  if (cmd.length() >= 2 && (isDigit(cmd[1]) || cmd[1] == '-')) {
    int v = cmd.substring(1).toInt();
    RobotCommand pos = CMD_STOP, neg = CMD_STOP;
    bool ok = true;
    switch (cmd[0]) {
      case 'W': pos = CMD_FORWARD;      neg = CMD_BACKWARD;     break;
      case 'S': pos = CMD_BACKWARD;     neg = CMD_FORWARD;      break;
      case 'T': pos = CMD_RIGHT;        neg = CMD_LEFT;         break;
      case 'H': pos = CMD_STRAFE_RIGHT; neg = CMD_STRAFE_LEFT;  break;
      default:  ok = false;
    }
    if (ok) {
      setRobotCommand(v > 10 ? pos : v < -10 ? neg : CMD_STOP);
      sendBLE(String(cmd[0]) + "\n");
      return;
    }
  }

  sendBLE("UNKNOWN COMMAND\n");
  Serial.println("BLE: unknown command: " + cmd);
}

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override {
    bleConnected = true;
    Serial.println("BLE: connected");
    sendBLE("CONNECTED\n");
  }
  void onDisconnect(BLEServer *s) override {
    bleConnected = false;
    Serial.println("BLE: disconnected");
    BLEDevice::startAdvertising();
  }
};

class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    static String buf;
    String rx = c->getValue().c_str();
    for (unsigned i = 0; i < rx.length(); i++) {
      char ch = rx[i];
      if (ch == '\n' || ch == '\r') { bleHandleCommand(buf); buf = ""; }
      else buf += ch;
    }
    // apps that send a command without a newline: handle it at packet end
    if (buf.length()) { bleHandleCommand(buf); buf = ""; }
  }
};

void bleInit() {
  BLEDevice::init("SpotMicro");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  BLEService *svc = server->createService(SVC_UUID);

  BLECharacteristic *rx = svc->createCharacteristic(
      RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCB());

  bleTx = svc->createCharacteristic(TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  bleTx->addDescriptor(new BLE2902());

  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("BLE ready. F B L R Q E T P C D S H Y | SPEED 0.5-6.0 | PRY,pitch,roll,yaw");
}

// ---- Main ----
void executeRobotCommand() {
  switch (robotCommand) {
    case CMD_FORWARD:      walkGait(1);        break;
    case CMD_BACKWARD:     walkGait(-1);       break;
    case CMD_LEFT:         rotateGait(false);  break;
    case CMD_RIGHT:        rotateGait(true);   break;
    case CMD_STRAFE_LEFT:  sideWalkGait(false);break;
    case CMD_STRAFE_RIGHT: sideWalkGait(true); break;
    case CMD_TROT:         trotGait();         break;
    case CMD_PACE:         paceGait();         break;
    case CMD_CRAWL:        crawlGait();        break;
    case CMD_BOUND:        boundGait();        break;
    case CMD_PRY_TEST:     testPRY(H);          break;
    case CMD_PRY_SET:      applyPRY(pryP, pryR, pryY, H); delay(20); break;
    case CMD_STOP:
    case CMD_STAND:
    default:               moveAllLegs(X, Y, H); delay(20); break;
  }
}

void ibusUpdate() {
  while (IBUS.available()) {
    uint8_t b = IBUS.read();

    if (framePos == 0) {                         // first byte = packet length
      if (b < 4 || b > 64) continue;
      frame[framePos++] = b;
      frameLength = b;
      continue;
    }
    frame[framePos++] = b;

    if (framePos == 2 && b != 0x40) {            // not a channel packet: resync
      framePos = frameLength = 0;
      continue;
    }

    if (framePos >= frameLength) {               // complete frame
      uint16_t sum = 0xFFFF;
      for (int i = 0; i < frameLength - 2; i++) sum -= frame[i];
      uint16_t rx = frame[frameLength - 2] | (frame[frameLength - 1] << 8);

      if (frame[1] == 0x40 && sum == rx) {
        int n = min((frameLength - 4) / 2, 10);
        for (int c = 0; c < n; c++)
          channels[c] = frame[2 + c * 2] | (frame[3 + c * 2] << 8);
      }
      lastFrameMs = millis();
      framePos = frameLength = 0;
    }
  }
}

bool ibusAlive() {
  ibusUpdate();
  return lastFrameMs && millis() - lastFrameMs < 500;
}

int ibusChannel(int channel) {                   // channel 1..10 -> ~1000-2000, 0 if invalid
  ibusUpdate();
  return (channel < 1 || channel > 10) ? 0 : channels[channel - 1];
}

void setup() {
  Serial.begin(115200);
  Serial.printf("Reset reason: %d\n", (int)esp_reset_reason());
  Wire.begin();
  lcd.init(); lcd.backlight();
  lcdPrint("SpotMicro", "Starting...");
  pca.begin();
  pca.setPWMFreq(50);
  moveAllLegs(X, Y, H);
  bleInit();                                   // BLE works even if the MPU is missing
  IBUS.begin(115200, SERIAL_8N1, IBUS_RX, -1);
  if (mpu.begin() != 0) {
    Serial.println("MPU6050 not found. Check wiring!");
    lcdPrint("MPU6050", "Not found!");
  } else {
    Serial.println("Calibrating MPU6050, do not move...");
    lcdPrint("Calibrating MPU", "Do not move...");
    delay(1000);
    mpu.calcOffsets();
    Serial.println("Done.");
  }
  lcdPrint("SpotMicro", "Ready");
}

float axis(int v, float maxv){
  float a=(constrain(v,1000,2000)-1500)/500.0f;
  return fabs(a)<0.08f ? 0.0f : a*maxv;        // deadband around stick center
}

void loop() {
  static int height = 150;                     // held when there is no signal
  static bool ch9Last = false;
  static unsigned long lcdT = 0;

  if (!ibusAlive()) {
    moveAllLegs(X, Y, H);
    lcdPrint("NO SIGNAL", "Waiting for RC");
    return;
  }

  int c1 = ibusChannel(1), c2 = ibusChannel(2), c3 = ibusChannel(3),
      c4 = ibusChannel(4), c5 = ibusChannel(5);

  // CH9 one-push handshake trigger
  int c9 = ibusChannel(9);

  bool ch9Pressed = (c9 >= 1500);

  if (ch9Pressed && !ch9Last && !handshakeActive) {
    robotCommand = CMD_HANDSHAKE;
  }

  ch9Last = ch9Pressed;

  // HANDSHAKE HAS MASTER PRIORITY
  if (robotCommand == CMD_HANDSHAKE) {
    if (!handshakeActive) {
      doHandshake();
    }
    return;
  }

  height = map(constrain(c3, 1000, 2000), 1000, 2000, 100, 220);

  if (millis() - lcdT > 150) {                 // limit LCD refresh to ~7 Hz
    lcdT = millis();
    char h[17]; snprintf(h, 17, "Height: %d", height);
    lcdPrint(c5 >= 1500 ? "Trot Mode" : "Posture Mode", h);
  }

  if (c5 >= 1500) {                            // gait mode
    float vx = axis(c2, 60);                   // forward / back
    float vy = axis(c1, 60);                   // strafe
    float wz = axis(c4, 40);                   // rotate
    float n = sqrt((vx/60)*(vx/60) + (vy/60)*(vy/60) + (wz/40)*(wz/40));
    if (n > 1) { vx /= n; vy /= n; wz /= n; }  // keep the mix within limits
    omniTrot(30, vx, vy, wz, height);
  } else {                                     // posture mode
    applyPRY(map(constrain(c2,1000,2000),1000,2000,-30,30),   // pitch
             map(constrain(c1,1000,2000),1000,2000,-25,25),   // roll
             map(constrain(c4,1000,2000),1000,2000,-25,25),   // yaw
             height);
  }
}
