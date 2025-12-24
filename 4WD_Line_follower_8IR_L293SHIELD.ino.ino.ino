#include <Arduino.h>
#include <AFMotor.h>

/*
  8-IR LINE FOLLOWER (NO PID) - "MAX SENSOR" STEERING
  Update requested:
    - When line is lost: STOP first, then TURN toward last sensor that saw line.

  Sensors:
    S1 = RIGHT ... S8 = LEFT
    index: 0=S1(right) ... 7=S8(left)

  Black line => higher ADC values.
*/

/* ===================== IR CONFIG ===================== */
static constexpr uint8_t IR_PINS[8] = {/* A8, A9, A10, A11, A12, A13, A14, A15 */};

// Optional IR enable pin. Set to 255 if unused.
static constexpr uint8_t PIN_IR_ENABLE = 22;
static constexpr bool    IR_ENABLE_ACTIVE_HIGH = true;

static constexpr uint32_t SAMPLE_STEP_MS = 2;
static constexpr uint8_t  FILTER_SHIFT  = 3;   // 1/8 smoothing
static constexpr uint32_t PRINT_MS      = 120;

// Adaptive "line present" by contrast (max-min)
static constexpr int CONTRAST_MIN = 12;  // tune 10..20 from your logs

/* ===================== MOTOR CONFIG ===================== */
AF_DCMotor g_m1(1, MOTOR12_1KHZ); // BR
AF_DCMotor g_m2(2, MOTOR12_1KHZ); // FR
AF_DCMotor g_m3(3, MOTOR34_1KHZ); // FL
AF_DCMotor g_m4(4, MOTOR34_1KHZ); // BL

bool g_revM1 = false; // BR
bool g_revM2 = true;  // FR
bool g_revM3 = false; // FL
bool g_revM4 = false; // BL

static int16_t SPD_FWD   = 180;
static int16_t SPD_TURN  = 190;
static int16_t SPD_INNER = 90;
static int16_t SPD_SPIN  = 150;

// Lost-line behavior
static constexpr uint16_t LOST_STOP_MS = 120; // stop briefly before turning (non-blocking)

/* ===================== STATE ===================== */
static int  rawv[8]  = {0};
static int  filt[8]  = {0};

static uint32_t lastSampleMs = 0;
static uint32_t lastPrintMs  = 0;

// last "best" sensor index that saw the line
static int8_t lastImax = -1; // -1 = unknown

// Lost-line state machine
enum LostState : uint8_t { LOST_NONE, LOST_STOPPING, LOST_TURNING };
static LostState g_lostState = LOST_NONE;
static uint32_t  g_lostT0    = 0;

/* ===================== HELPERS ===================== */
static void setIrLeds(bool on) {
  if (PIN_IR_ENABLE == 255) return;
  digitalWrite(PIN_IR_ENABLE, (on == IR_ENABLE_ACTIVE_HIGH) ? HIGH : LOW);
}

static inline void applyMotor(AF_DCMotor &m, int16_t speed, bool reverseFlag)
{
  int16_t s = speed;
  if (reverseFlag) s = (int16_t)-s;

  if (s > 255)  s = 255;
  if (s < -255) s = -255;

  if (s == 0) {
    m.setSpeed(0);
    m.run(RELEASE);
    return;
  }

  const uint8_t pwm = (uint8_t)(s > 0 ? s : -s);
  m.setSpeed(pwm);
  m.run((s > 0) ? FORWARD : BACKWARD);
}

static void setMotorSpeeds(int16_t leftMotorSpeed, int16_t rightMotorSpeed)
{
  applyMotor(g_m3, leftMotorSpeed,  g_revM3);
  applyMotor(g_m4, leftMotorSpeed,  g_revM4);
  applyMotor(g_m2, rightMotorSpeed, g_revM2);
  applyMotor(g_m1, rightMotorSpeed, g_revM1);
}

static void updateSensorsAll()
{
  uint32_t now = millis();
  if ((uint32_t)(now - lastSampleMs) < SAMPLE_STEP_MS) return;
  lastSampleMs = now;

  for (uint8_t i = 0; i < 8; i++) {
    rawv[i] = analogRead(IR_PINS[i]);
    if (filt[i] == 0) filt[i] = rawv[i];
    filt[i] = filt[i] + ((rawv[i] - filt[i]) >> FILTER_SHIFT);
  }
}

static void analyzeSensors(int &minV, int &maxV, uint8_t &imax)
{
  minV = filt[0];
  maxV = filt[0];
  imax = 0;
  for (uint8_t i = 1; i < 8; i++) {
    int v = filt[i];
    if (v < minV) minV = v;
    if (v > maxV) { maxV = v; imax = i; }
  }
}

/* ===================== CONTROL ===================== */

static void doLostBehavior(bool hasLine)
{
  // If line is back, exit lost behavior
  if (hasLine) {
    g_lostState = LOST_NONE;
    return;
  }

  uint32_t now = millis();

  if (g_lostState == LOST_NONE) {
    // Enter lost: STOP immediately
    g_lostState = LOST_STOPPING;
    g_lostT0 = now;
    setMotorSpeeds(0, 0);
    return;
  }

  if (g_lostState == LOST_STOPPING) {
    // Hold stop for LOST_STOP_MS then start turning toward lastImax
    setMotorSpeeds(0, 0);
    if ((uint32_t)(now - g_lostT0) >= LOST_STOP_MS) {
      g_lostState = LOST_TURNING;
    }
    return;
  }

  // LOST_TURNING: turn toward last sensor side until line returns
  // lastImax: 0..7 (0 right, 7 left)
  if (lastImax < 0) {
    // no history -> just stop
    setMotorSpeeds(0, 0);
    return;
  }

  if (lastImax <= 3) {
    // last seen on RIGHT -> turn RIGHT (left fwd, right back)
    setMotorSpeeds(+SPD_SPIN, -SPD_SPIN);
  } else {
    // last seen on LEFT -> turn LEFT (left back, right fwd)
    setMotorSpeeds(-SPD_SPIN, +SPD_SPIN);
  }
}

static void followLineByMax()
{
  int minV, maxV;
  uint8_t imax;
  analyzeSensors(minV, maxV, imax);

  int contrast = maxV - minV;
  bool hasLine = (contrast >= CONTRAST_MIN);

  // Update lastImax only when we are confident we have the line
  if (hasLine) lastImax = (int8_t)imax;

  // Lost behavior state machine (stop then turn toward last detection)
  if (!hasLine) {
    doLostBehavior(false);
    return;
  }

  // Line is present -> normal steering (and cancel lost mode)
  doLostBehavior(true);

  // imax: 0=S1(right) ... 7=S8(left)
  // Desired behavior:
  //  - if imax on right -> turn RIGHT (left faster, right slower)
  //  - if imax on left  -> turn LEFT  (right faster, left slower)

  if (imax <= 1) {
    // far right -> hard right
    setMotorSpeeds(+SPD_SPIN, -SPD_SPIN);
  }
  else if (imax <= 3) {
    // right -> gentle right
    setMotorSpeeds(SPD_TURN, SPD_INNER);
  }
  else if (imax == 3 || imax == 4) {
    // near center -> forward
    setMotorSpeeds(SPD_FWD, SPD_FWD);
  }
  else if (imax <= 6) {
    // left -> gentle left
    setMotorSpeeds(SPD_INNER, SPD_TURN);
  }
  else {
    // far left -> hard left
    setMotorSpeeds(-SPD_SPIN, +SPD_SPIN);
  }
}

static void debugPrint()
{
  uint32_t now = millis();
  if ((uint32_t)(now - lastPrintMs) < PRINT_MS) return;
  lastPrintMs = now;

  int minV, maxV;
  uint8_t imax;
  analyzeSensors(minV, maxV, imax);

  Serial.print("imax=");
  Serial.print(imax);
  Serial.print(" lastImax=");
  Serial.print(lastImax);
  Serial.print(" state=");
  Serial.print((int)g_lostState);

  Serial.print(" max=");
  Serial.print(maxV);
  Serial.print(" min=");
  Serial.print(minV);
  Serial.print(" contrast=");
  Serial.print(maxV - minV);

  Serial.print(" | filt:");
  for (uint8_t i = 0; i < 8; i++) { Serial.print(' '); Serial.print(filt[i]); }
  Serial.println();
}

/* ===================== ARDUINO ===================== */

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < 8; i++) pinMode(IR_PINS[i], INPUT);

  if (PIN_IR_ENABLE != 255) {
    pinMode(PIN_IR_ENABLE, OUTPUT);
    setIrLeds(true);
  }

  setMotorSpeeds(0, 0);

  Serial.println("LineFollower NO PID (MAX sensor) | lost: STOP then TURN to last sensor");
  Serial.print("CONTRAST_MIN="); Serial.println(CONTRAST_MIN);
}

void loop() {
  /* updateSensorsAll();
  followLineByMax();
  debugPrint(); */
  setMotorSpeeds(200, 200);
}
