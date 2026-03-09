/*
 * =============================================================
 * AMR Arduino Firmware for ROS2 - RMCS-2305 Driver
 * Closed-loop wheel speed control on Arduino
 * Input: cmd_vel (linear, angular) over USB serial (Pi <-> Arduino Mega)
 * Output: encoder ticks feedback over USB serial
 * =============================================================
 *
 * Receive:
 *   "C,v,w\n"
 *     v = linear velocity (m/s)
 *     w = angular velocity (rad/s)
 *
 * Send:
 *   "E,left_ticks,right_ticks\n"
 *
 * Notes:
 * - Left/Right pins are SWAPPED (logical left is old right wiring).
 * - Arduino does all wheel-speed control via feedforward + PID.
 */

// =============================================================
// ORIGINAL PIN DEFINITIONS (FROM YOUR OLD CODE)
// =============================================================
// Old "LEFT" motor pins
#define OLD_LEFT_SLEEP  2
#define OLD_LEFT_DIR    3
#define OLD_LEFT_PWM    5

// Old "RIGHT" motor pins
#define OLD_RIGHT_PWM   6
#define OLD_RIGHT_DIR   7
#define OLD_RIGHT_SLEEP 8

// Old encoder pins
#define OLD_LEFT_ENC_A   18
#define OLD_LEFT_ENC_B   22
#define OLD_RIGHT_ENC_A  19
#define OLD_RIGHT_ENC_B  23

#define STATUS_LED       13

// =============================================================
// NEW LOGICAL MAPPING (LEFT/RIGHT INTERCHANGED)
// =============================================================
// LOGICAL LEFT motor is physically wired like old RIGHT
#define LEFT_SLEEP   OLD_RIGHT_SLEEP
#define LEFT_DIR     OLD_RIGHT_DIR
#define LEFT_PWM     OLD_RIGHT_PWM

// LOGICAL RIGHT motor is physically wired like old LEFT
#define RIGHT_SLEEP  OLD_LEFT_SLEEP
#define RIGHT_DIR    OLD_LEFT_DIR
#define RIGHT_PWM    OLD_LEFT_PWM

// LOGICAL LEFT encoder is physically old RIGHT encoder
#define LEFT_ENC_A   OLD_RIGHT_ENC_A
#define LEFT_ENC_B   OLD_RIGHT_ENC_B

// LOGICAL RIGHT encoder is physically old LEFT encoder
#define RIGHT_ENC_A  OLD_LEFT_ENC_A
#define RIGHT_ENC_B  OLD_LEFT_ENC_B

// =============================================================
// ROBOT CONSTANTS
// =============================================================
const long   TICKS_PER_REV  = 2350;
const float  WHEEL_DIAM_MM  = 73.5f;
const float  PI_F           = 3.14159265f;

const float  WHEEL_CIRC_MM  = PI_F * WHEEL_DIAM_MM;                   // ~230.91 mm
const float  TICKS_PER_MM   = (float)TICKS_PER_REV / WHEEL_CIRC_MM;   // ~10.18 ticks/mm
const float  MM_PER_TICK    = WHEEL_CIRC_MM / (float)TICKS_PER_REV;   // ~0.0983 mm/tick

const float  WHEELBASE_MM   = 325.0f;   // center-to-center wheel spacing

// Max achievable wheel speed (mm/s) — set to your motor's real limit
const float  MAX_WHEEL_SPEED_MMPS = 1000.0f;

// =============================================================
// TIMING
// =============================================================
const unsigned long WATCHDOG_TIMEOUT  = 500;   // ms — stop if no cmd for this long
const unsigned long PUBLISH_INTERVAL  = 50;    // ms — encoder feedback rate
const unsigned long CONTROL_INTERVAL  = 20;    // ms — control loop rate (50 Hz)

// =============================================================
// CONTROL PARAMETERS
// =============================================================
const int PWM_MAX = 255;

// Minimum PWM that actually moves each motor (overcome static friction)
const int LEFT_PWM_MIN  = 45;
const int RIGHT_PWM_MIN = 45;

// Below this PWM magnitude, just output zero (noise rejection)
const int PWM_DEADBAND = 8;

// ---------- Feedforward ----------
// kV maps target ticks/sec -> approximate PWM.
// Calibration: command a known speed, measure steady-state PWM / measured ticks_per_sec.
// Example: if PWM 200 gives ~1100 ticks/s, then kV ~ 200/1100 ~ 0.18
float LEFT_kV  = 0.10f;
float RIGHT_kV = 0.10f;

// ---------- Per-wheel PID gains ----------
// These act on error in ticks/sec.
float LEFT_KP  = 0.12f;
float LEFT_KI  = 0.40f;
float LEFT_KD  = 0.0015f;
float RIGHT_KP = 0.12f;
float RIGHT_KI = 0.40f;
float RIGHT_KD = 0.0015f;

// Anti-windup integrator clamp (in accumulated ticks units)
const float I_CLAMP = 800.0f;

// ---------- Command shaping ----------
// Slew-rate limits for smooth start/stop (mm/s^2)
const float ACCEL_LIMIT_MMPS2 = 650.0f;
const float DECEL_LIMIT_MMPS2 = 900.0f;
const float TARGET_ZERO_EPS_MMPS = 0.8f;
const int PWM_SLEW_PER_STEP = 10;  // max PWM change every control step
const float FF_PWM_HEADROOM = 15.0f; // reserve PWM for PID/sync correction

// ---------- Straightness balancing ----------
// When w ~= 0 and robot is commanded to go straight, trim wheel setpoints using
// measured tick-rate difference so both wheels track the same speed.
const float STRAIGHT_W_THRESHOLD_RPS = 0.05f;
const float STRAIGHT_MIN_MMPS = 25.0f;
const float STRAIGHT_BAL_SPEED_KP = 0.18f;
const float STRAIGHT_BAL_SPEED_KI = 0.04f;
const float STRAIGHT_BAL_SPEED_I_CLAMP = 400.0f;
const float STRAIGHT_BAL_TICK_KP = 0.030f;      // tps correction per tick mismatch
const float STRAIGHT_BAL_TICK_KI = 0.004f;
const float STRAIGHT_BAL_TICK_I_CLAMP = 2000.0f;
const float STRAIGHT_BAL_CORR_MAX_TPS = 300.0f;

// ---------- Low-pass filter on measured speed ----------
// alpha = 0.0 -> no filtering, 1.0 -> infinite smoothing
// 0.3-0.5 is a good starting point to reject encoder noise
const float SPEED_FILTER_ALPHA = 0.3f;

// Encoder sign correction.
// If forward motion yields negative measured speed, flip to -1 for that side.
const int LEFT_ENC_SIGN  = +1;
const int RIGHT_ENC_SIGN = +1;

// =============================================================
// STATE
// =============================================================
volatile long leftEncoder  = 0;
volatile long rightEncoder = 0;

// Wheel speed targets (mm/s)
float leftTarget_mmps  = 0.0f;
float rightTarget_mmps = 0.0f;

// Slew-limited setpoints (mm/s) actually fed to PID
float leftSetpoint_mmps  = 0.0f;
float rightSetpoint_mmps = 0.0f;

// Latest cmd_vel
float cmd_v_mps = 0.0f;   // m/s
float cmd_w_rps = 0.0f;   // rad/s

unsigned long lastCommandTime = 0;
unsigned long lastPublishTime = 0;
unsigned long lastControlTime = 0;

String inputBuffer;

// PI controller state
long  leftEncPrev  = 0;
long  rightEncPrev = 0;
float leftI  = 0.0f;
float rightI = 0.0f;
float leftMeasPrev_tps = 0.0f;
float rightMeasPrev_tps = 0.0f;

// Straight-line balance controller state
bool straightModeActive = false;
long straightLeftStartEnc = 0;
long straightRightStartEnc = 0;
float straightSpeedI = 0.0f;
float straightTickI = 0.0f;

// Applied PWM state (for smooth PWM ramp)
int leftPwmApplied = 0;
int rightPwmApplied = 0;

// Filtered speed (ticks/sec)
float leftMeasFilt_tps  = 0.0f;
float rightMeasFilt_tps = 0.0f;

// =============================================================
// SETUP
// =============================================================
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(LEFT_SLEEP, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);

  pinMode(RIGHT_SLEEP, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);

  // Encoder pins
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  pinMode(STATUS_LED, OUTPUT);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  leftISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightISR, CHANGE);

  stopMotors();

  noInterrupts();
  leftEncPrev  = leftEncoder;
  rightEncPrev = rightEncoder;
  interrupts();

  lastCommandTime = millis();
  lastPublishTime = millis();
  lastControlTime = millis();

  // Boot indicator
  digitalWrite(STATUS_LED, HIGH);
  delay(250);
  digitalWrite(STATUS_LED, LOW);

  Serial.println("AMR_READY_CMDVEL");
}

// =============================================================
// LOOP
// =============================================================
void loop() {
  // --- Serial read ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }

  // --- Watchdog: stop if no command received recently ---
  if (millis() - lastCommandTime > WATCHDOG_TIMEOUT) {
    if (leftTarget_mmps != 0.0f || rightTarget_mmps != 0.0f) {
      cmd_v_mps        = 0.0f;
      cmd_w_rps        = 0.0f;
      leftTarget_mmps  = 0.0f;
      rightTarget_mmps = 0.0f;
      leftSetpoint_mmps = 0.0f;
      rightSetpoint_mmps = 0.0f;
      leftI  = 0.0f;
      rightI = 0.0f;
      leftMeasPrev_tps = 0.0f;
      rightMeasPrev_tps = 0.0f;
      leftMeasFilt_tps  = 0.0f;
      rightMeasFilt_tps = 0.0f;
      straightModeActive = false;
      straightLeftStartEnc = 0;
      straightRightStartEnc = 0;
      straightSpeedI = 0.0f;
      straightTickI = 0.0f;
      leftPwmApplied = 0;
      rightPwmApplied = 0;
      stopMotors();
    }
  }

  // --- Closed-loop speed control ---
  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_INTERVAL) {
    unsigned long actualDt_ms = now - lastControlTime;
    lastControlTime = now;
    speedControlStep(actualDt_ms);
  }

  // --- Publish encoder ticks ---
  if (millis() - lastPublishTime >= PUBLISH_INTERVAL) {
    publishEncoders();
    lastPublishTime = millis();
  }

  // --- Status LED: blink when moving ---
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  if (leftTarget_mmps != 0.0f || rightTarget_mmps != 0.0f) {
    if (millis() - lastBlink >= 120) {
      ledState = !ledState;
      digitalWrite(STATUS_LED, ledState);
      lastBlink = millis();
    }
  } else {
    digitalWrite(STATUS_LED, LOW);
  }
}

// =============================================================
// ENCODER ISRs
// =============================================================
void leftISR() {
  if (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B))
    leftEncoder--;
  else
    leftEncoder++;
}

void rightISR() {
  if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B))
    rightEncoder++;
  else
    rightEncoder--;
}

// =============================================================
// COMMAND PROCESSING
// =============================================================
// Protocol:
//   "C,v,w"   v in m/s, w in rad/s
//   "STOP"    immediate stop
//   "RESET"   zero encoders
//   "PING"    liveness check
void processCommand(String cmd) {
  cmd.trim();

  if (cmd.startsWith("C,")) {
    int c1 = cmd.indexOf(',');
    int c2 = cmd.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > c1) {
      cmd_v_mps = cmd.substring(c1 + 1, c2).toFloat();
      cmd_w_rps = cmd.substring(c2 + 1).toFloat();

      // Safety clamp on inputs
      cmd_v_mps = constrain(cmd_v_mps, -2.0f, 2.0f);
      cmd_w_rps = constrain(cmd_w_rps, -6.0f, 6.0f);

      // Differential drive kinematics: cmd_vel -> wheel speeds (mm/s)
      //   v_left  = v - w * (L / 2)
      //   v_right = v + w * (L / 2)
      float v_mmps = cmd_v_mps * 1000.0f;         // m/s -> mm/s
      float half_L = WHEELBASE_MM * 0.5f;          // 162.5 mm

      leftTarget_mmps  = v_mmps - (cmd_w_rps * half_L);
      rightTarget_mmps = v_mmps + (cmd_w_rps * half_L);

      // Clamp individual wheel speeds
      leftTarget_mmps  = constrain(leftTarget_mmps,  -MAX_WHEEL_SPEED_MMPS, MAX_WHEEL_SPEED_MMPS);
      rightTarget_mmps = constrain(rightTarget_mmps, -MAX_WHEEL_SPEED_MMPS, MAX_WHEEL_SPEED_MMPS);

      lastCommandTime = millis();
    }
  }
  else if (cmd == "STOP") {
    cmd_v_mps        = 0.0f;
    cmd_w_rps        = 0.0f;
    leftTarget_mmps  = 0.0f;
    rightTarget_mmps = 0.0f;
    leftSetpoint_mmps = 0.0f;
    rightSetpoint_mmps = 0.0f;
    leftI  = 0.0f;
    rightI = 0.0f;
    leftMeasPrev_tps = 0.0f;
    rightMeasPrev_tps = 0.0f;
    leftMeasFilt_tps  = 0.0f;
    rightMeasFilt_tps = 0.0f;
    straightModeActive = false;
    straightLeftStartEnc = 0;
    straightRightStartEnc = 0;
    straightSpeedI = 0.0f;
    straightTickI = 0.0f;
    leftPwmApplied = 0;
    rightPwmApplied = 0;
    stopMotors();
    lastCommandTime = millis();
  }
  else if (cmd == "RESET") {
    noInterrupts();
    leftEncoder  = 0;
    rightEncoder = 0;
    leftEncPrev  = 0;
    rightEncPrev = 0;
    interrupts();

    leftI  = 0.0f;
    rightI = 0.0f;
    leftMeasPrev_tps = 0.0f;
    rightMeasPrev_tps = 0.0f;
    leftMeasFilt_tps  = 0.0f;
    rightMeasFilt_tps = 0.0f;
    straightModeActive = false;
    straightLeftStartEnc = 0;
    straightRightStartEnc = 0;
    straightSpeedI = 0.0f;
    straightTickI = 0.0f;
    leftSetpoint_mmps = 0.0f;
    rightSetpoint_mmps = 0.0f;
    leftPwmApplied = 0;
    rightPwmApplied = 0;
    Serial.println("RESET_OK");
  }
  else if (cmd == "PING") {
    Serial.println("PONG");
  }
}

// =============================================================
float rampSetpoint(float current_mmps, float target_mmps, float dt) {
  float delta = target_mmps - current_mmps;
  float movingAway = (fabsf(target_mmps) > fabsf(current_mmps)) ? 1.0f : 0.0f;
  float limit = (movingAway > 0.5f) ? ACCEL_LIMIT_MMPS2 : DECEL_LIMIT_MMPS2;
  float maxStep = limit * dt;
  delta = constrain(delta, -maxStep, maxStep);
  float next = current_mmps + delta;
  if (fabsf(target_mmps) < TARGET_ZERO_EPS_MMPS && fabsf(next) < TARGET_ZERO_EPS_MMPS) {
    return 0.0f;
  }
  return next;
}

int applyPwmSlew(int desired, int current) {
  int delta = desired - current;
  if (delta > PWM_SLEW_PER_STEP) delta = PWM_SLEW_PER_STEP;
  else if (delta < -PWM_SLEW_PER_STEP) delta = -PWM_SLEW_PER_STEP;
  return current + delta;
}

// =============================================================
// SPEED CONTROL — Feedforward + PID with low-pass filtered feedback
// =============================================================
void speedControlStep(unsigned long actualDt_ms) {
  // Use actual elapsed time for accurate velocity measurement
  float dt = (float)actualDt_ms / 1000.0f;
  if (dt < 0.001f) dt = 0.001f;  // safety floor

  // --- Read encoders atomically ---
  long lEnc, rEnc;
  noInterrupts();
  lEnc = leftEncoder;
  rEnc = rightEncoder;
  interrupts();

  // --- Compute raw delta ticks ---
  long dl_raw = lEnc - leftEncPrev;
  long dr_raw = rEnc - rightEncPrev;
  leftEncPrev  = lEnc;
  rightEncPrev = rEnc;

  // --- Measured speed in ticks/sec (with sign correction) ---
  float leftMeasRaw_tps  = (float)(dl_raw * LEFT_ENC_SIGN)  / dt;
  float rightMeasRaw_tps = (float)(dr_raw * RIGHT_ENC_SIGN) / dt;

  // --- Low-pass filter to smooth encoder noise ---
  // filtered = alpha * old + (1 - alpha) * new
  leftMeasFilt_tps  = SPEED_FILTER_ALPHA * leftMeasFilt_tps
                    + (1.0f - SPEED_FILTER_ALPHA) * leftMeasRaw_tps;
  rightMeasFilt_tps = SPEED_FILTER_ALPHA * rightMeasFilt_tps
                    + (1.0f - SPEED_FILTER_ALPHA) * rightMeasRaw_tps;

  // --- Smooth start/stop using slew-limited setpoints ---
  leftSetpoint_mmps = rampSetpoint(leftSetpoint_mmps, leftTarget_mmps, dt);
  rightSetpoint_mmps = rampSetpoint(rightSetpoint_mmps, rightTarget_mmps, dt);

  // --- Convert targets from mm/s -> ticks/sec ---
  float leftTarget_tps  = leftSetpoint_mmps  * TICKS_PER_MM;
  float rightTarget_tps = rightSetpoint_mmps * TICKS_PER_MM;

  // Keep commands inside controllable range so PID can still correct mismatch.
  // If feedforward is already saturated, wheel synchronization cannot work.
  float leftKvAbs = fabsf(LEFT_kV);
  float rightKvAbs = fabsf(RIGHT_kV);
  if (leftKvAbs < 0.01f) leftKvAbs = 0.01f;
  if (rightKvAbs < 0.01f) rightKvAbs = 0.01f;

  float leftMaxCtrlPwm = (float)PWM_MAX - (float)LEFT_PWM_MIN - FF_PWM_HEADROOM;
  float rightMaxCtrlPwm = (float)PWM_MAX - (float)RIGHT_PWM_MIN - FF_PWM_HEADROOM;
  if (leftMaxCtrlPwm < 20.0f) leftMaxCtrlPwm = 20.0f;
  if (rightMaxCtrlPwm < 20.0f) rightMaxCtrlPwm = 20.0f;

  float leftMaxCtrl_tps = leftMaxCtrlPwm / leftKvAbs;
  float rightMaxCtrl_tps = rightMaxCtrlPwm / rightKvAbs;
  float maxCtrl_tps = (leftMaxCtrl_tps < rightMaxCtrl_tps) ? leftMaxCtrl_tps : rightMaxCtrl_tps;

  float maxReq_tps = fabsf(leftTarget_tps);
  if (fabsf(rightTarget_tps) > maxReq_tps) maxReq_tps = fabsf(rightTarget_tps);
  if (maxReq_tps > maxCtrl_tps) {
    float s = maxCtrl_tps / maxReq_tps;
    leftTarget_tps *= s;
    rightTarget_tps *= s;
  }

  // --- Straight-line balancing (enforce equal wheel speed when w ~ 0) ---
  bool straightCmd = (fabsf(cmd_w_rps) < STRAIGHT_W_THRESHOLD_RPS) &&
                     (fabsf(leftSetpoint_mmps) > STRAIGHT_MIN_MMPS) &&
                     (fabsf(rightSetpoint_mmps) > STRAIGHT_MIN_MMPS) &&
                     ((leftSetpoint_mmps * rightSetpoint_mmps) > 0.0f);
  if (straightCmd) {
    if (!straightModeActive) {
      straightModeActive = true;
      straightLeftStartEnc = lEnc;
      straightRightStartEnc = rEnc;
      straightSpeedI = 0.0f;
      straightTickI = 0.0f;
    }

    // Use both instantaneous speed mismatch and cumulative tick mismatch.
    float speedErr_tps = leftMeasFilt_tps - rightMeasFilt_tps;  // +ve => left faster
    long leftTicksFromStart = (lEnc - straightLeftStartEnc) * LEFT_ENC_SIGN;
    long rightTicksFromStart = (rEnc - straightRightStartEnc) * RIGHT_ENC_SIGN;
    float tickErr = (float)(leftTicksFromStart - rightTicksFromStart); // +ve => left traveled more

    straightSpeedI += speedErr_tps * dt;
    straightSpeedI = constrain(straightSpeedI, -STRAIGHT_BAL_SPEED_I_CLAMP, STRAIGHT_BAL_SPEED_I_CLAMP);
    straightTickI += tickErr * dt;
    straightTickI = constrain(straightTickI, -STRAIGHT_BAL_TICK_I_CLAMP, STRAIGHT_BAL_TICK_I_CLAMP);

    float corr_tps =
      STRAIGHT_BAL_SPEED_KP * speedErr_tps +
      STRAIGHT_BAL_SPEED_KI * straightSpeedI +
      STRAIGHT_BAL_TICK_KP * tickErr +
      STRAIGHT_BAL_TICK_KI * straightTickI;
    corr_tps = constrain(corr_tps, -STRAIGHT_BAL_CORR_MAX_TPS, STRAIGHT_BAL_CORR_MAX_TPS);

    leftTarget_tps -= corr_tps;
    rightTarget_tps += corr_tps;
  } else {
    straightModeActive = false;
    straightSpeedI = 0.0f;
    straightTickI = 0.0f;
  }

  int pwmL_desired = 0;
  int pwmR_desired = 0;

  // --- LEFT wheel ---
  if (fabsf(leftSetpoint_mmps) < TARGET_ZERO_EPS_MMPS && fabsf(leftMeasFilt_tps) < 5.0f) {
    leftI = 0.0f;
    leftMeasPrev_tps = 0.0f;
    pwmL_desired = 0;
  } else {
    float eL = leftTarget_tps - leftMeasFilt_tps;
    float dMeasL = (leftMeasFilt_tps - leftMeasPrev_tps) / dt;
    leftMeasPrev_tps = leftMeasFilt_tps;

    // Conditional integration anti-windup
    float ffL = LEFT_kV * leftTarget_tps;
    float uNoI_L = ffL + LEFT_KP * eL - LEFT_KD * dMeasL;
    float iCandL = constrain(leftI + eL * dt, -I_CLAMP, I_CLAMP);
    float uCandL = uNoI_L + LEFT_KI * iCandL;
    if (!((uCandL > PWM_MAX && eL > 0.0f) || (uCandL < -PWM_MAX && eL < 0.0f))) {
      leftI = iCandL;
    }

    // Feedforward + PID output
    float uL = ffL + LEFT_KP * eL + LEFT_KI * leftI - LEFT_KD * dMeasL;
    pwmL_desired = shapePwm((int)lroundf(uL), LEFT_PWM_MIN);
  }

  // --- RIGHT wheel ---
  if (fabsf(rightSetpoint_mmps) < TARGET_ZERO_EPS_MMPS && fabsf(rightMeasFilt_tps) < 5.0f) {
    rightI = 0.0f;
    rightMeasPrev_tps = 0.0f;
    pwmR_desired = 0;
  } else {
    float eR = rightTarget_tps - rightMeasFilt_tps;
    float dMeasR = (rightMeasFilt_tps - rightMeasPrev_tps) / dt;
    rightMeasPrev_tps = rightMeasFilt_tps;

    float ffR = RIGHT_kV * rightTarget_tps;
    float uNoI_R = ffR + RIGHT_KP * eR - RIGHT_KD * dMeasR;
    float iCandR = constrain(rightI + eR * dt, -I_CLAMP, I_CLAMP);
    float uCandR = uNoI_R + RIGHT_KI * iCandR;
    if (!((uCandR > PWM_MAX && eR > 0.0f) || (uCandR < -PWM_MAX && eR < 0.0f))) {
      rightI = iCandR;
    }

    float uR = ffR + RIGHT_KP * eR + RIGHT_KI * rightI - RIGHT_KD * dMeasR;
    pwmR_desired = shapePwm((int)lroundf(uR), RIGHT_PWM_MIN);
  }

  // Smooth PWM transitions to avoid jerk at start/stop and reversals.
  leftPwmApplied = applyPwmSlew(pwmL_desired, leftPwmApplied);
  rightPwmApplied = applyPwmSlew(pwmR_desired, rightPwmApplied);
  setMotors(leftPwmApplied, rightPwmApplied);
}

// =============================================================
// PWM SHAPING — deadband + minimum PWM enforcement
// =============================================================
int shapePwm(int pwm, int pwm_min) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm == 0) return 0;
  if (abs(pwm) < PWM_DEADBAND) return 0;

  // Enforce minimum PWM to overcome static friction
  if (abs(pwm) < pwm_min) {
    pwm = (pwm > 0) ? pwm_min : -pwm_min;
  }
  return pwm;
}

// =============================================================
// MOTOR OUTPUT — individual motor helpers
// =============================================================
void setLeftMotor(int pwm) {
  if (pwm == 0) {
    digitalWrite(LEFT_SLEEP, HIGH);
    analogWrite(LEFT_PWM, 0);
  } else {
    digitalWrite(LEFT_SLEEP, LOW);
    digitalWrite(LEFT_DIR, pwm > 0 ? HIGH : LOW);
    analogWrite(LEFT_PWM, abs(pwm));
  }
}

void setRightMotor(int pwm) {
  if (pwm == 0) {
    digitalWrite(RIGHT_SLEEP, HIGH);
    analogWrite(RIGHT_PWM, 0);
  } else {
    digitalWrite(RIGHT_SLEEP, LOW);
    digitalWrite(RIGHT_DIR, pwm > 0 ? HIGH : LOW);
    analogWrite(RIGHT_PWM, abs(pwm));
  }
}

void setMotors(int left_pwm, int right_pwm) {
  setLeftMotor(left_pwm);
  setRightMotor(right_pwm);
}

void stopMotors() {
  digitalWrite(LEFT_SLEEP, HIGH);
  digitalWrite(RIGHT_SLEEP, HIGH);
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
}

// =============================================================
// FEEDBACK — publish raw encoder ticks
// =============================================================
void publishEncoders() {
  long lEnc, rEnc;
  noInterrupts();
  lEnc = leftEncoder;
  rEnc = rightEncoder;
  interrupts();

  Serial.print("E,");
  Serial.print(lEnc);
  Serial.print(",");
  Serial.println(rEnc);
}
