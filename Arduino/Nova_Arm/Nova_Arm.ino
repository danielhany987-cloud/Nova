/*
 * ============================================================
 *  NOVA — Robot Arm Motion Controller v2.0
 *  ESP32 + 4 Servos + Analog Joystick
 *
 *  Architecture:
 *    Acceleration-limited velocity control
 *    + light EMA smoothing for servo micro-jitter removal
 *
 *  Pipeline (every 20ms):
 *    1. Read joystick (8-sample average, normalize, deadzone)
 *    2. Process button (claw toggle, debounced)
 *    3. Update base velocity & target (FB axis)
 *    4. Update arm LR — unified shoulder + elbow (LR axis)
 *       - Shoulder velocity & target
 *       - Measure actual shoulder travel
 *       - Enable elbow after ~20° actual rotation
 *       - Elbow velocity & target
 *    5. Smooth & write servos (light EMA + safeWrite)
 *    6. Debug output
 *
 *  Both joystick axes are completely independent.
 *  No dominant axis lock. No axis zeroing.
 *  Joystick release holds position — nothing returns to 90°.
 * ============================================================
 */

#include <ESP32Servo.h>

/* ---- Debug ------------------------------------------------ */
#define DEBUG_MODE 1

/* ---- GPIO Pins (VERIFIED — do not change) ----------------- */
#define PIN_BASE_SERVO     26
#define PIN_SHOULDER_SERVO 18
#define PIN_ELBOW_SERVO    19
#define PIN_CLAW_SERVO     14

#define PIN_JOY_A          36   /* X axis (VP) */
#define PIN_JOY_B          39   /* Y axis (VN) */
#define PIN_JOY_BTN        22

/* ---- Servo Ranges ----------------------------------------- */
#define BASE_MIN           0
#define BASE_MAX           180
#define BASE_CENTER        90

#define SHOULDER_MIN       20
#define SHOULDER_MAX       160
#define SHOULDER_CENTER    90

#define ELBOW_MIN          20
#define ELBOW_MAX          160
#define ELBOW_CENTER       90

#define CLAW_OPEN_ANGLE    90
#define CLAW_CLOSE_ANGLE   74

/* ---- Timing ----------------------------------------------- */
#define LOOP_MS            20       /* 50 Hz control loop      */
#define DT                 0.02f    /* Loop period in seconds   */
#define DEBUG_INTERVAL_MS  150      /* ms between debug prints  */
#define DEBOUNCE_MS        300      /* Button debounce window   */

/* ---- Joystick --------------------------------------------- */
#define DEADZONE           0.12f    /* Normalized deadzone      */
#define JOY_SAMPLES        8       /* ADC oversampling count    */

/* ---- Motion: Velocity Limits (degrees / second) ----------- */
/*  ~17% increase for snappier response (was 42/45/28)         */
#define BASE_MAX_VEL       49.0f
#define SHOULDER_MAX_VEL   53.0f
#define ELBOW_MAX_VEL      33.0f   /* Slower than shoulder     */

/* ---- Motion: Acceleration Limits (degrees / second²) ------ */
/*  Controls how quickly velocity ramps up/down.               */
/*  250 deg/s² → 0 to max in ~180ms (smooth ramp, no jerk)    */
#define BASE_ACCEL         250.0f
#define SHOULDER_ACCEL     250.0f
#define ELBOW_ACCEL        200.0f

/* ---- Motion: EMA Smoothing -------------------------------- */
/*  Light filter for servo micro-jitter removal.               */
/*  The acceleration limiter handles macro smoothness.          */
/*  α = 0.45 gives ~90ms convergence — slightly faster tracking. */
#define SMOOTH_ALPHA       0.45f

/* ---- Elbow Staging ---------------------------------------- */
/*  Elbow waits until shoulder has physically rotated ~20°      */
#define ELBOW_DELAY_DEG    20.0f

/* ============================================================
 *  GLOBALS
 * ============================================================ */

/* Servo objects */
Servo sBase, sShoulder, sElbow, sClaw;

/* Joystick ADC center calibration */
int centerA = 2048;
int centerB = 2048;

/* Joystick normalized inputs [-1.0 .. +1.0] after deadzone */
float joyFB = 0.0f;    /* Forward / Backward axis */
float joyLR = 0.0f;    /* Left / Right axis       */

/* Velocities (degrees per second) */
float velBase     = 0.0f;
float velShoulder = 0.0f;
float velElbow    = 0.0f;

/* Targets — commanded positions, integrated from velocity */
float tgtBase     = (float)BASE_CENTER;
float tgtShoulder = (float)SHOULDER_CENTER;
float tgtElbow    = (float)ELBOW_CENTER;

/* Current — smoothed output positions (what the servos receive) */
float curBase     = (float)BASE_CENTER;
float curShoulder = (float)SHOULDER_CENTER;
float curElbow    = (float)ELBOW_CENTER;
float curClaw     = (float)CLAW_OPEN_ANGLE;

/* Elbow staging state */
float shoulderAnchor = (float)SHOULDER_CENTER;
bool  elbowEnabled   = false;
int   prevActiveDir  = 0;   /* Last nonzero LR direction (persists through release) */

/* Claw state */
bool clawClosed = false;

/* Button debouncing */
bool          lastBtnState = HIGH;
unsigned long lastBtnTime  = 0;

/* Servo write cache — avoid redundant servo.write() calls */
int lastWrBase     = -1;
int lastWrShoulder = -1;
int lastWrElbow    = -1;
int lastWrClaw     = -1;

/* Loop timing */
unsigned long lastLoopTime  = 0;
unsigned long lastDebugTime = 0;

/* ============================================================
 *  HELPER FUNCTIONS
 * ============================================================ */

/*
 * Apply deadzone with linear remapping.
 * Inside deadzone → 0.
 * Outside → smoothly scaled from 0 to ±1.
 */
float applyDeadzone(float raw) {
  float mag = fabs(raw);
  if (mag < DEADZONE) return 0.0f;
  float remapped = (mag - DEADZONE) / (1.0f - DEADZONE);
  return (raw > 0.0f) ? remapped : -remapped;
}

/*
 * Clamp float to [lo, hi].
 */
float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/*
 * Ramp current velocity toward desired velocity at a bounded rate.
 *
 * This is the core of the acceleration limiter:
 *   - If the difference is small enough, snap to desired (prevents creep)
 *   - Otherwise, move toward desired by at most (accel * dt) per call
 *   - Works correctly in both directions and across zero crossings
 *
 * Produces trapezoidal velocity profiles:
 *   smooth start → constant cruise → smooth stop
 */
float rampVelocity(float currentVel, float desiredVel, float accel, float dt) {
  float diff = desiredVel - currentVel;
  float maxChange = accel * dt;
  if (fabs(diff) <= maxChange) {
    return desiredVel;
  }
  return currentVel + ((diff > 0.0f) ? maxChange : -maxChange);
}

/*
 * Write a degree value to a servo, but only if the integer
 * angle has actually changed. Avoids redundant I²C / PWM writes
 * that can cause micro-glitches on some servo controllers.
 */
void safeWrite(Servo &s, float deg, int &lastWr) {
  int d = (int)(deg + 0.5f);
  if (d != lastWr) {
    s.write(d);
    lastWr = d;
  }
}

/* ============================================================
 *  PIPELINE STAGE 1: Read Joystick
 *
 *  - 8-sample oversampling for noise rejection
 *  - Normalize to [-1, +1] relative to calibrated center
 *  - Apply deadzone with linear remapping
 *
 *  joyFB and joyLR are completely independent outputs.
 * ============================================================ */
void readJoystick() {
  long sumA = 0, sumB = 0;
  for (int i = 0; i < JOY_SAMPLES; i++) {
    sumA += analogRead(PIN_JOY_A);
    sumB += analogRead(PIN_JOY_B);
  }

  float rawA = ((float)(sumA / JOY_SAMPLES) - (float)centerA) / 2048.0f;
  float rawB = ((float)(sumB / JOY_SAMPLES) - (float)centerB) / 2048.0f;

  joyFB = applyDeadzone(clampf(rawA, -1.0f, 1.0f));
  joyLR = applyDeadzone(clampf(rawB, -1.0f, 1.0f));
}

/* ============================================================
 *  PIPELINE STAGE 2: Process Button (Claw Toggle)
 *
 *  Debounced toggle on falling edge (press).
 *  Completely independent of joystick state.
 * ============================================================ */
void processButton() {
  bool btnState = digitalRead(PIN_JOY_BTN);
  unsigned long now = millis();

  if (btnState == LOW && lastBtnState == HIGH && (now - lastBtnTime > DEBOUNCE_MS)) {
    clawClosed = !clawClosed;
    lastBtnTime = now;
  }
  lastBtnState = btnState;
}

/* ============================================================
 *  PIPELINE STAGE 3: Update Base (Forward / Backward)
 *
 *  The base is entirely independent.
 *  FB joystick axis → base velocity → base target.
 *
 *  - Joystick sets desired velocity (proportional to deflection)
 *  - Acceleration limiter ramps actual velocity smoothly
 *  - Target integrates velocity
 *  - On joystick release, desired velocity = 0, velocity decelerates
 *  - Target freezes at final position
 * ============================================================ */
void updateBase() {
  float desiredVel = joyFB * BASE_MAX_VEL;

  velBase = rampVelocity(velBase, desiredVel, BASE_ACCEL, DT);

  tgtBase += velBase * DT;
  tgtBase = clampf(tgtBase, (float)BASE_MIN, (float)BASE_MAX);
}

/* ============================================================
 *  PIPELINE STAGE 4: Update Arm LR (Shoulder + Elbow Unified)
 *
 *  This is the core arm controller. Shoulder and elbow are
 *  treated as ONE mechanical system with staged engagement:
 *
 *    1. LR joystick → shoulder starts moving immediately
 *    2. Measure actual shoulder rotation (curShoulder - anchor)
 *    3. After ~20° actual rotation → enable elbow
 *    4. Elbow then follows at its own (slower) speed
 *
 *  Direction reversal:
 *    - Detected when LR sign changes between two nonzero values
 *    - Resets anchor to current actual shoulder position
 *    - Disables elbow (must re-earn 20° travel)
 *
 *  Joystick release:
 *    - Velocities decelerate to 0 naturally
 *    - elbowEnabled state is PRESERVED (not reset)
 *    - Resuming in the same direction keeps elbow active
 *    - Resuming in the opposite direction triggers reversal logic
 * ============================================================ */
void updateArmLR() {
  /* ---- Direction tracking ---- */
  int currentDir = 0;
  if (joyLR > 0.0f) currentDir =  1;
  if (joyLR < 0.0f) currentDir = -1;

  /*
   * Detect direction REVERSAL:
   * Only between two nonzero directions.
   * Joystick passing through the deadzone does NOT count
   * as a reversal — prevActiveDir persists through releases.
   */
  if (currentDir != 0) {
    if (prevActiveDir != 0 && currentDir != prevActiveDir) {
      /* True direction reversal — reset elbow staging */
      shoulderAnchor = curShoulder;   /* Anchor at ACTUAL position */
      elbowEnabled   = false;
    }
    prevActiveDir = currentDir;
  }
  /* On release (currentDir == 0): prevActiveDir unchanged, elbowEnabled unchanged */

  /* ---- Shoulder velocity & target ---- */
  float shoulderDesiredVel = joyLR * SHOULDER_MAX_VEL;
  velShoulder = rampVelocity(velShoulder, shoulderDesiredVel, SHOULDER_ACCEL, DT);

  tgtShoulder += velShoulder * DT;
  tgtShoulder = clampf(tgtShoulder, (float)SHOULDER_MIN, (float)SHOULDER_MAX);

  /* ---- Elbow staging check ---- */
  /*
   * Use curShoulder (the smoothed/actual position) to measure travel,
   * NOT tgtShoulder. This ensures the elbow only engages after the
   * shoulder has PHYSICALLY rotated ~20°.
   *
   * Only check while the joystick is active (currentDir != 0).
   * If the joystick is released, travel measurement pauses —
   * it resumes when the user pushes again in the same direction.
   */
  if (!elbowEnabled && currentDir != 0) {
    float travel = fabs(curShoulder - shoulderAnchor);
    if (travel >= ELBOW_DELAY_DEG) {
      elbowEnabled = true;
    }
  }

  /* ---- Elbow velocity & target ---- */
  float elbowDesiredVel = 0.0f;
  if (elbowEnabled) {
    /* Elbow follows the same joystick axis but at its own speed */
    elbowDesiredVel = joyLR * ELBOW_MAX_VEL;
  }
  /* When not enabled: desired = 0 → velocity decelerates smoothly to stop */

  velElbow = rampVelocity(velElbow, elbowDesiredVel, ELBOW_ACCEL, DT);

  tgtElbow += velElbow * DT;
  tgtElbow = clampf(tgtElbow, (float)ELBOW_MIN, (float)ELBOW_MAX);
}

/* ============================================================
 *  PIPELINE STAGE 5: Smooth & Write Servos
 *
 *  Apply light EMA smoothing to remove servo micro-jitter on
 *  the heavy arm, then write to hardware.
 *
 *  The acceleration limiter in stages 3-4 handles macro smoothness
 *  (no jerk on start/stop/reversal). This EMA handles the last-mile
 *  micro-stepping artifacts that can cause audible servo chatter
 *  on high-torque servos.
 *
 *  α = 0.40 → time constant ~50ms → 95% convergence in ~150ms
 *  This is fast enough to feel responsive, slow enough to
 *  eliminate single-degree step noise.
 * ============================================================ */
void writeServos() {
  float tgtClaw = clawClosed ? (float)CLAW_CLOSE_ANGLE : (float)CLAW_OPEN_ANGLE;

  /* Light EMA smoothing */
  curBase     += SMOOTH_ALPHA * (tgtBase     - curBase);
  curShoulder += SMOOTH_ALPHA * (tgtShoulder - curShoulder);
  curElbow    += SMOOTH_ALPHA * (tgtElbow    - curElbow);
  curClaw     += SMOOTH_ALPHA * (tgtClaw     - curClaw);

  /* Write to servos (only if integer angle changed) */
  safeWrite(sBase,     curBase,     lastWrBase);
  safeWrite(sShoulder, curShoulder, lastWrShoulder);
  safeWrite(sElbow,    curElbow,    lastWrElbow);
  safeWrite(sClaw,     curClaw,     lastWrClaw);
}

/* ============================================================
 *  PIPELINE STAGE 6: Debug Output
 *
 *  Prints all relevant state for tuning and diagnostics.
 *  Rate-limited to avoid flooding the serial monitor.
 * ============================================================ */
void printDebug() {
  unsigned long now = millis();
  if (now - lastDebugTime < DEBUG_INTERVAL_MS) return;
  lastDebugTime = now;

  Serial.print("Joy FB:");    Serial.print(joyFB, 2);
  Serial.print("  LR:");      Serial.print(joyLR, 2);

  Serial.print("  | Vel B:");  Serial.print(velBase, 1);
  Serial.print(" S:");         Serial.print(velShoulder, 1);
  Serial.print(" E:");         Serial.print(velElbow, 1);

  Serial.print("  | Tgt B:");  Serial.print(tgtBase, 1);
  Serial.print(" S:");         Serial.print(tgtShoulder, 1);
  Serial.print(" E:");         Serial.print(tgtElbow, 1);

  Serial.print("  | Cur B:");  Serial.print(curBase, 1);
  Serial.print(" S:");         Serial.print(curShoulder, 1);
  Serial.print(" E:");         Serial.print(curElbow, 1);

  Serial.print("  | ShldTravel:");  Serial.print(fabs(curShoulder - shoulderAnchor), 1);
  Serial.print("  Anchor:");        Serial.print(shoulderAnchor, 1);
  Serial.print("  ElbEn:");         Serial.print(elbowEnabled ? "Y" : "N");
  Serial.print("  Dir:");           Serial.println(prevActiveDir);
}

/* ============================================================
 *  SETUP
 * ============================================================ */
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("===================================");
  Serial.println(" NOVA Arm — Motion Controller v2.0");
  Serial.println("===================================");

  /* Joystick pins */
  pinMode(PIN_JOY_A,   INPUT);
  pinMode(PIN_JOY_B,   INPUT);
  pinMode(PIN_JOY_BTN, INPUT_PULLUP);

  /* Joystick center calibration — 64 samples, ~192ms settle time */
  Serial.print("Calibrating joystick center... ");
  long sumA = 0, sumB = 0;
  for (int i = 0; i < 64; i++) {
    sumA += analogRead(PIN_JOY_A);
    sumB += analogRead(PIN_JOY_B);
    delay(3);
  }
  centerA = (int)(sumA / 64);
  centerB = (int)(sumB / 64);
  Serial.print("A=");  Serial.print(centerA);
  Serial.print("  B="); Serial.println(centerB);

  /* Attach servos with 500–2500µs pulse range */
  sBase.attach(PIN_BASE_SERVO,         500, 2500);
  sShoulder.attach(PIN_SHOULDER_SERVO, 500, 2500);
  sElbow.attach(PIN_ELBOW_SERVO,       500, 2500);
  sClaw.attach(PIN_CLAW_SERVO,         500, 2500);

  /* Move to center positions */
  sBase.write(BASE_CENTER);
  sShoulder.write(SHOULDER_CENTER);
  sElbow.write(ELBOW_CENTER);
  sClaw.write(CLAW_OPEN_ANGLE);

  lastWrBase     = BASE_CENTER;
  lastWrShoulder = SHOULDER_CENTER;
  lastWrElbow    = ELBOW_CENTER;
  lastWrClaw     = CLAW_OPEN_ANGLE;

  /* Allow servos to physically reach center before starting the loop */
  delay(500);

  Serial.println("Ready.");
  Serial.println();

  lastLoopTime  = millis();
  lastDebugTime = millis();
}

/* ============================================================
 *  MAIN LOOP — 50 Hz
 * ============================================================ */
void loop() {
  unsigned long now = millis();
  if (now - lastLoopTime < LOOP_MS) return;
  lastLoopTime = now;

  /* Pipeline — each stage is independent and ordered */
  readJoystick();       /* Stage 1: Inputs           */
  processButton();      /* Stage 2: Claw toggle      */
  updateBase();         /* Stage 3: FB → Base        */
  updateArmLR();        /* Stage 4: LR → Shoulder+Elbow */
  writeServos();        /* Stage 5: Smooth & output  */

#if DEBUG_MODE
  printDebug();         /* Stage 6: Diagnostics      */
#endif
}
