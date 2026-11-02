#include "../Config.h"
#include "Navigator.h"
#ifdef OTV_HOST_TEST
// [HOST TEST] Explicit fake adapters only in test/test_controller.cpp.
#include "../test/stubs/Arduino.h"
#include "../test/stubs/Enes100.h"
#else
#include <Arduino.h>
#include <Enes100.h>
#endif
#include <math.h>
#include <stdio.h>
#include <string.h>

// [INTEGRATION / 整合批注] One setup/loop for ArUco, sensors, motor and mission
// tests. Original teammates' files remain at the repository root as reference
// examples. BOOT NEVER STARTS MOTION. Use the serial menu (9600 baud) to select
// a demo.
enum class Mode {
  IDLE,
  DIAGNOSTICS,
  WIRELESS,
  FORWARD,
  TURN,
  NAV_CLEAR,
  NAV_AVOID,
  PUMP
};
enum class Phase { PLAN, SCAN, DRIVE };
Mode mode = Mode::IDLE;
Phase phase = Phase::PLAN;
OTV::Navigator navigator;
OTV::Point target = {0, 0}, scanOrigin = {0, 0}, forwardOrigin = {0, 0};
unsigned long modeStarted = 0, lastLog = 0, lastPing = 0, pumpStarted = 0,
              turnPauseUntil = 0;
float scanLast = 0, scanTotal = 0, turnBase = 0;
int stage = 0, turnCount = 0;
bool pumpOn = false;

void stopMotors() {
  analogWrite(MOTOR_ENA_PIN, 0);
  analogWrite(MOTOR_ENB_PIN, 0);
}
uint8_t pumpPin() { return PUMP_USE_RELAY ? PUMP_RELAY_PIN : WATER_PUMP_PIN; }
void stopPump() {
  if (PUMP_WIRING_CONFIRMED)
    digitalWrite(pumpPin(), PUMP_ACTIVE_HIGH ? LOW : HIGH);
  pumpOn = false;
}
void finish(const char *message) {
  stopMotors();
  stopPump();
  mode = Mode::IDLE;
  Serial.println(message);
}
void fault(const char *message) {
  Serial.print("FAULT: ");
  finish(message);
}
void help() {
  Serial.println("\nOctonauts UNO R4 WiFi -- select one demonstration:");
  Serial.println(
      "d diagnostics | w wireless receive | x labelled transmit test");
  Serial.println(
      "f forward to pre-limbo point | t three successive 90-degree turns");
  Serial.println(
      "n navigation NO obstacles | a navigation WITH obstacle avoidance");
  Serial.println("k <known mm> obstacle calibration (10 samples), e.g. k 200");
  Serial.println("m water depth samples/report | c raw color/LDR calibration");
  Serial.println("p bounded pump test | q STOP | h help");
  Serial.println("All movement requires confirmed motor wiring and measured "
                 "body geometry.");
}

// [MOTORS / 电机] Standard L298N backend; missing IN1/IN2 deliberately stay
// unassigned.
bool motorReady() {
  if (!MOTOR_WIRING_CONFIRMED || MOTOR_IN1_PIN < 0 || MOTOR_IN2_PIN < 0) {
    Serial.println("Motor test refused: confirm IN1/IN2, enable wiring and "
                   "wheel polarity in Config.h.");
    return false;
  }
  const int pins[] = {MOTOR_IN1_PIN, MOTOR_IN2_PIN, MOTOR_IN3_PIN,
                      MOTOR_IN4_PIN, MOTOR_ENA_PIN, MOTOR_ENB_PIN};
  const int reserved[] = {A0, A1, A2, A3, A4, 2, 4, 7, 8, 10, 11, 12, 13};
  for (unsigned i = 0; i < 6; ++i) {
    if (pins[i] < 0 || pins[i] >= NUM_DIGITAL_PINS)
      return false;
    for (unsigned j = i + 1; j < 6; ++j)
      if (pins[i] == pins[j])
        return false;
    for (int used : reserved)
      if (pins[i] == used) {
        Serial.println("Motor pin conflicts with sensor/pump pin");
        return false;
      }
  }
  if (LEFT_FORWARD_SIGN * LEFT_FORWARD_SIGN != 1 ||
      RIGHT_FORWARD_SIGN * RIGHT_FORWARD_SIGN != 1)
    return false;
  return true;
}
void channel(int a, int b, int enable, int pwm) {
  analogWrite(enable, 0); // Disable before reversing direction.
  digitalWrite(a, pwm >= 0 ? HIGH : LOW);
  digitalWrite(b, pwm >= 0 ? LOW : HIGH);
  analogWrite(enable, constrain(abs(pwm), 0, 255));
}
void pulseWheels(int left, int right) {
  // [BOUNDED MOTION] Motors stop before networking, printing or path planning.
  // Enable-channel jumpers must be removed for the Arduino to control ENA/ENB.
  left *= LEFT_FORWARD_SIGN;
  right *= RIGHT_FORWARD_SIGN;
  int a = CHANNEL_A_IS_LEFT ? left : right,
      b = CHANNEL_A_IS_LEFT ? right : left;
  channel(MOTOR_IN1_PIN, MOTOR_IN2_PIN, MOTOR_ENA_PIN, a);
  channel(MOTOR_IN3_PIN, MOTOR_IN4_PIN, MOTOR_ENB_PIN, b);
  delay(MOTOR_PULSE_MS);
  stopMotors();
}

bool readPose(OTV::Pose &p) {
  stopMotors();
  if (!Enes100.isConnected())
    return false;
  p.x = Enes100.getX();
  p.y = Enes100.getY();
  p.theta = OTV::wrap(Enes100.getTheta() + MARKER_HEADING_OFFSET_RAD);
  return Enes100.isVisible() && isfinite(p.x) && isfinite(p.y) &&
         isfinite(p.theta) && p.x >= 0 && p.x <= ARENA_WIDTH_M && p.y >= 0 &&
         p.y <= ARENA_HEIGHT_M;
}
bool readDistanceMm(uint8_t trig, uint8_t echo, float &mm,
                    bool calibrated = true) {
  // [REAL SENSOR] Zero duration means invalid. It NEVER means a clear path.
  unsigned long since = millis() - lastPing;
  if (since < ULTRASONIC_GAP_MS)
    delay(ULTRASONIC_GAP_MS - since);
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long duration = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
  lastPing = millis();
  if (!duration)
    return false;
  float raw = duration * SOUND_SPEED_MM_PER_US * .5f;
  if (raw < MIN_VALID_DISTANCE_MM || raw > MAX_VALID_DISTANCE_MM)
    return false;
  mm = raw;
  if (calibrated)
    mm = trig == FRONT_TRIG_PIN ? raw * FRONT_SCALE + FRONT_OFFSET_MM
                                : raw * DOWN_SCALE + DOWN_OFFSET_MM;
  return isfinite(mm) && mm >= MIN_VALID_DISTANCE_MM &&
         mm <= MAX_VALID_DISTANCE_MM;
}
float colorHz(bool s2, bool s3) {
  digitalWrite(COLOR_S2_PIN, s2 ? HIGH : LOW);
  digitalWrite(COLOR_S3_PIN, s3 ? HIGH : LOW);
  delay(5);
  unsigned long highUs = pulseIn(COLOR_OUT_PIN, HIGH, 25000UL),
                lowUs = pulseIn(COLOR_OUT_PIN, LOW, 25000UL);
  return highUs && lowUs ? 1000000.0f / (highUs + lowUs) : -1.0f;
}
void colorReport() {
  stopMotors();
  Serial.println("COLOR_RAW: R_Hz,G_Hz,B_Hz,CLEAR_Hz,LDR_raw");
  Serial.print(colorHz(false, false));
  Serial.print(',');
  Serial.print(colorHz(true, true));
  Serial.print(',');
  Serial.print(colorHz(false, true));
  Serial.print(',');
  Serial.print(colorHz(true, false));
  Serial.print(',');
  Serial.println(analogRead(LDR_PIN));
  Serial.println("Frequency is NOT pulse width; classify only after actual "
                 "sample calibration.");
}
void calibration(float knownMm) {
  stopMotors();
  if (knownMm < MIN_VALID_DISTANCE_MM || knownMm > MAX_VALID_DISTANCE_MM) {
    Serial.println("Known distance outside sensor range");
    return;
  }
  Serial.println("CAL: known_mm,raw_mm,error_mm");
  int valid = 0;
  float sum = 0, maximum = 0;
  for (int i = 0; i < 10; ++i) {
    float raw;
    Serial.print(knownMm);
    Serial.print(',');
    if (readDistanceMm(FRONT_TRIG_PIN, FRONT_ECHO_PIN, raw, false)) {
      float error = raw - knownMm;
      ++valid;
      sum += error;
      maximum = fmaxf(maximum, fabsf(error));
      Serial.print(raw);
      Serial.print(',');
      Serial.println(error);
    } else
      Serial.println("INVALID,INVALID");
  }
  Serial.print("Valid samples: ");
  Serial.print(valid);
  Serial.print("/10; mean error mm: ");
  if (valid)
    Serial.print(sum / valid);
  else
    Serial.print("NA");
  Serial.print("; max absolute error mm: ");
  Serial.println(maximum);
  Serial.println("Save results; update FRONT_SCALE/OFFSET and FRONT_CALIBRATED "
                 "after review.");
}
void measureWater() {
  stopMotors();
  float values[7];
  int count = 0;
  Serial.println("WATER: surface_mm,estimated_depth_mm");
  for (int i = 0; i < 7; ++i) {
    float mm;
    if (readDistanceMm(DOWN_TRIG_PIN, DOWN_ECHO_PIN, mm) &&
        mm <= SENSOR_TO_BOTTOM_MM) {
      values[count++] = SENSOR_TO_BOTTOM_MM - mm;
      Serial.print(mm);
      Serial.print(',');
      Serial.println(SENSOR_TO_BOTTOM_MM - mm);
    } else
      Serial.println("INVALID,INVALID");
  }
  if (count < 5) {
    Serial.println("Depth rejected: need at least 5/7 valid readings");
    return;
  }
  for (int i = 1; i < count; ++i) {
    float v = values[i];
    int j = i;
    while (j > 0 && values[j - 1] > v) {
      values[j] = values[j - 1];
      --j;
    }
    values[j] = v;
  }
  float median = count % 2 ? values[count / 2]
                           : (values[count / 2 - 1] + values[count / 2]) * .5f;
  Serial.print("Median mm: ");
  Serial.print(median);
  Serial.print("; spread mm: ");
  Serial.println(values[count - 1] - values[0]);
  if (values[count - 1] - values[0] > MAX_DEPTH_SPREAD_MM) {
    Serial.println("Unstable water surface; not transmitted");
    return;
  }
  if (!DEPTH_CALIBRATED) {
    Serial.println("Calibration mode: set true only after known-depth tests; "
                   "not transmitted");
    return;
  }
  if (!Enes100.isConnected()) {
    Serial.println("No connection; not transmitted");
    return;
  }
  Enes100.mission(DEPTH, (int)lroundf(median));
  Serial.println("Measured water depth transmitted with mission(DEPTH, mm).");
}

void driveTo(OTV::Pose p, OTV::Point destination) {
  float error =
      OTV::wrap(atan2f(destination.y - p.y, destination.x - p.x) - p.theta);
  if (fabsf(error) > .14f) {
    int turn = (int)constrain(fabsf(error) * 80, 25.0f, (float)TURN_PWM);
    if (error < 0)
      turn = -turn;
    pulseWheels(-turn, turn);
  } else {
    int speed = (int)constrain(OTV::distance(OTV::xy(p), destination) * 500,
                               65.0f, (float)DRIVE_PWM);
    int correction = (int)constrain(error * 150, -35.0f, 35.0f);
    pulseWheels(speed - correction, speed + correction);
  }
}
void beginScan(OTV::Pose p) {
  stopMotors();
  phase = Phase::SCAN;
  scanOrigin = OTV::xy(p);
  scanLast = p.theta;
  scanTotal = 0;
  Serial.println(
      "SCAN: rotating front sensor; not using any hidden obstacle coordinates");
}
bool geometryReady() {
  if (!OTV_GEOMETRY_CONFIRMED) {
    Serial.println("Measure OTV length/width, front probe offset and margin "
                   "before motion");
    return false;
  }
  return OTV_LENGTH_M > 0 && OTV_WIDTH_M > 0 && FRONT_SENSOR_OFFSET_M >= 0 &&
         BODY_CLEARANCE_M > .5f * hypotf(OTV_LENGTH_M, OTV_WIDTH_M) &&
         BODY_CLEARANCE_M < .4f;
}
void startMotion(Mode requested) {
  stopMotors();
  if (!motorReady() || !geometryReady())
    return;
  if ((requested == Mode::NAV_CLEAR || requested == Mode::NAV_AVOID) &&
      !MISSION_POINT_CONFIRMED) {
    Serial.println("Set confirmed Water Mission approach coordinates before "
                   "real autonomous navigation");
    return;
  }
  if (requested != Mode::TURN && !FRONT_CALIBRATED) {
    Serial.println("Calibrate obstacle sensor before travel");
    return;
  }
  OTV::Pose p;
  if (!readPose(p)) {
    Serial.println("No valid ArUco pose; not started");
    return;
  }
  mode = requested;
  modeStarted = millis();
  turnCount = 0;
  turnBase = p.theta;
  stage = 0;
  turnPauseUntil = 0;
  navigator.clear();
  navigator.margin = BODY_CLEARANCE_M;
  if (mode == Mode::FORWARD) {
    forwardOrigin = OTV::xy(p);
    target = {FORWARD_TARGET_X_M, p.y};
    if (target.x <= p.x) {
      fault("Forward target must be ahead in +X direction");
      return;
    }
    Serial.print("Planned forward displacement m: ");
    Serial.println(target.x - p.x);
    if (target.x - p.x < REQUIRED_FORWARD_TRAVEL_M)
      Serial.println("NOTE: shorter than rubric's 3.4 m; do not claim this "
                     "proves the distance requirement.");
  } else if (mode == Mode::NAV_CLEAR || mode == Mode::NAV_AVOID) {
    target = {MISSION_X_M, MISSION_Y_M};
    if (!navigator.free(target) || !navigator.free({FINISH_X_M, FINISH_Y_M})) {
      fault("Route target outside planner area");
      return;
    }
    phase = Phase::PLAN;
    if (mode == Mode::NAV_AVOID)
      beginScan(p);
  }
  Serial.println("Motion demonstration started; q aborts between bounded "
                 "movement pulses.");
}
void motionStep() {
  OTV::Pose p;
  if (!readPose(p)) {
    fault("ArUco invisible/disconnected; motors stopped. Restore marker and "
          "restart.");
    return;
  }
  unsigned long elapsed = millis() - modeStarted;
  unsigned long limit =
      mode == Mode::TURN
          ? TURN_TIMEOUT_MS
          : (mode == Mode::FORWARD ? FORWARD_TIMEOUT_MS : NAV_TIMEOUT_MS);
  if (elapsed >= limit) {
    fault("Demonstration timeout");
    return;
  }
  if (mode == Mode::TURN) {
    if (turnPauseUntil && (long)(millis() - turnPauseUntil) < 0)
      return;
    float error = OTV::wrap(turnBase + (turnCount + 1) * PI / 2 - p.theta);
    if (fabsf(error) < ANGLE_TOLERANCE_RAD) {
      ++turnCount;
      turnPauseUntil = millis() + 700;
      Serial.print("90-degree turn complete: ");
      Serial.print(turnCount);
      Serial.println("/3");
      if (turnCount == 3)
        finish("DONE: three consecutive 90-degree turns");
    } else {
      int turn = (int)constrain(fabsf(error) * 80, 25.0f, (float)TURN_PWM);
      if (error < 0)
        turn = -turn;
      pulseWheels(-turn, turn);
    }
    return;
  }
  float mm;
  if (!readDistanceMm(FRONT_TRIG_PIN, FRONT_ECHO_PIN, mm)) {
    fault("Front ultrasonic invalid; cannot infer a clear path");
    return;
  }
  float frontM = mm / 1000.0f;
  if (mode == Mode::NAV_AVOID) {
    navigator.observe(p, frontM, FRONT_SENSOR_OFFSET_M);
    if (navigator.mapFull) {
      fault("Obstacle map full; stopped rather than discarding measurements");
      return;
    }
    if (phase == Phase::SCAN) {
      scanTotal += fabsf(OTV::wrap(p.theta - scanLast));
      scanLast = p.theta;
      if (scanTotal >= 2 * PI) {
        phase = Phase::PLAN;
        return;
      }
      pulseWheels(-TURN_PWM, TURN_PWM);
      return;
    }
  }
  if (OTV::distance(OTV::xy(p), target) < POSITION_TOLERANCE_M) {
    if (mode == Mode::FORWARD) {
      Serial.print("Actual endpoint displacement m: ");
      Serial.println(OTV::distance(forwardOrigin, OTV::xy(p)));
      Serial.print("Elapsed ms: ");
      Serial.println(elapsed);
      finish("DONE: pre-limbo point reached; compare measured distance/time to "
             "rubric");
    } else if (stage == 0) {
      Serial.println("MISSION APPROACH REACHED. Use m/c/p separately for "
                     "calibrated mission demonstrations.");
      // Navigation qualification does not require guessing an automatic pump
      // duration. Measure/pump tests are explicit stationary demonstrations,
      // not fabricated results.
      stage = 1;
      target = {FINISH_X_M, FINISH_Y_M};
      phase = Phase::PLAN;
    } else
      finish("DONE: mission approach -> stopped before log/limbo");
    return;
  }
  if (mode != Mode::NAV_AVOID) {
    if (mm < OBSTACLE_STOP_DISTANCE_MM) {
      fault("Obstacle encountered in clear-path/forward mode");
      return;
    }
    driveTo(p, target);
    return;
  }
  if (phase == Phase::PLAN) {
    // Motors already stopped while the A* calculation executes.
    if (!navigator.plan(p, target)) {
      fault("No measured-clear route");
      return;
    }
    phase = Phase::DRIVE;
    return;
  }
  if (OTV::distance(OTV::xy(p), scanOrigin) > RESCAN_TRAVEL_M) {
    beginScan(p);
    return;
  }
  OTV::Point waypoint;
  if (!navigator.next(p, waypoint)) {
    phase = Phase::PLAN;
    return;
  }
  float error = OTV::wrap(atan2f(waypoint.y - p.y, waypoint.x - p.x) - p.theta);
  float ahead =
      fminf(DETECTION_DISTANCE_M, OTV::distance(OTV::xy(p), waypoint) + .04f);
  if (fabsf(error) <= .14f && frontM < ahead) {
    beginScan(p);
    return;
  }
  driveTo(p, waypoint);
}

void handleCommand(char *command) {
  if (!strcmp(command, "q")) {
    finish("STOPPED by operator");
    return;
  }
  if (mode == Mode::PUMP)
    return; // No printing or other work may delay the pump deadline.
  if (!strcmp(command, "h")) {
    help();
    return;
  }
  if (mode == Mode::PUMP || mode == Mode::FORWARD || mode == Mode::TURN ||
      mode == Mode::NAV_CLEAR || mode == Mode::NAV_AVOID) {
    Serial.println("Busy: use q to stop before changing test");
    return;
  }
  if (!strcmp(command, "d"))
    mode = Mode::DIAGNOSTICS;
  else if (!strcmp(command, "w"))
    mode = Mode::WIRELESS;
  else if (!strcmp(command, "f"))
    startMotion(Mode::FORWARD);
  else if (!strcmp(command, "t"))
    startMotion(Mode::TURN);
  else if (!strcmp(command, "n"))
    startMotion(Mode::NAV_CLEAR);
  else if (!strcmp(command, "a"))
    startMotion(Mode::NAV_AVOID);
  else if (!strcmp(command, "c"))
    colorReport();
  else if (!strcmp(command, "m"))
    measureWater();
  else if (command[0] == 'k') {
    float known = 0;
    if (sscanf(command, "k %f", &known) == 1)
      calibration(known);
    else
      Serial.println("Use k 200 (known distance in mm)");
  } else if (!strcmp(command, "x")) {
    if (!Enes100.isConnected()) {
      Serial.println("No vision connection");
      return;
    }
    Serial.println("TEST ONLY: sending 25 mm; NOT a measured water depth");
    Enes100.println("TEST ONLY: following 25 mm payload is not a measurement");
    Enes100.mission(DEPTH, 25);
  } else if (!strcmp(command, "p")) {
    if (!PUMP_WIRING_CONFIRMED) {
      Serial.println(
          "Pump test refused: confirm driver topology and active level first");
      return;
    }
    if (PUMP_TEST_MS == 0 || PUMP_TEST_MS > MAX_PUMP_MS) {
      Serial.println("Invalid pump duration");
      return;
    }
    stopMotors();
    Serial.println("PUMP TEST ON");
    digitalWrite(pumpPin(), PUMP_ACTIVE_HIGH ? HIGH : LOW);
    pumpOn = true;
    pumpStarted = millis();
    mode = Mode::PUMP;
  } else
    Serial.println("Unknown command; h for help");
}
void serialCommands() {
  // Fixed-size command parser; no blocking readStringUntil() while a motor/pump
  // runs.
  static char buffer[32];
  static unsigned count = 0;
  static bool overflow = false;
  unsigned processed = 0;
  while (Serial.available() && processed++ < 64) {
    char c = (char)Serial.read();
    if (c == 'q') {
      finish("STOPPED by operator");
      count = 0;
      overflow = false;
      continue;
    }
    if (c == '\r' || c == '\n') {
      if (overflow) {
        if (mode != Mode::PUMP)
          Serial.println("Command too long; discarded");
      } else if (count) {
        buffer[count] = '\0';
        handleCommand(buffer);
      }
      count = 0;
      overflow = false;
    } else if (count < sizeof(buffer) - 1)
      buffer[count++] = c;
    else
      overflow = true;
  }
}
void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_ENA_PIN, OUTPUT);
  pinMode(MOTOR_ENB_PIN, OUTPUT);
  stopMotors();
  // Set known direction pins only after disables are low.
  pinMode(MOTOR_IN3_PIN, OUTPUT);
  pinMode(MOTOR_IN4_PIN, OUTPUT);
  digitalWrite(MOTOR_IN3_PIN, LOW);
  digitalWrite(MOTOR_IN4_PIN, LOW);
  if (motorReady()) {
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
  }
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  digitalWrite(FRONT_TRIG_PIN, LOW);
  pinMode(DOWN_TRIG_PIN, OUTPUT);
  pinMode(DOWN_ECHO_PIN, INPUT);
  digitalWrite(DOWN_TRIG_PIN, LOW);
  pinMode(COLOR_OUT_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  const int colorPins[] = {COLOR_LED_PIN, COLOR_S0_PIN, COLOR_S1_PIN,
                           COLOR_S2_PIN, COLOR_S3_PIN};
  for (int pin : colorPins)
    pinMode(pin, OUTPUT);
  digitalWrite(COLOR_LED_PIN, HIGH);
  digitalWrite(COLOR_S0_PIN, HIGH);
  digitalWrite(COLOR_S1_PIN, LOW);
  if (PUMP_WIRING_CONFIRMED) {
    digitalWrite(pumpPin(), PUMP_ACTIVE_HIGH ? LOW : HIGH);
    pinMode(pumpPin(), OUTPUT);
    stopPump();
  } else {
    pinMode(WATER_PUMP_PIN, INPUT);
    pinMode(PUMP_RELAY_PIN, INPUT);
  }
  help();
  // Arduino R4 uses onboard WiFi. Motors are disabled BEFORE begin().
  Enes100.begin(TEAM_NAME, WATER, ARUCO_ID, ROOM_NUMBER);
  Enes100.println("Octonauts integrated demonstration controller ready");
}
void loop() {
  stopMotors();
  // Pump deadline checked before serial handling or any potentially blocking
  // I/O.
  if (pumpOn && millis() - pumpStarted >= PUMP_TEST_MS)
    finish("DONE: pump OFF after bounded test");
  serialCommands();
  if (mode == Mode::PUMP)
    return; // Do not call WiFi or pulseIn() while pump is active.
  if (mode == Mode::FORWARD || mode == Mode::TURN || mode == Mode::NAV_CLEAR ||
      mode == Mode::NAV_AVOID) {
    motionStep();
    return;
  }
  if (mode != Mode::DIAGNOSTICS && mode != Mode::WIRELESS)
    return;
  if (millis() - lastLog < 1000)
    return;
  lastLog = millis();
  OTV::Pose p;
  if (readPose(p)) {
    char text[100];
    snprintf(text, sizeof(text), "POSE x=%.3f y=%.3f theta=%.3f", (double)p.x,
             (double)p.y, (double)p.theta);
    Serial.println(text);
    Enes100.println(text);
  } else
    Serial.println("POSE INVALID: not visible or disconnected");
  if (mode == Mode::DIAGNOSTICS) {
    float mm;
    Serial.print("Front mm: ");
    if (readDistanceMm(FRONT_TRIG_PIN, FRONT_ECHO_PIN, mm))
      Serial.println(mm);
    else
      Serial.println("INVALID");
    Serial.print("Surface distance mm: ");
    if (readDistanceMm(DOWN_TRIG_PIN, DOWN_ECHO_PIN, mm))
      Serial.println(mm);
    else
      Serial.println("INVALID");
  }
}
