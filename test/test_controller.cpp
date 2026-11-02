// Host-only checks: excluded from the Arduino board's compilation context.
#ifndef ARDUINO
// Exercises the actual integrated main.cpp with fake pin/vision/serial
// adapters. This verifies software behavior; it does not claim hardware
// calibration.
#define OTV_HOST_TEST 1
#include "../src/main.cpp"
#include <assert.h>
#include <stdio.h>
void command(const char *text) {
  Serial.send(text);
  loop();
}
int main() {
  setup();
  assert(mode == Mode::IDLE && !pumpOn);
  assert(Fake::pwm[MOTOR_ENA_PIN] == 0 && Fake::pwm[MOTOR_ENB_PIN] == 0);
  assert(Fake::modes[WATER_PUMP_PIN] == INPUT &&
         Fake::modes[PUMP_RELAY_PIN] == INPUT);
  for (const char *test : {"f\n", "t\n", "n\n", "a\n", "p\n"}) {
    command(test);
    assert(mode == Mode::IDLE && !pumpOn);
    assert(Fake::pwm[MOTOR_ENA_PIN] == 0 && Fake::pwm[MOTOR_ENB_PIN] == 0);
  }
  command("x\n");
  assert(Enes100.missions == 1 && Enes100.lastType == DEPTH &&
         Enes100.lastValue == 25);
  command("m\n"); // Valid raw samples, but calibration flag is false: no
                  // mission sent.
  assert(Enes100.missions == 1);
  float mm = 0;
  Fake::echo = 0;
  assert(!readDistanceMm(FRONT_TRIG_PIN, FRONT_ECHO_PIN, mm));
  Fake::echo = 583;
  assert(readDistanceMm(FRONT_TRIG_PIN, FRONT_ECHO_PIN, mm));
  assert(fabsf(mm - 99.9845f) < .01f);
  OTV::Pose p;
  Enes100.visible = false;
  assert(!readPose(p));
  Enes100.visible = true;
  Enes100.x = NAN;
  assert(!readPose(p));
  Enes100.x = .55f;
  command("w\n");
  assert(mode == Mode::WIRELESS);
  command("q"); // Abort without newline.
  assert(mode == Mode::IDLE);
  Serial.send("hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh\n");
  loop();
  assert(mode == Mode::IDLE);
  // A large incoming stream is processed in bounded chunks, not an infinite
  // loop.
  for (int i = 0; i < 200; ++i)
    Serial.input.push_back('z');
  loop();
  assert(Serial.input.size() == 136);
  printf("PASS: idle boot, hardware gates, explicit test transmit, "
         "uncalibrated depth suppression, invalid echo/pose, immediate q, "
         "command overflow, bounded parser\n");
}

#endif // !ARDUINO

