#pragma once
#ifdef OTV_HOST_TEST
#include "test/stubs/Arduino.h"
#else
#include <Arduino.h>
#endif

// [CONFIG / 配置] All dimensions and wiring assumptions live here.
// Pin assignments below preserve your team's supplied wiring table.
constexpr uint8_t LDR_PIN = A0;
constexpr uint8_t FRONT_TRIG_PIN = A1, FRONT_ECHO_PIN = A2;
constexpr uint8_t DOWN_TRIG_PIN = A3, DOWN_ECHO_PIN = A4;
constexpr uint8_t COLOR_OUT_PIN = 2, COLOR_LED_PIN = 4;
constexpr uint8_t COLOR_S0_PIN = 7, COLOR_S1_PIN = 8;
constexpr uint8_t COLOR_S2_PIN = 11, COLOR_S3_PIN = 12;
constexpr uint8_t MOTOR_IN3_PIN = 3, MOTOR_IN4_PIN = 9;
constexpr uint8_t MOTOR_ENA_PIN = 6, MOTOR_ENB_PIN = 5;
constexpr uint8_t WATER_PUMP_PIN = 10, PUMP_RELAY_PIN = 13;

// [HARDWARE GATE / 待实车核对] UNO R4 WiFi does not determine H-bridge wiring.
// Standard L298N: IN1/IN2 control channel A; IN3/IN4 control channel B.
// Keep disabled until both missing pins, left/right channel and polarity are
// known.
constexpr bool MOTOR_WIRING_CONFIRMED = false;
constexpr int MOTOR_IN1_PIN = -1,
              MOTOR_IN2_PIN = -1; // -1 is deliberately unassigned.
constexpr bool CHANNEL_A_IS_LEFT = true;
constexpr int LEFT_FORWARD_SIGN = 1, RIGHT_FORWARD_SIGN = 1;
constexpr uint8_t DRIVE_PWM = 110, TURN_PWM = 80;
constexpr unsigned long MOTOR_PULSE_MS =
    40; // Every command ends with an explicit stop.

// [PUMP / 水泵] Select exactly one driver after checking the physical circuit.
// The pump must use a rated external driver; never power it from an Arduino
// pin.
constexpr bool PUMP_WIRING_CONFIRMED = false;
constexpr bool PUMP_USE_RELAY =
    true; // true -> D13, false -> external driver on D10.
constexpr bool PUMP_ACTIVE_HIGH =
    true; // Check module; active-low modules are common.
constexpr unsigned long PUMP_TEST_MS = 1500, MAX_PUMP_MS = 3000;

// [VISION / 定位] Units: meters, radians. Positive theta is counterclockwise.
constexpr char TEAM_NAME[] = "Octonauts";
constexpr int ARUCO_ID = 106, ROOM_NUMBER = 1116;
constexpr float MARKER_HEADING_OFFSET_RAD =
    0.0f; // Calibrate marker vs vehicle nose.
constexpr float ARENA_WIDTH_M = 4.0f, ARENA_HEIGHT_M = 2.0f;
constexpr float OTV_LENGTH_M = .15f,
                OTV_WIDTH_M = .12f; // REPLACE with actual OTV dimensions.
constexpr bool OTV_GEOMETRY_CONFIRMED = false;
constexpr float FRONT_SENSOR_OFFSET_M =
    .075f; // Marker center to front probe, along nose.
constexpr float BODY_CLEARANCE_M =
    .19f; // Radius = half body diagonal + practical margin.

// [ROUTE / 路线] Website draws log/limbo around x=3.4; stop before its near
// edge. The current MicroPython simulator does not publish a Water Mission
// coordinate.
constexpr bool MISSION_POINT_CONFIRMED = false;
constexpr float MISSION_X_M = .55f,
                MISSION_Y_M = 1.45f; // Demonstration only; confirm real point.
constexpr float FINISH_X_M = 3.15f, FINISH_Y_M = 1.45f;
constexpr float POSITION_TOLERANCE_M = .06f, ANGLE_TOLERANCE_RAD = .07f;
constexpr float FORWARD_TARGET_X_M = 3.15f;
constexpr float REQUIRED_FORWARD_TRAVEL_M =
    3.4f; // Rubric distance, not arena x coordinate.
constexpr unsigned long FORWARD_TIMEOUT_MS = 180000UL,
                        NAV_TIMEOUT_MS = 300000UL;
constexpr unsigned long TURN_TIMEOUT_MS = 45000UL;

// [CALIBRATION / 标定] measured_mm = raw_mm * scale + offset.
constexpr float FRONT_SCALE = 1.0f, FRONT_OFFSET_MM = 0.0f;
constexpr float DOWN_SCALE = 1.0f, DOWN_OFFSET_MM = 0.0f;
constexpr bool FRONT_CALIBRATED = false, DEPTH_CALIBRATED = false;
constexpr float SENSOR_TO_BOTTOM_MM =
    62.0f; // Measure in actual water-sensing posture.
constexpr float MIN_VALID_DISTANCE_MM = 20.0f, MAX_VALID_DISTANCE_MM = 4000.0f;
constexpr float SOUND_SPEED_MM_PER_US = .343f;
constexpr unsigned long ECHO_TIMEOUT_US = 30000UL, ULTRASONIC_GAP_MS = 60UL;
constexpr float OBSTACLE_STOP_DISTANCE_MM = 200.0f;
constexpr float RESCAN_TRAVEL_M = .30f, DETECTION_DISTANCE_M = .42f;
constexpr float MAX_DEPTH_SPREAD_MM = 5.0f;
// RGB frequencies are logged for real-sample calibration; no invented water
// classifier.
