
#pragma once

#include <Arduino.h>

// Front Ultrasonic Sensor
constexpr uint8_t FRONT_TRIG_PIN = A1;
constexpr uint8_t FRONT_ECHO_PIN = A2;

// Downward Ultrasonic Sensor (Water Depth)
constexpr uint8_t DOWN_TRIG_PIN = A3;
constexpr uint8_t DOWN_ECHO_PIN = A4;

// Measure and verify this value on the actual OTV.
constexpr float SENSOR_TO_BOTTOM_MM = 62.0f;

// GY-31 Color Sensor
constexpr uint8_t COLOR_OUT_PIN = 2;
constexpr uint8_t COLOR_S3_PIN = 12;

// L298N Dual H-Bridge
// Only pins listed in the current wiring table, full motor direction control is not yet confirmed.
constexpr uint8_t MOTOR_IN3_PIN = 3;
constexpr uint8_t MOTOR_ENB_PIN = 5;
constexpr uint8_t MOTOR_ENA_PIN = 6;
constexpr uint8_t MOTOR_IN4_PIN = 9;

// Water Pump / Relay, keep separate until actual wiring is confirmed.
constexpr uint8_t WATER_PUMP_PIN = 10;
constexpr uint8_t PUMP_RELAY_PIN = 13;

// Ultrasonic Settings
constexpr unsigned long ECHO_TIMEOUT_US = 30000UL;
constexpr float SOUND_SPEED_MM_PER_US = 0.343f;

// Navigation Settings
constexpr float OBSTACLE_STOP_DISTANCE_MM = 200.0f;
constexpr float POSITION_TOLERANCE_M = 0.10f;
constexpr float ANGLE_TOLERANCE_RAD = 0.12f;
