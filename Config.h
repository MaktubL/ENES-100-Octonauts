// Config.h

#pragma once

#include <Arduino.h>

// Front ultrasonic sensor
constexpr int FRONT_TRIG_PIN = A1;
constexpr int FRONT_ECHO_PIN = A2;

// Downward ultrasonic sensor
constexpr int DOWN_TRIG_PIN = A3;
constexpr int DOWN_ECHO_PIN = A4;

// GY-31 color sensor
constexpr int COLOR_OUT_PIN = 2;
constexpr int COLOR_S3_PIN = 12;

// Water pump relay
constexpr int PUMP_RELAY_PIN = 13;