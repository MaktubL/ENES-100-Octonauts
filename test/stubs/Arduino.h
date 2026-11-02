#pragma once
#include <algorithm>
#include <array>
#include <deque>
#include <math.h>
#include <sstream>
#include <stdint.h>
#include <string>
constexpr int A0 = 14, A1 = 15, A2 = 16, A3 = 17, A4 = 18;
constexpr int HIGH = 1, LOW = 0, OUTPUT = 1, INPUT = 0, NUM_DIGITAL_PINS = 24;
constexpr float PI = 3.14159265358979323846f;
namespace Fake {
inline unsigned long clock = 0, echo = 175;
inline std::array<int, 24> modes = {}, pwm = {}, levels = {};
} // namespace Fake
struct SerialStub {
  std::deque<char> input;
  std::string output;
  void begin(int) {}
  int available() { return (int)input.size(); }
  int read() {
    char c = input.front();
    input.pop_front();
    return c;
  }
  void send(const char *s) {
    while (*s)
      input.push_back(*s++);
  }
  template <typename T> void print(T value) {
    std::ostringstream text;
    text << value;
    output += text.str();
  }
  template <typename T> void println(T value) {
    print(value);
    output += '\n';
  }
  void println() { output += '\n'; }
};
inline SerialStub Serial;
inline unsigned long millis() { return Fake::clock; }
inline void delay(unsigned long ms) { Fake::clock += ms; }
inline void delayMicroseconds(unsigned long) {}
inline void pinMode(int p, int m) { Fake::modes.at(p) = m; }
inline void digitalWrite(int p, int v) { Fake::levels.at(p) = v; }
inline void analogWrite(int p, int v) { Fake::pwm.at(p) = v; }
inline int analogRead(int) { return 512; }
inline unsigned long pulseIn(int, int, unsigned long) { return Fake::echo; }
template <typename T> inline T constrain(T n, T low, T high) {
  return std::max(low, std::min(high, n));
}
