#pragma once
#define WATER 4
#define DEPTH 0
struct VisionStub {
  bool connected = true, visible = true;
  float x = .55f, y = .55f, theta = 0;
  int missions = 0, lastType = -1, lastValue = -1;
  void begin(const char *, int, int, int) {}
  bool isConnected() { return connected; }
  bool isVisible() { return visible; }
  float getX() { return x; }
  float getY() { return y; }
  float getTheta() { return theta; }
  void println(const char *) {}
  void mission(int type, int value) {
    ++missions;
    lastType = type;
    lastValue = value;
  }
};
inline VisionStub Enes100;
