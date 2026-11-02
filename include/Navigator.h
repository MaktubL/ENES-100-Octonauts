#pragma once
#include <math.h>
#include <stdint.h>

// [NAVIGATION CORE] No Arduino/WiFi calls: inputs are pose + measured surface
// points. Fixed storage keeps memory bounded on the UNO R4 (32 KB RAM).
namespace OTV {
struct Point {
  float x, y;
};
struct Pose {
  float x, y, theta;
};
inline float distance(Point a, Point b) { return hypotf(a.x - b.x, a.y - b.y); }
inline float wrap(float a) { return atan2f(sinf(a), cosf(a)); }
inline Point xy(Pose p) { return {p.x, p.y}; }
class Navigator {
public:
  static constexpr int NX = 31, NY = 17, N = NX * NY, MAX_HITS = 180;
  float margin = .19f;
  bool mapFull = false;
  Point hits[MAX_HITS];
  int hitCount = 0;
  // Path is stored goal-to-start; cursor walks backward.
  uint16_t route[N];
  int cursor = -1;
  Point goal = {0, 0};
  void clear() {
    hitCount = 0;
    cursor = -1;
    mapFull = false;
  }
  float clearance(Point p) const {
    float value = fminf(fminf(p.x, 4 - p.x), fminf(p.y, 2 - p.y));
    for (int i = 0; i < hitCount; ++i)
      value = fminf(value, distance(p, hits[i]));
    return value;
  }
  bool free(Point p) const {
    return p.x >= .15f && p.x <= 3.22f && p.y >= .15f && p.y <= 1.85f &&
           clearance(p) > margin;
  }
  bool segment(Point a, Point b) const {
    float required = margin, initial = clearance(a);
    // A newly observed surface may be inside the planning buffer.
    // Allow escape away from it, never motion further into the buffer.
    if (initial <= margin) {
      if (initial < .115f || clearance(b) <= initial + .005f)
        return false;
      required = initial - .003f;
    }
    int steps = (int)(distance(a, b) / .025f) + 1;
    for (int i = 0; i <= steps; ++i) {
      float f = (float)i / steps;
      Point p = {a.x + f * (b.x - a.x), a.y + f * (b.y - a.y)};
      if (p.x < .15f || p.x > 3.22f || p.y < .15f || p.y > 1.85f ||
          clearance(p) <= required)
        return false;
    }
    return true;
  }
  bool observe(Pose p, float frontM, float offsetM) {
    if (!isfinite(frontM) || frontM < 0)
      return false;
    Point hit = {p.x + (frontM + offsetM) * cosf(p.theta),
                 p.y + (frontM + offsetM) * sinf(p.theta)};
    if (hit.x < .08f || hit.x > 3.92f || hit.y < .08f || hit.y > 1.92f)
      return false;
    for (int i = 0; i < hitCount; ++i)
      if (distance(hit, hits[i]) < .045f)
        return false;
    if (hitCount == MAX_HITS) {
      mapFull = true;
      return false;
    }
    hits[hitCount++] = hit;
    return true;
  }
  static Point point(int node) {
    return {.2f + .1f * (node % NX), .2f + .1f * (node / NX)};
  }
  static int node(Point p) {
    int x = (int)lroundf((p.x - .2f) * 10), y = (int)lroundf((p.y - .2f) * 10);
    if (x < 0)
      x = 0;
    if (x >= NX)
      x = NX - 1;
    if (y < 0)
      y = 0;
    if (y >= NY)
      y = NY - 1;
    return y * NX + x;
  }
  bool plan(Pose p, Point target) {
    // A*: sensed obstacles only. No simulator obstacle coordinates are used.
    goal = target;
    cursor = -1;
    if (!free(goal))
      return false;
    int start = -1, end = node(goal);
    float nearest = 1e9f;
    for (int i = 0; i < N; ++i) {
      Point q = point(i);
      float d = distance(xy(p), q);
      if (d <= .35f && d < nearest && free(q) && segment(xy(p), q)) {
        start = i;
        nearest = d;
      }
    }
    if (start < 0)
      return false;
    for (int i = 0; i < N; ++i) {
      costs[i] = 1e9f;
      parents[i] = -1;
      flags[i] = 0;
    }
    costs[start] = 0;
    flags[start] = 1;
    for (int iteration = 0; iteration < N; ++iteration) {
      int current = -1;
      float best = 1e9f;
      for (int i = 0; i < N; ++i)
        if (flags[i] == 1) {
          float score = costs[i] + distance(point(i), point(end)) * 10;
          if (score < best) {
            current = i;
            best = score;
          }
        }
      if (current < 0)
        return false;
      if (current == end) {
        int count = 0;
        for (int n = end; n >= 0; n = parents[n]) {
          route[count++] = (uint16_t)n;
          if (n == start)
            break;
        }
        cursor = count - 1;
        return segment(point(end), goal);
      }
      flags[current] = 2;
      for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy) {
          int x = current % NX + dx, y = current / NX + dy;
          if ((dx == 0 && dy == 0) || x < 0 || x >= NX || y < 0 || y >= NY)
            continue;
          int next = y * NX + x;
          if (flags[next] == 2 || !segment(point(current), point(next)))
            continue;
          float cost = costs[current] + hypotf((float)dx, (float)dy);
          if (cost < costs[next]) {
            costs[next] = cost;
            parents[next] = (int16_t)current;
            flags[next] = 1;
          }
        }
    }
    return false;
  }
  bool next(Pose p, Point &target) {
    while (cursor >= 0 && distance(xy(p), point(route[cursor])) < .055f)
      --cursor;
    if (cursor < 0) {
      target = goal;
      return segment(xy(p), target);
    }
    if (!segment(xy(p), point(route[cursor])))
      return false;
    int chosen = cursor;
    for (int i = cursor - 1; i >= 0; --i) {
      Point q = point(route[i]);
      if (distance(xy(p), q) > .45f)
        break;
      if (segment(xy(p), q))
        chosen = i;
    }
    cursor = chosen;
    target = point(route[cursor]);
    return true;
  }

private:
  float costs[N];
  int16_t parents[N];
  uint8_t flags[N]; // 0 unseen, 1 open, 2 closed.
};
} // namespace OTV
