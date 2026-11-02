#include "Navigator.h"
#include <assert.h>
#include <stdio.h>

// Host-side geometry and planner regression tests. No hardware or WiFi
// required.
int main() {
  using namespace OTV;
  const float pi = 3.14159265358979323846f;
  assert(fabsf(wrap(-pi + .02f - (pi - .02f)) - .04f) < .0001f);
  Navigator nav;
  Pose start = {.55f, .55f, 0};
  assert(nav.plan(start, {3.15f, 1.45f}));
  assert(!nav.plan(start, {4.0f, 2.0f}));
  nav.clear();
  // Surface samples describe a rectangle at x=1.4..1.6, y=.75..1.25.
  for (int i = 0; i <= 10; ++i) {
    float y = .75f + i * .05f;
    assert(nav.observe({.55f, y, 0}, .775f, .075f));
    nav.observe({2.4f, y, pi}, .725f, .075f);
  }
  for (int i = 0; i <= 4; ++i) {
    float x = 1.4f + i * .05f;
    nav.observe({x, .2f, pi / 2}, .475f, .075f);
    nav.observe({x, 1.8f, -pi / 2}, .475f, .075f);
  }
  assert(!nav.free({1.5f, 1.0f}));
  assert(nav.plan({.55f, 1.0f, 0}, {3.15f, 1.45f}));
  Point previous = {.55f, 1.0f};
  for (int i = nav.cursor; i >= 0; --i) {
    Point next = Navigator::point(nav.route[i]);
    assert(nav.segment(previous, next));
    previous = next;
  }
  // An already-seen point is not duplicated, and an arena-wall hit is ignored.
  int saved = nav.hitCount;
  assert(!nav.observe({.55f, .75f, 0}, .775f, .075f));
  assert(nav.hitCount == saved);
  assert(!nav.observe({.55f, .55f, pi}, .475f, .075f));
  nav.clear();
  nav.hitCount = Navigator::MAX_HITS;
  for (int i = 0; i < nav.hitCount; ++i)
    nav.hits[i] = {.5f, .5f};
  nav.observe({.55f, 1.0f, 0}, .5f, .075f);
  assert(nav.mapFull);
  printf("PASS: angle wrap, clear route, invalid goal, obstacle detour, path "
         "clearance, hit deduplication, wall rejection, bounded storage\n");
  printf("Navigator storage: %u bytes\n", (unsigned)sizeof(Navigator));
}
