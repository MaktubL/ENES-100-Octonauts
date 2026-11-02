# Paste this entire file into the UMD MicroPython simulator.
# The website supplies enes100, tank, time and mission constants.
# This is simulator code; do not upload it to an Arduino UNO R4.
import math
import js

MODE = "NAVIGATION"  # NAVIGATION / TURN_TEST / FORWARD_TEST / WIRELESS_TEST / SENSOR_TEST
# The current simulator has no water-site marker/API. This default is only
# a demonstration approach point, opposite the initial landing position.
# Replace with (x, y) from your instructor's Water Mission arena plan.
MISSION_POINT = None
FINISH_POINT = (3.15, 1.45)  # Stop before the log/limbo at x=3.4.
ENABLE_AVOIDANCE = True
SEND_DEMO_MISSION = False  # True only for explicitly labelled transmit tests.
DEMO_DEPTH_MM = 25         # Test payload, NOT a measured water depth.

GRID = 0.10
NX, NY = 31, 17           # Centers x=.2..3.2, y=.2..1.8
BODY_MARGIN = 0.19        # For simulator's .15 x .12 m body, not your OTV.
SENSOR_OFFSET = 0.075     # Simulator front sensor offset from vehicle center.
STOP_RANGE = 0.42
POSITION_TOL = 0.06
ANGLE_TOL = 0.07
TIMEOUT_MS = 180000

def hypot(x, y):
    return math.sqrt(x*x + y*y)

def now_ms():
    return js.ArduinoAPI.millis()

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def wheels(left, right):
    tank.set_left_motor_pwm(int(max(-255, min(255, left))))
    tank.set_right_motor_pwm(int(max(-255, min(255, right))))

def stop():
    tank.turn_off_motors()

def pose():
    if not enes100.is_connected() or not enes100.is_visible():
        return None
    p = (enes100.get_x(), enes100.get_y(), enes100.get_theta())
    if not all(math.isfinite(v) for v in p):
        return None
    if not (0 <= p[0] <= 4 and 0 <= p[1] <= 2):
        return None
    return p

def front():
    d = tank.read_distance_sensor(1)
    if not math.isfinite(d) or (d < 0 and d != -1):
        raise ValueError("Invalid simulator range")
    return d

def remember(p, d):
    if d < 0:
        return False  # Simulator: -1 means no hit in its 1 m range.
    x = p[0] + (d + SENSOR_OFFSET) * math.cos(p[2])
    y = p[1] + (d + SENSOR_OFFSET) * math.sin(p[2])
    # Arena wall returns need not be stored: planner already respects walls.
    if x < 0.08 or x > 3.92 or y < 0.08 or y > 1.92:
        return False
    for a, b in hits:
        if hypot(x-a, y-b) < 0.045:
            return False
    hits.append((x, y))
    return True

def point(cell):
    return (0.2 + cell[0]*GRID, 0.2 + cell[1]*GRID)

def cell(p):
    return (int(max(0, min(NX-1, round((p[0]-.2)/GRID)))),
            int(max(0, min(NY-1, round((p[1]-.2)/GRID)))))

def free(x, y):
    if not (.15 <= x <= 3.22 and .15 <= y <= 1.85):
        return False
    for a, b in hits:
        if hypot(x-a, y-b) <= BODY_MARGIN:
            return False
    return True

def clearance(p):
    result = min(p[0], 4-p[0], p[1], 2-p[1])
    for a, b in hits:
        result = min(result, hypot(p[0]-a, p[1]-b))
    return result

def line_free(a, b):
    margin = BODY_MARGIN
    initial = clearance(a)
    if initial <= margin:
        # A newly discovered surface can place the current pose inside the
        # planning buffer. Permit an outward escape, never closer approach.
        if initial < .115 or clearance(b) <= initial+.005:
            return False
        margin = initial-.003
    steps = max(1, int(hypot(b[0]-a[0], b[1]-a[1])/.025)+1)
    for i in range(steps+1):
        f = i/steps
        x, y = a[0]+f*(b[0]-a[0]), a[1]+f*(b[1]-a[1])
        if not (.15 <= x <= 3.22 and .15 <= y <= 1.85) or clearance((x,y)) <= margin:
            return False
    return True

def plan(p, goal):
    # A* uses only ultrasonic hit points collected by remember().
    # Unknown space is provisionally free; a new detection triggers a scan/replan.
    start, end = cell(p), cell(goal)
    best_distance = 1e9
    best_start = None
    for ix in range(NX):
        for iy in range(NY):
            xy = point((ix, iy))
            distance = hypot(xy[0]-p[0], xy[1]-p[1])
            if distance <= .35 and distance < best_distance and free(xy[0], xy[1]) and line_free(p, xy):
                best_distance, best_start = distance, (ix, iy)
    if best_start is None:
        return []
    start = best_start
    opened = [start]
    cost = {start: 0.0}
    parent = {}
    closed = set()
    while opened:
        best_score = 1e9
        current = opened[0]
        for candidate in opened:
            score = cost[candidate]+hypot(candidate[0]-end[0], candidate[1]-end[1])
            if score < best_score:
                best_score, current = score, candidate
        opened.remove(current)
        if current == end:
            path = [goal]
            while current != start:
                path.append(point(current))
                current = parent[current]
            path.append(point(start))
            path.reverse()
            # Confirm attachment from exact pose and to exact destination.
            if path and not line_free(p, path[0]):
                return []
            return path
        closed.add(current)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                q = (current[0]+dx, current[1]+dy)
                if (dx == 0 and dy == 0) or q in closed:
                    continue
                if not (0 <= q[0] < NX and 0 <= q[1] < NY):
                    continue
                if not line_free(point(current), point(q)):
                    continue
                new_cost = cost[current] + hypot(dx, dy)
                if new_cost < cost.get(q, 1e9):
                    cost[q], parent[q] = new_cost, current
                    if q not in opened:
                        opened.append(q)
    return []

def fail(message):
    global state
    stop()
    state = "FAULT"
    enes100.println("FAULT: " + message)

def start_scan(p):
    global state, scan_total, scan_last, scan_origin
    stop()
    state, scan_total, scan_last = "SCAN", 0.0, p[2]
    scan_origin = (p[0], p[1])
    enes100.println("Scanning with FRONT sensor")

def drive_to(p, target):
    error = wrap(math.atan2(target[1]-p[1], target[0]-p[0])-p[2])
    if abs(error) > .14:
        turn = max(24, min(95, abs(error)*90))
        if error < 0:
            turn = -turn
        wheels(-turn, turn)
    else:
        distance = hypot(target[0]-p[0], target[1]-p[1])
        speed = max(45, min(160, distance*550))
        correction = max(-45, min(45, error*180))
        wheels(speed-correction, speed+correction)

def setup():
    global state, hits, targets, stage, route, started, last_log
    global scan_total, scan_last, turn_base, turn_count, turn_wait_until
    global forward_goal, forward_start
    enes100.begin("Octonauts", WATER, 106, 1116)
    tank.begin()
    stop()
    hits, route, stage = [], [], 0
    state = "START"
    started, last_log = now_ms(), -10000
    scan_total, scan_last, turn_count, turn_wait_until = 0.0, 0.0, 0, 0
    p = pose()
    if p is None:
        fail("No initial pose; restore marker then restart")
        return
    mission = MISSION_POINT
    if mission is None:
        mission = (.55, 1.45 if p[1] < 1.0 else .55)
        enes100.println("DEMO mission approach; confirm actual Water Mission coordinates")
    targets = [mission, FINISH_POINT]
    if not all(.2 <= g[0] <= 3.2 and .2 <= g[1] <= 1.8 for g in targets):
        fail("Targets outside configured planner area")
        return
    turn_base = p[2]
    forward_start = (p[0], p[1])
    forward_goal = (3.15, p[1])
    enes100.println("MODE: " + MODE)
    if SEND_DEMO_MISSION:
        enes100.println("TEST PAYLOAD ONLY; not a sensor measurement")
        enes100.mission(DEPTH, DEMO_DEPTH_MM)
    if MODE == "NAVIGATION":
        if ENABLE_AVOIDANCE:
            start_scan(p)
        else:
            state = "PLAN"
    elif MODE not in ("TURN_TEST", "FORWARD_TEST", "WIRELESS_TEST", "SENSOR_TEST"):
        fail("Unknown MODE")

def step():
    global state, stage, route, scan_last, scan_total, last_log
    global turn_count, turn_wait_until
    if state in ("DONE", "FAULT"):
        stop()
        return
    if now_ms()-started > TIMEOUT_MS:
        fail("180 s timeout")
        return
    p = pose()
    if p is None:
        fail("Pose lost; restore marker then restart")
        return
    d = front()
    if now_ms()-last_log > 1500:
        enes100.println("Pose x=%.2f y=%.2f theta=%.2f front=%.2f" % (p[0], p[1], p[2], d))
        last_log = now_ms()
    if MODE in ("WIRELESS_TEST", "SENSOR_TEST"):
        stop()
        return
    if MODE == "TURN_TEST":
        if now_ms() < turn_wait_until:
            stop()
            return
        error = wrap(turn_base+(turn_count+1)*math.pi/2-p[2])
        if abs(error) < ANGLE_TOL:
            stop()
            turn_count += 1
            enes100.println("90-degree turn %d/3 complete" % turn_count)
            turn_wait_until = now_ms()+700
            if turn_count == 3:
                state = "DONE"
        else:
            turn = max(22, min(85, abs(error)*80))
            if error < 0:
                turn = -turn
            wheels(-turn, turn)
        return
    if MODE == "FORWARD_TEST":
        if hypot(p[0]-forward_goal[0], p[1]-forward_goal[1]) < POSITION_TOL:
            stop()
            state = "DONE"
            enes100.println("Forward demo complete; verify actual required travel distance")
        elif 0 <= d < .20:
            fail("Obstacle in forward test")
        else:
            drive_to(p, forward_goal)
        return
    if ENABLE_AVOIDANCE:
        remember(p, d)
    if state == "SCAN":
        scan_total += abs(wrap(p[2]-scan_last))
        scan_last = p[2]
        if scan_total >= 2*math.pi:
            stop()
            state = "PLAN"
        else:
            wheels(-110, 110)
        return
    goal = targets[stage]
    if hypot(goal[0]-p[0], goal[1]-p[1]) < POSITION_TOL:
        stop()
        if stage == 0:
            enes100.println("MISSION APPROACH REACHED; simulator does not measure water")
            stage = 1
            state = "PLAN"
        else:
            state = "DONE"
            enes100.println("DONE: stopped before log/limbo. Elapsed ms: %d" % (now_ms()-started))
        return
    if state == "PLAN":
        stop()
        route = plan(p, goal)
        if not route:
            fail("No known safe route; check goals, clearance and map")
        else:
            state = "DRIVE"
            enes100.println("Route planned from measured obstacle points: %d" % len(hits))
        return
    if ENABLE_AVOIDANCE and hypot(p[0]-scan_origin[0], p[1]-scan_origin[1]) > .30:
        start_scan(p)
        return
    if ENABLE_AVOIDANCE and route and not line_free(p, route[0]):
        stop()
        state = "PLAN"
        return
    while route and hypot(route[0][0]-p[0], route[0][1]-p[1]) < .055:
        route.pop(0)
    if not route:
        state = "PLAN"
        stop()
        return
    # Follow the farthest known-clear point within 45 cm to reduce grid zigzags.
    index = 0
    for i in range(1, len(route)):
        if hypot(route[i][0]-p[0], route[i][1]-p[1]) > .45:
            break
        if line_free(p, route[i]):
            index = i
    if index:
        route = route[index:]
    error = wrap(math.atan2(route[0][1]-p[1], route[0][0]-p[0])-p[2])
    # A close return while turning away must not trigger endless scans.
    # In-place turns are permitted only from the planner's clearance area.
    target_distance = hypot(route[0][0]-p[0], route[0][1]-p[1])
    if abs(error) <= .14 and 0 <= d < min(STOP_RANGE, target_distance+.04):
        stop()
        if ENABLE_AVOIDANCE:
            start_scan(p)
        else:
            fail("Obstacle; enable avoidance or remove obstacles")
        return
    drive_to(p, route[0])

def loop():
    try:
        step()
    except Exception as error:
        fail(str(error))

