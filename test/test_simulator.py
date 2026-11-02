import math, sys, types, random, json
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / 'simulator' / 'main.py').read_text(encoding='utf-8-sig')

def run(seed, mode='NAVIGATION', obstacles=True):
    rng = random.Random(seed)
    high = rng.randrange(2)
    x, y, angle = .55, .55+.9*high, rng.uniform(-math.pi, math.pi)
    boxes = []
    # Same size/placement constraints as official randomize(), no map supplied
    # to student code. Include both solid obstacles; omit rumble perturbation.
    if obstacles:
        col = rng.randrange(2) + (0 if high else 1)
        cols = [c for c in range(3) if c != col]
        rumble_row, r1 = rng.randrange(2), rng.randrange(2)
        r2 = 1-r1 if r1 == rumble_row else rng.randrange(2)
        for c, r in zip(cols, [r1, r2]):
            boxes.append((1.4+.8*r, .25+.5*c, .2, .5))
    left = right = clock = 0
    logs, collisions = [], 0
    class Enes:
        begin = lambda *args: None
        is_connected = lambda *args: True
        is_visible = lambda *args: True
        get_x = lambda *args: x
        get_y = lambda *args: y
        get_theta = lambda *args: angle
        println = lambda _, s: logs.append(str(s))
        mission = lambda *args: None
    class Tank:
        begin = lambda *args: None
        def turn_off_motors(self):
            nonlocal left, right
            left = right = 0
        def set_left_motor_pwm(self, value):
            nonlocal left
            left = value/255
        def set_right_motor_pwm(self, value):
            nonlocal right
            right = value/255
        def read_distance_sensor(self, _):
            sx, sy = x+.075*math.cos(angle), y+.075*math.sin(angle)
            vx, vy = math.cos(angle), math.sin(angle)
            ranges = []
            for bx, by, bw, bh in [(0, 0, 4, 2)]+boxes:
                for wallx in (bx, bx+bw):
                    if abs(vx) > 1e-8:
                        t = (wallx-sx)/vx
                        if t >= 0 and by-1e-8 <= sy+t*vy <= by+bh+1e-8:
                            ranges.append(t)
                for wally in (by, by+bh):
                    if abs(vy) > 1e-8:
                        t = (wally-sy)/vy
                        if t >= 0 and bx-1e-8 <= sx+t*vx <= bx+bw+1e-8:
                            ranges.append(t)
            nearest = min(ranges, default=1)
            return nearest if nearest < 1 else -1
    sys.modules['js'] = types.SimpleNamespace(ArduinoAPI=types.SimpleNamespace(millis=lambda: clock))
    ns = {'enes100': Enes(), 'tank': Tank(), 'WATER': 4, 'DEPTH': 0}
    exec(compile(SOURCE, 'simulator_main.py', 'exec'), ns)
    ns['MODE'] = mode
    if not obstacles:
        ns['ENABLE_AVOIDANCE'] = False
    ns['setup']()
    for frame in range(1801):
        ns['loop']()
        for _ in range(6):
            dt = .1/6
            angle = math.atan2(math.sin(angle+(right-left)*dt), math.cos(angle+(right-left)*dt))
            x += math.cos(angle)*(left+right)/4*dt
            y += math.sin(angle)*(left+right)/4*dt
            # Conservative circular body check (official body half-diagonal .096m).
            hit = not (.096 <= x <= 3.904 and .096 <= y <= 1.904)
            for bx, by, bw, bh in boxes:
                cx, cy = max(bx, min(bx+bw, x)), max(by, min(by+bh, y))
                hit |= math.hypot(x-cx, y-cy) < .096
            if hit:
                collisions += 1
        clock += 100
        if ns['state'] in ('DONE', 'FAULT'):
            break
    return dict(seed=seed, mode=mode, state=ns['state'], seconds=clock/1000,
                collisions=collisions, pose=[round(x,3),round(y,3)], last=logs[-1], logs=logs)

results = [run(i) for i in range(30)]
results += [run(i, obstacles=False) for i in range(4)]
results += [run(i, mode='TURN_TEST', obstacles=False) for i in range(4)]
result_dir = ROOT / '.pio' / 'test-results'
result_dir.mkdir(parents=True, exist_ok=True)
(result_dir / 'simulator_results.json').write_text(json.dumps(results, indent=2))
for r in results:
    print({k:v for k,v in r.items() if k != 'logs'})


