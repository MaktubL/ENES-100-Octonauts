ENES-100 Octonauts — UNO R4 WiFi / Water Mission

This project corresponds to your nine-part stage assessment and includes two independent modes: **obstacle-free navigation** and **obstacle navigation**. Comments in the code, such as `[CONFIG]`, `[INTEGRATION]`, `[MOTORS]`, `[REAL SENSOR]`, and `[NAVIGATION CORE]`, explain the reasons for modifications and hardware assumptions.

What to Open in VS Code
- `src/main.cpp`: The main program compiled by PlatformIO.
- `Config.h`: Your pin assignments, calibration, vehicle dimensions, route, and hardware verification switches.
- `include/Navigator.h`: A* planning with a fixed memory limit and obstacle surface point logging.
- `simulator/main.py`: Copy this to your specified MicroPython website to run; it cannot be uploaded to the UNO R4.
- `docs/Assessment_CN.md`: Nine requirements, on-site procedures, and items that still require physical verification.
- `test/test_navigator.cpp`, `test/test_simulator.py`: Navigation core and offline scenario testing.

The original Aruco Marker, Color Sensor Code, and Depth Sensor Code are retained for team members’ reference. The original `Main Code` did not have a `.cpp` extension and was not located in the `src` directory, so PlatformIO would not automatically compile it into the main program; now, it is uniformly built from `src/main.cpp`.

Test navigation in the simulator first
1. Open `simulator/main.py` and copy all content to `https://enes100.umd.edu/tools/micropythonsimulator/`.
2. Obstacle-free test: `MODE=“NAVIGATION”`, `ENABLE_AVOIDANCE=False`; click Randomize → Remove Obstacles → Run.
3. Obstacle-included test: `MODE=“NAVIGATION”`, `ENABLE_AVOIDANCE=True`; click Randomize → Run.
4. Turning: `MODE=“TURN_TEST”`; observe the 90° arrival information at 1/3, 2/3, and 3/3.
5. Wireless Interface Test: `MODE=“WIRELESS_TEST”`; set `SEND_DEMO_MISSION=True` only when you need to send a test message.

The `enes100`, `tank`, and mission constants are already built into the webpage; do not copy Arduino functions such as `digitalWrite()` into MicroPython. Keep the web page visible; runs where the backend timer continues but the screen is paused must be retested and cannot be counted as passes.

Mission points are still to be confirmed: The current web page source code does not include the Water Mission coordinates. `MISSION_POINT=None` selects only the demonstration approach point opposite the starting point; the log will provide a clear prompt. Replace this with `(x,y)` once the official locations are known. The default finish point is `(3.15,1.45)`, located ahead of the x≈3.4 log/limbo mark drawn on the webpage. Do not treat the demonstration proximity point as a confirmed official location.

Real-world Use
After building on PlatformIO, upload to the UNO R4 WiFi; set the serial port to 9600 baud, and ensure commands end with a line break. The robot will not move automatically upon power-up. Press `q` to request a stop without waiting for a line feed; the physical kill switch is still used to cut off the drive power.

| Command | Function |
|---|---|
| `h` | Display menu |
| `d` | ArUco, front, and downward ultrasonic diagnostics |
| `w` | Periodically reads and displays X/Y/theta, while outputting to the Vision System |
| `x` | Explicitly labeled 25 mm test task message; not actual water depth |
| `f` | Navigate to the X-coordinate of pre-limbo; record actual displacement and elapsed time |
| `t` | Perform three consecutive 90° turns relative to the initial heading |
| `n` | Obstacle-free: Current position → Confirmed task approach point → Before limbo/log |
| `a` | Obstacle present: Scan with forward sensor, record surface, plan a detour, and complete the same route |
| `k 200` | Take 10 measurements at the known 200 mm mark; output error and valid reading rate |
| `c` | Output raw R/G/B/clear frequencies and LDR values for task perception calibration |
| `m` | Seven water depth measurements; submit task results only after at least five valid and stable readings, with calibration confirmed |
| `p` | Pump power-on control test with an upper limit; default 1500 ms |
| `q` | Stop motors and pump, return to IDLE |

Current default settings
`MOTOR_WIRING_CONFIRMED=false`, `OTV_GEOMETRY_CONFIRMED=false`, `PUMP_WIRING_CONFIRMED=false`, `FRONT_CALIBRATED=false`, `DEPTH_CALIBRATED=false`, `MISSION_POINT_CONFIRMED=false`.
These default values indicate facts that require verification; do not set them all to `true` just to make the program run. You do not need to fill in the wiring from memory at this stage; you can verify each item on-site. You may proceed with stationary readings, wireless interface setup, color calibration, and water depth sampling first. The system will provide prompts regarding any unconfirmed configurations during autonomous vehicle movement, pump operation, and formal measurement submission.
On a standard L298N, IN3/IN4 control the direction of the same motor, while ENB controls its speed. The missing IN1/IN2 pins are explicitly reserved with a value of -1. After confirming the actual board layout, fill in the direction pins, left/right channels, and forward/reverse polarity; check the ENA/ENB jumpers and common ground to avoid assigning the same pin to two components.
Confirm whether the water pump is controlled by the D13 relay or the external driver on D10, verify the active levels, and then enable `PUMP_WIRING_CONFIRMED`. The default input state does not guarantee that the external relay is off; disconnect the water pump load before performing stationary diagnostics. The program will not drive the motor or water pump by connecting them directly to the Arduino’s power supply pins.

Regarding 3.4 m
The **actual travel distance of 3.4 m** specified in the requirements is not the same as the **x=3.4 m** coordinate on the map. The default starting point on the webpage is x=0.55; traveling to x=3.15 is only approximately 2.60 m. `f` will accurately print the planned and actual displacements and indicate that the distance is less than 3.4 m; this cannot be used to claim that the first requirement has been met. The first requirement allows for obstacle removal and manual vehicle assistance; proof must be provided using a ruler and stopwatch over the actual distance specified by the instructor.

Verification and Limitations
In the previous phase, the default web-based scenario’s full navigation reached approximately (3.102, 1.415), with a log time of 118,303 ms. The original Python controller passed offline verification for 30 random obstacle scenarios, 4 obstacle-free scenarios, and 4 triple-turn scenarios; the offline model does not include random rumble disturbances, real ultrasonic blind spots, wheel slippage, or network latency.
The newly added C++ navigation core passed verification on a standalone host, covering: angle ranges of ±π, open-field routes, invalid targets, obstacle avoidance, segment spacing, deduplication, wall filtering, and map capacity limits. The navigation core object is approximately 6.2 KB; additional memory requirements for WiFi/RTOS depend on the actual compilation and runtime results on the UNO R4.
The following tests have not yet been completed: real-vehicle upload, actual 3.4 m driving, physical calibration, and pump water intake testing; the code and simulator cannot replace these demonstrations. For more test evidence, see `docs/Verification_CN.md`.