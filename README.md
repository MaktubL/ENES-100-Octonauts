# ENES-100 Octonauts — UNO R4 WiFi / Water Mission

本工程对应你们的九项阶段考核，包含 **无障碍导航** 和 **有障碍导航** 两个独立模式。代码中的 `[CONFIG]`、`[INTEGRATION]`、`[MOTORS]`、`[REAL SENSOR]`、`[NAVIGATION CORE]` 等批注解释了修改原因与硬件假设。

## 在 VS Code 中打开什么

- `src/main.cpp`：实际由 PlatformIO 编译的整合主程序。
- `Config.h`：你们的引脚、校准、车身尺寸、路线和硬件确认开关。
- `include/Navigator.h`：有固定内存上限的 A* 规划与障碍表面点记录。
- `simulator/main.py`：复制到你指定的 MicroPython 网站运行；不能上传到 UNO R4。
- `docs/Assessment_CN.md`：九项要求、现场步骤、哪些仍需实物验证。
- `test/test_navigator.cpp`、`test/test_simulator.py`：导航核心和离线场景检查。

原有 Aruco Marker、Color Sensor Code、Depth Sensor Code 保留作组员参考。原来的 `Main Code` 没有 `.cpp` 扩展名，也不在 `src` 目录，PlatformIO 不会自动把它编译成主程序；现在统一从 `src/main.cpp` 构建。

## 先在模拟器测试导航

1. 打开 `simulator/main.py`，复制全部内容到 https://enes100.umd.edu/tools/micropythonsimulator/。
2. 无障碍考核：`MODE="NAVIGATION"`，`ENABLE_AVOIDANCE=False`；点击 Randomize → Remove Obstacles → Run。
3. 有障碍考核：`MODE="NAVIGATION"`，`ENABLE_AVOIDANCE=True`；点击 Randomize → Run。
4. 转弯：`MODE="TURN_TEST"`；观察 1/3、2/3、3/3 的 90° 到达信息。
5. 无线接口流程：`MODE="WIRELESS_TEST"`；需要发送测试消息时才设 `SEND_DEMO_MISSION=True`。

网页的 `enes100`、`tank` 与任务常量已经内置，不要把 Arduino 的 `digitalWrite()` 等函数粘贴到 MicroPython 中。保持网页可见；后端计时继续而画面暂停的运行应重测，不能记为通过。

**任务点尚待确认**：当前网页源码没有 Water Mission 坐标。`MISSION_POINT=None` 只选择起点对面的演示接近点，日志会明确提示。知道正式位置后替换为 `(x,y)`。终点默认 `(3.15,1.45)`，在网页绘制的 x≈3.4 log/limbo 前方。不得把演示接近点当作已经确认的正式站点。

## 实车使用

PlatformIO Build 后上传到 UNO R4 WiFi；串口设 9600 baud，命令以换行结束。上电不会自动移动。`q` 不需要等待换行，即可请求停止；实体 kill switch 仍用于切断驱动电源。

| 命令 | 功能 |
|---|---|
| `h` | 显示菜单 |
| `d` | ArUco、前方与向下超声波诊断 |
| `w` | 周期性读取并显示 X/Y/theta，同时输出到 Vision System |
| `x` | 明确标注的 25 mm 测试任务消息，不是实际水深 |
| `f` | 导航至 pre-limbo 的 X 坐标，记录实际位移与耗时 |
| `t` | 从初始 heading 起连续完成三个相对 90° 转弯 |
| `n` | 无障碍：当前位置 → 已确认任务接近点 → limbo/log 前 |
| `a` | 有障碍：前方探头扫描、记录表面、规划绕行并完成同一路线 |
| `k 200` | 已知 200 mm 处测 10 次，输出误差和有效读数率 |
| `c` | 输出 R/G/B/clear 原始频率和 LDR，供任务感知校准 |
| `m` | 七次水深测量；至少五次有效且稳定、校准已确认才提交任务结果 |
| `p` | 有上限的水泵通电控制测试，默认 1500 ms |
| `q` | 停止电机、水泵，回到 IDLE |

### 当前默认开关

`MOTOR_WIRING_CONFIRMED=false`、`OTV_GEOMETRY_CONFIRMED=false`、`PUMP_WIRING_CONFIRMED=false`、`FRONT_CALIBRATED=false`、`DEPTH_CALIBRATED=false`、`MISSION_POINT_CONFIRMED=false`。

这些默认值表示待验证的事实，不能为了让程序运行就全部改成 true。现在不需要你凭记忆补充接线；到了现场逐项核实即可。静止读数、无线接口、颜色和水深采样可以先进行。实车自主移动、水泵和正式测量提交会对相应未确认配置给出提示。

标准 L298N 的 IN3/IN4 控制同一电机方向，ENB 控制它的速度。缺失的 IN1/IN2 用 -1 明确保留。确认实际板型后填写两个方向引脚、左右通道和正反转极性；检查 ENA/ENB 跳线和共地，避免把同一引脚分配给两个部件。

确认水泵是由 D13 继电器控制，还是 D10 的外部驱动器控制，并核对有效电平，再启用 `PUMP_WIRING_CONFIRMED`。默认输入状态不保证外部继电器断开；静止诊断时先断开水泵负载。程序不会把电机或水泵直接接到 Arduino 供电引脚上驱动。

### 关于 3.4 m

要求中的 **实际行驶 3.4 m** 与地图的 **x=3.4 m** 不是同一个量。网页默认起点 x=.55，行驶到 x=3.15 只有约 2.60 m。`f` 会如实打印计划和实际位移，并提示不足 3.4 m；不能以此声称已经通过第一项。第一项允许移除障碍、人工接车，应在老师规定的实车距离上用尺子和计时器完成证明。

## 验证与限制

原阶段的网页默认场景完整导航曾到达约 (3.102,1.415)，日志耗时 118303 ms。原 Python 控制器的 30 个随机障碍场景、4 个无障碍场景和4个三次转弯场景通过离线检查；离线模型不含 rumble 随机扰动、真实超声波盲区、车轮打滑及网络延迟。

本次新增的 C++ 导航核心通过独立主机检查：角度跨越 ±π、空场地路线、无效目标、障碍绕行、路段间距、去重、墙面过滤、地图容量上限。导航核心对象约 6.2 KB；WiFi/RTOS 的额外内存需以实际 UNO R4 编译和运行结果为准。

还没有完成实车上传、实际 3.4 m 行驶、实物校准、泵取水量测试；代码与模拟器不能替代这些演示。更多测试证据见 `docs/Verification_CN.md`。
