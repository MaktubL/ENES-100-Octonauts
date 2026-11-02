# 验证记录与实际限制

## 已验证

- C++ `Navigator.h`：主机 GCC 严格警告模式编译并运行通过。检查角度跨越 ±π、空场地路线、目标越界、障碍绕行、逐段安全余量、障碍点去重、墙面点过滤和地图容量上限。对象大小 6212 bytes。
- 实际 `src/main.cpp`：通过 `test/test_controller.cpp` 以假串口/假引脚/假 Vision System 适配器直接执行。验证上电 IDLE、电机和泵的未确认接线拦截、测试 mission 消息、未标定水深不提交、无效回波/定位拒绝、无需换行的 q、命令溢出丢弃以及每轮串口输入处理上限。
- `simulator/main.py`：30 个随机有障碍场景、4 个关闭避障的无障碍场景、4 个连续三次 90° 转弯场景全部完成；本次离线检查未记录保守圆形车身接触。无障碍关闭扫描后约 19–26 s，有障碍扫描路线约 103–137 s；这些是离线模型时间，不是实车成绩。
- 同一 Python 模拟器程序此前在官网默认障碍布局完整运行一次，到约 (3.102,1.415) 停车，记录 118303 ms。后续网页随机运行曾遇动画停止、控制台继续计时并超时，没有计为通过。

## ARM 编译检查的范围

本机 PlatformIO 标准包的 ARM GCC 7 可执行程序无法启动，Windows 报“无效的系统 DLL 重定位”。因此不能把当前工程说成已经通过默认工具链的完整 Build。

为检查 Arduino API 与头文件，使用独立工作副本中的 ARM GCC 15.2，`src/main.cpp` 已生成 UNO R4 的目标文件。新版编译器还要求对工作副本的框架做 `::byte` 消歧义及旧 C 返回值诊断兼容处理；最终链接又因该测试工具链缺少所需 C++ 支持库失败。**这些框架改动、编译器替换和测试配置都没有写入你的项目或全局 PlatformIO 安装，也没有交付固件二进制。**

交付工程保持官方 Renesas RA 1.9.0、ENES100 v6.2.4、ArduinoHttpClient 0.6.2。你的正常 VS Code 环境仍需完成完整 Build；如果出现同一 Windows DLL 启动错误，需要修复或重装对应 ARM 编译器，而不是绕过接线确认开关。

## 尚须实物完成

没有执行上传、实车移动、实际 WiFi 通信演示、传感器标定、泵取水或被动采集演示。各默认确认开关保持 false；有限程序检查不构成九项实物考核全部通过。

本次没有执行 Git commit/push。

## 可重复检查

安装常规 Python 3 后，在项目目录运行 `python test/test_simulator.py`，结果保存在 `.pio/test-results/simulator_results.json`。该模型不包含 rumble 随机扰动、真实声学反射、网络延迟和车轮打滑。

有主机 C++ 编译器时，可用 C++11 编译 `test/test_navigator.cpp`，包含路径为 `include`；C++17 编译 `test/test_controller.cpp`，包含路径为 `test/stubs` 和 `include`。不要把测试 stubs 用于实车构建。
