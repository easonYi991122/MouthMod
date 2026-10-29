# MouthMod 头部 IMU 视角：交接文档

交接日期：2026-10-16（2026-10-28 更新）　|　固件 `MouthMod-ImuBench 0.8.0`　|　板子：Seeed XIAO nRF52840 Sense

本包最初是「头部倾角 → 鼠标视角」这一路的独立调试工程，现在已经接入摇杆、吹吸、咬合和 Minecraft Education 键鼠适配，并配有一个调试悬浮窗 app。
**新电脑从这里开始**：运行 `./scripts/setup-macos.sh`，或者让 Codex / Claude Code 按 [AGENTS.md](AGENTS.md) 部署；接线和日常使用见 [docs/USAGE.md](docs/USAGE.md)。
详细用法见 [README.md](README.md)；规格里没写死、本工程先暂定的实现选择见 [docs/imu-decisions.md](docs/imu-decisions.md)。

## 1. 当前状态

| 项目 | 状态 | 证据 / 说明 |
|---|---|---|
| 纯逻辑主机测试 | ✅ 通过 | `./scripts/test-native.sh`（macOS clang，`-Wall -Wextra -Werror`） |
| 固件编译（非 mbed core） | ✅ 通过 | 73.8 KB flash；我们自己的代码没有警告 |
| 烧录到实板 | ✅ 完成 | 用 UF2 拖拽烧录；USB 显示为 `Seeed / XIAO nRF52840 Sense` |
| IMU 读数 | ✅ 正常 | WHO_AM_I=0x6A；静止时 \|a\| ≈ 1.02 g；陀螺 Y 零偏约 −2.9 dps；约 100 个新样本/s |
| 串口 → 本地桥 → 测试页 | ✅ 跑通 | 页面实时显示姿态、陀螺数据、状态 |
| **倾头 → 视角动作** | ⚠️ **原因已找到，修复版待实板验证** | 校准静止阈值太严，见第 4 节；单文件版 `ImuLookMini` 已改为 3 dps |
| 校准、真实 HID 鼠标、停用路径、延迟 | ⬜ 未测 | 按 README 第 3 节逐项记录 |
| v0.8.0：加速度+角速度融合（力臂补偿、加速度拒绝）、倾角/角速度/混合控制、五点精细校准（D37–D39），开源方案调研 docs/imu-fusion-survey.md | ⚠️ 主机测试、固件编译通过（98.7 KB）；面板和桥用模拟板联调通过；**没接真板测试**；默认参数与 v0.7.0 相同 | 用法见 docs/USAGE.md §3.4 |
| v0.7.0：面板精简页 + 调整页、视角轴选择/取反/分轴灵敏度、摇杆分轴阈值、吹气模式、吸气跳跃、参数保存（D33–D36） | ⚠️ 主机测试、固件编译通过；面板和桥用模拟板联调通过；**没接真板测试** | 用法见 docs/USAGE.md |
| 悬浮窗 app（`build/MouthMod Float.app`，D32） | ✅ 2026-10-20 组内实测正常（先用模拟板验证了启动、加载、发命令，以及强杀后桥自动退出） | 用法见 docs/USAGE.md §3.2；备用方案仍是浏览器版 |
| 吹气改左键、吹气回弹不再误停用、上电默认打开真实输出（v0.6.1） | ⚠️ 主机测试通过，回弹问题来自实测日志；修复后还没有接真板复测 | 见 D29–D31 |
| 起转区间 6°+80 ms、只对 Minecraft 生效、悬浮窗（v0.6.0） | ⚠️ 主机测试通过；门控和悬浮窗在模拟板上跑通，**没接真板测试** | 用法见 docs/USAGE.md |
| 长吸启用、看门狗（v0.5.0） | ⚠️ 已写好、主机测试通过，**没有实测**；看门狗默认关闭 | decisions D25、D26 |
| 摇杆、吹吸移植（v0.4.0） | ⚠️ 已接入、主机测试通过，**没有接实物测** | 组员代码的问题和改动见 docs/minecraft-education.md §3 |
| Minecraft Education 适配（v0.3.0） | ⚠️ 已写好、主机测试通过，**未在游戏里测** | `src/logic/GameInput`，串口命令 `act/tap/turn`，见 [docs/minecraft-education.md](docs/minecraft-education.md) |
| 遥测频率修复 | ⚠️ 只在源码里 | 原设 50 Hz，实测约 38 Hz，已修复。修复版只在本包源码和 `build/ImuBench.uf2` 里；**板子上还是修复前的版本** |

## 2. 在另一台电脑上开始

### 2.1 只烧录（不需要装任何工具）
1. 用 USB-C 数据线连接板子，**快速双击** RESET 按钮。电脑上会出现一个叫 `XIAO-SENSE` 的盘。
2. 把 `ImuBench.uf2` 拖进这个盘。这个文件从 GitHub 最新 Release 下载；`build/` 不在仓库里，要本地编译才会有（见 2.2）。拷完后板子会自动重启，这个盘也会消失，这是正常的。
3. 先把盘里 `INFO_UF2.TXT` 的内容记下来（bootloader 版本）。交接指南要求记录这一项，目前还没记。

### 2.2 从源码编译
- **macOS 推荐**：在仓库根目录运行 `./scripts/setup-macos.sh`。它会检查并安装下面几项，然后跑测试、编译出 `build/ImuBench.uf2` 和悬浮窗 app；最后一行是 `SUMMARY fails=0` 就说明完成了。Apple Silicon 的 Mac 需要先装 Rosetta 2，脚本会提示。下面是手动步骤，供排查用。
- Arduino IDE 中添加板包网址 `https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json` → 安装 **Seeed nRF52 Boards 1.1.13**（不是带 mbed 的那个）。
- 安装库 **Seeed Arduino LSM6DS3 2.0.7**。**不要**另装 Adafruit TinyUSB，用板包自带的 1.7.0。
- 开发板选 *Seeed XIAO nRF52840 Sense*（不是 Plus），打开 `firmware/ImuBench/ImuBench.ino`。
- **macOS**：编译会报 `exec: "python"` 找不到。把 `tools/macos/platform.local.txt` 复制到 `~/Library/Arduino15/packages/Seeeduino/hardware/nrf52/1.1.13/`，然后重启 IDE。
- **Windows**：如果遇到同样的 `python` 报错，装好 Python 并确认 `python` 在 PATH 里。这一步还没在 Windows 上验证过。
- 自动上传失败（报 `No data received on serial port`）时，按 2.1 双击 RESET，然后选新出现的端口再上传，或者直接拖 UF2。

### 2.3 打开测试页
| 方式 | 适用系统 | 命令 / 操作 |
|---|---|---|
| 本地桥（推荐） | macOS / Linux | `python3 tools/look_demo/bridge.py`，然后打开 http://localhost:8765。任何浏览器都可以，页面会自动连上板子 |
| Web Serial | 任何系统的桌面版 Chrome / Edge | `python3 -m http.server 8765 --directory tools/look_demo`，打开 http://localhost:8765，点「连接板子 (Web Serial)」 |

> `bridge.py` 用到了 `termios`，**在 Windows 上不能运行**，Windows 请用 Web Serial 方式。
> 同一个串口一次只能被一个程序打开：Arduino 串口监视器、Chrome、bridge.py 三者只留一个。烧录前要先停掉桥。

## 3. 工作原理速览

```
LSM6DS3TR-C (Wire1 0x6A, 104 Hz)
  → ImuReader：查 STATUS_REG，有新样本才读，每次 I2C 都检查
  → MountFrame：传感器轴 → 头部坐标系（x 前、y 左、z 上）
  → ComplementaryFilter：α=0.98，得到 roll/pitch，不积分偏航
  → LookController：DISARMED → CALIBRATING(静止 3 s) → WAIT_NEUTRAL(回中 300 ms) → ACTIVE；FAULT 锁存
  → MouseLookMapper：死区 3° / 迟滞 2° / 饱和 20°，v = k·e（单位：计数/秒/度）
  → DeltaAccumulator：按真实 dt 换算成整数增量
  → HID 鼠标（需要 `hid 1` 才真正输出）或「虚拟增量」（通过串口发给页面，默认方式）
```

- 上电默认 DISARMED，**不会产生任何动作**。必须点「启用」，静止 3 s，再回中，才会进入 ACTIVE。
- 真实 USB 鼠标输出**默认关闭**。默认只有测试页里的 3D 视角会动，系统光标不会动。
- 停用方式：Esc / X、关闭页面、拔线、串口断开。IMU 故障会锁存为 FAULT，需要点「确认故障」才能恢复。

## 4. 倾头没有转成视角动作

**已定位的原因（2026-10-16 实板复现）：** 发送 `arm` 后，3 s 校准结束时报 `fail_motion`，状态退回 DISARMED，所以一直进不了 ACTIVE。
当时实测的陀螺标准差为 1.30 / 1.95 / 1.11 dps，而 `ImuBench` 暂定的上限是 1.0 dps（docs/imu-decisions.md D5）。板子拿在手上或戴在头上时，自然的微动和呼吸就会超过这个值。
- 单文件版 `firmware/ImuLookMini/ImuLookMini.ino` 已把上限改为 **3 dps**，失败时会打印实测标准差和上限。它已编译通过，**还没在实板上验证**。
- 完整版 `ImuBench` 从 v0.3.0 起也改为 3 dps（`src/logic/StillCalibration.cpp`，D5 已更新），同样还没在实板上验证。

修复后如果还是不动，按下面的清单逐项排查：

本轮按要求没有改代码。下次接手时请按顺序排查，把每一步看到的现象记下来：

1. **状态是不是 ACTIVE？** 页面左上角的状态标签必须是绿色「启用 ACTIVE」。DISARMED 下所有角度照常显示，但速度恒为 0，这是设计如此。
2. **校准有没有失败？** 点「启用」后看日志里的 `校准: status=...`：
   - `fail_motion` 表示 3 s 内没拿稳（陀螺标准差超过上限，v0.3.0 起为 3 dps）。拿在手里很容易失败，请把板子放在桌上，或者戴好后保持静止。
   - `fail_accel` 表示加速度模长不在 0.9–1.1 g 之间。
3. **是不是卡在 WAIT_NEUTRAL？** 校准结束后，头要回到校准时的姿势（两轴误差都小于 3°）并保持 300 ms，才会进入 ACTIVE。
4. **倾斜角度够不够？** 小于 3° 不会动。ACTIVE 时右下方曲线图里的 vx/vy 应该随倾斜变成非零，左下方十字图中的圆点会变绿。
5. **视角输入源对不对？** 「视角输入源」选「自动」或「固件串口增量」时，3D 画面靠串口传来的 `dx/dy` 转动。如果选成「真实鼠标」而 HID 又没开，画面就不会动。
6. **指望系统光标会动？** 需要先点击 3D 画面锁定指针，再勾「真实 USB 鼠标输出」（相当于发 `hid 1`）。
7. **安装方向对不对？** 如果倾斜后动的是另一根轴，或者方向反了，先按 README 第 3 节 c 步重新设 mount，最后才考虑用「反转」。
8. 以上都正常还是不动：用「开始记录 CSV」录一段「启用 → 倾头」的过程。看 `D,` 行里的 `state, look_x_err, look_y_err, vx_cps, vy_cps, dx, dy` 这几列，就能判断是固件没算出速度，还是页面没有显示。

## 5. 文件清单

| 路径 | 内容 |
|---|---|
| `firmware/ImuLookMini/` | 单文件精简版控制代码（约 350 行）：同样的状态机、滤波和映射，没有可切换的迟滞/速度模式，也没有测试；测试页可以直接连它 |
| `firmware/ImuBench/` | Arduino sketch。`src/logic/` 是纯 C++，不依赖 Arduino，之后可以原样并入完整的 MouthMod 固件 |
| `firmware/ImuBench/src/Config.h` | 引脚和默认参数，注明了哪些是规格冻结值、哪些是暂定值 |
| `tests/native/`、`scripts/test-native.sh` | 逻辑的主机端测试，共 18 组 |
| `tools/look_demo/index.html` | 测试页：3D 体素场景、调参滑条、图表、瞄准测试、CSV 记录、Config.h 片段导出、模拟模式 |
| `tools/look_demo/bridge.py` | 串口 → 浏览器本地桥（只用标准库） |
| `tools/macos/platform.local.txt` | macOS 编译时 `python` 报错的修复 |
| `build/ImuBench.uf2` | 最新固件，可直接拖拽烧录 |
| `docs/imu-decisions.md` | D1–D15：待组内确认的实现选择 |
| `README.md` | 安装、台架步骤表、串口协议、安全设计、验证状态 |

## 6. 版本锁定

Arduino IDE 2.3.10（自带 arduino-cli 1.5.1）· Seeeduino:nrf52 1.1.13 · Seeed Arduino LSM6DS3 2.0.7 · Adafruit TinyUSB 1.7.0（板包自带）· FQBN `Seeeduino:nrf52:xiaonRF52840Sense` · bootloader 版本：**未记录**

## 7. 下一步

1. 解决第 4 节的问题，然后完成 README 第 3 节 c–i 步，记录实测值：安装方向、静止噪声、每人的 k、真实 HID 到达率。
2. 组内确认 `docs/imu-decisions.md` 里的 D1（迟滞公式）和 D2（速度曲线）。
3. Oct 23–25 和压力组合并：把 `src/logic/` 并入 `firmware/MouthMod/`；用 SafetyStateMachine 的长吸启用和 D6 逻辑，替换调试版的 `arm`/`ack` 串口命令；开启 500 ms 看门狗；在 Minecraft Education 里确定灵敏度并记录。
