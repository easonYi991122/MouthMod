# MouthMod：头部 IMU 视角调试（ImuBench）

本目录是 MouthMod 固件中「头部倾角 → 鼠标视角」这一路的独立调试工程，配套一个浏览器测试页。
吹吸、咬合、摇杆不在这里；这里的 `src/logic/` 模块写成了不依赖 Arduino 的纯 C++，之后可以原样搬进完整的 `firmware/MouthMod/`。

```text
MouthMod/
  firmware/ImuLookMini/         单文件精简版（一个 .ino，不含测试；遥测格式与测试页兼容）
  firmware/ImuBench/            Arduino sketch（文件夹名 = .ino 名）
    ImuBench.ino                只转发 setup()/loop()
    src/Config.h                引脚、默认参数（标明哪些是规格冻结值、哪些是暂定值）
    src/App.cpp                 100 Hz 非阻塞主循环、串口命令、CSV 遥测、HID 调度
    src/hardware/ImuReader.*    板载 LSM6DS3TR-C（内部 Wire1, 0x6A），轮询 + 每次 I2C 都检查
    src/hardware/HidOutput.*    TinyUSB 键盘+鼠标复合 HID，release-all 优先
    src/logic/                  纯逻辑：安装方向、互补滤波、静止校准、视角映射、状态机
    src/logic/FineCalibration.* 五点精细校准：只求左右/上下转轴（D39、D40）
    src/logic/HeadDeviation.*   中立位起的头部偏离量，不受安装方向影响（D40）
    src/logic/GameInput.*       Minecraft Education 适配：各通道电平 → 完整键盘状态 + 鼠标报告
    src/logic/JoystickMapper.*  摇杆 → W/A/S/D（移植自组员代码，按规格归一化与迟滞）
    src/logic/PressureChannel.* HX710B 压力 → 吹/吸/咬（移植自组员代码，带符号、基线跟踪）
    src/hardware/Hx710b.*       HX710B 非阻塞读取（27 脉冲 = 40 Hz）
  tests/native/                 逻辑的主机端测试
  scripts/test-native.sh        编译并运行主机端测试
  scripts/setup-macos.sh        新 Mac 一键部署和自检（板包、库、修复、测试、编译固件和悬浮窗 app）
  scripts/build-firmware.sh     编译固件，生成 build/ImuBench.uf2
  scripts/build-float-app.sh    生成 build/MouthMod Float.app（悬浮窗 app）
  tools/float_app/              悬浮窗 app 的 Swift 源码：置顶、不抢焦点，内部启动 bridge.py 并显示 float.html
  tools/look_demo/index.html    浏览器测试页（3D 体素场景、调参、图表）
  tools/look_demo/bridge.py     串口→浏览器本地桥（标准库，无需安装；macOS 上检测 Minecraft 是否在前台）
  tools/look_demo/float.html    调试悬浮窗页面（悬浮窗 app 里显示它；备用方案是在 Chrome/Edge 里用画中画置顶）
  tools/macos/platform.local.txt  macOS 编译修复（见下）
  docs/imu-decisions.md         规格没写死、本工程暂定的实现选择
  docs/minecraft-education.md   为什么保持键鼠、按键映射、合并接口、今晚实机测试步骤
  docs/imu-fusion-survey.md     六轴融合、头鼠、力臂补偿的开源方案调研（v0.8.0）
```

## 要不要现在装 Minecraft？

> **接线与使用说明：[docs/USAGE.md](docs/USAGE.md)**。悬浮窗：双击 `build/MouthMod Float.app`（备用：桥运行后在 Chrome 里打开 `http://localhost:8765/float.html`）。
>
> **新 Mac 部署**：在仓库根目录运行 `./scripts/setup-macos.sh`。它会检查并安装板包、库和 macOS 修复，然后跑测试、编译出 `build/ImuBench.uf2`；不能自动处理的项会标成 `[FAIL]`，并给出要执行的命令。用 Codex 或 Claude Code 部署时，让它按根目录的 [AGENTS.md](AGENTS.md) 来做。
>
> 2026-10-19 更新：已经进入实机阶段。键鼠还是手柄、游戏设置、合并接口和测试表，见 [docs/minecraft-education.md](docs/minecraft-education.md)。

调 IMU 这一阶段不需要。测试页能覆盖这阶段的检查项：

- **方向和符号**：右倾→右转、低头→向下看，安装方向是否正确。
- **死区、迟滞、饱和、k**：拖滑条后实时下发到板子，不用重新烧录。
- **回中停转、无漂移**：图表同时显示 roll/pitch 误差、速度、事件。
- **真实 USB 鼠标通路**：在 Pointer Lock 下接收板子发出的 HID 移动，对比「固件发出的计数 / 浏览器收到的计数」。
- **手感**：按 Minecraft Java 的灵敏度公式把计数换算成角度，内置 10 次瞄准测试，用来比较不同 k。

需要真实游戏的只有两件事，留到计划中的 Oct 23–25 联调阶段做：
1. 提案写的是 **Minecraft Education**（基于 Bedrock）。它的灵敏度公式、是否使用原始输入，都和 Java 版不同。测试页里的换算只用来做相对比较，最终灵敏度要在游戏里定，然后记录下来。
2. 和键盘通道（W/A/S/D、Space、左键）同时工作的组合演示。

## 1. 安装（Arduino IDE，macOS）

1. Arduino IDE → Settings → *Additional boards manager URLs* 加入
   `https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json`
2. Boards Manager 安装 **Seeed nRF52 Boards 1.1.13**。注意不要装名字里带 *mbed* 的那个。
3. Library Manager 安装 **Seeed Arduino LSM6DS3 2.0.7**。
   **不要**另外装 Adafruit TinyUSB Library。板包自带的 1.7.0 就是已验证能编译的组合；另装一份 3.x 会和自带的冲突。
4. **macOS 必做**：板包的 platform.txt 里调用的是 `python`，而 macOS 只有 `python3`，不处理的话编译到最后一步会报 `exec: "python": executable file not found`。
   把 `tools/macos/platform.local.txt` 复制到
   `~/Library/Arduino15/packages/Seeeduino/hardware/nrf52/1.1.13/`，然后重启 IDE。
   **Apple Silicon 的 Mac 还需要 Rosetta 2**：板包里的编译器和上传工具都是 x86 程序。没装的话，编译会报 `bad CPU type in executable`。装法：`softwareupdate --install-rosetta --agree-to-license`。
5. 开发板选 *Seeed nRF52 Boards → Seeed XIAO nRF52840 Sense*（不是 Sense Plus）。端口选 `/dev/cu.usbmodem…`。
6. 打开 `firmware/ImuBench/ImuBench.ino` 并上传。如果上传卡住，快速双击板上的 RESET 进入 bootloader，再上传一次。

> 已验证：用 arduino-cli 1.5.1 + Seeeduino:nrf52@1.1.13 + LSM6DS3 2.0.7 + 自带 TinyUSB 1.7.0 编译通过，占用 73.8 KB flash，我们自己的代码没有警告。已在实板上运行（见第 7 节）。

## 2. 打开测试页

**推荐：用本地桥（任何浏览器都能用，包括 Safari 和 app 内置浏览器）。** 只依赖 Python 标准库，不用另外安装任何东西：

```bash
python3 tools/look_demo/bridge.py   # 在仓库根目录运行
```

然后打开 <http://localhost:8765>，页面会自动连上板子。`bridge.py` 会自动找 `/dev/cu.usbmodem*`；板子拔掉或复位后会自动重连。
所有页面都关掉时，以及按 Ctrl+C 停掉桥时，桥都会给板子发 `disarm`；真实输出的开关保持不变（D30）。

**备选：Web Serial 直连。** 只能用**桌面版 Chrome 或 Edge**，页面也要通过 localhost 打开（可以直接用上面桥的地址，点「连接板子 (Web Serial)」）。app 内置浏览器不会弹出端口选择框，所以这种方式在 app 里用不了。

常见问题：
- 一个串口同一时间只能被一方打开。Arduino IDE 的串口监视器/绘图器、Chrome、`bridge.py` 三者只能留一个，否则会提示「端口被占用」。上传固件之前要先停掉桥。
- 页面看起来还是旧版本（比如没有「连接板子（本地桥）」按钮）：强制刷新（Cmd+Shift+R）。
- 3D 视图要从 cdnjs 加载 three.js，所以需要联网；断网时其余功能照常可用。
- 没接板子时，点「模拟模式」可以先熟悉界面：在十字图上拖动就相当于转头。

## 3. 台架调试步骤

| 步骤 | 操作 | 合格标准（实测后记录） |
|---|---|---|
| a. 裸板 | 插 USB，点「连接板子」 | 日志出现 `#I … imu_ok=1, whoami=0x6A`；IMU 新样本约 100/s；样本年龄 ≤ 10 ms |
| b. 静止读数 | 板子芯片面朝上平放，mount 保持默认 `+Z / +X` | 加速度约为 `0 / 0 / +1.00`，模长 1.00±0.03；陀螺 \|值\| 只有几 dps（这时还没去零偏） |
| c. 定安装方向 | 先固定在头带上，mount 设为 `+Z/+X`，这时显示的就是传感器原始轴。坐正后，读数约 +1 g 的那根轴是「朝上」；然后低头，读数变成负的那根轴是「朝前」。设好后点「应用」 | 设好后，右倾时 roll 为正，低头时 pitch 为正 |
| d. 启用 | 点「启用」，静止 3 s，然后头回中 | 状态依次为 CALIBRATING → WAIT_NEUTRAL → ACTIVE；`#C` 中 gyro_std 小于 1 dps |
| e. 方向 | 虚拟模式下（不勾「真实 USB 鼠标」）倾头，看 3D 视图 | 右倾右转、左倾左转、低头下看、抬头上看；回中后停住。方向反了先检查 mount，最后才用「反转」 |
| f. 调参 | 调 kx/ky、死区、迟滞模式、速度曲线；做瞄准测试 | 小倾角慢转，回中停转，静坐不漂；记下每个人的 k |
| g. 真实 HID | 点 3D 画面锁定指针 → 勾「真实 USB 鼠标输出」 | 「HID 实发到达率」接近 100%（前提是「指针原始输入=是」，否则系统鼠标加速会让这个比例偏离） |
| h. 安全 | Esc / X / 关页面 / 拔线 / 断开串口 | 每种情况都立即停用、视角停止；FAULT 需要点「确认故障」才回到 DISARMED |
| i. 记录 | 「开始记录 CSV」→ 做动作 →「停止并下载」 | 文件里有 `#I/#P/#C` 元数据行和 `D,` 数据行 |

调好之后点「生成 Config.h 片段」，把结果贴回 `src/Config.h`。参数只保存在浏览器 localStorage 和本次运行的固件里，板子断电后会恢复成 Config.h 里的默认值。

## 4. 串口协议（USB CDC，换行结尾）

| 命令 | 作用 |
|---|---|
| `arm` | DISARMED → 开始 3 s 静止校准 → 回中 300 ms → ACTIVE（替代长吸；如果打开了 D6，还需要 D6 闭合） |
| `disarm` / `x` | 停用，释放全部 |
| `ack` | 确认 FAULT → DISARMED（替代 D6 拨动） |
| `hid 0\|1` | 关闭/开启真实 USB 键鼠输出（v0.6.1 起上电默认开启，见 D30） |
| `set <k> <v>` | `kx ky dead hyst sat alpha hmode vmode invx invy rate` |
| `mount <up> <fwd>` | 例：`mount -Y +Z`；修改后自动停用，需要重新校准 |
| `get` | 打印 `#I` 身份、`#P` 参数、`#C` 校准结果、`#H` 列名 |
| `act <w\|a\|s\|d\|jump\|use> <0\|1\|ms>` | 仅 ACTIVE 时可用：模拟摇杆、咬合、吹气的电平。`1` 表示一直保持，直到发 `0`；≥2 表示保持这么多毫秒；`act clear` 全部清除 |
| `tap jump\|use` | 仅 ACTIVE 时可用：一次很短的咬合或吹气，用来检查 60 ms 最短按住 |
| `chan <joy\|breath\|bite> <0\|1>` | 打开或关闭某个通道（默认摇杆、吹吸打开，咬合关闭）。没接的传感器要先关掉；切换后会停用 |
| `set <hsrc\|vsrc\|deadx\|deady\|yawtau> <v>` | v0.7.0：视角左右/上下来源（0 roll、1 pitch、2 yaw）、分轴起转角、yaw 回零时间（D33）；`D` 行第 14/15 列改为映射后的左右/上下误差（`look_x_err,look_y_err`） |
| `set <jonx\|jony\|joffx\|joffy\|pmode\|prep\|sjump\|sdis> <v>` | v0.7.0：摇杆分轴阈值、吹气模式和连点间隔、吸气跳跃、吸气停用时长（D33–D35）；桥的 `GET/POST /profile` 读写保存的参数（D36） |
| `set <fuse\|armx\|armz\|arej\|lmode\|kgx\|kgy\|gdead> <v>` | v0.8.0：融合模式与力臂、加速度拒绝；控制方式（倾角 / 角速度 / 混合）与角速度增益、死区（D37、D38）。`fauto` 自 v0.8.1 起不再使用，仍接受 |
| `fcal [n\|l\|r\|d\|u\|on\|off\|clear]` / `fcal val n <roll> <pitch> 0` / `fcal rel <l\|r\|d\|u> <x> <y> <yaw>` | 五点精细校准，只求左右/上下转轴，五步齐了自动启用；回 `#FC,on=..,have=..,h=x/y/z,v=x/y/z,n=roll/pitch/0,l=..`（l/r/d/u 为相对中立的偏离，单位度）；基础校准后另有 `#FC,neutral,n`（D39、D40） |
| `fgate 0\|1` / `focus 0\|1` | 前台门控开关 / 由桥周期发送的「Minecraft 在前台」心跳（见 D28） |
| `hang` | 仅在看门狗开启时可用：让主循环卡死，用来测试看门狗复位（复位后应为 DISARMED，`#I` 里 `reset=watchdog`） |
| `jspan` | 仅 DISARMED 时可用：先保持松杆，再在 6 s 内把四个方向各推到舒适的极限，记录每人的摇杆行程 |
| `set <jon\|joff\|jswap\|jinvx\|jinvy> <v>` | 摇杆的触发/释放阈值（按跨度归一化）、交换轴、反转方向 |
| `set <bsign\|bon\|boff\|son\|soff\|brel> <v>` | 吹吸：符号（±1）、吹气触发/释放、吸气触发/释放（counts）、释放延迟（ms）。`ksign/kon/koff` 对应咬合；`slock` 是吹气结束后多久内不算吸气（ms，D31） |
| `turn <counts> [cps]` | 仅 ACTIVE 时可用：以给定速率发出正好这么多的鼠标 X 计数（期间头部视角暂停），用来测游戏里的度/计数和鼠标线性 |

板子输出的行：`#I` 身份/元数据，`#P` 参数，`#Q` 摇杆/压力通道参数，`S,` 通道数据（20 Hz，列名见 `#HS`），`#C` 校准结果，`#E` 事件，`#K` 主机实际收到的按键/右键状态（变化时才输出），`#F` 前台门控变化，`#OK/#ERR` 命令回执，`D,` 数据行。
`fuse 1` 时另有 10 Hz 的 `F,<t_ms>,<w>,<lin_g>,<rot_g>` 行：加速度信任、线加速度、力臂加速度（D37）。

`D` 行各列：`t_ms, loop_dt_us, state, imu_new, imu_age_ms, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, roll_deg, pitch_deg, look_x_err, look_y_err, vx_cps, vy_cps, dx, dy, hid, engaged, log_dropped, event`。`look_x_err, look_y_err` 是按 `hsrc/vsrc` 选好来源后的左右、上下误差（v0.7.0 之前叫 `roll_err, pitch_err`，默认设置下数值相同）。
其中加速度和陀螺是头部坐标系下的值（x 前、y 左、z 上），陀螺已去零偏；`dx/dy` 是相邻两行之间实际提交的计数之和；`engaged` 是两位数字，分别表示 X 轴和 Y 轴是否越过死区。
发送是有界的：缓冲区不够时丢弃这一行并计入 `log_dropped`，不会阻塞主循环。

## 5. 安全设计（调试版）

- 上电默认 DISARMED；HID 默认开启但必须先启用才有输出；非 ACTIVE 状态鼠标增量恒为 0。
- 串口 DTR 断开（关页面、断开、拔线）会立即停用。这是仅在调试版启用的保护，可以在 `Config.h` 里关掉。
- IMU 连续 100 ms 没有新样本，或者初始化失败，都会进入 FAULT 并锁存。样本恢复后也不会自动恢复，必须确认故障才回到 DISARMED，之后还要重新校准。
- 进入停用、校准或故障时，会先发出键盘全松和鼠标零按钮报告；这两个报告被端点接受之前会一直保持待发。
- 指针锁定退出时，测试页会自动发 `hid 0` 和 `disarm`。
- 调试版不启动看门狗（交接指南 §f 要求 bring-up 阶段先不开），正式集成时再开。

## 6. 主机端测试

```bash
./scripts/test-native.sh
```

覆盖：安装方向映射、滤波（静止收敛、陀螺跟踪、时间间断重置、非法值、线加速度抑制、α 与 dt 无关）、静止校准（成功、运动、样本不足、加速度异常）、两种迟滞和两种速度曲线、饱和、反转、增量累积器的残差和上限、状态机（上电无输出、完整启用流程、回中门控、三种停用来源、故障锁存、启动时无 IMU、校准失败）。

## 7. 验证状态

| 项目 | 结果 |
|---|---|
| 主机端逻辑测试 | 通过（macOS clang，`-Wall -Wextra -Werror`） |
| 固件编译（非 mbed，FQBN `Seeeduino:nrf52:xiaonRF52840Sense`） | 通过 |
| 测试页（模拟模式） | 在浏览器中跑通 |
| 实板：IMU 读数 → 本地桥 → 测试页 | 跑通：WHO_AM_I=0x6A，静止时 \|a\|≈1.02 g，陀螺 Y 零偏约 −2.9 dps |
| 实板：IMU 轴向、校准、HID、DTR 停用、延迟 | **未测**，按第 3 节记录 |
