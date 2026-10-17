# 车载平衡滚球系统 · MSPM0G3507

2026 年全国大学生电子设计竞赛 **H 题**参赛车完整固件。

双轮差速底盘 + 步进摇臂机构 + 板球平衡平台。完成题目要求的循迹一圈、定点停车、
平衡球定点保持与抗扰、以及多阶段搬运任务。

| 项 | 数值 |
|---|---|
| 主控 | MSPM0G3507 @ 80 MHz |
| 固件语言 | C（TI Arm Clang / DriverLib） |
| C/H 源码 | 78 文件 / 约 8,900 行 |
| Python 策略模型与测试 | 12 文件 / 约 1,450 行 |
| 实时环 | 姿态 200 Hz ／ 控制 100 Hz |

---

## 硬件平台

| 模块 | 型号 / 参数 |
|---|---|
| 主控 | MSPM0G3507，HFXT 40 MHz → PLL → **80 MHz CPU**，ULPCLK 40 MHz |
| 姿态 | **IMU660RB（LSM6DSR）或 MPU6050**，编译期二选一，共用 I2C1 (PB2/PB3) |
| 底盘 | 双直流减速电机 + 正交编码器，TIMG8 双路 PWM |
| 执行 | 2 路步进电机（球杆倾角 / 摇臂） |
| 感知 | 8 路灰度循迹、4 个按键 |
| 显示 | SSD1306 OLED（软件 I2C，PA0/PA1） |
| 存储 | AT24C02 EEPROM（参数掉电保存） |
| 通信 | UART1 摄像头、UART2 上位机 |

完整引脚分配见 [docs/quickstart.md](docs/quickstart.md)。

---

## 软件结构

```
main.c                        入口，只调 system_init() + system_run()
│
├─ APP/                       应用层
│   ├─ system.c               按层聚合初始化 + 无阻塞主循环
│   ├─ scheduler.c            软定时任务
│   ├─ Control.c              转向 + 速度控制环（100 Hz 中断）
│   ├─ ball.c                 平衡球闭环
│   ├─ ball_target_task.c     球定点任务
│   ├─ course_speed.c         赛道分段速度规划
│   ├─ parking.c              定点停车判定
│   ├─ task4_ab.c             任务 4 A/B 分支
│   ├─ pid_tuning.c           PID 在线调参
│   ├─ vision_motor_trigger.c 视觉触发（检测红色目标 → 定时启停电机）
│   └─ menu/                  表驱动 OLED 菜单
│
├─ MID/                       中间件
│   ├─ attitude.c             Mahony 姿态融合（200 Hz 中断）
│   ├─ pid.c                  PID 内核
│   ├─ at_params.c            参数 ↔ EEPROM 映射
│   ├─ imu_select.h           IMU 编译期切换宏
│   ├─ mechanism.c            机构运动解算
│   └─ Fusion/                Madgwick AHRS（第三方，MIT）
│
├─ BSP/                       驱动层（不含业务逻辑）
│   ├─ oled / key / uart / led_buzzer / delay
│   ├─ imu660rb / mpu6050
│   ├─ Motor / Encoder / step_motor
│   ├─ track                  8 路循迹加权误差
│   └─ at24c02                软件 I2C EEPROM
│
├─ tests/                     Python 策略模型 + 集成测试（不参与固件编译）
├─ targetConfigs/             XDS110 下载配置
└─ empty_mspm0g3507.syscfg    全部引脚/外设配置（SysConfig）
```

---

## 运行节奏

| 环 | 频率 | 定时器 | 位置 |
|---|---|---|---|
| 姿态融合（Mahony） | **200 Hz** | TIMG0 | `MID/attitude.c` |
| 转向 + 速度控制 | **100 Hz** | TIMA0 | `APP/Control.c` |
| 编码器计数 | GPIO 中断 | GROUP1 | `BSP/Encoder.c` |
| 主循环 | 无阻塞轮询 | — | `APP/system.c` |

主循环里没有任何 `delay_ms` 阻塞调用，心跳与实时页刷新都走时间戳比较。

---

## 几个实现要点

**表驱动菜单** —— 新增一页只需 4 步、不写渲染代码：加枚举 → 定义 `MenuItem[]` →
`pages[]` 注册 → `main_items[]` 加入口。四种项类型（只读 / 可编辑参数 / 动作 /
子菜单）覆盖全部交互。详见 [APP/menu/README.md](APP/menu/README.md)。

**参数掉电保存** —— `at_params` 把变量地址集中登记，读写统一走 AT24C02；
读回值带范围校验，遇到空白 EEPROM 的 `0xFF` 或 NaN 会自动回落默认值，
不会把脏数据送进控制环。

**IMU 编译期切换** —— 改 `MID/imu_select.h` 一个宏即可在 IMU660RB 与 MPU6050
之间切换，驱动经统一宏（`imu_acc_x` / `imu_gyro_x` / `imu_dev_init()`）映射，
`attitude.c` 与 `Control.c` 一行都不用改。

**循迹盲区保护** —— 8 路探头之间有物理间隙，黑线恰好落在间隙时会出现全白。
固件在每 10 ms 快照里统计连续全白：前 3 帧（约 30 ms）保持最后一次有效误差，
之后按最后方向输出 ±10 主动找线，任一路重新检测到黑线立即恢复。
避免误差从中心突然跳到极值造成摆动。

**上电安全开关** —— `start_run = false`，上电后控制环直接 return，不会乱冲；
需要显式启动后才接管。

---

## 编译与烧录

1. 安装 **Code Composer Studio**（Theia 版）与 **MSPM0 SDK 2.x**
2. `File → Import → CCS Projects`，选择本仓库根目录（含 `.ccsproject`）
3. 引脚 / 外设改动一律在 **SysConfig** 里做（`empty_mspm0g3507.syscfg`），
   不要手改生成的 `ti_msp_dl_config.h`
4. `Build` → 连接 XDS110 → `Flash`

> 构建产物（`Debug/`）已在 `.gitignore` 中排除，克隆后首次编译会重新生成。

---

## ⚠️ 上电注意

`mid_init()` 会做约 **2.25 秒**的陀螺零偏标定。

**期间小车必须完全静止**，否则零偏标定不准 → 姿态角持续漂移 → 后续所有任务失准。
标定失败时 OLED 显示 `IMU init FAIL`。

---

## 测试

`tests/` 下是 Python 写的**策略模型**与**集成测试**，用来在电脑上验证判定逻辑
（不参与固件编译）：

```bash
cd tests
python -m unittest discover -v
```

**无第三方依赖**，只用标准库 `unittest`。两层测试：

| 类型 | 文件 | 作用 |
|---|---|---|
| 策略模型 | `*_policy_model.py` | 把停车判定、赛道分段减速、球定点控制等逻辑抽成纯函数 |
| 集成测试 | `test_*_policy.py` / `test_*_integration.py` | 跑边界条件 |

其中一类测试会**直接解析 C 头文件**做交叉校验——例如 `test_parking_policy.py`
读取 `APP/parking.h` 的宏，断言 Python 模型里的常量与固件完全一致。
这样"策略先在 PC 上跑通、再落固件"的流程不会两边数值飘掉。

---

## 文档

- [快速上手：引脚表 / 菜单操作 / 控制环 / IMU 切换](docs/quickstart.md)
- [菜单系统开发说明](APP/menu/README.md)

---

## 第三方组件

- `MID/Fusion/`：Madgwick AHRS 姿态融合库（MIT License，原作者 xioTechnologies）
- MSPM0 驱动与启动代码：TI MSPM0 SDK（BSD-3-Clause）
