# empty 工程快速上手

MSPM0G3507 / 2026 电赛工程。三层结构。

## 目录结构

```
main.c                      入口
BSP/                        纯驱动（oled key uart imu660rb mpu6050/ Motor Encoder step_motor track at24c02 led_buzzer delay）
MID/                        中间件（attitude+200Hz ISR  pid  imu_select  at_params  Fusion/）
APP/                        应用（system scheduler Control+100Hz ISR  menu/）
empty_mspm0g3507.syscfg     所有引脚/外设配置
```

## 时钟与启动

- HFXT 40MHz 外部晶振 (PA5/PA6) → PLL → **80MHz CPU**，ULPCLK 40MHz


## 引脚速查

| 模块 | 引脚 |
|---|---|
| OLED | PA0=SDA, PA1=SCL（软件 I2C）|
| IMU660RB | PB2=SCL, PB3=SDA（硬件 I2C1, 400kHz）|
| 按键 | K0=PB21, K1=PB11, K2=PA8, K3=PA7（按下=低）|
| 电机 PWM | PB15(bR), PB16(bL)（TIMG8 双 CCP）|
| 电机方向 | bL: PA14/PA15, bR: PA12/PB5 |
| 编码器 | bL: PB20(中断)/PA24, bR: PB0(中断)/PA13 |
| 步进 | STEP1=PA10, STEP2=PA23; DIR1=PB13, DIR2=PA25, EN1=PB1, EN2=PB24 |
| 循迹 | Track_1..8 = PB6/PB7/PB8/PB9/PB10/PB12/PA27/PA28 |
| UART | Cam=PA17/PA18(UART1), Comm=PB17/PB18(UART2) |
| AT24C02 | PA26=SCL, PA30=SDA（独立软件 I2C）|
| LED/蜂鸣器 | LED=PB25(低亮), BUZZER=PB27(高响), 心跳=PB14 |

## 上电流程

1. **IMU 标定**：`mid_init()` 自动做零偏标定 ~2.25s，**期间板子必须静止**，否则姿态漂移
2. **菜单**：OLED 显示 Main Menu，K1=UP / K0=DOWN 选，K2=ENTER 进，K3=BACK 返回
3. **运行**：默认 `start_run=false`（上电不跑），**K3 长按**切换运行/停止

IMU 标定失败显示 "IMU init FAIL"（当前未死循环，姿态不可信但继续跑）。

## 菜单页

- **Main Menu** → IMU Data / Encoder / Demo Params / Tasks / Track
- **IMU Data**：Roll/Pitch/Yaw 实时
- **Encoder**：bL/bR 速度 + 累计
- **Demo Params**：Kp/Speed 可编辑（ENTER 提交存 EEPROM，BACK 取消）
- **Tasks**：Buzz / LED / 电机测试
- **Track**：8 路电平 + 误差（调试用，自定义渲染）

实时页（IMU/Encoder/Track）每 100ms 自动刷新，无需手动操作。

## 添加菜单页

菜单是静态表驱动，加一页 4 步（全在 `APP/menu/app_menu.c`）：

1. 顶部枚举 `PAGE_COUNT` 前加一项，如 `PAGE_MYDATA`
2. 定义菜单项数组：
   ```c
   static const MenuItem mydata_items[] = {
       {"Val", MENU_ITEM_READONLY, {.param = {&my_var, MENU_VALUE_INT16, 0, 0, 0, 0}}},
   };
   ```
3. `pages[]` 表加一行：`[PAGE_MYDATA] = {"My Data", mydata_items, MENU_ARRAY_COUNT(mydata_items)},`
4. `main_items[]` 加入口：`{"My Data", MENU_ITEM_SUBMENU, {.page = &pages[PAGE_MYDATA]}},`

四种菜单项类型：

| 类型 | 用途 | data 字段 |
|---|---|---|
| `MENU_ITEM_READONLY` | 只读显示实时值 | `.param = {&var, TYPE, 0, 0, 0, 小数位}` |
| `MENU_ITEM_PARAM` | 可编辑（ENTER 进编辑，UP/DOWN 改，ENTER 提交 / BACK 取消）| `.param = {&var, TYPE, step, min, max, 小数位}` |
| `MENU_ITEM_ACTION` | 确认键执行一次回调 | `.action = func` |
| `MENU_ITEM_SUBMENU` | 进入子页面 | `.page = &pages[N]` |

变量类型用 `MENU_VALUE_FLOAT / INT16 / UINT8 / BOOL` 等。PARAM 项提交后触发 `on_committed` 回调，变量定义在 `at_params.c` 里即自动存 EEPROM。

整屏自定义显示（如 Track 页）：令 `item_count=0`，在 `app_menu_refresh()` 加 `if (menu_current_page(&g_menu) == &pages[PAGE_X]) { 自定义渲染; return; }`，参考 `track_page_render()`。

## 控制环

| 环 | 频率 | 定时器 | 位置 |
|---|---|---|---|
| 姿态环 (Mahony) | 200Hz | TIMER_ATT = TIMG0 (5ms) | `MID/attitude.c` ISR |
| 控制环 (turn+speed) | 100Hz | TIMER_0 = TIMA0 (10ms) | `APP/Control.c` ISR |
| 编码器 | 中断 | GROUP1 (GPIOB) | `BSP/Encoder.c` |

**start_run 安全开关**：控制环 ISR 顶部 `if(!start_run) return;`，上电不跑，K3 长按启用。

## IMU 切换

编译期二选一：imu660rb (LSM6DSR) 或 mpu6050 (自写非 DMP)。两者共用 I2C_IMU 总线 (PB2/PB3)，同一时刻只编译一个驱动，`attitude.c` 的 Mahony 对两者通用。

切换：改 `MID/imu_select.h` 的 `IMU_SELECT` 宏：
```c
#define IMU_SELECT  IMU_USE_MPU6050   /* 或 IMU_USE_IMU660RB */
```

- 驱动经抽象宏（`imu_acc_x` / `imu_gyro_x` / `imu_dev_init()` 等）映射到对应驱动，`attitude.c` / `Control.c` 无需改
- 轴方向符号 `IMU_AXIS_*_SIGN` 在对应分支按物理安装方向调整（两个 IMU 安装方向可能不同，实测调）
- MPU6050 走自写复用 attitude（不搬 DMP 库），yaw 永远是 `attitude.yaw`

## 空闲引脚

PA: 2, 9, 11, 16, 21, 22, 29, 31
PB: 4, 19, 22, 23, 26, 28, 29, 30, 31

（PA3/PA4 = LFXT 别用）

## H 题第二问停车

- K3 长按启动时开始计时；启动后的前 50ms 必须连续检测到有效路数不等于 4，才确认已经离开 A 点并允许识别终点。
- 正常循迹过程中，8 路红外中任意 4 路为 1 连续保持 5 个控制周期（约 50ms）后，把两轮输出置零；中途一次不是 4 路就重新计数。
- 保持零输出 100ms 后再次检查：仍为 4 路则停车成功；不再是 4 路则认为惯性冲过启停线。
- 冲过后左右轮以 `-4` 脉冲/10ms 的目标速度低速倒车，连续三个控制周期（约 30ms）检测到 4 路为 1 后停车。
- 倒车回找超过 1500ms 仍未找到启停线时强制停车，OLED 显示 `PARK FAIL`；成功时显示 `PARK OK` 和最终总时间。
- 停车策略参数定义在 `APP/parking.h`，状态机实现位于 `APP/parking.c`。

## 第二半圆减速

- 小车从 A 点顺时针出发，右侧 `B -> C` 第一半圆和底部 `C -> D` 仍使用菜单或串口设置的 `basicSpeed`。
- 航向状态机使用现有 `attitude.yaw`：先确认完成第一半圆，再确认底部直线，最后连续 8 个控制周期（约 80ms）检测第二次同方向转弯。
- 确认进入左侧 `D -> A` 第二半圆后，前进基础速度限制为 `10` 脉冲/10ms，并保持到 A 点停车。
- 停车制动和低速倒车不使用该限速；倒车目标仍是 `-4` 脉冲/10ms。
- 每次启动或手动停止都会重置赛道阶段。速度和航向判据集中定义在 `APP/course_speed.h`，状态机位于 `APP/course_speed.c`。
