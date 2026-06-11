# VOFA+ 在线调参与 PID 参数说明

## 1. 通信协议

### 1.1 VOFA+ JustFloat 波形协议
- **帧格式**：N × float(4字节) + 帧尾 `0x00 0x00 0x80 0x7F`
- **发送周期**：100ms（定时器中断10ms，每10次发送一次）
- **触发条件**：`debug_output_enabled = 1`（VOFA发送 `output:1` 或上位机发送 WirelessTune SET_OUTPUT）

### 1.2 VOFA+ 文本命令协议（滑块组件）
- **格式**：`name:value\n`（换行符结尾）
- **支持命令**：

| 滑块名称 | 作用 | 值范围 | 说明 |
|----------|------|--------|------|
| `kp` | 比例系数 | 0 ~ +∞ | 同时设置 kp_1 和 kp_2 |
| `ki` | 积分系数 | 0 ~ +∞ | 同时设置 ki_1 和 ki_2 |
| `kd` | 微分系数 | 0 ~ +∞ | 同时设置 kd_1 和 kd_2 |
| `speed` | 目标速度 | 0 ~ 500 | 同时设置两电机，**会关闭循迹** |
| `track` | 循迹开关 | 0 / 1 | 1=开启循迹，0=关闭 |
| `output` | 波形输出 | 0 / 1 | 1=发送波形，0=静默 |
| `kp_straight` | 直线KP | 0 ~ +∞ | 直线模式循迹比例系数 |
| `kd_straight` | 直线KD | 0 ~ +∞ | 直线模式循迹微分系数 |
| `kp_curve` | 弯道KP | 0 ~ +∞ | 弯道模式循迹比例系数 |
| `kd_curve` | 弯道KD | 0 ~ +∞ | 弯道模式循迹微分系数 |

### 1.3 WirelessTune 二进制协议
- **帧头**：`0xAA 0x55`
- **帧格式**：`[HEAD1][HEAD2][CMD][LEN][DATA...][CHECK]`
- **校验**：CHECK = CMD + LEN + SUM(DATA)
- **命令**：

| CMD | 功能 | DATA长度 | 说明 |
|-----|------|----------|------|
| 0x01 | 设置PID | 13字节 | motor_id(1) + kp(4) + ki(4) + kd(4)，支持单独设置 |
| 0x02 | 设置速度 | 8字节 | left_speed(4) + right_speed(4)，会关闭循迹 |
| 0x03 | 查询状态 | 0字节 | 返回54字节状态帧 |
| 0x04 | 波形开关 | 1字节 | 0=关，1=开 |
| 0x05 | 循迹开关 | 1字节 | 0=关，1=开 |

---

## 2. VOFA+ 波形通道映射

VOFA+ JustFloat 波形共6个通道（`VOFA_SendPIDWaveform`，`uart.c:132`）：

| 通道 | 变量 | 含义 |
|------|------|------|
| CH0 | `target_speed_1` | 电机1 目标速度 (mm/s) |
| CH1 | `speed_1` | 电机1 实际速度 (mm/s) |
| CH2 | `PWM_1_duty` | 电机1 PWM占空比 (0~4000) |
| CH3 | `target_speed_2` | 电机2 目标速度 (mm/s) |
| CH4 | `speed_2` | 电机2 实际速度 (mm/s) |
| CH5 | `PWM_2_duty` | 电机2 PWM占空比 (0~4000) |

---

## 3. PID 速度环参数

### 3.1 当前参数：第一组数据（2026-06-10）

| 参数 | 电机1 | 电机2 | 说明 |
|------|-------|-------|------|
| `kp` | 0.8 | 0.8 | 比例系数，加快响应 |
| `ki` | 0.3 | 0.3 | 积分系数，消除稳态误差 |
| `kd` | 0.12 | 0.12 | 微分系数，抑制超调 |

> VOFA 命令 `kp`/`ki`/`kd` 会同时设置两组参数为相同值。
> WirelessTune 二进制协议（CMD 0x01）支持通过 motor_id 单独设置。

### 3.2 PID 内部常量（`pid.c` #define）

| 常量 | 值 | 说明 |
|------|-----|------|
| `ERROR_THRESHOLD` | 30 | 积分限幅，单次最大积分增量 |
| `PWM_DEADZONE` | 500 | 死区补偿，PWM在此值以下强制启动 |
| `SPEED_FILTER_ALPHA` | 0.35 | 速度低通滤波系数，越小越平滑 |
| PWM上限 | 4000 | 硬件限制，占空比最大值 |

### 3.3 堵转保护
- 条件：PWM ≥ 3500 且 速度 < 15 mm/s，持续 15 个周期（150ms）
- 动作：PWM 清零，冷却 50 周期（500ms）

---

## 4. 循迹控制参数（`pid.c` 全局变量）

### 4.1 循迹 PD 参数（可通过VOFA在线调节）

| 模式 | VOFA名称 | 变量 | 初始值 | 切换条件 |
|------|----------|------|--------|----------|
| 直线模式 | `kp_straight` | `kp_straight` | 12.0 | \|position\| < 2.5 |
| 直线模式 | `kd_straight` | `kd_straight` | 50.0 | \|position\| < 2.5 |
| 弯道模式 | `kp_curve` | `kp_curve` | 12.0 | \|position\| ≥ 2.5 |
| 弯道模式 | `kd_curve` | `kd_curve` | 48.0 | \|position\| ≥ 2.5 |

> `MODE_SWITCH_THRESH = 2.5`（加权求和后 position 范围 ±15）

### 4.2 速度参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `BASE_SPEED` | 120 | 基础速度 (mm/s) |
| `OUTER_TURN_GAIN` | 1.45 | 外轮增益（转弯时外轮加速） |
| `INNER_TURN_GAIN` | 0.30 | 内轮增益（转弯时内轮减速） |
| `TARGET_SPEED_STEP_LIMIT` | 12.0 | 直线斜坡限制/周期 |
| 弯道斜坡 | 24.0 | 弯道模式斜坡限制 |
| 边缘救线斜坡 | 32.0 | 传感器边缘触发时斜坡限制 |

### 4.3 弯道双基准速度

| 参数 | 公式 | 下限 |
|------|------|------|
| 内轮减速因子 | `1.0 - 0.05 × |position|` | 0.7 |
| 外轮减速因子 | `1.0 - 0.025 × |position|` | 0.85 |

### 4.4 差速补偿（仅直线模式生效）

| 参数 | 值 | 说明 |
|------|-----|------|
| `DIFF_KP` | 0.5 | 差速比例 |
| `DIFF_KI` | 0.1 | 差速积分 |
| `DIFF_KD` | 0.05 | 差速微分 |
| `DIFF_LIMIT` | 30.0 | 补偿限幅 |

---

## 5. 变量位置索引

| 变量 | 类型 | 文件 | 初始值 | 可调 |
|------|------|------|--------|------|
| `kp_1` / `ki_1` / `kd_1` | float | pid.c / pid.h | 0.8 / 0.3 / 0.12 | VOFA/WirelessTune |
| `kp_2` / `ki_2` / `kd_2` | float | pid.c / pid.h | 0.8 / 0.3 / 0.12 | VOFA/WirelessTune |
| `kp_straight` / `kd_straight` | float | pid.c / pid.h | 12.0 / 50.0 | VOFA kp_straight/kd_straight |
| `kp_curve` / `kd_curve` | float | pid.c / pid.h | 12.0 / 48.0 | VOFA kp_curve/kd_curve |
| `speed_1` / `speed_2` | float | pid.c / pid.h | 0 | 只读 |
| `target_speed_1` / `target_speed_2` | float | pid.c / pid.h | 0 | VOFA speed/循迹 |
| `PWM_1_duty` / `PWM_2_duty` | int16_t | pid.c / pid.h | 0 | 只读 |
| `tracking_enabled` | volatile uint8_t | uart.c / pid.h | 1 | VOFA track |
| `debug_output_enabled` | uint8_t | uart.c | 0→1 | VOFA output |
| `debug_tracking_output` | float | pid.c / pid.h | 0 | 只读 |
| `uart_send_flag` | volatile uint8_t | pid.c / pid.h | 0 | 内部标志 |
| `led_tick` | volatile uint32_t | pid.c / pid.h | 0 | 内部计数 |
