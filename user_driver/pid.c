#include "pid.h"
#include "motor.h"
#include "uart.h"
#include "tracking.h"
#include <math.h>

/*
 *  ========================================
 *  电机1 PID参数 (可在运行时调整)
 *  ========================================
 */
float kp_1 = 2.0;   // 比例系数
float ki_1 = 0.15;  // 积分系数
float kd_1 = 2.5;   // 微分系数

/*
 *  ========================================
 *  电机2 PID参数 (可在运行时调整)
 *  ========================================
 */
float kp_2 = 2.0;   // 比例系数
float ki_2 = 0.15;  // 积分系数
float kd_2 = 2.5;   // 微分系数

#define ERROR_THRESHOLD     30     // 积分限幅阈值，限制积分项单次最大增量
#define PWM_DEADZONE        500    // 死区补偿阈值，PWM>0且<此值时强制启动
#define SPEED_FILTER_ALPHA  0.35f  // 速度测量低通滤波系数
#define D_FILTER_ALPHA      0.5f   // D项低通滤波系数，越小越平滑
#define DEADZONE_SPEED_THRESH 50.0f // 目标速度高于此值时才启用死区补偿

/*
 *  ========================================
 *  循迹控制参数
 *  ========================================
 */
// 循迹PD参数（可通过VOFA在线调节，统一不分直线弯道）
float kp_track = 12.0f;      // 比例系数
float kd_track = 50.0f;      // 微分系数

// 统一速度
#define BASE_SPEED    170     // 基础速度

// 弯道判定阈值（用于差速补偿和速度策略，不影响PD参数）
#define MODE_SWITCH_THRESH 2.5f  // |position| >= 此值判定为弯道

// 差速补偿参数（让左右轮实际速度一致）
#define DIFF_KP       0.5f    // 差速比例系数
#define DIFF_KI       0.1f    // 差速积分系数
#define DIFF_KD       0.05f   // 差速微分系数
#define DIFF_LIMIT    30.0f   // 差速补偿限幅

// 非对称差速参数：外轮多加速，内轮少减速，降低过弯顿挫
#define OUTER_TURN_GAIN 1.45f
#define INNER_TURN_GAIN 0.30f

// 目标速度斜坡限制：10ms周期内最大变化量，提升底盘平顺性
#define TARGET_SPEED_STEP_LIMIT 12.0f

// 循迹器实例（在main.c中定义，这里extern引用）
extern LineTracker line_tracker;
extern float tracking_last_error;

/*
 *  ========================================
 *  电机1 PID变量
 *  ========================================
 */
extern uint32_t counter_1_A;   // 编码器计数 (在key.c的GPIO中断中累加)
float speed_1 = 0;             // 当前速度 mm/s
float target_speed_1 = 0;      // 目标速度 mm/s
int16_t PWM_1_duty = 0;        // 当前PWM占空比

static float last_error_1 = 0;
static float prev_error_1 = 0;   // 上上次误差，用于D项

/*
 *  ========================================
 *  电机2 PID变量
 *  ========================================
 */
extern uint32_t counter_2_A;   // 编码器计数 (在key.c中定义)
float speed_2 = 0;             // 当前速度 mm/s
float target_speed_2 = 0;      // 目标速度 mm/s
int16_t PWM_2_duty = 0;        // 当前PWM占空比

static float last_error_2 = 0;
static float prev_error_2 = 0;

/*
 *  ========================================
 *  设置目标速度
 *  ========================================
 */
void pid_set_target_speed(uint8_t motor_id, float speed)
{
    if (motor_id == MOTOR_1) {
        target_speed_1 = speed;
    } else if (motor_id == MOTOR_2) {
        target_speed_2 = speed;
    }
}

/*
 *  ========================================
 *  获取当前速度
 *  ========================================
 */
float pid_get_speed(uint8_t motor_id)
{
    if (motor_id == MOTOR_1) return speed_1;
    if (motor_id == MOTOR_2) return speed_2;
    return 0;
}

/*
 *  ========================================
 *  获取当前误差
 *  ========================================
 */
float pid_get_error(uint8_t motor_id)
{
    if (motor_id == MOTOR_1) return target_speed_1 - speed_1;
    if (motor_id == MOTOR_2) return target_speed_2 - speed_2;
    return 0;
}

/*
 *  ========================================
 *  设置PID参数
 *  motor_id: MOTOR_1 / MOTOR_2 / 0(同时设置两个)
 *  ========================================
 */
void pid_set_params(uint8_t motor_id, float kp_val, float ki_val, float kd_val)
{
    if (motor_id == 0 || motor_id == MOTOR_1) {
        kp_1 = kp_val;
        ki_1 = ki_val;
        kd_1 = kd_val;
    }
    if (motor_id == 0 || motor_id == MOTOR_2) {
        kp_2 = kp_val;
        ki_2 = ki_val;
        kd_2 = kd_val;
    }
}

/*
 *  ========================================
 *  速度计算函数
 *  轮速 = 编码器计数 / 编码器线数 * π * 轮径 * 采样频率
 *  采样频率 = 1/0.01s = 100
 *  ========================================
 */
void calculate_speed(uint8_t motor_id)
{
    if (motor_id == MOTOR_1) {
        __disable_irq();
        uint32_t count = counter_1_A;
        counter_1_A = 0;
        __enable_irq();
        float instant_speed = (float)count / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 100;
        speed_1 = SPEED_FILTER_ALPHA * instant_speed + (1.0f - SPEED_FILTER_ALPHA) * speed_1;
    } else if (motor_id == MOTOR_2) {
        __disable_irq();
        uint32_t count = counter_2_A;
        counter_2_A = 0;
        __enable_irq();
        float instant_speed = (float)count / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 100;
        speed_2 = SPEED_FILTER_ALPHA * instant_speed + (1.0f - SPEED_FILTER_ALPHA) * speed_2;
    }
}

/*
 *  ========================================
 *  增量式PID控制函数
 *  ΔPWM = Kp*(e[n]-e[n-1]) + Ki*e[n] + Kd*(e[n]-2*e[n-1]+e[n-2])
 *  ========================================
 */
void DC_MOTOR_PID(uint8_t motor_id)
{
    float error;
    float delta_pwm;

    if (motor_id == MOTOR_1) {
        static uint8_t stall_cnt_1 = 0;   // 堵转持续计数
        static uint8_t cooldown_1 = 0;    // 冷却计数，防止重复触发
        static float d_filtered_1 = 0;    // D项滤波值

        error = target_speed_1 - speed_1;

        // D项低通滤波：抑制速度测量噪声导致的PWM抖动
        float d_raw = error - 2*last_error_1 + prev_error_1;
        d_filtered_1 = D_FILTER_ALPHA * d_raw + (1.0f - D_FILTER_ALPHA) * d_filtered_1;

        // 增量式PID
        delta_pwm = kp_1 * (error - last_error_1)
                  + ki_1 * error
                  + kd_1 * d_filtered_1;

        // 积分限幅
        if (ki_1 * error > ERROR_THRESHOLD) {
            delta_pwm = kp_1 * (error - last_error_1)
                      + ERROR_THRESHOLD
                      + kd_1 * d_filtered_1;
        } else if (ki_1 * error < -ERROR_THRESHOLD) {
            delta_pwm = kp_1 * (error - last_error_1)
                      - ERROR_THRESHOLD
                      + kd_1 * d_filtered_1;
        }

        // 积分抗饱和
        if ((PWM_1_duty >= 4000 && delta_pwm > 0) ||
            (PWM_1_duty <= 0 && delta_pwm < 0)) {
            delta_pwm = kp_1 * (error - last_error_1)
                      + kd_1 * d_filtered_1;
        }

        PWM_1_duty += (int16_t)delta_pwm;

        // 限幅输出
        if (PWM_1_duty > 4000) PWM_1_duty = 4000;
        if (PWM_1_duty < 0) PWM_1_duty = 0;

        // 死区补偿：仅目标速度较高时启用，避免低速目标超调
        if (target_speed_1 > DEADZONE_SPEED_THRESH && PWM_1_duty > 0 && PWM_1_duty < PWM_DEADZONE) {
            PWM_1_duty = PWM_DEADZONE;
        }

        // 堵转保护
        if (cooldown_1 > 0) {
            cooldown_1--;
        } else if (PWM_1_duty >= 3500 && speed_1 < 15.0f) {
            stall_cnt_1++;
            if (stall_cnt_1 > 15) {
                PWM_1_duty = 0;
                last_error_1 = 0;
                prev_error_1 = 0;
                stall_cnt_1 = 0;
                cooldown_1 = 50;
            }
        } else {
            stall_cnt_1 = 0;
        }

        prev_error_1 = last_error_1;
        last_error_1 = error;
        motor_set_duty(motor_id, (uint32_t)PWM_1_duty);

    } else if (motor_id == MOTOR_2) {
        static uint8_t stall_cnt_2 = 0;
        static uint8_t cooldown_2 = 0;
        static float d_filtered_2 = 0;    // D项滤波值

        error = target_speed_2 - speed_2;

        // D项低通滤波
        float d_raw = error - 2*last_error_2 + prev_error_2;
        d_filtered_2 = D_FILTER_ALPHA * d_raw + (1.0f - D_FILTER_ALPHA) * d_filtered_2;

        // 增量式PID
        delta_pwm = kp_2 * (error - last_error_2)
                  + ki_2 * error
                  + kd_2 * d_filtered_2;

        // 积分限幅
        if (ki_2 * error > ERROR_THRESHOLD) {
            delta_pwm = kp_2 * (error - last_error_2)
                      + ERROR_THRESHOLD
                      + kd_2 * d_filtered_2;
        } else if (ki_2 * error < -ERROR_THRESHOLD) {
            delta_pwm = kp_2 * (error - last_error_2)
                      - ERROR_THRESHOLD
                      + kd_2 * d_filtered_2;
        }

        // 积分抗饱和
        if ((PWM_2_duty >= 4000 && delta_pwm > 0) ||
            (PWM_2_duty <= 0 && delta_pwm < 0)) {
            delta_pwm = kp_2 * (error - last_error_2)
                      + kd_2 * d_filtered_2;
        }

        PWM_2_duty += (int16_t)delta_pwm;

        // 限幅输出
        if (PWM_2_duty > 4000) PWM_2_duty = 4000;
        if (PWM_2_duty < 0) PWM_2_duty = 0;

        // 死区补偿：仅目标速度较高时启用
        if (target_speed_2 > DEADZONE_SPEED_THRESH && PWM_2_duty > 0 && PWM_2_duty < PWM_DEADZONE) {
            PWM_2_duty = PWM_DEADZONE;
        }

        // 堵转保护
        if (cooldown_2 > 0) {
            cooldown_2--;
        } else if (PWM_2_duty >= 3500 && speed_2 < 15.0f) {
            stall_cnt_2++;
            if (stall_cnt_2 > 15) {
                PWM_2_duty = 0;
                last_error_2 = 0;
                prev_error_2 = 0;
                stall_cnt_2 = 0;
                cooldown_2 = 50;
            }
        } else {
            stall_cnt_2 = 0;
        }

        prev_error_2 = last_error_2;
        last_error_2 = error;
        motor_set_duty(motor_id, (uint32_t)PWM_2_duty);
    }
}

/*
 *  ========================================
 *  循迹控制函数（在定时器中断中调用）
 *  读取传感器 -> 计算位置 -> PD输出 -> 差速转向
 *  ========================================
 */
void tracking_control(void)
{
    // 斜坡状态（跨调用保持，需在函数顶层声明以便禁用时重置）
    static float last_left_speed = BASE_SPEED;
    static float last_right_speed = BASE_SPEED;
    static float diff_integral = 0;
    static float diff_last_error = 0;

    // 循迹禁用时重置斜坡状态并返回
    if (!tracking_enabled) {
        last_left_speed = BASE_SPEED;
        last_right_speed = BASE_SPEED;
        diff_integral = 0;
        diff_last_error = 0;
        return;
    }

    // 更新循迹传感器
    LineTracker_Update(&line_tracker);

    if (!line_tracker.on_line || line_tracker.cross_detected) {
        tracking_last_error = 0;
    }

    // 计算循迹PD输出（统一参数，不分直线弯道）
    uint8_t is_curve = (fabsf(line_tracker.position) >= MODE_SWITCH_THRESH) ? 1 : 0;
    float tracking_output = LineTracker_GetPIDOutput(&line_tracker,
                                kp_track, kd_track,
                                &tracking_last_error);
    debug_tracking_output = tracking_output;

    // 差速补偿：仅直线模式启用，弯道时禁用并清零积分项
    float diff_compensation = 0;
    if (!is_curve) {
        float diff_error = speed_1 - speed_2;
        diff_integral += diff_error;
        if (diff_integral > DIFF_LIMIT) diff_integral = DIFF_LIMIT;
        if (diff_integral < -DIFF_LIMIT) diff_integral = -DIFF_LIMIT;
        float diff_derivative = diff_error - diff_last_error;
        diff_last_error = diff_error;

        diff_compensation = DIFF_KP * diff_error
                          + DIFF_KI * diff_integral
                          + DIFF_KD * diff_derivative;

        if (diff_compensation > DIFF_LIMIT) diff_compensation = DIFF_LIMIT;
        if (diff_compensation < -DIFF_LIMIT) diff_compensation = -DIFF_LIMIT;
    } else {
        diff_integral = 0;
        diff_last_error = 0;
    }

    // 弯道双基准速度：内轮明显减速，外轮轻微减速，避免外轮过猛超调
    float abs_position = fabsf(line_tracker.position);
    float inner_speed_factor = 1.0f - 0.05f * abs_position;
    float outer_speed_factor = 1.0f - 0.025f * abs_position;
    if (inner_speed_factor < 0.7f) inner_speed_factor = 0.7f;
    if (outer_speed_factor < 0.85f) outer_speed_factor = 0.85f;
    float inner_base_speed = BASE_SPEED * inner_speed_factor;
    float outer_base_speed = BASE_SPEED * outer_speed_factor;

    // 非对称差速：外轮轻微降速后加速，内轮随偏移明显降速
    float left_speed, right_speed;
    if (tracking_output >= 0) {
        left_speed = outer_base_speed + tracking_output * OUTER_TURN_GAIN - diff_compensation;
        right_speed = inner_base_speed - tracking_output * INNER_TURN_GAIN + diff_compensation;
    } else {
        left_speed = inner_base_speed + tracking_output * INNER_TURN_GAIN - diff_compensation;
        right_speed = outer_base_speed - tracking_output * OUTER_TURN_GAIN + diff_compensation;
    }

    // 限幅保护（允许内轮停止或反转）
    if (left_speed < -100) left_speed = -100;
    if (left_speed > 400) left_speed = 400;
    if (right_speed < -100) right_speed = -100;
    if (right_speed > 400) right_speed = 400;

    // 目标速度动态斜坡：直线平顺，弯道快速建立差速，边缘急弯优先救线
    float target_step_limit = TARGET_SPEED_STEP_LIMIT;
    if (is_curve) {
        target_step_limit = 24.0f;
    }
    if (line_tracker.sensor_raw[0] || line_tracker.sensor_raw[SENSOR_COUNT - 1]) {
        target_step_limit = 32.0f;
    }

    float left_delta = left_speed - last_left_speed;
    float right_delta = right_speed - last_right_speed;
    if (left_delta > target_step_limit) left_delta = target_step_limit;
    if (left_delta < -target_step_limit) left_delta = -target_step_limit;
    if (right_delta > target_step_limit) right_delta = target_step_limit;
    if (right_delta < -target_step_limit) right_delta = -target_step_limit;
    left_speed = last_left_speed + left_delta;
    right_speed = last_right_speed + right_delta;
    last_left_speed = left_speed;
    last_right_speed = right_speed;

    // 设置目标速度
    pid_set_target_speed(MOTOR_1, left_speed);
    pid_set_target_speed(MOTOR_2, right_speed);
}

/*
 *  ========================================
 *  串口发送标志（由ISR设置，主循环清除）
 *  ========================================
 */
volatile uint8_t uart_send_flag = 0;

// 循迹PD输出（用于VOFA+调试）
float debug_tracking_output = 0;

/*
 *  ========================================
 *  LED闪烁计数器（由ISR递增，主循环使用）
 *  10ms中断一次，100次 = 1秒
 *  ========================================
 */
volatile uint32_t led_tick = 0;

/*
 *  ========================================
 *  PID定时器中断服务函数 (10ms周期)
 *  ========================================
 */
void MOTOR_PID_INST_IRQHandler()
{
    static uint8_t vofa_cnt = 0;

    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        DL_Timer_clearInterruptStatus(MOTOR_PID_INST, DL_TIMER_INTERRUPT_LOAD_EVENT);

        // 执行循迹控制（更新目标速度）
        tracking_control();

        // 计算速度
        calculate_speed(MOTOR_1);
        calculate_speed(MOTOR_2);

        // 执行PID控制
        DC_MOTOR_PID(MOTOR_1);
        DC_MOTOR_PID(MOTOR_2);

        // LED计时（10ms递增）
        led_tick++;

        // 每10次中断设置串口发送标志 (100ms)
        vofa_cnt++;
        if (vofa_cnt >= 10) {
            vofa_cnt = 0;
            uart_send_flag = 1;
        }
        break;

    default:
        break;
    }
}
