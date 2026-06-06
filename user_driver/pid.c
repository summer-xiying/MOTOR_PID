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
float kp_1 = 0.2;   // 比例系数
float ki_1 = 0.2;   // 积分系数
float kd_1 = 0.1;   // 微分系数

/*
 *  ========================================
 *  电机2 PID参数 (可在运行时调整)
 *  ========================================
 */
float kp_2 = 0.2;   // 比例系数
float ki_2 = 0.2;   // 积分系数
float kd_2 = 0.1;   // 微分系数

#define ERROR_THRESHOLD 30   // 积分限幅阈值，限制积分项单次最大增量

/*
 *  ========================================
 *  循迹控制参数
 *  ========================================
 */
#define TRACKING_KP  50.0f    // 循迹比例系数
#define TRACKING_KD  15.0f   // 循迹微分系数
#define SPEED_HIGH   180     // 直线速度
#define SPEED_LOW    80      // 弯道速度
#define SPEED_SWITCH_THRESH 1.0f  // 速度切换阈值

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
static float current_error_1 = 0;
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
static float current_error_2 = 0;
static float prev_error_2 = 0;

/*
 *  ========================================
 *  设置目标速度
 *  ========================================
 */
void pid_set_target_speed(uint8_t motor_id, float speed)
{
    if(motor_id == MOTOR_1){
        target_speed_1 = speed;
    }
    else if(motor_id == MOTOR_2){
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
    if(motor_id == MOTOR_1){
        return speed_1;
    }
    else if(motor_id == MOTOR_2){
        return speed_2;
    }
    return 0;
}

/*
 *  ========================================
 *  获取当前误差
 *  ========================================
 */
float pid_get_error(uint8_t motor_id)
{
    if(motor_id == MOTOR_1){
        return target_speed_1 - speed_1;
    }
    else if(motor_id == MOTOR_2){
        return target_speed_2 - speed_2;
    }
    return 0;
}

/*
 *  ========================================
 *  设置PID参数
 *  ========================================
 */
void pid_set_params(uint8_t motor_id, float kp, float ki, float kd)
{
    if(motor_id == MOTOR_1){
        kp_1 = kp;
        ki_1 = ki;
        kd_1 = kd;
    }
    else if(motor_id == MOTOR_2){
        kp_2 = kp;
        ki_2 = ki;
        kd_2 = kd;
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
        speed_1 = (float)counter_1_A / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 100;
        counter_1_A = 0;
    }
    else if (motor_id == MOTOR_2) {
        speed_2 = (float)counter_2_A / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 100;
        counter_2_A = 0;
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

        error = target_speed_1 - speed_1;
        current_error_1 = error;

        // 增量式PID
        delta_pwm = kp_1 * (current_error_1 - last_error_1)
                  + ki_1 * current_error_1
                  + kd_1 * (current_error_1 - 2*last_error_1 + prev_error_1);

        // 积分限幅
        if (ki_1 * current_error_1 > ERROR_THRESHOLD) {
            delta_pwm = kp_1 * (current_error_1 - last_error_1)
                      + ERROR_THRESHOLD
                      + kd_1 * (current_error_1 - 2*last_error_1 + prev_error_1);
        }
        else if (ki_1 * current_error_1 < -ERROR_THRESHOLD) {
            delta_pwm = kp_1 * (current_error_1 - last_error_1)
                      - ERROR_THRESHOLD
                      + kd_1 * (current_error_1 - 2*last_error_1 + prev_error_1);
        }

        // 积分抗饱和
        if ((PWM_1_duty >= 4000 && delta_pwm > 0) ||
            (PWM_1_duty <= 0 && delta_pwm < 0)) {
            delta_pwm = kp_1 * (current_error_1 - last_error_1)
                      + kd_1 * (current_error_1 - 2*last_error_1 + prev_error_1);
        }

        PWM_1_duty += (int16_t)delta_pwm;

        // 限幅输出
        if (PWM_1_duty > 4000) {
            PWM_1_duty = 4000;
        }
        if (PWM_1_duty < 0) {
            PWM_1_duty = 0;
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
        last_error_1 = current_error_1;
        motor_set_duty(motor_id, (uint32_t)PWM_1_duty);
    }
    else if (motor_id == MOTOR_2) {
        static uint8_t stall_cnt_2 = 0;
        static uint8_t cooldown_2 = 0;

        error = target_speed_2 - speed_2;
        current_error_2 = error;

        // 增量式PID
        delta_pwm = kp_2 * (current_error_2 - last_error_2)
                  + ki_2 * current_error_2
                  + kd_2 * (current_error_2 - 2*last_error_2 + prev_error_2);

        // 积分限幅
        if (ki_2 * current_error_2 > ERROR_THRESHOLD) {
            delta_pwm = kp_2 * (current_error_2 - last_error_2)
                      + ERROR_THRESHOLD
                      + kd_2 * (current_error_2 - 2*last_error_2 + prev_error_2);
        }
        else if (ki_2 * current_error_2 < -ERROR_THRESHOLD) {
            delta_pwm = kp_2 * (current_error_2 - last_error_2)
                      - ERROR_THRESHOLD
                      + kd_2 * (current_error_2 - 2*last_error_2 + prev_error_2);
        }

        // 积分抗饱和
        if ((PWM_2_duty >= 4000 && delta_pwm > 0) ||
            (PWM_2_duty <= 0 && delta_pwm < 0)) {
            delta_pwm = kp_2 * (current_error_2 - last_error_2)
                      + kd_2 * (current_error_2 - 2*last_error_2 + prev_error_2);
        }

        PWM_2_duty += (int16_t)delta_pwm;

        // 限幅输出
        if (PWM_2_duty > 4000) {
            PWM_2_duty = 4000;
        }
        if (PWM_2_duty < 0) {
            PWM_2_duty = 0;
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
        last_error_2 = current_error_2;
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
    // 更新循迹传感器
    LineTracker_Update(&line_tracker);

    // 计算循迹PD输出
    float tracking_output = LineTracker_GetPIDOutput(&line_tracker,
                                TRACKING_KP, TRACKING_KD,
                                &tracking_last_error);

    // 动态调速：直线高速，弯道低速
    float base_speed = (fabsf(line_tracker.position) < SPEED_SWITCH_THRESH)
                       ? SPEED_HIGH : SPEED_LOW;

    // 差速转向：左轮加修正，右轮减修正
    float left_speed = base_speed + tracking_output;
    float right_speed = base_speed - tracking_output;

    // 限幅保护
    if (left_speed < 0) left_speed = 0;
    if (left_speed > 400) left_speed = 400;
    if (right_speed < 0) right_speed = 0;
    if (right_speed > 400) right_speed = 400;

    // 设置目标速度
    pid_set_target_speed(MOTOR_1, left_speed);
    pid_set_target_speed(MOTOR_2, right_speed);
}

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
        // 执行循迹控制（更新目标速度）
        tracking_control();

        // 计算速度
        calculate_speed(MOTOR_1);
        calculate_speed(MOTOR_2);

        // 执行PID控制
        DC_MOTOR_PID(MOTOR_1);
        DC_MOTOR_PID(MOTOR_2);

        // 每10次中断发送一次串口数据 (100ms)
        vofa_cnt++;
        if (vofa_cnt >= 10) {
            vofa_cnt = 0;
            UART_SendSensorData(PRINT_INST);
        }
        break;

    default:
        break;
    }
}
