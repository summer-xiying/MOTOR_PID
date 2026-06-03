#include "pid.h"
#include "motor.h"
#include "uart.h"

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
 *  采样频率 = 1/0.05s = 20
 *  ========================================
 */
void calculate_speed(uint8_t motor_id)
{
    if (motor_id == MOTOR_1) {
        speed_1 = (float)counter_1_A / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 20;
        counter_1_A = 0;
    }
    else if (motor_id == MOTOR_2) {
        speed_2 = (float)counter_2_A / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 20;
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
 *  PID定时器中断服务函数 (50ms周期)
 *  ========================================
 */
void MOTOR_PID_INST_IRQHandler()
{
    static uint8_t vofa_cnt = 0;

    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        // 诊断：翻转LED1确认PID中断在触发
        DL_GPIO_togglePins(LED_PORT, LED_LED1_PIN);

        // 计算速度
        calculate_speed(MOTOR_1);
        calculate_speed(MOTOR_2);

        // 执行PID控制
        DC_MOTOR_PID(MOTOR_1);
        DC_MOTOR_PID(MOTOR_2);

        // 每10次中断发送一次VOFA数据 (500ms)
        vofa_cnt++;
        if (vofa_cnt >= 10) {
            vofa_cnt = 0;
            float vofa_data[4];
            vofa_data[0] = target_speed_1;  // 电机1目标速度
            vofa_data[1] = speed_1;         // 电机1实际速度
            vofa_data[2] = target_speed_2;  // 电机2目标速度
            vofa_data[3] = speed_2;         // 电机2实际速度
            VOFA_SendFrame(PRINT_INST, vofa_data, 4);
        }
        break;

    default:
        break;
    }
}
