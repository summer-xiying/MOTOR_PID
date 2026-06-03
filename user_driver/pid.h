#ifndef PID_H
#define PID_H

#include "ti_msp_dl_config.h"

/*
 *  ========================================
 *  PID控制模块
 *  ========================================
 *
 *  功能：
 *  - 增量式PID控制算法
 *  - 积分限幅
 *  - 积分抗饱和
 *  - 堵转保护
 *  - 速度计算
 */

// 电机1 PID参数 (可在运行时调整)
extern float kp_1;   // 比例系数
extern float ki_1;   // 积分系数
extern float kd_1;   // 微分系数

// 电机2 PID参数 (可在运行时调整)
extern float kp_2;   // 比例系数
extern float ki_2;   // 积分系数
extern float kd_2;   // 微分系数

// 电机1 PID状态变量
extern float speed_1;           // 当前速度 mm/s
extern float target_speed_1;    // 目标速度 mm/s
extern int16_t PWM_1_duty;     // 当前PWM占空比

// 电机2 PID状态变量
extern float speed_2;           // 当前速度 mm/s
extern float target_speed_2;    // 目标速度 mm/s
extern int16_t PWM_2_duty;     // 当前PWM占空比

/*
 *  ========================================
 *  函数声明
 *  ========================================
 */

// 速度计算 (由定时器中断调用)
void calculate_speed(uint8_t motor_id);

// PID控制算法 (由定时器中断调用)
void DC_MOTOR_PID(uint8_t motor_id);

// 设置目标速度
void pid_set_target_speed(uint8_t motor_id, float speed);

// 设置PID参数
void pid_set_params(uint8_t motor_id, float kp, float ki, float kd);

// 获取当前速度
float pid_get_speed(uint8_t motor_id);

// 获取当前误差
float pid_get_error(uint8_t motor_id);

#endif // PID_H
