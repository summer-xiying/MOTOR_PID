#include "motor.h"

/*
 *  ========================================
 *  电机初始化函数
 *  ========================================
 */
void motor_init(uint8_t motor_id)
{
    // 使能TB6612
    // DL_GPIO_setPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);

    if(motor_id == MOTOR_1){
        // 启动电机1的PWM定时器
        DL_Timer_startCounter(PWMA_INST);
        // 初始方向：停止 (AIN1=1, AIN2=1)
        DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
        DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        // 初始占空比为0
        DL_Timer_setCaptureCompareValue(PWMA_INST, 0, GPIO_PWMA_C0_IDX);
    }
    else if(motor_id == MOTOR_2){
        // 启动电机2的PWM定时器
        DL_Timer_startCounter(PWMB_INST);
        // 初始方向：停止 (BIN1=1, BIN2=1)
        DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
        DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        // 初始占空比为0
        DL_Timer_setCaptureCompareValue(PWMB_INST, 0, GPIO_PWMB_C0_IDX);
    }

    // 启动PID定时器（只需初始化一次）
    static uint8_t pid_timer_started = 0;
    if(!pid_timer_started){
        DL_Timer_clearInterruptStatus(MOTOR_PID_INST, DL_TIMER_INTERRUPT_LOAD_EVENT);
        NVIC_ClearPendingIRQ(MOTOR_PID_INST_INT_IRQN);
        DL_Timer_startCounter(MOTOR_PID_INST);
        NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);
        pid_timer_started = 1;
    }
}

/*
 *  ========================================
 *  设置电机占空比
 *  duty: 0~4000
 *  ========================================
 */
void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    // 限幅输出：确保duty在0~4000范围内
    if(duty > 4000){
        duty = 4000;
    }

    if(motor_id == MOTOR_1){
        DL_Timer_setCaptureCompareValue(PWMA_INST, duty, GPIO_PWMA_C0_IDX);
    }
    else if(motor_id == MOTOR_2){
        DL_Timer_setCaptureCompareValue(PWMB_INST, duty, GPIO_PWMB_C0_IDX);
    }
}

/*
 *  ========================================
 *  设置电机方向
 *  direction: 0 停止，1 正转，2 反转
 *  ========================================
 */
void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if(motor_id == MOTOR_1){
        if(direction == 0){
            // 停止：AIN1=1, AIN2=1
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if(direction == 1){
            // 正转（前进）：AIN1=0, AIN2=1
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if(direction == 2){
            // 反转（后退）：AIN1=1, AIN2=0
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
    }
    else if(motor_id == MOTOR_2){
        if(direction == 0){
            // 停止：BIN1=1, BIN2=1
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if(direction == 1){
            // 正转：BIN1=1, BIN2=0
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if(direction == 2){
            // 反转：BIN1=0, BIN2=1
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
    }
}
