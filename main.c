/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
// #include "oled.h"  // OLED已禁用，引脚已删除
#include <stdio.h>
#include "uart.h"
#include "key.h"
#include "motor.h"
#include "pid.h"
#include "tracking.h"

int status = 0;

// 循迹器实例
LineTracker line_tracker;
float tracking_last_error = 0;

int main(void)
{
    SYSCFG_DL_init();

    // OLED初始化（已禁用，引脚已删除）
    // OLED_Init();
    // OLED_ColorTurn(0);
    // OLED_DisplayTurn(0);
    // OLED_Clear();

    /*
     * 集中使能所有NVIC中断
     * GPIO_MULTIPLE_GPIOB_INT_IRQN = DC_MOTOR_GPIOA_INT_IRQN = GROUP1 (IRQ 1)
     * 处理：按键(PB6/PB7)、编码器(PA17/PB8/PB9)
     */
    DL_GPIO_disableInterrupt(GPIOB, DC_MOTOR_BB_PIN);
    DL_GPIO_clearInterruptStatus(GPIOB, KEY_KEY9_PIN |
        KEY_KEY10_PIN |
        DC_MOTOR_BA_PIN |
        DC_MOTOR_BB_PIN);
    DL_GPIO_clearInterruptStatus(GPIOA, DC_MOTOR_AA_PIN);
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_ClearPendingIRQ(PRINT_INST_INT_IRQN);
    NVIC_EnableIRQ(PRINT_INST_INT_IRQN);  // 使能UART中断

    // 初始化循迹器
    LineTracker_Init(&line_tracker);

    // 启动后立即发送测试字符串，验证UART硬件是否工作
    UART_send_string(PRINT_INST, "UART OK\r\n");
    while (!DL_UART_isTXFIFOEmpty(PRINT_INST)) {}
    while (DL_UART_isBusy(PRINT_INST)) {}

    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);

    motor_init(MOTOR_1);
    motor_init(MOTOR_2);

    // 设置电机方向：都前进
    motor_set_direction(MOTOR_1, 1);
    motor_set_direction(MOTOR_2, 1);

    /*
     * PID控制定时器 (TIMA0, IRQ 18)
     * 10ms周期，执行循迹控制、速度计算和PID控制
     * 循迹控制已在中断中集成，主循环无需处理
     */
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);

    uint32_t led_next_toggle = 0;
    uint8_t led_state = 0;

    // 启动后自动开启调试输出和循迹控制
    WirelessTune_SetDebugOutput(1);
    tracking_enabled = 1;

    while (1) {
        // 处理在线调参命令
        WirelessTune_Process();
        VOFA_TextCommandProcess();

        // 检查串口发送标志（由定时器ISR设置，每100ms一次）
        if (uart_send_flag) {
            uart_send_flag = 0;
            if (WirelessTune_IsDebugOutputEnabled()) {
                VOFA_SendPIDWaveform(PRINT_INST);
            }
        }

        // LED闪烁：1秒周期（亮0.5s灭0.5s）
        // led_tick由定时器ISR每10ms递增，50次=500ms
        if (led_tick >= led_next_toggle) {
            led_next_toggle = led_tick + 50;
            led_state = !led_state;
            if (led_state) {
                DL_GPIO_setPins(LED_PORT, LED_LED0_PIN);
            } else {
                DL_GPIO_clearPins(LED_PORT, LED_LED0_PIN);
            }
        }
    }
}
