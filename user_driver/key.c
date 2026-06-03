#include "key.h"
extern int status;

uint8_t get_key_state(uint32_t key) {
    uint32_t high_bits = DL_GPIO_readPins(KEY_PORT, key); //0x00000040 0b01000000 PB6 0~31
    if((high_bits & key) != 0) return 1;
    else return 0;
}

/*
 *  ========================================
 *  编码器计数变量
 *  ========================================
 */
uint32_t counter_1_A = 0;  // 电机1编码器计数
uint32_t counter_2_A = 0;  // 电机2编码器计数

/*
 *  ========================================
 *  GPIO中断服务函数
 *  处理按键和编码器信号
 *  ========================================
 */
void GROUP1_IRQHandler()
{
    // 处理GPIOB的中断 (按键)
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
    case KEY_KEY9_IIDX:
        status = (status + 1) % 3;
        break;
    case KEY_KEY10_IIDX:
        status = (status + 3 - 1) % 3;
        break;

    // 电机2编码器A相中断
    case DC_MOTOR_BA_IIDX:
        counter_2_A++;
        break;

    default:
        break;
    }

    // 处理GPIOA的中断 (电机1编码器)
    switch (DL_GPIO_getPendingInterrupt(GPIOA))
    {
    case DC_MOTOR_AA_IIDX:
        counter_1_A++;
        break;

    default:
        break;
    }
}
