#ifndef UART_H
#define UART_H

#include "ti_msp_dl_config.h"

void UART_send_string(UART_Regs *uart, const char *str);
void UART_send_char(UART_Regs *uart, const uint8_t chr);

// VOFA+ JustFloat协议发送函数
void VOFA_SendFloat(UART_Regs *uart, float data);
void VOFA_SendFrame(UART_Regs *uart, float *data, uint8_t len);

// 发送8路灰度传感器数据（ASCII文本格式）
void UART_SendSensorData(UART_Regs *uart);

#endif /* UART_H */
