#include "uart.h"

void UART_send_char(UART_Regs *uart, const uint8_t chr)
{
    DL_UART_transmitDataBlocking(uart, chr);
}

void UART_send_string(UART_Regs *uart, const char *str)
{
    while (*str) {
        UART_send_char(uart, (uint8_t) *str);
        str++;
    }
}

// VOFA+ JustFloat协议：发送单个float（小端序）
void VOFA_SendFloat(UART_Regs *uart, float data)
{
    uint8_t *p = (uint8_t *)&data;
    for (int i = 0; i < 4; i++) {
        UART_send_char(uart, p[i]);
    }
}

// VOFA+ JustFloat协议：发送一帧数据（带帧尾）
// data: float数组指针，len: 数据个数
void VOFA_SendFrame(UART_Regs *uart, float *data, uint8_t len)
{
    // 发送数据
    for (int i = 0; i < len; i++) {
        VOFA_SendFloat(uart, data[i]);
    }
    // 发送帧尾 0x00 0x00 0x80 0x7F
    UART_send_char(uart, 0x00);
    UART_send_char(uart, 0x00);
    UART_send_char(uart, 0x80);
    UART_send_char(uart, 0x7F);
}

void PRINT_INST_IRQHandler()
{
    switch (DL_UART_getPendingInterrupt(PRINT_INST))
    {
    case DL_UART_IIDX_RX:
        {   
            uint8_t rec = DL_UART_receiveData(PRINT_INST);
            UART_send_char(PRINT_INST, rec);
            break;
        }
    
    default:
        break;
    }
}

