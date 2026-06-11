#ifndef UART_H
#define UART_H

#include "ti_msp_dl_config.h"

/*
 *  ========================================
 *  WirelessTune 二进制协议定义
 *  帧格式: [HEAD1][HEAD2][CMD][LEN][DATA...][CHECK]
 *  校验: CHECK = CMD + LEN + SUM(DATA)
 *  ========================================
 */
#define WIRELESS_FRAME_HEAD1        0xAAu
#define WIRELESS_FRAME_HEAD2        0x55u
#define WIRELESS_FRAME_MAX_LEN      64u

// 命令字（上位机 -> 下位机）
#define WIRELESS_CMD_SET_PID        0x01u
#define WIRELESS_CMD_SET_SPEED      0x02u
#define WIRELESS_CMD_GET_STATUS     0x03u
#define WIRELESS_CMD_SET_OUTPUT     0x04u
#define WIRELESS_CMD_SET_TRACKING   0x05u

// 响应字（下位机 -> 上位机）
#define WIRELESS_RSP_ACK            0x80u
#define WIRELESS_RSP_ERROR          0x81u
#define WIRELESS_RSP_STATUS         0x82u

// 错误码
#define WIRELESS_ERR_BAD_LEN        0x01u
#define WIRELESS_ERR_BAD_MOTOR      0x02u
#define WIRELESS_ERR_BAD_VALUE      0x03u
#define WIRELESS_ERR_BAD_CMD        0x04u

/*
 *  ========================================
 *  基础UART发送函数
 *  ========================================
 */
void UART_send_string(UART_Regs *uart, const char *str);
void UART_send_char(UART_Regs *uart, const uint8_t chr);

/*
 *  ========================================
 *  VOFA+ JustFloat协议发送函数
 *  ========================================
 */
void VOFA_SendFloat(UART_Regs *uart, float data);
void VOFA_SendFrame(UART_Regs *uart, float *data, uint8_t len);

// 循迹调试数据（ASCII文本格式）
void UART_SendSensorData(UART_Regs *uart);

// PID波形数据（JustFloat格式，含PWM）
void VOFA_SendPIDWaveform(UART_Regs *uart);

/*
 *  ========================================
 *  WirelessTune 二进制协议处理
 *  ========================================
 */
void WirelessTune_Process(void);
uint8_t WirelessTune_IsDebugOutputEnabled(void);
uint8_t WirelessTune_IsTrackingEnabled(void);
void WirelessTune_SetDebugOutput(uint8_t enabled);
uint32_t WirelessTune_GetParseErrorCount(void);

/*
 *  ========================================
 *  VOFA+文本命令解析（滑块协议）
 *  支持 name:value 格式，如 kp:0.65
 *  命令：kp/ki/kd/speed/track/output
 *  ========================================
 */
void VOFA_TextCommandProcess(void);

#endif /* UART_H */
