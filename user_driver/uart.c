#include "uart.h"
#include "tracking.h"
#include "pid.h"
#include "motor.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/*
 *  ========================================
 *  WirelessTune 二进制协议 - 内部常量
 *  ========================================
 */
#define WIRELESS_PID_PAYLOAD_LEN       13u
#define WIRELESS_SPEED_PAYLOAD_LEN     8u
#define WIRELESS_SWITCH_PAYLOAD_LEN    1u
#define WIRELESS_STATUS_PAYLOAD_LEN    54u
#define WIRELESS_PID_PARAM_MAX         1000.0f
#define WIRELESS_SPEED_MAX             500.0f
#define WIRELESS_MOTOR_BOTH            0u

typedef enum {
    WIRELESS_STATE_HEAD1 = 0,
    WIRELESS_STATE_HEAD2,
    WIRELESS_STATE_CMD,
    WIRELESS_STATE_LEN,
    WIRELESS_STATE_DATA,
    WIRELESS_STATE_CHECK
} WirelessParseState;

// 循迹器实例（在main.c中定义）
extern LineTracker line_tracker;

// 二进制协议解析状态
static volatile WirelessParseState rx_state = WIRELESS_STATE_HEAD1;
static volatile uint8_t rx_cmd = 0;
static volatile uint8_t rx_len = 0;
static volatile uint8_t rx_idx = 0;
static volatile uint8_t rx_check = 0;
static volatile uint8_t rx_buf[WIRELESS_FRAME_MAX_LEN];
static volatile uint8_t frame_ready = 0;
static volatile uint8_t frame_cmd = 0;
static volatile uint8_t frame_len = 0;
static volatile uint8_t frame_buf[WIRELESS_FRAME_MAX_LEN];
static volatile uint32_t parse_error_count = 0;
static uint8_t debug_output_enabled = 0;
volatile uint8_t tracking_enabled = 1;

// 内部函数声明
static void WirelessTune_PushByte(uint8_t byte);
static void WirelessTune_SendFrame(uint8_t cmd, const uint8_t *data, uint8_t len);
static void WirelessTune_SendAck(uint8_t cmd);
static void WirelessTune_SendError(uint8_t cmd, uint8_t error);
static void WirelessTune_SendStatus(void);
static void WirelessTune_PutFloat(uint8_t *buf, uint8_t *idx, float value);
static float WirelessTune_GetFloat(const uint8_t *buf);
static uint8_t WirelessTune_FloatValid(float value, float abs_limit);

// VOFA+文本命令缓冲区
#define VOFA_TEXT_BUF_SIZE 64
static char vofa_text_buf[VOFA_TEXT_BUF_SIZE];
static uint8_t vofa_text_idx = 0;
static uint8_t vofa_text_ready = 0;

/*
 *  ========================================
 *  基础UART发送函数
 *  ========================================
 */
void UART_send_char(UART_Regs *uart, const uint8_t chr)
{
    while (DL_UART_isTXFIFOFull(uart)) {}
    DL_UART_transmitData(uart, chr);
}

void UART_send_string(UART_Regs *uart, const char *str)
{
    while (*str) {
        UART_send_char(uart, (uint8_t) *str);
        str++;
    }
}

/*
 *  ========================================
 *  VOFA+ JustFloat协议
 *  帧尾: 0x00 0x00 0x80 0x7F
 *  ========================================
 */
void VOFA_SendFloat(UART_Regs *uart, float data)
{
    uint8_t *p = (uint8_t *)&data;
    for (int i = 0; i < 4; i++) {
        UART_send_char(uart, p[i]);
    }
}

void VOFA_SendFrame(UART_Regs *uart, float *data, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        VOFA_SendFloat(uart, data[i]);
    }
    UART_send_char(uart, 0x00);
    UART_send_char(uart, 0x00);
    UART_send_char(uart, 0x80);
    UART_send_char(uart, 0x7F);
}

/*
 *  ========================================
 *  循迹调试数据（ASCII文本格式，保留原有功能）
 *  ========================================
 */
void UART_SendSensorData(UART_Regs *uart)
{
    float data[6];
    data[0] = line_tracker.position;
    data[1] = debug_tracking_output;
    data[2] = target_speed_1;
    data[3] = target_speed_2;
    data[4] = speed_1;
    data[5] = speed_2;
    VOFA_SendFrame(uart, data, 6);
}

/*
 *  ========================================
 *  VOFA PID波形发送
 *  通道：目标速度1, 实际速度1, PWM1, 目标速度2, 实际速度2, PWM2
 *  ========================================
 */
void VOFA_SendPIDWaveform(UART_Regs *uart)
{
    float waveform_data[6];
    waveform_data[0] = target_speed_1;
    waveform_data[1] = speed_1;
    waveform_data[2] = (float)PWM_1_duty;
    waveform_data[3] = target_speed_2;
    waveform_data[4] = speed_2;
    waveform_data[5] = (float)PWM_2_duty;
    VOFA_SendFrame(uart, waveform_data, 6);
}

/*
 *  ========================================
 *  WirelessTune 二进制协议处理（主循环调用）
 *  ========================================
 */
void WirelessTune_Process(void)
{
    uint8_t cmd;
    uint8_t len;
    uint8_t data[WIRELESS_FRAME_MAX_LEN];

    __disable_irq();
    if (!frame_ready) {
        __enable_irq();
        return;
    }
    cmd = frame_cmd;
    len = frame_len;
    for (uint8_t i = 0; i < len; i++) {
        data[i] = frame_buf[i];
    }
    frame_ready = 0;
    __enable_irq();

    if (cmd == WIRELESS_CMD_SET_PID) {
        if (len != WIRELESS_PID_PAYLOAD_LEN) {
            WirelessTune_SendError(cmd, WIRELESS_ERR_BAD_LEN);
            return;
        }
        uint8_t motor_id = data[0];
        float kp_val = WirelessTune_GetFloat(&data[1]);
        float ki_val = WirelessTune_GetFloat(&data[5]);
        float kd_val = WirelessTune_GetFloat(&data[9]);

        if (motor_id != WIRELESS_MOTOR_BOTH && motor_id != MOTOR_1 && motor_id != MOTOR_2) {
            WirelessTune_SendError(cmd, WIRELESS_ERR_BAD_MOTOR);
            return;
        }
        if (!WirelessTune_FloatValid(kp_val, WIRELESS_PID_PARAM_MAX) ||
            !WirelessTune_FloatValid(ki_val, WIRELESS_PID_PARAM_MAX) ||
            !WirelessTune_FloatValid(kd_val, WIRELESS_PID_PARAM_MAX) ||
            kp_val < 0.0f || ki_val < 0.0f || kd_val < 0.0f) {
            WirelessTune_SendError(cmd, WIRELESS_ERR_BAD_VALUE);
            return;
        }
        pid_set_params(motor_id, kp_val, ki_val, kd_val);
        WirelessTune_SendAck(cmd);

    } else if (cmd == WIRELESS_CMD_SET_SPEED) {
        if (len != WIRELESS_SPEED_PAYLOAD_LEN) {
            WirelessTune_SendError(cmd, WIRELESS_ERR_BAD_LEN);
            return;
        }
        float left_speed = WirelessTune_GetFloat(&data[0]);
        float right_speed = WirelessTune_GetFloat(&data[4]);
        if (!WirelessTune_FloatValid(left_speed, WIRELESS_SPEED_MAX) ||
            !WirelessTune_FloatValid(right_speed, WIRELESS_SPEED_MAX) ||
            left_speed < 0.0f || right_speed < 0.0f) {
            WirelessTune_SendError(cmd, WIRELESS_ERR_BAD_VALUE);
            return;
        }
        tracking_enabled = 0;
        pid_set_target_speed(MOTOR_1, left_speed);
        pid_set_target_speed(MOTOR_2, right_speed);
        WirelessTune_SendAck(cmd);

    } else if (cmd == WIRELESS_CMD_GET_STATUS) {
        if (len != 0) {
            WirelessTune_SendError(cmd, WIRELESS_ERR_BAD_LEN);
            return;
        }
        WirelessTune_SendStatus();

    } else if (cmd == WIRELESS_CMD_SET_OUTPUT) {
        if (len != WIRELESS_SWITCH_PAYLOAD_LEN) {
            WirelessTune_SendError(cmd, WIRELESS_ERR_BAD_LEN);
            return;
        }
        debug_output_enabled = data[0] ? 1 : 0;
        WirelessTune_SendAck(cmd);

    } else if (cmd == WIRELESS_CMD_SET_TRACKING) {
        if (len != WIRELESS_SWITCH_PAYLOAD_LEN) {
            WirelessTune_SendError(cmd, WIRELESS_ERR_BAD_LEN);
            return;
        }
        tracking_enabled = data[0] ? 1 : 0;
        WirelessTune_SendAck(cmd);

    } else {
        WirelessTune_SendError(cmd, WIRELESS_ERR_BAD_CMD);
    }
}

uint8_t WirelessTune_IsDebugOutputEnabled(void)
{
    return debug_output_enabled;
}

void WirelessTune_SetDebugOutput(uint8_t enabled)
{
    debug_output_enabled = enabled ? 1 : 0;
}

uint8_t WirelessTune_IsTrackingEnabled(void)
{
    return tracking_enabled;
}

uint32_t WirelessTune_GetParseErrorCount(void)
{
    return parse_error_count;
}

/*
 *  ========================================
 *  WirelessTune 状态机解析器（在UART RX中断中调用）
 *  ========================================
 */
static void WirelessTune_PushByte(uint8_t byte)
{
    switch (rx_state) {
    case WIRELESS_STATE_HEAD1:
        if (byte == WIRELESS_FRAME_HEAD1) {
            rx_state = WIRELESS_STATE_HEAD2;
        }
        break;

    case WIRELESS_STATE_HEAD2:
        rx_state = (byte == WIRELESS_FRAME_HEAD2) ? WIRELESS_STATE_CMD : WIRELESS_STATE_HEAD1;
        break;

    case WIRELESS_STATE_CMD:
        rx_cmd = byte;
        rx_check = byte;
        rx_state = WIRELESS_STATE_LEN;
        break;

    case WIRELESS_STATE_LEN:
        rx_len = byte;
        rx_check += byte;
        rx_idx = 0;
        if (rx_len > WIRELESS_FRAME_MAX_LEN) {
            rx_state = WIRELESS_STATE_HEAD1;
            parse_error_count++;
        } else if (rx_len == 0) {
            rx_state = WIRELESS_STATE_CHECK;
        } else {
            rx_state = WIRELESS_STATE_DATA;
        }
        break;

    case WIRELESS_STATE_DATA:
        rx_buf[rx_idx++] = byte;
        rx_check += byte;
        if (rx_idx >= rx_len) {
            rx_state = WIRELESS_STATE_CHECK;
        }
        break;

    case WIRELESS_STATE_CHECK:
        rx_state = WIRELESS_STATE_HEAD1;
        if (byte != rx_check || frame_ready) {
            parse_error_count++;
            break;
        }
        frame_cmd = rx_cmd;
        frame_len = rx_len;
        for (uint8_t i = 0; i < rx_len; i++) {
            frame_buf[i] = rx_buf[i];
        }
        frame_ready = 1;
        break;

    default:
        rx_state = WIRELESS_STATE_HEAD1;
        break;
    }
}

/*
 *  ========================================
 *  WirelessTune 帧发送（内部辅助函数）
 *  ========================================
 */
static void WirelessTune_SendFrame(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t check = cmd + len;

    if (len > WIRELESS_FRAME_MAX_LEN || (len > 0 && data == NULL)) {
        return;
    }

    UART_send_char(PRINT_INST, WIRELESS_FRAME_HEAD1);
    UART_send_char(PRINT_INST, WIRELESS_FRAME_HEAD2);
    UART_send_char(PRINT_INST, cmd);
    UART_send_char(PRINT_INST, len);
    for (uint8_t i = 0; i < len; i++) {
        check += data[i];
        UART_send_char(PRINT_INST, data[i]);
    }
    UART_send_char(PRINT_INST, check);
}

static void WirelessTune_SendAck(uint8_t cmd)
{
    uint8_t data[1] = {cmd};
    WirelessTune_SendFrame(WIRELESS_RSP_ACK, data, 1);
}

static void WirelessTune_SendError(uint8_t cmd, uint8_t error)
{
    uint8_t data[2] = {cmd, error};
    WirelessTune_SendFrame(WIRELESS_RSP_ERROR, data, 2);
}

static void WirelessTune_SendStatus(void)
{
    uint8_t data[WIRELESS_STATUS_PAYLOAD_LEN];
    uint8_t idx = 0;
    uint32_t errors = parse_error_count;

    data[idx++] = debug_output_enabled;
    data[idx++] = tracking_enabled;
    data[idx++] = (uint8_t)(errors & 0xFFu);
    data[idx++] = (uint8_t)((errors >> 8) & 0xFFu);
    data[idx++] = (uint8_t)((errors >> 16) & 0xFFu);
    data[idx++] = (uint8_t)((errors >> 24) & 0xFFu);

    // 电机1 PID + 状态
    WirelessTune_PutFloat(data, &idx, kp_1);
    WirelessTune_PutFloat(data, &idx, ki_1);
    WirelessTune_PutFloat(data, &idx, kd_1);
    WirelessTune_PutFloat(data, &idx, target_speed_1);
    WirelessTune_PutFloat(data, &idx, speed_1);
    WirelessTune_PutFloat(data, &idx, pid_get_error(MOTOR_1));

    // 电机2 PID + 状态
    WirelessTune_PutFloat(data, &idx, kp_2);
    WirelessTune_PutFloat(data, &idx, ki_2);
    WirelessTune_PutFloat(data, &idx, kd_2);
    WirelessTune_PutFloat(data, &idx, target_speed_2);
    WirelessTune_PutFloat(data, &idx, speed_2);
    WirelessTune_PutFloat(data, &idx, pid_get_error(MOTOR_2));

    WirelessTune_SendFrame(WIRELESS_RSP_STATUS, data, idx);
}

static void WirelessTune_PutFloat(uint8_t *buf, uint8_t *idx, float value)
{
    memcpy(&buf[*idx], &value, sizeof(value));
    *idx += sizeof(value);
}

static float WirelessTune_GetFloat(const uint8_t *buf)
{
    float value;
    memcpy(&value, buf, sizeof(value));
    return value;
}

static uint8_t WirelessTune_FloatValid(float value, float abs_limit)
{
    return isfinite(value) && value >= -abs_limit && value <= abs_limit;
}

/*
 *  ========================================
 *  UART RX 中断处理
 *  同时驱动：二进制协议状态机 + 文本命令缓冲
 *  ========================================
 */
void PRINT_INST_IRQHandler()
{
    switch (DL_UART_getPendingInterrupt(PRINT_INST))
    {
    case DL_UART_IIDX_RX:
        while (!DL_UART_isRXFIFOEmpty(PRINT_INST)) {
            uint8_t byte = DL_UART_receiveData(PRINT_INST);

            // 二进制协议解析
            WirelessTune_PushByte(byte);

            // 文本命令缓冲（以\n或\r结尾）
            if (!vofa_text_ready) {
                if (byte == '\n' || byte == '\r') {
                    if (vofa_text_idx > 0) {
                        vofa_text_buf[vofa_text_idx] = '\0';
                        vofa_text_ready = 1;
                    }
                } else if (vofa_text_idx < VOFA_TEXT_BUF_SIZE - 1) {
                    vofa_text_buf[vofa_text_idx++] = byte;
                }
            }
        }
        break;

    default:
        break;
    }
}

/*
 *  ========================================
 *  VOFA+文本命令解析（主循环调用）
 *  支持VOFA+滑块组件的 name:value 格式
 *
 *  滑块名称（在VOFA+中配置）：
 *    kp / ki / kd           - 速度环PID参数（同时设置两个电机）
 *    speed                  - 目标速度
 *    track                  - 循迹开关 (0/1)
 *    output                 - 波形输出开关 (0/1)
 *    kp_straight / kd_straight - 直线模式循迹PD参数
 *    kp_curve / kd_curve       - 弯道模式循迹PD参数
 *
 *  示例：kp:0.65\n  speed:150\n  track:1\n  kp_straight:12.0\n
 *  ========================================
 */
void VOFA_TextCommandProcess(void)
{
    if (!vofa_text_ready) return;

    char *buf = vofa_text_buf;
    char name[16];
    float value;

    if (sscanf(buf, "%15[^:]:%f", name, &value) == 2) {
        if (strcmp(name, "kp") == 0) {
            if (value >= 0 && isfinite(value)) {
                kp_1 = value;
                kp_2 = value;
                char ack[24];
                sprintf(ack, "kp=%.2f\r\n", value);
                UART_send_string(PRINT_INST, ack);
            }
        } else if (strcmp(name, "ki") == 0) {
            if (value >= 0 && isfinite(value)) {
                ki_1 = value;
                ki_2 = value;
                char ack[24];
                sprintf(ack, "ki=%.2f\r\n", value);
                UART_send_string(PRINT_INST, ack);
            }
        } else if (strcmp(name, "kd") == 0) {
            if (value >= 0 && isfinite(value)) {
                kd_1 = value;
                kd_2 = value;
                char ack[24];
                sprintf(ack, "kd=%.2f\r\n", value);
                UART_send_string(PRINT_INST, ack);
            }
        } else if (strcmp(name, "speed") == 0) {
            if (value >= 0 && isfinite(value)) {
                tracking_enabled = 0;
                pid_set_target_speed(MOTOR_1, value);
                pid_set_target_speed(MOTOR_2, value);
                char ack[24];
                sprintf(ack, "speed=%.1f\r\n", value);
                UART_send_string(PRINT_INST, ack);
            }
        } else if (strcmp(name, "track") == 0) {
            tracking_enabled = (value != 0) ? 1 : 0;
            char ack[16];
            sprintf(ack, "track=%d\r\n", tracking_enabled);
            UART_send_string(PRINT_INST, ack);
        } else if (strcmp(name, "output") == 0) {
            debug_output_enabled = (value != 0) ? 1 : 0;
            char ack[16];
            sprintf(ack, "output=%d\r\n", debug_output_enabled);
            UART_send_string(PRINT_INST, ack);
        } else if (strcmp(name, "kp_straight") == 0) {
            if (value >= 0 && isfinite(value)) {
                kp_straight = value;
                char ack[24];
                sprintf(ack, "kp_straight=%.2f\r\n", value);
                UART_send_string(PRINT_INST, ack);
            }
        } else if (strcmp(name, "kd_straight") == 0) {
            if (value >= 0 && isfinite(value)) {
                kd_straight = value;
                char ack[24];
                sprintf(ack, "kd_straight=%.2f\r\n", value);
                UART_send_string(PRINT_INST, ack);
            }
        } else if (strcmp(name, "kp_curve") == 0) {
            if (value >= 0 && isfinite(value)) {
                kp_curve = value;
                char ack[24];
                sprintf(ack, "kp_curve=%.2f\r\n", value);
                UART_send_string(PRINT_INST, ack);
            }
        } else if (strcmp(name, "kd_curve") == 0) {
            if (value >= 0 && isfinite(value)) {
                kd_curve = value;
                char ack[24];
                sprintf(ack, "kd_curve=%.2f\r\n", value);
                UART_send_string(PRINT_INST, ack);
            }
        }
    }

    vofa_text_idx = 0;
    vofa_text_ready = 0;
}
