/**
 * @file tracking.c
 * @brief 八路灰度传感器循迹模块实现
 * 支持HJ-DXJ8八路循迹模块
 */

#include "tracking.h"
#include <math.h>

/** @brief 传感器位置权重（由Init填充） */
static float sensor_weights[SENSOR_COUNT];

/** @brief 传感器引脚映射表（P1-P8对应GPIO引脚） */
static const struct {
    GPIO_Regs *port;
    uint32_t pin;
} sensor_pins[SENSOR_COUNT] = {
    {HUIDU_P1_PORT, HUIDU_P1_PIN},  // P1 - PA2
    {HUIDU_P2_PORT, HUIDU_P2_PIN},  // P2 - PB24
    {HUIDU_P3_PORT, HUIDU_P3_PIN},  // P3 - PB20
    {HUIDU_P4_PORT, HUIDU_P4_PIN},  // P4 - PB19
    {HUIDU_P5_PORT, HUIDU_P5_PIN},  // P5 - PB18
    {HUIDU_P6_PORT, HUIDU_P6_PIN},  // P6 - PA7
    {HUIDU_P7_PORT, HUIDU_P7_PIN},  // P7 - PB2
    {HUIDU_P8_PORT, HUIDU_P8_PIN},  // P8 - PB3
};

int LineTracker_Init(LineTracker *lt)
{
    if (lt == NULL) return -1;

    // 初始化权重：外侧更大，增强弯道响应
    // P1=-5.0, P2=-3.5, P3=-1.5, P4=-0.5, P5=0.5, P6=1.5, P7=3.5, P8=5.0
    sensor_weights[0] = -5.0f;
    sensor_weights[1] = -3.5f;
    sensor_weights[2] = -1.5f;
    sensor_weights[3] = -0.5f;
    sensor_weights[4] = 0.5f;
    sensor_weights[5] = 1.5f;
    sensor_weights[6] = 3.5f;
    sensor_weights[7] = 5.0f;

    lt->position = 0;
    lt->on_line = 0;
    lt->cross_detected = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        lt->sensor_raw[i] = 0;
    }
    return 0;
}

int LineTracker_Update(LineTracker *lt)
{
    if (lt == NULL) return -1;

    // 读取传感器原始值
    // HJ-DXJ8模块：检测到黑线输出低电平(0)，白底输出高电平(1)
    // 反转逻辑：sensor_raw=1表示检测到黑线，0表示白底
    for (int i = 0; i < SENSOR_COUNT; i++) {
        uint32_t pin_value = DL_GPIO_readPins(sensor_pins[i].port, sensor_pins[i].pin);
        lt->sensor_raw[i] = (pin_value == 0) ? 1 : 0;
    }

    // 根据传感器状态计算位置
    float sum = 0;
    int active_count = 0;
    static float filtered_position = 0;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (lt->sensor_raw[i]) {
            sum += sensor_weights[i];
            active_count++;
        }
    }

    lt->cross_detected = (active_count == SENSOR_COUNT) ? 1 : 0;

    if (active_count == 0) {
        lt->position = 0;
        lt->on_line = 0;
        filtered_position = 0;
        return 0;
    }

    if (lt->cross_detected) {
        lt->position = 0;
        lt->on_line = 1;
        filtered_position = 0;
        return 0;
    }

    // 单侧识别时按最外侧位置累计中心到边缘的权重；跨中心识别时用原始加权和
    int leftmost = SENSOR_COUNT;
    int rightmost = -1;
    uint8_t has_left = 0;
    uint8_t has_right = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (lt->sensor_raw[i]) {
            if (i < leftmost) leftmost = i;
            if (i > rightmost) rightmost = i;
            if (i < SENSOR_COUNT / 2) has_left = 1;
            else has_right = 1;
        }
    }

    if (has_left && !has_right) {
        sum = 0;
        for (int i = leftmost; i < SENSOR_COUNT / 2; i++) {
            sum += sensor_weights[i];
        }
    } else if (!has_left && has_right) {
        sum = 0;
        for (int i = SENSOR_COUNT / 2; i <= rightmost; i++) {
            sum += sensor_weights[i];
        }
    }

    // 低通滤波：平滑位置变化，减少剧烈摆动
    float alpha = 0.5f;
    filtered_position = alpha * sum + (1.0f - alpha) * filtered_position;
    lt->position = filtered_position;
    lt->on_line = 1;

    return 0;
}

float LineTracker_GetPIDOutput(LineTracker *lt,
                                float kp, float kd,
                                float *last_error)
{
    if (lt == NULL || last_error == NULL) return 0.0f;
    if (!isfinite(kp) || !isfinite(kd)) return 0.0f;
    if (!lt->on_line) return 0.0f;  // 丢线时不输出修正

    float error = lt->position;
    float derivative = error - *last_error;
    *last_error = error;

    // D项限幅：防止位置跳变时derivative尖峰
    #define DERIVATIVE_LIMIT 3.0f
    if (derivative > DERIVATIVE_LIMIT) derivative = DERIVATIVE_LIMIT;
    if (derivative < -DERIVATIVE_LIMIT) derivative = -DERIVATIVE_LIMIT;

    float output = kp * error + kd * derivative;

    // PD输出限幅：外侧传感器触发时临时增加急弯救援裕量
    #define TRACKING_OUTPUT_LIMIT       70.0f
    #define TRACKING_EDGE_OUTPUT_LIMIT  78.0f
    float output_limit = TRACKING_OUTPUT_LIMIT;
    if (lt->sensor_raw[0] || lt->sensor_raw[SENSOR_COUNT - 1]) {
        output_limit = TRACKING_EDGE_OUTPUT_LIMIT;
    }
    if (output > output_limit) output = output_limit;
    if (output < -output_limit) output = -output_limit;

    return output;
}

uint8_t LineTracker_GetSensorValue(uint8_t sensor_index)
{
    if (sensor_index >= SENSOR_COUNT) return 0;
    uint32_t pin_value = DL_GPIO_readPins(sensor_pins[sensor_index].port, sensor_pins[sensor_index].pin);
    return (pin_value == 0) ? 1 : 0;  // 1=检测到黑线，0=白底
}
