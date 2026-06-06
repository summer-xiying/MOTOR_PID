/**
 * @file tracking.h
 * @brief 八路灰度传感器循迹模块
 * 支持HJ-DXJ8八路循迹模块，含加权位置计算、十字检测
 */

#ifndef TRACKING_H
#define TRACKING_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define SENSOR_COUNT 8  /**< 传感器数量 */

/** @brief 循迹器状态结构 */
typedef struct {
    uint8_t sensor_raw[SENSOR_COUNT]; /**< 原始值（0/1） */
    float   position;                 /**< 偏离中心的位置 (-1.0 ~ 1.0) */
    uint8_t on_line;                  /**< 是否检测到线 */
    uint8_t cross_detected;          /**< 十字路口检测 */
} LineTracker;

/**
 * @brief 初始化循迹器
 * @param lt 循迹器结构体指针
 * @return 0=成功，-1=参数无效
 */
int LineTracker_Init(LineTracker *lt);

/**
 * @brief 读取传感器并计算位置
 * @param lt 循迹器结构体指针
 * @return 0=成功，-1=参数无效
 */
int LineTracker_Update(LineTracker *lt);

/**
 * @brief 循迹PD控制，将位置偏差转换为速度修正值
 * @param lt 循迹器结构体指针
 * @param kp 比例系数
 * @param kd 微分系数
 * @param last_error 上一次误差（会更新）
 * @return 速度修正值（加到左轮、从右轮减去），无效输入返回0
 */
float LineTracker_GetPIDOutput(LineTracker *lt,
                                float kp, float kd,
                                float *last_error);

/**
 * @brief 获取传感器原始值（用于调试）
 * @param sensor_index 传感器索引（0-7）
 * @return 传感器值（0或1）
 */
uint8_t LineTracker_GetSensorValue(uint8_t sensor_index);

#endif /* TRACKING_H */
