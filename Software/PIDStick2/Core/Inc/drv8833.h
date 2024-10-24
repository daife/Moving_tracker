
#ifndef __DRV8833_H_
#define __DRV8833_H_

#include <stdint.h>

/**
 * @brief 衰减模式枚举
 */
typedef enum {
    SLOW_DECAY,  /**< 慢衰减模式 */
    FAST_DECAY   /**< 快衰减模式 */
} DecayMode;

/**
 * @brief 初始化DRV8833
 */
void DRV8833_Init(void);

/**
 * @brief 设置衰减模式
 * @param mode 衰减模式
 */
void DRV8833_SetDecayMode(DecayMode mode);

/**
 * @brief 控制电机前进
 * @param speed 速度值（0-100）
 */
void motor1_Forward(uint16_t speed);
void motor2_Forward(uint16_t speed);

/**
 * @brief 控制电机后退
 * @param speed 速度值（0-100）
 */
void DRV8833_Backward(uint16_t speed);

/**
 * @brief 电机刹车
 */
void motor1_Brake(void);
void motor2_Brake(void);

/**
 * @brief 电机滑行
 */
void motor1__Coast(void);
void motor2__Coast(void);


void DRV8833_Control(float param);


#endif /* __DRV8833_H_ */
