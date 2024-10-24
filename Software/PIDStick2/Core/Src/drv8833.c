
/**
 * @file drv8833.c
 * @brief DRV8833双H桥电机驱动器控制实现
 */

#include "drv8833.h"

#include <math.h>

#include "tim.h"

/** @brief IN1引脚的定时器 */
#define IN1_TIM &htim2
/** @brief IN2引脚的定时器 */
#define IN2_TIM &htim2

// #define IN3_TIM &htim2
// #define IN4_TIM &htim2

/** @brief IN1引脚的定时器通道 */
#define IN1_CH TIM_CHANNEL_1
/** @brief IN2引脚的定时器通道 */
#define IN2_CH TIM_CHANNEL_2

// #define IN3_CH TIM_CHANNEL_3
//
// #define IN4_CH TIM_CHANNEL_4

/** @brief 最大速度值 */
#define MAX_SPEED 255
#define BackwardRatio 1.3//由于反转效率低，适当地增强
#define Ratio 0.90
/** @brief 当前衰减模式 */
static DecayMode currentDecayMode = FAST_DECAY;

/**
 * @brief 设置IN1引脚的PWM占空比
 * @param duty 占空比值
 */
static inline void __SetIn1PWM(uint16_t duty)
{
    __HAL_TIM_SET_COMPARE(IN1_TIM, IN1_CH, duty);
}

/**
 * @brief 设置IN2引脚的PWM占空比
 * @param duty 占空比值
 */
static inline void __SetIn2PWM(uint16_t duty)
{
    __HAL_TIM_SET_COMPARE(IN2_TIM, IN2_CH, duty);
}

// static inline void __SetIn3PWM(uint16_t duty) {
//     __HAL_TIM_SET_COMPARE(IN3_TIM, IN3_CH, duty);
// }
//
// static inline void __SetIn4PWM(uint16_t duty) {
//     __HAL_TIM_SET_COMPARE(IN4_TIM, IN4_CH, duty);
// }
/**
 * @brief 初始化DRV8833
 */
void DRV8833_Init(void)
{
    HAL_TIM_PWM_Start(IN1_TIM, IN1_CH);
    HAL_TIM_PWM_Start(IN2_TIM, IN2_CH);
    // HAL_TIM_PWM_Start(IN3_TIM, IN3_CH);
    // HAL_TIM_PWM_Start(IN4_TIM, IN4_CH);
}

/**
 * @brief 设置衰减模式
 * @param mode 衰减模式
 */
void DRV8833_SetDecayMode(DecayMode mode)
{
    currentDecayMode = mode;
}

/**
 * @brief 控制电机1前进
 * @param speed 速度值（0-100）
 */
void motor1_Forward(uint16_t speed)
{
    if (speed > MAX_SPEED)
        speed = MAX_SPEED;
    
    if (currentDecayMode == FAST_DECAY) {
      __SetIn1PWM(speed);
        __SetIn2PWM(0);
    } else {
        __SetIn1PWM(MAX_SPEED);
        __SetIn2PWM(MAX_SPEED - speed);
    }
}

// void motor2_Forward(uint16_t speed) {
//     if (speed > MAX_SPEED)
//         speed = MAX_SPEED;
//
//     if (currentDecayMode == FAST_DECAY) {
//         __SetIn3PWM(speed);
//         __SetIn4PWM(0);
//     } else {
//         __SetIn3PWM(MAX_SPEED);
//         __SetIn4PWM(MAX_SPEED - speed);
//     }
// }

/**
 * @brief 控制电机后退
 * @param speed 速度值（0-100）
 */
void motor1_Backward(uint16_t speed)
{
    if (speed > MAX_SPEED)
        speed = MAX_SPEED;

    if (currentDecayMode == FAST_DECAY) {
        __SetIn1PWM(0);
        __SetIn2PWM(speed);
    } else {
        __SetIn1PWM(MAX_SPEED - speed);
        __SetIn2PWM(MAX_SPEED);
    }
}

// void motor2_Backward(uint16_t speed) {
//     if (speed > MAX_SPEED)
//         speed = MAX_SPEED;
//
//     if (currentDecayMode == FAST_DECAY) {
//         __SetIn3PWM(0);
//         __SetIn4PWM(speed);
//     } else {
//         __SetIn3PWM(MAX_SPEED - speed);
//         __SetIn4PWM(MAX_SPEED);
//     }
// }
/**
 * @brief 电机刹车
 */
void motor1_Brake(void)
{
    __SetIn1PWM(MAX_SPEED);
    __SetIn2PWM(MAX_SPEED);
}

// void motor2_Brake(void) {
//     __SetIn3PWM(MAX_SPEED);
//     __SetIn4PWM(MAX_SPEED);
// }

/**
 * @brief 电机滑行
 */
void motor1__Coast(void)
{
    __SetIn1PWM(0);
    __SetIn2PWM(0);
}

// void motor2__Coast(void) {
//     __SetIn3PWM(0);
//     __SetIn4PWM(0);
// }

//定义正负
void DRV8833_Control(float param) {
uint16_t uint_param=(uint16_t) fabs(param)*Ratio;
    if (param>=0) {
        motor1_Forward(uint_param);//改单电机双通道
        // motor2_Forward(uint_param);
        /*如果是负数，应当换算滑行和刹车占比*/
     }else {
         motor1_Backward(uint_param * BackwardRatio*Ratio);//改单电机双通道
         // motor2_Backward(uint_param * BackwardRatio);

     }

}