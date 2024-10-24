
#ifndef MSBMPU6500_MOTION_H
#define MSBMPU6500_MOTION_H

#include "mpu6500/imu_types.h"
#include "main.h"

extern void mpu6500_init(I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *tim, sensor_accel_fifo_s *accel, sensor_gyro_fifo_s *gyro);
extern void mpu6500_run();
extern void mpu6500_exit_and_cleanup();

#endif //MSBMPU6500_MOTION_H
