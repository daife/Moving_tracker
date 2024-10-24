//
// Created by w2016 on 2024/9/4.
//

#ifndef MSBMPU6500_IMU_TYPES_H
#define MSBMPU6500_IMU_TYPES_H

#include "stdint.h"


typedef struct {
    uint64_t timestamp;
    uint64_t timestamp_sample;

    uint32_t device_id;

    float dt;
    float scale;

    uint8_t samples;
    int16_t  x[32];
    int16_t  y[32];
    int16_t  z[32];

} sensor_accel_fifo_s;

typedef struct {
    uint64_t timestamp;
    uint64_t timestamp_sample;

    uint32_t device_id;

    float dt;
    float scale;

    uint8_t samples;
    int16_t  x[32];
    int16_t  y[32];
    int16_t  z[32];

    uint8_t ORB_QUEUE_LENGTH;


} sensor_gyro_fifo_s;

#endif //MSBMPU6500_IMU_TYPES_H
