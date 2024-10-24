#include "motion/motion_types.h"

int cali_count = 0;
long long int cali_x_sum = 0;
long long int cali_y_sum = 0;
long long int cali_z_sum = 0;
long long int cali_x = 0;
long long int cali_y = 0;
long long int cali_z = 0;

ekf_t ekf;
float euler_ekf[3];

volatile uint8_t fusion_flag;

void fusion_init(void) {
    EKF_init(&ekf, 1.0f, 1.0f, 1.0f, 0.1, 0.1, 1000);
}


void fusion_cali(sensor_gyro_fifo_s *gyro_data) {

    for (int i = 0; i < gyro_data->samples; i++) {
        cali_x_sum += gyro_data->x[i];
        cali_y_sum += gyro_data->y[i];
        cali_z_sum += gyro_data->z[i];
    }
    cali_count += gyro_data->samples;
    cali_x = cali_x_sum / cali_count;
    cali_y = cali_y_sum / cali_count;
    cali_z = cali_z_sum / cali_count;

}

void fusion_update(sensor_accel_fifo_s *accel_data, sensor_gyro_fifo_s *gyro_data, float *rpy) {

    /* Convert angular velocity from [mdps] to [dps] */

    float delta_gyro[3] = {0, 0, 0};
    for (int i = 0; i < gyro_data->samples; i++) {
        delta_gyro[0] += (float) (gyro_data->x[i] - cali_x)* gyro_data->scale;
        delta_gyro[1] += (float) (gyro_data->y[i]- cali_y)* gyro_data->scale;
        delta_gyro[2] += (float) (gyro_data->z[i] -cali_z)* gyro_data->scale;

    }
    delta_gyro[0] /= (float) gyro_data->samples;
    delta_gyro[1] /= (float) gyro_data->samples;
    delta_gyro[2] /= (float) gyro_data->samples;
    float gyro_delta_time = gyro_data->dt * gyro_data->samples / 1e6f;


    float accel[3] = {0, 0, 0};
    for (int i = 0; i < accel_data->samples; i++) {
        accel[0] += ((float) accel_data->x[i]) * accel_data->scale;
        accel[1] += ((float) accel_data->y[i]) * accel_data->scale;
        accel[2] += ((float) accel_data->z[i]) * accel_data->scale;
    }
    accel[0] /= (float) accel_data->samples;
    accel[1] /= (float) accel_data->samples;
    accel[2] /= (float) accel_data->samples;

//    accel[0] = accel[0] * accel_data->scale;
//    accel[1] = accel[1] * accel_data->scale;
//    accel[2] = accel[2] * accel_data->scale;

    EKF_update(&ekf, euler_ekf, accel[0], accel[1] , accel[2], delta_gyro[0],delta_gyro[1],delta_gyro[2],1.0f,1.0f,1.0f, gyro_delta_time);
//
//        Serial.print("Yaw: ");
//      Serial.print(data_out.rotation[0]);
//      Serial.print("  Pitch: ");
//      Serial.print(data_out.rotation[1]);
//      Serial.print("  Roll: ");
//      Serial.print(data_out.rotation[2]);
//      Serial.println("");

    rpy[0] = euler_ekf[0] * 180.0f / 3.14159265f;
    rpy[1] = euler_ekf[1]* 180.0f / 3.14159265f;
    rpy[2] = euler_ekf[2]* 180.0f / 3.14159265f;
}