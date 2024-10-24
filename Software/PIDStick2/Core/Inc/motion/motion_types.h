
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MOTION_FX_H_
#define _MOTION_FX_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stddef.h"

#include "mpu6500/imu_types.h"
#include "motion/EKF.h"

/** @addtogroup MIDDLEWARES
 * @{
 */

void fusion_init(void);
void fusion_update(sensor_accel_fifo_s *accel_data, sensor_gyro_fifo_s *gyro_data, float *rpy);
void fusion_cali(sensor_gyro_fifo_s *gyro_data);

#ifdef __cplusplus
}
#endif

#endif /* _MOTION_FX_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
