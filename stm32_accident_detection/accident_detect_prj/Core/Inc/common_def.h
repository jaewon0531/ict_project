/*
 * common_def.h
 *
 *  Created on: Mar 22, 2023
 *      Author: IoT02
 */

#ifndef INC_COMMON_DEF_H_
#define INC_COMMON_DEF_H_

typedef struct{
  uint32_t time;
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t mag_x;
  int16_t mag_y;
  int16_t mag_z;
  int16_t accel200g_x;
  int16_t accel200g_y;
  int16_t accel200g_z;
  uint32_t longitude;
  uint32_t latitude;
  int16_t speed;
  int16_t engine_load;
  int16_t car_break;
  int16_t car_info;
}IMUDATA;


#endif /* INC_COMMON_DEF_H_ */
