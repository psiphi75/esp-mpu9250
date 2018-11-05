/*****************************************************************************
 *                                                                           *
 *  Copyright 2018 Simon M. Werner                                           *
 *                                                                           *
 *  Licensed under the Apache License, Version 2.0 (the "License");          *
 *  you may not use this file except in compliance with the License.         *
 *  You may obtain a copy of the License at                                  *
 *                                                                           *
 *      http://www.apache.org/licenses/LICENSE-2.0                           *
 *                                                                           *
 *  Unless required by applicable law or agreed to in writing, software      *
 *  distributed under the License is distributed on an "AS IS" BASIS,        *
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
 *  See the License for the specific language governing permissions and      *
 *  limitations under the License.                                           *
 *                                                                           *
 *****************************************************************************/

#ifndef __MPU9250_H
#define __MPU9250_H

#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"

/*****************/
/** MPU9250 MAP **/
/*****************/
// documentation:
//   https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/
//   https://www.invensense.com/wp-content/uploads/2015/02/MPU-9250-Datasheet.pdf
//   https://www.invensense.com/wp-content/uploads/2015/02/MPU-9250-Register-Map.pdf

#define MPU9250_I2C_ADDRESS_AD0_LOW (0x68)
#define MPU9250_I2C_ADDR MPU9250_I2C_ADDRESS_AD0_LOW
#define MPU9250_I2C_ADDRESS_AD0_HIGH (0x69)
#define MPU9250_WHO_AM_I (0x75)

#define MPU9250_RA_CONFIG (0x1A)
#define MPU9250_RA_GYRO_CONFIG (0x1B)
#define MPU9250_RA_ACCEL_CONFIG_1 (0x1C)
#define MPU9250_RA_ACCEL_CONFIG_2 (0x1D)

#define MPU9250_RA_INT_PIN_CFG (0x37)

#define MPU9250_INTCFG_ACTL_BIT (7)
#define MPU9250_INTCFG_OPEN_BIT (6)
#define MPU9250_INTCFG_LATCH_INT_EN_BIT (5)
#define MPU9250_INTCFG_INT_ANYRD_2CLEAR_BIT (4)
#define MPU9250_INTCFG_ACTL_FSYNC_BIT (3)
#define MPU9250_INTCFG_FSYNC_INT_MODE_EN_BIT (2)
#define MPU9250_INTCFG_BYPASS_EN_BIT (1)
#define MPU9250_INTCFG_NONE_BIT (0)

#define MPU9250_ACCEL_XOUT_H (0x3B)
#define MPU9250_ACCEL_XOUT_L (0x3C)
#define MPU9250_ACCEL_YOUT_H (0x3D)
#define MPU9250_ACCEL_YOUT_L (0x3E)
#define MPU9250_ACCEL_ZOUT_H (0x3F)
#define MPU9250_ACCEL_ZOUT_L (0x40)
#define MPU9250_TEMP_OUT_H (0x41)
#define MPU9250_TEMP_OUT_L (0x42)
#define MPU9250_GYRO_XOUT_H (0x43)
#define MPU9250_GYRO_XOUT_L (0x44)
#define MPU9250_GYRO_YOUT_H (0x45)
#define MPU9250_GYRO_YOUT_L (0x46)
#define MPU9250_GYRO_ZOUT_H (0x47)
#define MPU9250_GYRO_ZOUT_L (0x48)

#define MPU9250_RA_USER_CTRL (0x6A)
#define MPU9250_RA_PWR_MGMT_1 (0x6B)
#define MPU9250_RA_PWR_MGMT_2 (0x6C)
#define MPU9250_PWR1_DEVICE_RESET_BIT (7)
#define MPU9250_PWR1_SLEEP_BIT (6)
#define MPU9250_PWR1_CYCLE_BIT (5)
#define MPU9250_PWR1_TEMP_DIS_BIT (3)
#define MPU9250_PWR1_CLKSEL_BIT (0)
#define MPU9250_PWR1_CLKSEL_LENGTH (3)

#define MPU9250_GCONFIG_FS_SEL_BIT (3)
#define MPU9250_GCONFIG_FS_SEL_LENGTH (2)
#define MPU9250_GYRO_FS_250 (0x00)
#define MPU9250_GYRO_FS_500 (0x01)
#define MPU9250_GYRO_FS_1000 (0x02)
#define MPU9250_GYRO_FS_2000 (0x03)
#define MPU9250_GYRO_SCALE_FACTOR_0 (131)
#define MPU9250_GYRO_SCALE_FACTOR_1 (65.5)
#define MPU9250_GYRO_SCALE_FACTOR_2 (32.8)
#define MPU9250_GYRO_SCALE_FACTOR_3 (16.4)

#define MPU9250_ACONFIG_FS_SEL_BIT (3)
#define MPU9250_ACONFIG_FS_SEL_LENGTH (2)
#define MPU9250_ACCEL_FS_2 (0x00)
#define MPU9250_ACCEL_FS_4 (0x01)
#define MPU9250_ACCEL_FS_8 (0x02)
#define MPU9250_ACCEL_FS_16 (0x03)
#define MPU9250_ACCEL_SCALE_FACTOR_0 (16384)
#define MPU9250_ACCEL_SCALE_FACTOR_1 (8192)
#define MPU9250_ACCEL_SCALE_FACTOR_2 (4096)
#define MPU9250_ACCEL_SCALE_FACTOR_3 (2048)

#define MPU9250_CLOCK_INTERNAL (0x00)
#define MPU9250_CLOCK_PLL_XGYRO (0x01)
#define MPU9250_CLOCK_PLL_YGYRO (0x02)
#define MPU9250_CLOCK_PLL_ZGYRO (0x03)
#define MPU9250_CLOCK_KEEP_RESET (0x07)
#define MPU9250_CLOCK_PLL_EXT32K (0x04)
#define MPU9250_CLOCK_PLL_EXT19M (0x05)

#define MPU9250_I2C_SLV0_DO (0x63)
#define MPU9250_I2C_SLV1_DO (0x64)
#define MPU9250_I2C_SLV2_DO (0x65)

#define MPU9250_USERCTRL_DMP_EN_BIT (7)
#define MPU9250_USERCTRL_FIFO_EN_BIT (6)
#define MPU9250_USERCTRL_I2C_MST_EN_BIT (5)
#define MPU9250_USERCTRL_I2C_IF_DIS_BIT (4)
#define MPU9250_USERCTRL_DMP_RESET_BIT (3)
#define MPU9250_USERCTRL_FIFO_RESET_BIT (2)
#define MPU9250_USERCTRL_I2C_MST_RESET_BIT (1)
#define MPU9250_USERCTRL_SIG_COND_RESET_BIT (0)

#define BYTE_2_INT_BE(byte, i) ((int16_t)((byte[i] << 8) + (byte[i + 1])))
#define BYTE_2_INT_LE(byte, i) ((int16_t)((byte[i + 1] << 8) + (byte[i])))

typedef struct
{
  float x, y, z;
} vector_t;

typedef struct
{
  // Magnetometer
  vector_t mag_offset;
  vector_t mag_scale;

  // Gryoscope
  vector_t gyro_bias_offset;

  // Accelerometer
  vector_t accel_offset;
  vector_t accel_scale_lo;
  vector_t accel_scale_hi;

} calibration_t;

esp_err_t i2c_mpu9250_init(calibration_t *cal);
esp_err_t set_clock_source(uint8_t adrs);
esp_err_t set_full_scale_gyro_range(uint8_t adrs);
esp_err_t set_full_scale_accel_range(uint8_t adrs);
esp_err_t set_sleep_enabled(bool state);
esp_err_t get_device_id(uint8_t *val);
esp_err_t get_temperature_raw(uint16_t *val);
esp_err_t get_temperature_celsius(float *val);

esp_err_t get_bypass_enabled(bool *state);
esp_err_t set_bypass_enabled(bool state);
esp_err_t get_i2c_master_mode(bool *state);
esp_err_t set_i2c_master_mode(bool state);

esp_err_t get_accel(vector_t *v);
esp_err_t get_gyro(vector_t *v);
esp_err_t get_mag(vector_t *v);
esp_err_t get_accel_gyro(vector_t *va, vector_t *vg);
esp_err_t get_accel_gyro_mag(vector_t *va, vector_t *vg, vector_t *vm);
esp_err_t get_mag_raw(uint8_t bytes[6]);

void print_settings(void);

#endif // __MPU9250_H
