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

#ifndef __AK8963_H
#define __AK8963_H

#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"

#define AK8963_ADDRESS (0x0c)
#define AK8963_WHO_AM_I (0x00) // should return 0x48
#define AK8963_WHO_AM_I_RESPONSE (0x48)
#define AK8963_INFO (0x01)
#define AK8963_ST1 (0x02)    // data ready status bit 0
#define AK8963_XOUT_L (0x03) // data
#define AK8963_XOUT_H (0x04)
#define AK8963_YOUT_L (0x05)
#define AK8963_YOUT_H (0x06)
#define AK8963_ZOUT_L (0x07)
#define AK8963_ZOUT_H (0x08)
#define AK8963_ST2 (0x09)    // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL (0x0a)   // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC (0x0c)   // Self test control
#define AK8963_I2CDIS (0x0f) // I2C disable
#define AK8963_ASAX (0x10)   // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY (0x11)   // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ (0x12)

#define AK8963_ST1_DRDY_BIT (0)
#define AK8963_ST1_DOR_BIT (1)

#define AK8963_CNTL_MODE_OFF (0x00)                // Power-down mode
#define AK8963_CNTL_MODE_SINGLE_MEASURE (0x01)     // Single measurement mode
#define AK8963_CNTL_MODE_CONTINUE_MEASURE_1 (0x02) // Continuous measurement mode 1 - Sensor is measured periodically at 8Hz
#define AK8963_CNTL_MODE_CONTINUE_MEASURE_2 (0x06) // Continuous measurement mode 2 - Sensor is measured periodically at 100Hz
#define AK8963_CNTL_MODE_EXT_TRIG_MEASURE (0x04)   // External trigger measurement mode
#define AK8963_CNTL_MODE_SELF_TEST_MODE (0x08)     // Self-test mode
#define AK8963_CNTL_MODE_FUSE_ROM_ACCESS (0x0f)    // Fuse ROM access mode

esp_err_t ak8963_init(i2c_port_t i2c_number, calibration_t *c);

/**
 * @name ak8963_get_data_ready
 */
esp_err_t ak8963_get_data_ready(bool *val);

/**
 * @name ak8963_get_device_id
 */
esp_err_t ak8963_get_device_id(uint8_t *val);

/**
 * Get the Sensitivity Adjustment values.  These were set during manufacture and allow us to get the actual H values
 * from the magnetometer.
 * @name ak8963_get_sensitivity_adjustment_values
 */
esp_err_t ak8963_get_sensitivity_adjustment_values();

/**
 * Get the raw magnetometer values
 * @name ak8963_get_mag_raw
 */
esp_err_t ak8963_get_mag(vector_t *v);
esp_err_t ak8963_get_mag_raw(uint8_t bytes[6]);

/**
 * @name getCNTL
 */
esp_err_t ak8963_get_cntl(uint8_t *mode);

/**---------------------|[ SET ]|--------------------**/

/**
 * @name setCNTL
 * CNTL_MODE_OFF: 0x00, // Power-down mode
 * CNTL_MODE_SINGLE_MEASURE: 0x01, // Single measurement mode
 * CNTL_MODE_CONTINUE_MEASURE_1: 0x02, // Continuous measurement mode 1
 * CNTL_MODE_CONTINUE_MEASURE_2: 0x06, // Continuous measurement mode 2
 * CNTL_MODE_EXT_TRIG_MEASURE: 0x04, // External trigger measurement mode
 * CNTL_MODE_SELF_TEST_MODE: 0x08, // Self-test mode
 * CNTL_MODE_FUSE_ROM_ACCESS: 0x0F  // Fuse ROM access mode
 * @return undefined | false
 */
esp_err_t ak8963_set_cntl(uint8_t mode);

void ak8963_print_settings(void);

#endif