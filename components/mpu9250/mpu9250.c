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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c-easy.h"
#include "mpu9250.h"
#include "ak8963.h"

#define I2C_MASTER_SCL_IO 22      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21     /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */

static const char *TAG = "mpu9250";

static bool initialised = false;
calibration_t *cal;

static float gyro_inv_scale = 1.0;
static float accel_inv_scale = 1.0;

static esp_err_t enable_magnetometer(void);

esp_err_t i2c_mpu9250_init(calibration_t *c)
{
  ESP_LOGI(TAG, "Initializating MPU9250");
  vTaskDelay(100 / portTICK_RATE_MS);

  i2c_master_init(I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

  if (initialised)
  {
    ESP_LOGE(TAG, "i2c_mpu9250_init has already been called");
    return ESP_ERR_INVALID_STATE;
  }
  initialised = true;
  cal = c;

  ESP_LOGD(TAG, "i2c_mpu9250_init");

  ESP_ERROR_CHECK(i2c_write_bit(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_DEVICE_RESET_BIT, 1));
  vTaskDelay(10 / portTICK_RATE_MS);

  // define clock source
  ESP_ERROR_CHECK(set_clock_source(MPU9250_CLOCK_PLL_XGYRO));
  vTaskDelay(10 / portTICK_RATE_MS);

  // define gyro range
  ESP_ERROR_CHECK(set_full_scale_gyro_range(MPU9250_GYRO_FS_250));
  vTaskDelay(10 / portTICK_RATE_MS);

  // define accel range
  ESP_ERROR_CHECK(set_full_scale_accel_range(MPU9250_ACCEL_FS_4));
  vTaskDelay(10 / portTICK_RATE_MS);

  // disable sleepEnabled
  ESP_ERROR_CHECK(set_sleep_enabled(false));
  vTaskDelay(10 / portTICK_RATE_MS);

  ESP_LOGD(TAG, "END of MPU9250 initialization");

  ESP_ERROR_CHECK(enable_magnetometer());

  print_settings();

  return ESP_OK;
}

esp_err_t set_clock_source(uint8_t adrs)
{
  return i2c_write_bits(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CLKSEL_BIT, MPU9250_PWR1_CLKSEL_LENGTH, adrs);
}

esp_err_t get_clock_source(uint8_t *clock_source)
{
  uint8_t byte;
  esp_err_t ret = i2c_read_byte(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_RA_PWR_MGMT_1, &byte);
  if (ret != ESP_OK)
  {
    return ret;
  }

  *clock_source = byte & 0x07;
  return ESP_OK;
}

float get_gyro_inv_scale(uint8_t scale_factor)
{
  switch (scale_factor)
  {
  case MPU9250_GYRO_FS_250:
    return 1.0 / MPU9250_GYRO_SCALE_FACTOR_0;
  case MPU9250_GYRO_FS_500:
    return 1.0 / MPU9250_GYRO_SCALE_FACTOR_1;
  case MPU9250_GYRO_FS_1000:
    return 1.0 / MPU9250_GYRO_SCALE_FACTOR_2;
  case MPU9250_GYRO_FS_2000:
    return 1.0 / MPU9250_GYRO_SCALE_FACTOR_3;
  default:
    ESP_LOGE(TAG, "get_gyro_inv_scale(): invalid value (%d)", scale_factor);
    return 1;
  }
}

esp_err_t set_full_scale_gyro_range(uint8_t adrs)
{
  gyro_inv_scale = get_gyro_inv_scale(adrs);
  return i2c_write_bits(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_RA_GYRO_CONFIG, MPU9250_GCONFIG_FS_SEL_BIT, MPU9250_GCONFIG_FS_SEL_LENGTH, adrs);
}

float get_accel_inv_scale(uint8_t scale_factor)
{
  switch (scale_factor)
  {
  case MPU9250_ACCEL_FS_2:
    return 1.0 / MPU9250_ACCEL_SCALE_FACTOR_0;
  case MPU9250_ACCEL_FS_4:
    return 1.0 / MPU9250_ACCEL_SCALE_FACTOR_1;
  case MPU9250_ACCEL_FS_8:
    return 1.0 / MPU9250_ACCEL_SCALE_FACTOR_2;
  case MPU9250_ACCEL_FS_16:
    return 1.0 / MPU9250_ACCEL_SCALE_FACTOR_3;
  default:
    ESP_LOGE(TAG, "get_accel_inv_scale(): invalid value (%d)", scale_factor);
    return 1;
  }
}

esp_err_t set_full_scale_accel_range(uint8_t adrs)
{
  accel_inv_scale = get_accel_inv_scale(adrs);
  return i2c_write_bits(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_RA_ACCEL_CONFIG_1, MPU9250_ACONFIG_FS_SEL_BIT, MPU9250_ACONFIG_FS_SEL_LENGTH, adrs);
}

esp_err_t set_sleep_enabled(bool state)
{
  return i2c_write_bit(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_SLEEP_BIT, state ? 0x01 : 0x00);
}

esp_err_t get_sleep_enabled(bool *state)
{
  uint8_t bit;
  esp_err_t ret = i2c_read_bit(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_SLEEP_BIT, &bit);
  if (ret != ESP_OK)
  {
    return ret;
  }
  *state = (bit == 0x01);

  return ESP_OK;
}

float scale_accel(float value, float offset, float scale_lo, float scale_hi)
{
  if (value < 0)
  {
    return -(value * accel_inv_scale - offset) / (scale_lo - offset);
  }
  else
  {
    return (value * accel_inv_scale - offset) / (scale_hi - offset);
  }
}

void align_accel(uint8_t bytes[6], vector_t *v)
{
  int16_t xi = BYTE_2_INT_BE(bytes, 0);
  int16_t yi = BYTE_2_INT_BE(bytes, 2);
  int16_t zi = BYTE_2_INT_BE(bytes, 4);

  v->x = scale_accel((float)xi, cal->accel_offset.x, cal->accel_scale_lo.x, cal->accel_scale_hi.x);
  v->y = scale_accel((float)yi, cal->accel_offset.y, cal->accel_scale_lo.y, cal->accel_scale_hi.y);
  v->z = scale_accel((float)zi, cal->accel_offset.z, cal->accel_scale_lo.z, cal->accel_scale_hi.z);
}

esp_err_t get_accel(vector_t *v)
{

  esp_err_t ret;
  uint8_t bytes[6];

  ret = i2c_read_bytes(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_ACCEL_XOUT_H, bytes, 6);
  if (ret != ESP_OK)
  {
    return ret;
  }

  align_accel(bytes, v);

  return ESP_OK;
}

void align_gryo(uint8_t bytes[6], vector_t *v)
{
  int16_t xi = BYTE_2_INT_BE(bytes, 0);
  int16_t yi = BYTE_2_INT_BE(bytes, 2);
  int16_t zi = BYTE_2_INT_BE(bytes, 4);

  v->x = (float)xi * gyro_inv_scale + cal->gyro_bias_offset.x;
  v->y = (float)yi * gyro_inv_scale + cal->gyro_bias_offset.y;
  v->z = (float)zi * gyro_inv_scale + cal->gyro_bias_offset.z;
}

esp_err_t get_gyro(vector_t *v)
{
  esp_err_t ret;
  uint8_t bytes[6];
  ret = i2c_read_bytes(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_GYRO_XOUT_H, bytes, 6);
  if (ret != ESP_OK)
  {
    return ret;
  }

  align_gryo(bytes, v);

  return ESP_OK;
}

esp_err_t get_accel_gyro(vector_t *va, vector_t *vg)
{
  esp_err_t ret;
  uint8_t bytes[14];
  ret = i2c_read_bytes(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_ACCEL_XOUT_H, bytes, 14);
  if (ret != ESP_OK)
  {
    return ret;
  }

  // Accelerometer - bytes 0:5
  align_accel(bytes, va);

  // Skip Temperature - bytes 6:7

  // Gyroscope - bytes 9:13
  align_gryo(&bytes[8], vg);

  return ESP_OK;
}

esp_err_t get_accel_gyro_mag(vector_t *va, vector_t *vg, vector_t *vm)
{
  esp_err_t ret;
  ret = get_accel_gyro(va, vg);
  if (ret != ESP_OK)
  {
    return ret;
  }

  return ak8963_get_mag(vm);
}

esp_err_t get_mag(vector_t *v)
{
  return ak8963_get_mag(v);
}

esp_err_t get_mag_raw(uint8_t bytes[6])
{
  return ak8963_get_mag_raw(bytes);
}

esp_err_t get_device_id(uint8_t *val)
{
  return i2c_read_byte(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_WHO_AM_I, val);
}

esp_err_t get_temperature_raw(uint16_t *val)
{
  uint8_t bytes[2];
  esp_err_t ret = i2c_read_bytes(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_TEMP_OUT_H, bytes, 2);
  if (ret != ESP_OK)
  {
    return ret;
  }
  *val = BYTE_2_INT_BE(bytes, 0);
  return ESP_OK;
}

esp_err_t get_temperature_celsius(float *val)
{
  uint16_t raw_temp;
  esp_err_t ret = get_temperature_raw(&raw_temp);
  if (ret != ESP_OK)
  {
    return ret;
  }
  *val = ((float)raw_temp) / 333.87 + 21.0;

  return ESP_OK;
}

static esp_err_t enable_magnetometer(void)
{
  ESP_LOGI(TAG, "Enabling magnetometer");

  ESP_ERROR_CHECK(set_i2c_master_mode(false));
  vTaskDelay(100 / portTICK_RATE_MS);

  ESP_ERROR_CHECK(set_bypass_enabled(true));
  vTaskDelay(100 / portTICK_RATE_MS);

  bool is_enabled;
  ESP_ERROR_CHECK(get_bypass_enabled(&is_enabled));
  if (is_enabled)
  {
    ak8963_init(I2C_MASTER_NUM, cal);
    ESP_LOGI(TAG, "Magnetometer enabled");
    return ESP_OK;
  }
  else
  {
    ESP_LOGE(TAG, "Can't turn on RA_INT_PIN_CFG.");
    return ESP_ERR_INVALID_STATE;
  }
}

esp_err_t get_bypass_enabled(bool *state)
{
  uint8_t bit;
  esp_err_t ret = i2c_read_bit(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_BYPASS_EN_BIT, &bit);
  if (ret != ESP_OK)
  {
    return ret;
  }
  *state = (bit == 0x01);

  return ESP_OK;
}

esp_err_t set_bypass_enabled(bool state)
{
  return i2c_write_bit(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_BYPASS_EN_BIT, state ? 1 : 0);
}

esp_err_t get_i2c_master_mode(bool *state)
{
  uint8_t bit;
  esp_err_t ret = i2c_read_bit(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_I2C_MST_EN_BIT, &bit);
  if (ret != ESP_OK)
  {
    return ret;
  }
  *state = (bit == 0x01);

  return ESP_OK;
}

esp_err_t set_i2c_master_mode(bool state)
{
  return i2c_write_bit(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_I2C_MST_EN_BIT, state ? 1 : 0);
}

/**
 * @name get_gyro_power_settings
 */
esp_err_t get_gyro_power_settings(uint8_t bytes[3])
{
  uint8_t byte;
  esp_err_t ret = i2c_read_byte(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_RA_PWR_MGMT_2, &byte);
  if (ret != ESP_OK)
  {
    return ret;
  }

  byte = byte & 0x07;

  bytes[0] = (byte >> 2) & 1; // X
  bytes[1] = (byte >> 1) & 1; // Y
  bytes[2] = (byte >> 0) & 1; // Z

  return ESP_OK;
}

/**
 * @name get_accel_power_settings
 */
esp_err_t get_accel_power_settings(uint8_t bytes[3])
{
  uint8_t byte;
  esp_err_t ret = i2c_read_byte(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_RA_PWR_MGMT_2, &byte);
  if (ret != ESP_OK)
  {
    return ret;
  }

  byte = byte & 0x38;

  bytes[0] = (byte >> 5) & 1; // X
  bytes[1] = (byte >> 4) & 1; // Y
  bytes[2] = (byte >> 3) & 1; // Z

  return ESP_OK;
}

/**
 * @name get_full_scale_accel_range
 */
esp_err_t get_full_scale_accel_range(uint8_t *full_scale_accel_range)
{
  uint8_t byte;
  esp_err_t ret = i2c_read_byte(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_RA_ACCEL_CONFIG_1, &byte);
  if (ret != ESP_OK)
  {
    return ret;
  }

  byte = byte & 0x18;
  byte = byte >> 3;

  *full_scale_accel_range = byte;

  return ESP_OK;
}

/**
 * @name get_full_scale_gyro_range
 */
esp_err_t get_full_scale_gyro_range(uint8_t *full_scale_gyro_range)
{
  uint8_t byte;
  esp_err_t ret = i2c_read_byte(I2C_MASTER_NUM, MPU9250_I2C_ADDR, MPU9250_RA_GYRO_CONFIG, &byte);
  if (ret != ESP_OK)
  {
    return ret;
  }

  byte = byte & 0x18;
  byte = byte >> 3;

  *full_scale_gyro_range = byte;

  return ESP_OK;
}

#define YN(yn) (yn == 0 ? "Yes" : "No")

void mpu9250_print_settings(void)
{
  const char *CLK_RNG[] = {
      "0 (Internal 20MHz oscillator)",
      "1 (Auto selects the best available clock source)",
      "2 (Auto selects the best available clock source)",
      "3 (Auto selects the best available clock source)",
      "4 (Auto selects the best available clock source)",
      "5 (Auto selects the best available clock source)",
      "6 (Internal 20MHz oscillator)",
      "7 (Stops the clock and keeps timing generator in reset)"};

  uint8_t device_id;
  ESP_ERROR_CHECK(get_device_id(&device_id));

  bool bypass_enabled;
  ESP_ERROR_CHECK(get_bypass_enabled(&bypass_enabled));

  bool sleep_enabled;
  ESP_ERROR_CHECK(get_sleep_enabled(&sleep_enabled));

  bool i2c_master_mode;
  ESP_ERROR_CHECK(get_i2c_master_mode(&i2c_master_mode));

  uint8_t clock_source;
  ESP_ERROR_CHECK(get_clock_source(&clock_source));

  uint8_t accel_power_settings[3];
  ESP_ERROR_CHECK(get_accel_power_settings(accel_power_settings));

  uint8_t gyro_power_settings[3];
  ESP_ERROR_CHECK(get_gyro_power_settings(gyro_power_settings));

  ESP_LOGI(TAG, "MPU9250:");
  ESP_LOGI(TAG, "--> i2c bus: 0x%02x", I2C_MASTER_NUM);
  ESP_LOGI(TAG, "--> Device address: 0x%02x", MPU9250_I2C_ADDR);
  ESP_LOGI(TAG, "--> Device ID: 0x%02x", device_id);
  ESP_LOGI(TAG, "--> initialised: %s", initialised ? "Yes" : "No");
  ESP_LOGI(TAG, "--> BYPASS enabled: %s", bypass_enabled ? "Yes" : "No");
  ESP_LOGI(TAG, "--> SleepEnabled Mode: %s", sleep_enabled ? "On" : "Off");
  ESP_LOGI(TAG, "--> i2c Master Mode: %s", i2c_master_mode ? "Enabled" : "Disabled");
  ESP_LOGI(TAG, "--> Power Management (0x6B, 0x6C):");
  ESP_LOGI(TAG, "  --> Clock Source: %s", CLK_RNG[clock_source]);
  ESP_LOGI(TAG, "  --> Accel enabled (x, y, z): (%s, %s, %s)",
           YN(accel_power_settings[0]),
           YN(accel_power_settings[1]),
           YN(accel_power_settings[2]));
  ESP_LOGI(TAG, "  --> Gyro enabled (x, y, z): (%s, %s, %s)",
           YN(gyro_power_settings[0]),
           YN(gyro_power_settings[1]),
           YN(gyro_power_settings[2]));
}

void print_accel_settings(void)
{
  const char *FS_RANGE[] = {"±2g (0)", "±4g (1)", "±8g (2)", "±16g (3)"};

  uint8_t full_scale_accel_range;
  ESP_ERROR_CHECK(get_full_scale_accel_range(&full_scale_accel_range));

  ESP_LOGI(TAG, "Accelerometer:");
  ESP_LOGI(TAG, "--> Full Scale Range (0x1C): %s", FS_RANGE[full_scale_accel_range]);
  ESP_LOGI(TAG, "--> Scalar: 1/%f", 1.0 / accel_inv_scale);
  ESP_LOGI(TAG, "--> Calibration:");
  ESP_LOGI(TAG, "  --> Offset: ");
  ESP_LOGI(TAG, "    --> x: %f", cal->accel_offset.x);
  ESP_LOGI(TAG, "    --> y: %f", cal->accel_offset.y);
  ESP_LOGI(TAG, "    --> z: %f", cal->accel_offset.z);
  ESP_LOGI(TAG, "  --> Scale: ");
  ESP_LOGI(TAG, "    --> x: (%f, %f)", cal->accel_scale_lo.x, cal->accel_scale_hi.x);
  ESP_LOGI(TAG, "    --> y: (%f, %f)", cal->accel_scale_lo.y, cal->accel_scale_hi.y);
  ESP_LOGI(TAG, "    --> z: (%f, %f)", cal->accel_scale_lo.z, cal->accel_scale_hi.z);
};

void print_gyro_settings(void)
{
  const char *FS_RANGE[] = {
      "+250 dps (0)",
      "+500 dps (1)",
      "+1000 dps (2)",
      "+2000 dps (3)"};

  uint8_t full_scale_gyro_range;
  ESP_ERROR_CHECK(get_full_scale_gyro_range(&full_scale_gyro_range));

  ESP_LOGI(TAG, "Gyroscope:");
  ESP_LOGI(TAG, "--> Full Scale Range (0x1B): %s", FS_RANGE[full_scale_gyro_range]);
  ESP_LOGI(TAG, "--> Scalar: 1/%f", 1.0 / gyro_inv_scale);
  ESP_LOGI(TAG, "--> Bias Offset:");
  ESP_LOGI(TAG, "  --> x: %f", cal->gyro_bias_offset.x);
  ESP_LOGI(TAG, "  --> y: %f", cal->gyro_bias_offset.y);
  ESP_LOGI(TAG, "  --> z: %f", cal->gyro_bias_offset.z);
};

void print_settings(void)
{
  mpu9250_print_settings();
  print_accel_settings();
  print_gyro_settings();
  ak8963_print_settings();
}
