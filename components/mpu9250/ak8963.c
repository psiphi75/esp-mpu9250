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

static const char *TAG = "ak8963";

static bool initialised = false;
static uint8_t i2c_num;
calibration_t *cal;
vector_t asa;

esp_err_t ak8963_init(i2c_port_t i2c_number, calibration_t *c)
{

  if (initialised)
  {
    ESP_LOGE(TAG, "ak8963_init has already been called");
    return ESP_ERR_INVALID_STATE;
  }
  i2c_num = i2c_number;
  cal = c;

  // connection with magnetometer
  uint8_t id;
  ak8963_get_device_id(&id);

  if (id & AK8963_WHO_AM_I_RESPONSE)
  {
    ak8963_get_sensitivity_adjustment_values();
    vTaskDelay(10 / portTICK_RATE_MS);
    ak8963_set_cntl(AK8963_CNTL_MODE_CONTINUE_MEASURE_2);
    initialised = true;
    return ESP_OK;
  }
  else
  {
    ESP_LOGE(TAG, "AK8963: Device ID is not equal to 0x%02x, device value is 0x%02x", AK8963_WHO_AM_I_RESPONSE, id);
    return ESP_ERR_INVALID_STATE;
  }
}

esp_err_t ak8963_get_data_ready(bool *val)
{
  uint8_t bit;
  esp_err_t ret = i2c_read_bit(i2c_num, AK8963_ADDRESS, AK8963_ST1, AK8963_ST1_DRDY_BIT, &bit);
  if (ret != ESP_OK)
  {
    return ret;
  }
  *val = (bit == 0x01);

  return ESP_OK;
}

esp_err_t ak8963_get_device_id(uint8_t *val)
{
  return i2c_read_byte(i2c_num, AK8963_ADDRESS, AK8963_WHO_AM_I, val);
}

esp_err_t ak8963_get_sensitivity_adjustment_values()
{

  esp_err_t ret;

  // Need to set to Fuse mode to get valid values from this.
  uint8_t current_mode;
  ret = ak8963_get_cntl(&current_mode);
  if (ret != ESP_OK)
    return ret;

  ret = ak8963_set_cntl(AK8963_CNTL_MODE_FUSE_ROM_ACCESS);
  if (ret != ESP_OK)
    return ret;

  vTaskDelay(20 / portTICK_RATE_MS);

  uint8_t xi, yi, zi;
  ret = i2c_read_byte(i2c_num, AK8963_ADDRESS, AK8963_ASAX, &xi);
  if (ret != ESP_OK)
    return ret;

  ret = i2c_read_byte(i2c_num, AK8963_ADDRESS, AK8963_ASAY, &yi);
  if (ret != ESP_OK)
    return ret;

  ret = i2c_read_byte(i2c_num, AK8963_ADDRESS, AK8963_ASAZ, &zi);
  if (ret != ESP_OK)
    return ret;

  // Get the ASA* values
  asa.x = (((float)xi - 128.0) * 0.5) / 128.0 + 1.0;
  asa.y = (((float)yi - 128.0) * 0.5) / 128.0 + 1.0;
  asa.z = (((float)zi - 128.0) * 0.5) / 128.0 + 1.0;

  return ak8963_set_cntl(current_mode);
}

esp_err_t ak8963_get_mag(vector_t *v)
{

  esp_err_t ret;
  uint8_t bytes[6];

  ret = ak8963_get_mag_raw(bytes);
  if (ret != ESP_OK)
  {
    return ret;
  }

  float xi = (float)BYTE_2_INT_LE(bytes, 0);
  float yi = (float)BYTE_2_INT_LE(bytes, 2);
  float zi = (float)BYTE_2_INT_LE(bytes, 4);

  v->x = (xi * asa.x - cal->mag_offset.x) * cal->mag_scale.x;
  v->y = (yi * asa.y - cal->mag_offset.y) * cal->mag_scale.y;
  v->z = (zi * asa.z - cal->mag_offset.z) * cal->mag_scale.z;
  // ESP_LOGW(TAG, "mag     -> %0.4f %0.4f %0.4f", v->x, v->y, v->z);

  return ESP_OK;
}

esp_err_t ak8963_get_mag_raw(uint8_t bytes[6])
{
  i2c_read_bytes(i2c_num, AK8963_ADDRESS, AK8963_XOUT_L, bytes, 6);
  // ESP_LOGW(TAG, "mag raw -> %02x %02x %02x %02x %02x %02x", bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5]);

  // For some reason when we read ST2 (Status 2) just after reading byte, this ensures the
  // next reading is fresh.  If we do it before without a pause, only 1 in 15 readings will
  // be fresh.  The setTimeout ensures this read goes to the back of the queue, once all other
  // computation is done.
  vTaskDelay(1 / portTICK_RATE_MS);
  uint8_t b;
  i2c_read_byte(i2c_num, AK8963_ADDRESS, AK8963_ST2, &b);

  return ESP_OK;
}

esp_err_t ak8963_get_cntl(uint8_t *mode)
{
  return i2c_read_byte(i2c_num, AK8963_ADDRESS, AK8963_CNTL, mode);
}

esp_err_t ak8963_set_cntl(uint8_t mode)
{
  return i2c_write_byte(i2c_num, AK8963_ADDRESS, AK8963_CNTL, mode);
}

void ak8963_print_settings(void)
{
  char *cntl_modes[] = {"0x00 (Power-down mode)",
                        "0x01 (Single measurement mode)",
                        "0x02 (Continuous measurement mode 1: 8Hz)",
                        "0x03 Invalid mode",
                        "0x04 (External trigger measurement mode)",
                        "0x05 Invalid mode",
                        "0x06 (Continuous measurement mode 2: 100Hz)",
                        "0x07 Invalid mode",
                        "0x08 (Self-test mode)",
                        "0x09 Invalid mode",
                        "0x0A Invalid mode",
                        "0x0B Invalid mode",
                        "0x0C Invalid mode",
                        "0x0D Invalid mode",
                        "0x0E Invalid mode",
                        "0x0F Invalid mode",
                        "0x0F (Fuse ROM access mode)"};

  uint8_t device_id;
  ESP_ERROR_CHECK(ak8963_get_device_id(&device_id));

  uint8_t cntl;
  ESP_ERROR_CHECK(ak8963_get_cntl(&cntl));

  ESP_LOGI(TAG, "Magnetometer (Compass):");
  ESP_LOGI(TAG, "--> i2c address: 0x%02d", i2c_num);
  ESP_LOGI(TAG, "--> initialised: %s", initialised ? "true" : "false");
  ESP_LOGI(TAG, "--> Device ID: 0x%02x", device_id);
  ESP_LOGI(TAG, "--> Mode: %s", cntl_modes[cntl]);
  ESP_LOGI(TAG, "--> ASA Scalars:");
  ESP_LOGI(TAG, "  --> x: %f", asa.x);
  ESP_LOGI(TAG, "  --> y: %f", asa.y);
  ESP_LOGI(TAG, "  --> z: %f", asa.z);
  ESP_LOGI(TAG, "--> Offset:");
  ESP_LOGI(TAG, "  --> x: %f", cal->mag_offset.x);
  ESP_LOGI(TAG, "  --> y: %f", cal->mag_offset.y);
  ESP_LOGI(TAG, "  --> z: %f", cal->mag_offset.z);
  ESP_LOGI(TAG, "--> Scale:");
  ESP_LOGI(TAG, "  --> x: %f", cal->mag_scale.x);
  ESP_LOGI(TAG, "  --> y: %f", cal->mag_scale.y);
  ESP_LOGI(TAG, "  --> z: %f", cal->mag_scale.z);
}