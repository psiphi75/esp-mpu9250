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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "i2c-easy.h"

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */
#define LAST_NACK_VAL 0x2          /*!< I2C last_nack value */

#define I2C_FREQ_HZ 200000 /* I2C master clock frequency */
#define I2C_TX_BUF_DISABLE 0 /* I2C master do not need buffer */
#define I2C_RX_BUF_DISABLE 0

/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(uint8_t i2c_num, uint8_t gpio_sda, uint8_t gpio_scl)
{
  int i2c_master_port = i2c_num;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = gpio_sda;
  conf.sda_pullup_en = 0;
  conf.scl_io_num = gpio_scl;
  conf.scl_pullup_en = 0;
  conf.master.clk_speed = I2C_FREQ_HZ;
  ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode,I2C_RX_BUF_DISABLE, I2C_TX_BUF_DISABLE,0));
  return ESP_OK;
}

esp_err_t i2c_write_bytes(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t *data, size_t data_len)
{
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, periph_address << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
  i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

esp_err_t i2c_write_byte(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t data)
{
  return i2c_write_bytes(i2c_num, periph_address, reg_address, &data, 1);
}

esp_err_t i2c_read_bytes(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t *data, size_t data_len)
{
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, periph_address << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK)
  {
    return ret;
  }

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, periph_address << 1 | READ_BIT, ACK_CHECK_EN);
  i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

esp_err_t i2c_read_byte(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t *data)
{
  return i2c_read_bytes(i2c_num, periph_address, reg_address, data, 1);
}

uint8_t get_bit_mask(uint8_t bit, uint8_t length)
{
  return ((1 << length) - 1) << bit;
};

esp_err_t i2c_write_bits(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t bit, uint8_t length, uint8_t value)
{
  uint8_t data[1];

  int ret = i2c_read_bytes(i2c_num, periph_address, reg_address, data, 1);
  if (ret != ESP_OK)
  {
    return ret;
  }

  uint8_t mask = get_bit_mask(bit, length);
  data[0] = data[0] ^ ((data[0] ^ (value << bit)) & mask);
  return i2c_write_bytes(i2c_num, periph_address, reg_address, data, 1);
}

esp_err_t i2c_write_bit(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t bit, uint8_t value)
{
  return i2c_write_bits(i2c_num, periph_address, reg_address, bit, 1, value);
}

esp_err_t i2c_read_bit(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t bit, uint8_t *result)
{
  uint8_t data[1];

  int ret = i2c_read_byte(i2c_num, periph_address, reg_address, data);
  if (ret != ESP_OK)
  {
    return ret;
  }
  *result = (data[0] >> bit) & 1;
  return ret;
};
