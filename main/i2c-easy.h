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

#ifndef __I2C_EASY_H
#define __I2C_EASY_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"

esp_err_t i2c_master_init(uint8_t i2c_num, uint8_t gpio_sda, uint8_t gpio_scl);

/**
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to send
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
esp_err_t i2c_write_bytes(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t *data, size_t data_len);
esp_err_t i2c_write_byte(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t data);

/**
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to read
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
esp_err_t i2c_read_bytes(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t *data, size_t data_len);
esp_err_t i2c_read_byte(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t *data);

/**
 * Write one bit.  Note, this will do a read to get the existing value, then a write.
 * @param  i2c_num  The i2c number
 * @param  reg_address The address of the byte to write.
 * @param  bit      The nth bit.
 * @param  value    The new value, 1 or 0.
 */
esp_err_t i2c_write_bits(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t bit, uint8_t length, uint8_t value);
esp_err_t i2c_write_bit(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t bit, uint8_t value);

/**
 * Return the bit value, 1 or 0.
 * @param  adrs     The address of the byte to read.
 * @param  bit      The nth bit.
 * @param  result   The place where the result is stored.
 */
esp_err_t i2c_read_bit(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t bit, uint8_t *result);

#endif // __I2C_EASY_H