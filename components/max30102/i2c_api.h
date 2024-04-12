#ifndef I2C_API_H
#define I2C_API_H

#include "esp_err.h"
#include <driver/i2c.h>

#define MAX30102_ADDR 0x57

#define ACK_CHECK_EN 0x1
#define ACK_VAL 0x0
#define NACK_VAL 0x1

esp_err_t i2c_sensor_read();
esp_err_t i2c_sensor_write(uint8_t *data_wr, size_t size);

#endif
