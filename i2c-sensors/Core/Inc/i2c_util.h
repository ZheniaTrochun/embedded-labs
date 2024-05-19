#ifndef I2C_SENSORS_I2C_UTIL_H
#define I2C_SENSORS_I2C_UTIL_H

#include "main.h"

int16_t read2ByteValueI2c(I2C_HandleTypeDef hi2c1, uint8_t address, uint8_t registerAddress);

#endif //I2C_SENSORS_I2C_UTIL_H
