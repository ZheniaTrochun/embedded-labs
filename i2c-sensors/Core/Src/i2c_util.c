#include "i2c_util.h"

int16_t read2ByteValueI2c(I2C_HandleTypeDef hi2c1, uint8_t address, uint8_t registerAddress) {
    uint8_t data[2];

    HAL_StatusTypeDef res = HAL_I2C_Mem_Read(&hi2c1, address, registerAddress, 1, data, 2, 100);

    if (res == HAL_OK) {
        return ((int16_t) data[0] << 8) + data[1];
    } else {
        return 0;
    }
}
