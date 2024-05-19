#include "accelerometer.h"
#include "i2c_util.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

int configureMPU(I2C_HandleTypeDef hi2c1) {
    uint8_t exitSleepModeCommand = 0;
    HAL_StatusTypeDef res1 = HAL_I2C_Mem_Write(&hi2c1,
                                               MPU_GYRO_ACCELEROMETER_ADDRESS + 0,
                                               MPU_PWR_MANAGEMENT_REGISTER,
                                               1,
                                               &exitSleepModeCommand,
                                               1,
                                               100);

    uint8_t masterModeOffCommand = 0;
    HAL_StatusTypeDef res2 = HAL_I2C_Mem_Write(&hi2c1,
                                               MPU_GYRO_ACCELEROMETER_ADDRESS + 0,
                                               MPU_USER_CTRL_REGISTER,
                                               1,
                                               &masterModeOffCommand,
                                               1,
                                               100);

    uint8_t i2cBypassModeCommand = 2;
    HAL_StatusTypeDef res3 = HAL_I2C_Mem_Write(&hi2c1,
                                               MPU_GYRO_ACCELEROMETER_ADDRESS + 0,
                                               MPU_INT_PIN_CFG_REGISTER,
                                               1,
                                               &i2cBypassModeCommand,
                                               1,
                                               100);


    if ((res1 == HAL_OK) && (res2 == HAL_OK) && (res3 == HAL_OK)) {
        return 1;
    } else {
        return 0;
    }
}

int isAccelerometerReady(I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart1) {
    HAL_StatusTypeDef res = HAL_I2C_IsDeviceReady(&hi2c1, MPU_GYRO_ACCELEROMETER_ADDRESS, 1, 100);
    char msg[50] = {'\0'};

    if (res == HAL_OK) {
        sprintf(msg, "MPU device is ready \r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);

        return 1;
    } else {
        sprintf(msg, "MPU device is not ready \r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);

        return 0;
    }
}

struct AccelerometerData getAccelerometerData(I2C_HandleTypeDef hi2c1) {
    int16_t xAccRaw = read2ByteValueI2c(hi2c1, MPU_GYRO_ACCELEROMETER_ADDRESS + 1, MPU_X_ACC_REGISTER);
    int16_t yAccRaw = read2ByteValueI2c(hi2c1, MPU_GYRO_ACCELEROMETER_ADDRESS + 1, MPU_Y_ACC_REGISTER);
    int16_t zAccRaw = read2ByteValueI2c(hi2c1, MPU_GYRO_ACCELEROMETER_ADDRESS + 1, MPU_Z_ACC_REGISTER);

    double xAcc = xAccRaw / 16384.0;
    double yAcc = yAccRaw / 16384.0;
    double zAcc = zAccRaw / 16384.0;

    int16_t tempRaw = read2ByteValueI2c(hi2c1, MPU_GYRO_ACCELEROMETER_ADDRESS + 1, MPU_TEMP_REGISTER);

    double temp = (tempRaw / 340.0) + 35;

    int16_t xGyroRaw = read2ByteValueI2c(hi2c1, MPU_GYRO_ACCELEROMETER_ADDRESS + 1, MPU_X_GYRO_REGISTER);
    int16_t yGyroRaw = read2ByteValueI2c(hi2c1, MPU_GYRO_ACCELEROMETER_ADDRESS + 1, MPU_Y_GYRO_REGISTER);
    int16_t zGyroRaw = read2ByteValueI2c(hi2c1, MPU_GYRO_ACCELEROMETER_ADDRESS + 1, MPU_Z_GYRO_REGISTER);

    double xGyro = xGyroRaw / 131.0;
    double yGyro = yGyroRaw / 131.0;
    double zGyro = zGyroRaw / 131.0;

    struct AccelerometerData result;

    result.xAcc = xAcc;
    result.yAcc = yAcc;
    result.zAcc = zAcc;

    result.temp = temp;

    result.xGyro = xGyro;
    result.yGyro = yGyro;
    result.zGyro = zGyro;

    return result;
}
