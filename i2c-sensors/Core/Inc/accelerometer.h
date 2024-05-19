#ifndef I2C_SENSORS_ACCELEROMETER_H
#define I2C_SENSORS_ACCELEROMETER_H

#include "main.h"
#include "cmsis_os.h"

static const uint8_t MPU_GYRO_ACCELEROMETER_ADDRESS = 0b11010000;
static const uint8_t MPU_PWR_MANAGEMENT_REGISTER = 0x6B;
static const uint8_t MPU_USER_CTRL_REGISTER = 0x6A;
static const uint8_t MPU_INT_PIN_CFG_REGISTER = 0x37;
static const uint8_t MPU_X_ACC_REGISTER = 0x3B;
static const uint8_t MPU_Y_ACC_REGISTER = 0x3D;
static const uint8_t MPU_Z_ACC_REGISTER = 0x3F;
static const uint8_t MPU_TEMP_REGISTER = 0x41;
static const uint8_t MPU_X_GYRO_REGISTER = 0x43;
static const uint8_t MPU_Y_GYRO_REGISTER = 0x45;
static const uint8_t MPU_Z_GYRO_REGISTER = 0x47;

struct AccelerometerData {
    double xAcc;
    double yAcc;
    double zAcc;

    double temp;

    double xGyro;
    double yGyro;
    double zGyro;
};

int configureMPU(I2C_HandleTypeDef hi2c1);

int isAccelerometerReady(I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart1);

struct AccelerometerData getAccelerometerData(I2C_HandleTypeDef hi2c1);

#endif //I2C_SENSORS_ACCELEROMETER_H
