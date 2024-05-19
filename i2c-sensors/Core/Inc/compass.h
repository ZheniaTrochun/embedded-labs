#ifndef I2C_SENSORS_COMPASS_H
#define I2C_SENSORS_COMPASS_H

#include "main.h"
#include "cmsis_os.h"

static const uint8_t HMC_COMPASS_ADDRESS = 0b00111100;
static const uint8_t HMC_X_REGISTER = 0x03;
static const uint8_t HMC_Y_REGISTER = 0x07;
static const uint8_t HMC_Z_REGISTER = 0x05;

void configureCompass(I2C_HandleTypeDef hi2c1);

int isCompassReady(I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart1);

double getHeadingDegrees(I2C_HandleTypeDef hi2c1);

#endif //I2C_SENSORS_COMPASS_H
