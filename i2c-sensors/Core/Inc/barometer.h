#ifndef I2C_SENSORS_BAROMETER_H
#define I2C_SENSORS_BAROMETER_H

#include "main.h"
#include "cmsis_os.h"

static const uint8_t BMP_BAROMETER_ADDRESS = 0b11101110;

int isBarometerReady(I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart1);

long getPressure(I2C_HandleTypeDef hi2c1);

long getPressureAvg(I2C_HandleTypeDef hi2c1, int measurements, int delay);

double computeAltitude(long pa);

#endif //I2C_SENSORS_BAROMETER_H
