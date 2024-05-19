#include "compass.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static const double scale = 2.56;

// magnetic declination of Kyiv
// https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
static const int declinationDegrees = 8;
static const int declinationMinutes = 30;
static const double declicationOffsetRadians = (declinationDegrees + (1.0 / 60 * declinationMinutes)) * (M_PI / 180);

void configureCompass(I2C_HandleTypeDef hi2c1) {
    uint8_t continuousMode = 0;
    HAL_I2C_Mem_Write(&hi2c1,
                      HMC_COMPASS_ADDRESS + 0,
                      0x02,
                      1,
                      &continuousMode,
                      1,
                      100);

    uint8_t measurementsAvg = 0x70;
    HAL_I2C_Mem_Write(&hi2c1,
                      HMC_COMPASS_ADDRESS + 0,
                      0x00,
                      1,
                      &measurementsAvg,
                      1,
                      100);

    uint8_t gainParam = 0xA0;
    HAL_I2C_Mem_Write(&hi2c1,
                      HMC_COMPASS_ADDRESS + 0,
                      0x01,
                      1,
                      &gainParam,
                      1,
                      100);
}

int isCompassReady(I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart1) {
    HAL_StatusTypeDef res = HAL_I2C_IsDeviceReady(&hi2c1, HMC_COMPASS_ADDRESS, 1, 100);
    char msg[50] = {'\0'};

    if (res == HAL_OK) {
        sprintf(msg, "Compass device is ready \r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);

        return 1;
    } else {
        sprintf(msg, "Compass device is not ready \r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);

        return 0;
    }
}

double getHeadingDegrees(I2C_HandleTypeDef hi2c1) {
    uint8_t data[6];

    HAL_I2C_Mem_Read(&hi2c1, HMC_COMPASS_ADDRESS + 1, HMC_X_REGISTER, 1, data, 6, 100);

    int16_t xOrientationRaw = ((int16_t) data[0] << 8) + data[1];
    int16_t zOrientationRaw = ((int16_t) data[2] << 8) + data[3];
    int16_t yOrientationRaw = ((int16_t) data[4] << 8) + data[5];

    double xOrientation = xOrientationRaw * scale;
    double yOrientation = yOrientationRaw * scale;
    double zOrientation = zOrientationRaw * scale;

    double heading = atan2(xOrientation, yOrientation);
    heading -= declicationOffsetRadians;
    // Correct for when signs are reversed.
    if (heading < 0)
        heading += 2 * M_PI;

    // Check for wrap due to addition of declination.
    if (heading > 2 * M_PI)
        heading -= 2 * M_PI;

    // Convert radians to degrees for readability.
    double degrees = heading * 180 / M_PI;

    return degrees;
}
