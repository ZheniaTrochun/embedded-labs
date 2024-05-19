#include "barometer.h"
#include "i2c_util.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

int OVERSAMPLING_SETTING = 3;


int isBarometerReady(I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart1) {
    HAL_StatusTypeDef res = HAL_I2C_IsDeviceReady(&hi2c1, BMP_BAROMETER_ADDRESS, 1, 100);
    char msg[50] = {'\0'};

    if (res == HAL_OK) {
        sprintf(msg, "Barometer device is ready \r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);

        return 1;
    } else {
        sprintf(msg, "Barometer device is not ready \r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);

        return 0;
    }
}

long getPressure(I2C_HandleTypeDef hi2c1) {
    int16_t ac1 = read2ByteValueI2c(hi2c1, BMP_BAROMETER_ADDRESS, 0xAA);
    int16_t ac2 = read2ByteValueI2c(hi2c1, BMP_BAROMETER_ADDRESS, 0xAC);
    int16_t ac3 = read2ByteValueI2c(hi2c1, BMP_BAROMETER_ADDRESS, 0xAE);
    uint16_t ac4 = read2ByteValueI2c(hi2c1, BMP_BAROMETER_ADDRESS, 0xB0);
    uint16_t ac5 = read2ByteValueI2c(hi2c1, BMP_BAROMETER_ADDRESS, 0xB2);
    uint16_t ac6 = read2ByteValueI2c(hi2c1, BMP_BAROMETER_ADDRESS, 0xB4);

    int16_t b1 = read2ByteValueI2c(hi2c1, BMP_BAROMETER_ADDRESS, 0xB6);
    int16_t b2 = read2ByteValueI2c(hi2c1, BMP_BAROMETER_ADDRESS, 0xB8);

    int16_t mb = read2ByteValueI2c(hi2c1, BMP_BAROMETER_ADDRESS, 0xBA);
    int16_t mc = read2ByteValueI2c(hi2c1, BMP_BAROMETER_ADDRESS, 0xBC);
    int16_t md = read2ByteValueI2c(hi2c1, BMP_BAROMETER_ADDRESS, 0xBE);

    uint8_t readUncompensatedTempCommand = 0x2E;

    HAL_I2C_Mem_Write(&hi2c1, BMP_BAROMETER_ADDRESS, 0xF4, 1, &readUncompensatedTempCommand, 1, 100);
    osDelay(5);

    int16_t ut = read2ByteValueI2c(hi2c1, BMP_BAROMETER_ADDRESS, 0xF6);

    uint8_t readUncompensatedPressureCommand = 0x34 + (OVERSAMPLING_SETTING << 6);
    HAL_I2C_Mem_Write(&hi2c1, BMP_BAROMETER_ADDRESS, 0xF4, 1, &readUncompensatedPressureCommand, 1, 100);
    osDelay(26);

    uint8_t upData[3];
    HAL_I2C_Mem_Read(&hi2c1, BMP_BAROMETER_ADDRESS, 0xF6, 1, upData, 3, 100);
    int32_t up = (((int32_t) upData[0] << 16) + ((int32_t) upData[1] << 8) + upData[2]) >> (8 - OVERSAMPLING_SETTING);

    int32_t x1 = (ut - ac6) * ac5 / (int32_t) pow(2, 15);
    int32_t x2 = (mc * (int32_t) pow(2, 11)) / (x1 + md);
    int32_t b5 = x1 + x2;
    int32_t t = (b5 + 8) / (int32_t) pow(2, 4);

    int32_t b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 / (int32_t) pow(2, 12))) / (int32_t) pow(2, 11);
    x2 = ac2 * b6 / (int32_t) pow(2, 11);
    int32_t x3 = x1 + x2;
    int32_t b3 = (((ac1 * 4 + x3) << OVERSAMPLING_SETTING) + 2) / 4;
    x1 = ac3 * b6 / (int32_t) pow(2, 13);
    x2 = (b1 * (b6 * b6 / (int32_t) pow(2, 12))) / (int32_t) pow(2, 16);
    x3 = (x1 + x2 + 2) / (2 * 2);
    unsigned long b4 = ac4 * (unsigned long) (x3 + 32768) / (int32_t) pow(2, 15);
    unsigned long b7 = ((unsigned long) up - b3) * (50000 >> OVERSAMPLING_SETTING);

    int32_t p;

    if (b7 < 0x80000000) {
        p = (int32_t) ((b7 * 2) / b4);
    } else {
        p = (int32_t) ((b7 / b4) * 2);
    }

    x1 = (p / (int32_t) pow(2, 8)) * (p / (int32_t) pow(2, 8));
    x1 = (x1 * 3038) / (int32_t) pow(2, 16);
    x2 = (-7357 * p) / (int32_t) pow(2, 16);
    p = p + (x1 + x2 + 3791) / (int32_t) pow(2, 4);

    return p;
}

double computeAltitude(long p) {
    long p0 = 101325;

    return 44330 * (1 - (pow((double) p / p0, 1 / 5.255)));
}
