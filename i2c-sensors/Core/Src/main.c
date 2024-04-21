/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId sensorsReadTaskHandle;
osThreadId outputTaskHandle;
osMessageQId pressureQueueHandle;
osMessageQId accelerationQueueHandle;
osMessageQId gyroscopeQueueHandle;
/* USER CODE BEGIN PV */

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

static const uint8_t HMC_COMPASS_ADDRESS = 0b00111101;

static const uint8_t BMP_BAROMETER_ADDRESS = 0b11101110;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_USART2_UART_Init(void);

static void MX_I2C1_Init(void);

static void MX_USART1_UART_Init(void);

void StartSensorsReadTask(void const *argument);

void StartOutputTask(void const *argument);

/* USER CODE BEGIN PFP */

void println(char *msg);

int testI2cDevice(uint8_t address, char *deviceName);

void configureMPU();

int16_t read2ByteValueI2c(uint8_t address, uint8_t registerAddress);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* definition and creation of pressureQueue */
    osMessageQDef(pressureQueue, 16, uint16_t);
    pressureQueueHandle = osMessageCreate(osMessageQ(pressureQueue), NULL);

    /* definition and creation of accelerationQueue */
    osMessageQDef(accelerationQueue, 16, uint16_t);
    accelerationQueueHandle = osMessageCreate(osMessageQ(accelerationQueue), NULL);

    /* definition and creation of gyroscopeQueue */
    osMessageQDef(gyroscopeQueue, 16, uint16_t);
    gyroscopeQueueHandle = osMessageCreate(osMessageQ(gyroscopeQueue), NULL);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* definition and creation of sensorsReadTask */
    osThreadDef(sensorsReadTask, StartSensorsReadTask, osPriorityNormal, 0, 128);
    sensorsReadTaskHandle = osThreadCreate(osThread(sensorsReadTask), NULL);

    /* definition and creation of outputTask */
    osThreadDef(outputTask, StartOutputTask, osPriorityNormal, 0, 128);
    outputTaskHandle = osThreadCreate(osThread(outputTask), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void) {

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void) {

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void) {

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : LD2_Pin */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void println(char *msg) {
    HAL_UART_Transmit(&huart1, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);
}

int testI2cDevice(uint8_t address, char *deviceName) {
    HAL_StatusTypeDef res = HAL_I2C_IsDeviceReady(&hi2c1, address, 1, 100);
    char msg[50] = {'\0'};

    if (res == HAL_OK) {
        sprintf(msg, "%s device is ready \r\n", deviceName);
        println(msg);

        return 1;
    } else {
        sprintf(msg, "%s device is not ready \r\n", deviceName);
        println(msg);

        return 0;
    }
}

void configureMPU() {
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
        println("MPU device configured successfully \r\n");
    } else {
        println("Failed to configure MPU \r\n");
    }
}

int16_t read2ByteValueI2c(uint8_t address, uint8_t registerAddress) {
    uint8_t data[2];

    HAL_StatusTypeDef res = HAL_I2C_Mem_Read(&hi2c1, address, registerAddress, 1, data, 2, 100);

    if (res == HAL_OK) {
        return ((int16_t) data[0] << 8) + data[1];
    } else {
        println("Failed to read measurement \r\n");
        return 0;
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSensorsReadTask */
/**
  * @brief  Function implementing the sensorsReadTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSensorsReadTask */
void StartSensorsReadTask(void const *argument) {
    /* USER CODE BEGIN 5 */

//    todo: move to i2c init function
    configureMPU();

    int gyroAccelerometerReady = testI2cDevice(MPU_GYRO_ACCELEROMETER_ADDRESS, "MPU");
    int barometerReady = testI2cDevice(BMP_BAROMETER_ADDRESS, "Barometer");
    int compassReady = testI2cDevice(HMC_COMPASS_ADDRESS, "Compass");

    char msg[75] = {'\0'};

    if (!gyroAccelerometerReady || !barometerReady || !compassReady) {
        println("[ERROR] At least one of devices is not ready. Stopping application... \r\n");

        osDelay(1000);

        return;
    } else {
        println("===================== \r\n");
        println("All devices are ready \r\n");
        println("===================== \r\n");
    }

    osDelay(1000);
    HAL_StatusTypeDef res;
    /* Infinite loop */
    for (;;) {
        int16_t xAccRaw = read2ByteValueI2c(MPU_GYRO_ACCELEROMETER_ADDRESS + 1, MPU_X_ACC_REGISTER);
        int16_t yAccRaw = read2ByteValueI2c(MPU_GYRO_ACCELEROMETER_ADDRESS + 1, MPU_Y_ACC_REGISTER);
        int16_t zAccRaw = read2ByteValueI2c(MPU_GYRO_ACCELEROMETER_ADDRESS + 1, MPU_Z_ACC_REGISTER);

        double xAcc = xAccRaw / 16384.0;
        double yAcc = yAccRaw / 16384.0;
        double zAcc = zAccRaw / 16384.0;

        println("===================== \r\n");

        sprintf(msg, "X acceleration: %.3f g \r\n", xAcc);
        println(msg);
        sprintf(msg, "Y acceleration: %.3f g \r\n", yAcc);
        println(msg);
        sprintf(msg, "Z acceleration: %.3f g \r\n", zAcc);
        println(msg);

        int16_t tempRaw = read2ByteValueI2c(MPU_GYRO_ACCELEROMETER_ADDRESS + 1, MPU_TEMP_REGISTER);

        double temp = (tempRaw / 340.0) + 35;
        sprintf(msg, "temperature: %.3f C \r\n", temp);
        println(msg);

        int16_t xGyroRaw = read2ByteValueI2c(MPU_GYRO_ACCELEROMETER_ADDRESS + 1, MPU_X_GYRO_REGISTER);
        int16_t yGyroRaw = read2ByteValueI2c(MPU_GYRO_ACCELEROMETER_ADDRESS + 1, MPU_Y_GYRO_REGISTER);
        int16_t zGyroRaw = read2ByteValueI2c(MPU_GYRO_ACCELEROMETER_ADDRESS + 1, MPU_Z_GYRO_REGISTER);

        double xGyro = xGyroRaw / 131.0;
        double yGyro = yGyroRaw / 131.0;
        double zGyro = zGyroRaw / 131.0;

        sprintf(msg, "X rotation: %.3f %% \r\n", xGyro);
        println(msg);
        sprintf(msg, "Y rotation: %.3f %% \r\n", yGyro);
        println(msg);
        sprintf(msg, "Z rotation: %.3f %% \r\n", zGyro);
        println(msg);

        println("===================== \r\n");

        osDelay(500);
    }
    /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartOutputTask */
/**
* @brief Function implementing the outputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOutputTask */
void StartOutputTask(void const *argument) {
    /* USER CODE BEGIN StartOutputTask */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END StartOutputTask */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
