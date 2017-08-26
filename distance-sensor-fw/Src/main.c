/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#define MEASUREMENT_RATE_HZ 30
#define USE_STATUS_LED      1

/* Private variables ---------------------------------------------------------*/


uint8_t last_msg[256];
int last_msg_len = -1;

static void LOG_MSG(const char* msg)
{
    last_msg[0] = '#';
    strncpy(last_msg + 1, msg, sizeof(last_msg) - 3);
    last_msg[sizeof(last_msg) - 1] = '\0';
    int msg_len = strlen(last_msg);
    last_msg[msg_len] = '\n';
    msg_len++;
    last_msg[msg_len] = '\0';
    CDC_Transmit_FS(last_msg, msg_len);
    HAL_UART_Transmit(&huart2, last_msg, msg_len, 5);
    last_msg_len = msg_len;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
    bool sensor_error = false;

    //reset sensor
    HAL_GPIO_WritePin(I2C1_RST_GPIO_Port, I2C1_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(I2C1_RST_GPIO_Port, I2C1_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(5);

    VL53L0X_Dev_t dev;
    dev.addr = 0x52;
    VL53L0X_Error status = VL53L0X_i2c_init();
    status = VL53L0X_DataInit(&dev);
    if (status != VL53L0X_ERROR_NONE) {
        LOG_MSG("data init failed");
        sensor_error = true;
    }

    status = VL53L0X_StaticInit(&dev);
    if (status != VL53L0X_ERROR_NONE) {
        LOG_MSG("static init failed");
        sensor_error = true;
    }

    uint8_t vhv_settings;
    uint8_t phase_cal;
    status = VL53L0X_PerformRefCalibration(&dev, &vhv_settings, &phase_cal);
    if (status != VL53L0X_ERROR_NONE) {
        LOG_MSG("reference calibration failed");
        sensor_error = true;
    }

    uint32_t ref_spad_count;
    uint8_t is_aperture_spads;
    status = VL53L0X_PerformRefSpadManagement(&dev,
        		&ref_spad_count, &is_aperture_spads);
    if (status != VL53L0X_ERROR_NONE) {
        LOG_MSG("spad management failed");
        sensor_error = true;
    }


    status = VL53L0X_SetDeviceMode(&dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    if (status != VL53L0X_ERROR_NONE) {
        LOG_MSG("set device mode failed");
        sensor_error = true;
    }

    status = VL53L0X_StartMeasurement(&dev);
    if (status != VL53L0X_ERROR_NONE) { 
        LOG_MSG("start measurement failed");
        sensor_error = true;
    }

    if (sensor_error) {
        HAL_GPIO_WritePin(LED_USR_GPIO_Port, LED_USR_Pin, 0);
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
        if (sensor_error && last_msg_len > 0) {
            //sensor has logged an error, report last error message 
            //in case user has not seen it if using USB
            CDC_Transmit_FS(last_msg, last_msg_len);
            HAL_UART_Transmit(&huart2, last_msg, last_msg_len, 5);
            HAL_Delay(1000);
            continue;
        }

        uint8_t new_data_ready = 0;
        status = VL53L0X_GetMeasurementDataReady(&dev, &new_data_ready);
        if (status != VL53L0X_ERROR_NONE) {
            LOG_MSG("getting measurement data ready flag failed");
        }

        if (new_data_ready) {
            VL53L0X_RangingMeasurementData_t data;
            status = VL53L0X_GetRangingMeasurementData(&dev, &data);
            if (status != VL53L0X_ERROR_NONE)
                LOG_MSG("getting measurement data failed");
#if USE_STATUS_LED
            HAL_GPIO_WritePin(LED_USR_GPIO_Port, LED_USR_Pin, 0);
#endif
            uint8_t buf[32];
            itoa(data.RangeMilliMeter, buf, 10);
            int str_len = strlen(buf);
            buf[str_len] = '\n';
            str_len++;
            CDC_Transmit_FS(buf, str_len);
            HAL_UART_Transmit(&huart2, buf, str_len, 5);
            if (status == VL53L0X_ERROR_NONE)
            {
                status = VL53L0X_ClearInterruptMask(&dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
            }

#if USE_STATUS_LED
            HAL_GPIO_WritePin(LED_USR_GPIO_Port, LED_USR_Pin, 1);
#endif
        }

        HAL_Delay(1000 / MEASUREMENT_RATE_HZ);
    }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
