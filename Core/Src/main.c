/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AllHead.h" // 自定义硬件驱动（OLED、MPU6050、软件I2C等）
#include <string.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t SenData[MAXLEN + 1];
uint8_t ReciveData[MAXLEN + 1];

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // 自定义初始化
  HAL_Delay(200); // 等待外设上电稳定
  ALL_Init();     // 各模块初始化
  ALL_Start();    // PWM、编码器、定时器中断、串口接收启动
  // OLED_FastFresh(LINE1, ROLL8, "OK");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*******************************代码调试开始********************************/
    static uint32_t last_tick = 0;
    // 蓝牙测试与MPU测试
    if (HAL_GetTick() - last_tick >= 10)
    {
      sprintf((char *)SenData, "[plot,%.1f]", Angle);
      // sprintf((char *)SenData, "[plot,%d,%d]", Gx, Ay);
      OLED_FastFresh(LINE1, ROLL1, SenData);
      Serial_SenData((uint8_t *)SenData, (uint16_t)strlen((char *)SenData));

      // static uint32_t last_gyro_display_tick = 0;
      // if (HAL_GetTick() - last_gyro_display_tick >= 100U)
      // {
      //   OLED_ClearArea(ROLL9, LINE5, 64, 16);
      //   sprintf((char *)SenData, "Gx:%d", Gx);
      //   OLED_ShowString(ROLL9, LINE5, (char *)SenData, OLED_6X8);
      //   sprintf((char *)SenData, "Ay:%d", Ay);
      //   OLED_ShowString(ROLL9, LINE6, (char *)SenData, OLED_6X8);
      //   OLED_UpdateArea(ROLL9, LINE5, 64, 16);
      //   last_gyro_display_tick = HAL_GetTick();
      // }

      uint8_t RxData[MAXLEN + 1];
      uint8_t RxReady = 0;
      uint32_t primask = __get_PRIMASK();
      __disable_irq();
      if (Serial_RxReady)
      {
        memcpy(RxData, ReciveData, Serial_RxLength + 1);
        Serial_RxReady = 0;
        RxReady = 1;
      }
      if (!primask)
        __enable_irq();

      // 蓝牙Alpha调试
      // if (RxReady)
      // {
      //   float Alpha_Value;
      //   int Alpha_End = 0;

      //   if (sscanf((char *)RxData, "[slider,Alpha,%f]%n", &Alpha_Value, &Alpha_End) == 1 &&
      //       Alpha_End > 0 && RxData[Alpha_End] == '\0' &&
      //       Alpha_Value >= 0.0f && Alpha_Value <= 1.0f)
      //   {
      //     Alpha = Alpha_Value;
      //     OLED_ClearArea(ROLL1, LINE7, 128, 8);
      //     sprintf((char *)SenData, "Alpha:%.2f", Alpha);
      //     OLED_ShowString(ROLL1, LINE7, (char *)SenData, OLED_6X8);
      //     OLED_UpdateArea(ROLL1, LINE7, 128, 8);
      //   }
      // }
      // 蓝牙调整直立环pid
      if (RxReady)
      {
        char Key[3] = {0};
        float Value;
        int End = 0;
        uint8_t Updated = 0;
        if (sscanf((char *)RxData, "[slider,%2[^,],%f]%n", Key, &Value, &End) == 2 &&
            End > 0 && RxData[End] == '\0')
        {
          if (strcmp(Key, "Kp") == 0 && Value >= 1.0f && Value <= 400.0f)
          {
            Vert.Kp = Value;
            Updated = 1;
          }
          else if (strcmp(Key, "Ki") == 0 && Value >= 0.0f && Value <= 1.0f)
          {
            Vert.Ki = Value;
            Updated = 1;
          }
          else if (strcmp(Key, "Kd") == 0 && Value >= 0.0f)
          {
            Vert.Kd = Value;
            Updated = 1;
          }
        }
        // // 蓝牙死区测试
        // char temp_pwm[4] = {0};
        // float pwm_value = 0.0f;
        // int pwm_end = 0;
        // if (sscanf((char *)RxData, "[slider,%3[^,],%f]%n",
        //            temp_pwm, &pwm_value, &pwm_end) == 2 &&
        //     pwm_end > 0 && RxData[pwm_end] == '\0' &&
        //     strcmp(temp_pwm, "PWM") == 0 &&
        //     pwm_value >= -2700.0f && pwm_value <= 2700.0f)
        // {
        //   int16_t pwm = (int16_t)pwm_value;
        //   Motor_SetSpeed(1, pwm);
        //   Motor_SetSpeed(2, pwm);

        //   /* 显示蓝牙PWM命令、TIM1通道4实际比较值及控制开关状态。 */
        //   OLED_ClearArea(ROLL1, LINE7, 128, 8);
        //   sprintf((char *)SenData, "P:%d C:%lu", pwm,
        //           (unsigned long)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4));
        //   OLED_ShowString(ROLL1, LINE7, (char *)SenData, OLED_6X8);
        //   OLED_UpdateArea(ROLL1, LINE7, 128, 8);
        // }

        if (Updated)
        {
          OLED_ClearArea(ROLL1, LINE3, 128, 24);
          sprintf((char *)SenData, "Kp:%.2f", Vert.Kp);
          OLED_ShowString(ROLL1, LINE3, (char *)SenData, OLED_6X8);
          sprintf((char *)SenData, "Ki:%.2f", Vert.Ki);
          OLED_ShowString(ROLL1, LINE4, (char *)SenData, OLED_6X8);
          sprintf((char *)SenData, "Kd:%.2f", Vert.Kd);
          OLED_ShowString(ROLL1, LINE5, (char *)SenData, OLED_6X8);
          OLED_UpdateArea(ROLL1, LINE3, 128, 24);
        }
      }
      //------------------------------------------//

      last_tick = HAL_GetTick();
    }

    // motor调试
    // Motor_SetSpeed(1, 1000);
    // Motor_SetSpeed(2, 1000);
    /*******************************代码调试结束********************************/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
