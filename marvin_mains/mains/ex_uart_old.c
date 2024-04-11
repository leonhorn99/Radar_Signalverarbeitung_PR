/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "sai.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_hal.h"
#include "wm8731.h"
#include <math.h>
#include "arm_math.h"
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
const float32_t coeffs[64] = {
  -0.00030995, -0.00079491, -0.00089012, -0.0004301 ,  0.00051534,
        0.00151232,  0.00184793,  0.00093461, -0.00113637, -0.00331563,
       -0.00398011, -0.00196547,  0.00232788,  0.00661611,  0.00774692,
        0.00373992, -0.00434217, -0.01213514, -0.0140197 , -0.00670273,
        0.0077386 ,  0.02160706,  0.02507625,  0.0121237 , -0.0142744 ,
       -0.04109939, -0.0499559 , -0.02588554,  0.03390254,  0.11618024,
        0.1954186 ,  0.24394961,  0.24394961,  0.1954186 ,  0.11618024,
        0.03390254, -0.02588554, -0.0499559 , -0.04109939, -0.0142744 ,
        0.0121237 ,  0.02507625,  0.02160706,  0.0077386 , -0.00670273,
       -0.0140197 , -0.01213514, -0.00434217,  0.00373992,  0.00774692,
        0.00661611,  0.00232788, -0.00196547, -0.00398011, -0.00331563,
       -0.00113637,  0.00093461,  0.00184793,  0.00151232,  0.00051534,
       -0.0004301 , -0.00089012, -0.00079491, -0.00030995
};

int16_t in_buf[256];
float64_t left_buf[191];
float64_t left_buf_old[64];
int16_t out_buf[256];
float32_t pState[128+128+64-1]; // n_Coeffs + n_X -1
float32_t dspBufLeft[128];
static float32_t dspOut[128];
arm_fir_instance_f32 S;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void overlap_add(void);
void manual_conv(void);
void overlap_add_half(void);
void overlap_add_library(void);
void manual_conv_half(void);

const int method = 3;  // 1 Full FIR convolution, 2 Half FIR Convolution, 3 Library Function
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
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_UART4_Init();
  MX_SAI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  struct i2c_dev_s i2c_dev;
  i2c_init(&i2c_dev, &hi2c2);
  struct wm8731_dev_s wm8731_dev;
  wm8731_init(&wm8731_dev, &i2c_dev, &hsai_BlockB1, &hsai_BlockA1, 0b00110100);
  
  arm_fir_init_f32(&S, 64, &coeffs[0], pState, 128);
  
  wm8731_dev.setup(&wm8731_dev, ADC8_DAC8); //initialize audio codec and set sampling rate
  wm8731_dev.startDacDma(&wm8731_dev); //start audio output
  wm8731_dev.startAdcDma(&wm8731_dev); //start audio input

  // set buffers to zero
  for(int i=0; i < 191; i++) {
    dspBufLeft[i] = 0.0;
  }
  for(int i=0; i < 191; i++) {
    left_buf[i] = 0.0;
  }
  for(int i=0; i < 64; i++) {
    left_buf_old[i] = 0.0;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    wm8731_waitInBuf(&wm8731_dev);
    wm8731_getInBuf(&wm8731_dev, &in_buf);
    switch (method){
      case 1: {
        overlap_add(); 
        break;
      }
      case 2: {
        overlap_add_half(); 
        break;
      }
      case 3: {
        overlap_add_library();
        break;
      }
    }

    wm8731_waitOutBuf(&wm8731_dev);
    wm8731_putOutBuf(&wm8731_dev, &out_buf);

    //HAL_UART_Transmit(&huart4, &left, sizeof(left), HAL_MAX_DELAY);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void overlap_add(void) {
  manual_conv(); // convolution result stored in left_buf

  for(int i=0; i < 63; i++) {
    left_buf[i] += left_buf_old[i]; // add saved overlap sample
    left_buf_old[i] = left_buf[i+128]; // save new overlap sample
  }

  for(int i=0; i < 128; i++) {
    out_buf[2*i] = (int16_t)(left_buf[i]);
    out_buf[2*i+1] = (int16_t)(left_buf[i]);

    // if(i == 120){
    //   break;
    // }
  }
}

void manual_conv(void) {
  for(int i=0; i < 191; i++) {
    left_buf[i] = 0;
  }
  for(int n=0; n < 128; n++) {
    for(int m=0; m < 64; m++) {
      left_buf[n+m] += (coeffs[m] * in_buf[2*n]);
    }
  }
}

void overlap_add_half(void) {
  manual_conv_half(); // convolution result stored in left_buf

  for(int i=0; i < 63; i++) {
    left_buf[i] += left_buf_old[i]; // add saved overlap sample
    left_buf_old[i] = left_buf[i+128]; // save new overlap sample
  }

  for(int i=0; i < 128; i++) {
    out_buf[2*i] = (int16_t)(left_buf[i]);
    out_buf[2*i+1] = (int16_t)(left_buf[i]);

    // if(i == 120){
    //   break;
    // }
  }
}

void manual_conv_half(void) {
  for(int i=0; i < 191; i++) {
    left_buf[i] = 0;
  }
  for(int n=0; n < 128; n++) {
    for(int m=0; m < 32; m++) {
      left_buf[n+m] += coeffs[m] * in_buf[2*(n)];
      left_buf[n+m+32] += coeffs[m + 32]*in_buf[2*(n)];
    }
  }
}

void overlap_add_library(void) {

  for(int i=0; i < (128); i++) {
    dspBufLeft[i] = (float32_t)in_buf[2*i];
  }


  arm_fir_f32(&S, &dspBufLeft[0], &dspOut[0], 128);
  

  //for(int i=0; i < 63; i++) {
  //   left_buf[i] = dspOut[i] + left_buf_old[i]; // add saved overlap sample
  //   left_buf_old[i] = dspOut[i+128]; // save new overlap sample
  //}

  for(int i=0; i < 128; i++) {
    out_buf[2*i] = (int16_t)(dspOut[i]);
    out_buf[2*i+1] = (int16_t)(dspOut[i]);

    // if(i == 120){
    //   break;
    // }
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
