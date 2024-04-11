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
#include "arm_const_structs.h"
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
const float32_t coeffs[64] = { 8.02421002e-04,  7.25938064e-04,  5.12894046e-04,  1.31785153e-04,
 -4.31837866e-04, -1.13951343e-03, -1.87208380e-03, -2.42621876e-03,
 -2.54591366e-03, -1.98909127e-03, -6.16700551e-04,  1.51941922e-03,
  4.11934253e-03,  6.63558316e-03,  8.34733050e-03,  8.50650154e-03,
  6.53307221e-03,  2.21910582e-03, -4.10925132e-03, -1.15209924e-02,
 -1.85355123e-02, -2.33136428e-02, -2.39634396e-02, -1.89102476e-02,
 -7.25969981e-03,  1.09242204e-02,  3.44933495e-02,  6.12825307e-02,
  8.83731243e-02,  1.12515544e-01,  1.30638711e-01,  1.40353271e-01,
  1.40353271e-01,  1.30638711e-01,  1.12515544e-01,  8.83731243e-02,
  6.12825307e-02,  3.44933495e-02,  1.09242204e-02, -7.25969981e-03,
 -1.89102476e-02, -2.39634396e-02, -2.33136428e-02, -1.85355123e-02,
 -1.15209924e-02, -4.10925132e-03,  2.21910582e-03,  6.53307221e-03,
  8.50650154e-03,  8.34733050e-03,  6.63558316e-03,  4.11934253e-03,
  1.51941922e-03, -6.16700551e-04, -1.98909127e-03, -2.54591366e-03,
 -2.42621876e-03, -1.87208380e-03, -1.13951343e-03, -4.31837866e-04,
  1.31785153e-04,  5.12894046e-04,  7.25938064e-04,  8.02421002e-04};

int16_t in_buf[256];
int16_t out_buf[256];
static float32_t overlap[256];
static float32_t fftbuf[256*2];
static float32_t filtered_spect[256*2];



static const int subtask = 3;

arm_cfft_instance_f32 *S;


static float32_t shift_coeffs[512];
static float32_t shift_coeffs_inv[512];
static float32_t shift_buf[512];

static uint8_t button_up_flag = 0;
static uint8_t button_down_flag = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */



/* USER CODE END PFP */
void overlap_add(void);
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
  
  S = &arm_cfft_sR_f32_len256;
  wm8731_dev.setup(&wm8731_dev, ADC48_DAC48); //initialize audio codec and set sampling rate
  wm8731_dev.startDacDma(&wm8731_dev); //start audio output
  wm8731_dev.startAdcDma(&wm8731_dev); //start audio input

  float32_t coeff_buf[512];
  for(int i = 0; i<256; i++){
    if(i < 64){
      coeff_buf[2*i+1] = 0;
      coeff_buf[2*i] = coeffs[i];
    } else {
      coeff_buf[2*i] = 0;
      coeff_buf[2*i+1] = 0;
    }
  }
  arm_cfft_f32(S, coeff_buf, 0, 1);

  float32_t wc = 15000.0/48000.0*PI*2;
  float32_t w_max = 2 * PI * 17000.0;
  float32_t increment = 500.0/48000.0;

  // set buffers to zero
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (button_up_flag || button_down_flag) {

      wc += (button_up_flag - button_down_flag) * increment * PI * 2;

      if(wc < 0){ wc = 0; }
      if(wc > w_max){ wc = w_max; }

      for (int i = 0; i<256; i++){
        shift_coeffs[2*i] = cos(wc*i);
        shift_coeffs[2*i+1] = -sin(wc*i);
        shift_coeffs_inv[2*i] = cos(wc*i);
        shift_coeffs_inv[2*i+1] = sin(wc*i);
      }

      button_up_flag = 0;
      button_down_flag = 0;
    }
    
    wm8731_waitInBuf(&wm8731_dev);
    wm8731_getInBuf(&wm8731_dev, &in_buf);
    for (int i = 0; i<512; i++){
      if (i < 256){
        fftbuf[i] = (float32_t)in_buf[i];
      } else {
        fftbuf[i] = 0;
      }
    }
    switch (subtask){
      case 1:{
        arm_cfft_f32(S, fftbuf, 0, 1);
        arm_cfft_f32(S, fftbuf, 1, 1);
      }
      case 2:{
        arm_cfft_f32(S, fftbuf, 0, 1);
        arm_cmplx_mult_cmplx_f32(fftbuf,coeff_buf, filtered_spect,256);
        arm_cfft_f32(S, filtered_spect, 1, 1);

        overlap_add();
        for (int i = 0; i<256; i++) {
          out_buf[i] = (int16_t)filtered_spect[i];
        }
      }
      case 3:{
        arm_cmplx_mult_cmplx_f32(fftbuf, shift_coeffs, shift_buf,256);

        arm_cfft_f32(S, shift_buf, 0, 1);
        arm_cmplx_mult_cmplx_f32(shift_buf,coeff_buf, filtered_spect,256);
        arm_cfft_f32(S, filtered_spect, 1, 1);

        arm_cmplx_mult_cmplx_f32(filtered_spect, shift_coeffs_inv, shift_buf,256);
        
        overlap_add();
        for (int i = 0; i<256; i++) {
          out_buf[i] = (int16_t) shift_buf[i];
        }
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
void overlap_add(void){
   for(int i=0; i < 256; i++) {
    shift_buf[i] += overlap[i]; // add saved overlap sample
    overlap[i] = shift_buf[i+256]; // save new overlap sample
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if(GPIO_Pin == BTN1_Pin) {
    button_up_flag = 1;
  }
   if(GPIO_Pin == BTN2_Pin) {
    button_down_flag = 1;
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
