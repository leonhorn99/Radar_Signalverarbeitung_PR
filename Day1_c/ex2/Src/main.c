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
uint16_t period_offset = 0;
const uint16_t fs = 48000;
const uint16_t f1 = 800;
const uint16_t f2 = 8000;
uint8_t quarter = 1;
float64_t theta_1 = 2 * PI * 100 / 48000;
float64_t y_step_1[3] = {0, 0, 0};
float64_t theta_2 = 2 * PI * 300 / 48000;
float64_t y_step_2[3] = {0, 0, 0};
float64_t r = 1; // --> r^x = 1

uint16_t max = 0;
uint16_t min = 0;


 uint16_t sine_lut[480] = { 0,   429,   859,  1289,  1718,  2147,  2576,  3004,  3432,  3859,
   4285,  4711,  5136,  5560,  5983,  6405,  6826,  7246,  7664,  8082,
   8498,  8912,  9325,  9736, 10145, 10553, 10959, 11363, 11766, 12166,
  12564, 12960, 13353, 13745, 14134, 14520, 14904, 15286, 15664, 16041,
  16414, 16785, 17152, 17517, 17879, 18237, 18593, 18945, 19294, 19640,
  19982, 20321, 20657, 20988, 21317, 21641, 21962, 22279, 22592, 22902,
  23207, 23509, 23806, 24099, 24388, 24673, 24954, 25230, 25503, 25770,
  26033, 26292, 26546, 26796, 27041, 27282, 27517, 27748, 27974, 28196,
  28412, 28624, 28831, 29032, 29229, 29421, 29608, 29789, 29966, 30137,
  30303, 30464, 30620, 30770, 30915, 31055, 31189, 31318, 31442, 31560,
  31673, 31780, 31882, 31979, 32070, 32155, 32235, 32309, 32378, 32441,
  32499, 32551, 32597, 32638, 32673, 32703, 32727, 32745, 32758, 32765,
  32766, 32762, 32752, 32737, 32716, 32689, 32656, 32618, 32575, 32526,
  32471, 32410, 32344, 32273, 32196, 32113, 32025, 31931, 31832, 31727,
  31617, 31502, 31381, 31254, 31123, 30986, 30843, 30695, 30542, 30384,
  30221, 30052, 29878, 29699, 29515, 29326, 29131, 28932, 28728, 28519,
  28305, 28086, 27862, 27633, 27400, 27162, 26919, 26672, 26420, 26163,
  25902, 25637, 25367, 25093, 24814, 24531, 24244, 23953, 23658, 23358,
  23055, 22748, 22436, 22121, 21802, 21479, 21153, 20823, 20489, 20152,
  19812, 19467, 19120, 18769, 18416, 18058, 17698, 17335, 16969, 16600,
  16228, 15853, 15475, 15095, 14712, 14327, 13939, 13549, 13157, 12762,
  12365, 11966, 11565, 11162, 10757, 10350,  9941,  9531,  9118,  8705,
   8290,  7873,  7455,  7036,  6616,  6194,  5772,  5348,  4924,  4498,
   4072,  3645,  3218,  2790,  2361,  1933,  1503,  1074,   644,   214,
   -214,  -644, -1074, -1503, -1933, -2361, -2790, -3218, -3645, -4072,
  -4498, -4924, -5348, -5772, -6194, -6616, -7036, -7455, -7873, -8290,
  -8705, -9118, -9531, -9941,-10350,-10757,-11162,-11565,-11966,-12365,
 -12762,-13157,-13549,-13939,-14327,-14712,-15095,-15475,-15853,-16228,
 -16600,-16969,-17335,-17698,-18058,-18416,-18769,-19120,-19467,-19812,
 -20152,-20489,-20823,-21153,-21479,-21802,-22121,-22436,-22748,-23055,
 -23358,-23658,-23953,-24244,-24531,-24814,-25093,-25367,-25637,-25902,
 -26163,-26420,-26672,-26919,-27162,-27400,-27633,-27862,-28086,-28305,
 -28519,-28728,-28932,-29131,-29326,-29515,-29699,-29878,-30052,-30221,
 -30384,-30542,-30695,-30843,-30986,-31123,-31254,-31381,-31502,-31617,
 -31727,-31832,-31931,-32025,-32113,-32196,-32273,-32344,-32410,-32471,
 -32526,-32575,-32618,-32656,-32689,-32716,-32737,-32752,-32762,-32766,
 -32765,-32758,-32745,-32727,-32703,-32673,-32638,-32597,-32551,-32499,
 -32441,-32378,-32309,-32235,-32155,-32070,-31979,-31882,-31780,-31673,
 -31560,-31442,-31318,-31189,-31055,-30915,-30770,-30620,-30464,-30303,
 -30137,-29966,-29789,-29608,-29421,-29229,-29032,-28831,-28624,-28412,
 -28196,-27974,-27748,-27517,-27282,-27041,-26796,-26546,-26292,-26033,
 -25770,-25503,-25230,-24954,-24673,-24388,-24099,-23806,-23509,-23207,
 -22902,-22592,-22279,-21962,-21641,-21317,-20988,-20657,-20321,-19982,
 -19640,-19294,-18945,-18593,-18237,-17879,-17517,-17152,-16785,-16414,
 -16041,-15664,-15286,-14904,-14520,-14134,-13745,-13353,-12960,-12564,
 -12166,-11766,-11363,-10959,-10553,-10145, -9736, -9325, -8912, -8498,
  -8082, -7664, -7246, -6826, -6405, -5983, -5560, -5136, -4711, -4285,
  -3859, -3432, -3004, -2576, -2147, -1718, -1289,  -859,  -429,     0};

uint16_t sine_lut_quarter = { 0,   429,   859,  1289,  1718,  2147,  2576,  3004,  3432,  3859,
   4285,  4711,  5136,  5560,  5983,  6405,  6826,  7246,  7664,  8082,
   8498,  8912,  9325,  9736, 10145, 10553, 10959, 11363, 11766, 12166,
  12564, 12960, 13353, 13745, 14134, 14520, 14904, 15286, 15664, 16041,
  16414, 16785, 17152, 17517, 17879, 18237, 18593, 18945, 19294, 19640,
  19982, 20321, 20657, 20988, 21317, 21641, 21962, 22279, 22592, 22902,
  23207, 23509, 23806, 24099, 24388, 24673, 24954, 25230, 25503, 25770,
  26033, 26292, 26546, 26796, 27041, 27282, 27517, 27748, 27974, 28196,
  28412, 28624, 28831, 29032, 29229, 29421, 29608, 29789, 29966, 30137,
  30303, 30464, 30620, 30770, 30915, 31055, 31189, 31318, 31442, 31560,
  31673, 31780, 31882, 31979, 32070, 32155, 32235, 32309, 32378, 32441,
  32499, 32551, 32597, 32638, 32673, 32703, 32727, 32745, 32758, 32765,
  32766, 32762, 32752, 32737, 32716, 32689, 32656, 32618, 32575, 32526
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int get_sin(uint8_t type, uint8_t n_period);
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
  
  wm8731_dev.setup(&wm8731_dev, ADC48_DAC48); //initialize audio codec and set sampling rate
  wm8731_dev.startDacDma(&wm8731_dev); //start audio output

  uint8_t type = 3;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    wm8731_waitOutBuf(&wm8731_dev);
    wm8731_putOutBuf(&wm8731_dev, get_sin(type, period_offset));
    period_offset =  (period_offset+1) % ((fs/f1)*(fs/f2)); // "k"GV
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
int get_sin(uint8_t type, uint8_t n_period) {

  int16_t sine_1[128];
  int16_t sine_2[128];
  int16_t sine[256];
  float32_t amplitude = 0.1;

  if(type == 1) {  

    for(int16_t i=0; i < 128; i++) {
      sine_1[i] = (int16_t) (sin(2 * PI * (float32_t)(f1)/fs * (i + period_offset*128)) * (amplitude * 32767));
    }

    for(int16_t i=0; i < 128; i++) {
      sine_2[i] = (int16_t) (sin(2 * PI * (float32_t)(f2)/fs * (i + period_offset*128)) * (amplitude * 32767));
    }

    for(int i=0; i < 128; i++) {
      sine[i*2] = sine_1[i];
      sine[i*2+1] = sine_2[i];
    }
  }

  if(type == 2) {

    if(quarter){
      for(int16_t i=0; i < 128; i++) {
        sine[2*i] = (int16_t) (sine_lut[(i + period_offset*128)*8 % 479]);
        sine[2*i+1] = (int16_t) (sine_lut[(i + period_offset*128)*20 % 479]);
      }
        // TODO
      } else {
      for(int16_t i=0; i < 128; i++) {
        sine[2*i] = (int16_t) (sine_lut[(i + period_offset*128)*8 % 479]);
        sine[2*i+1] = (int16_t) (sine_lut[(i + period_offset*128)*20 % 479]);
      }
    }

  }

  if(type == 3) {
    for(int i=0; i < 128; i++) {
      y_step_1[0] = 2 * 1 * (float64_t) cos(theta_1) * y_step_1[1] - 1 * y_step_1[2] + 1;
      y_step_2[0] = 2 * 1 * (float64_t) cos(theta_2) * y_step_2[1] - 1 * y_step_2[2] + 1;
      sine[2*i] = (int16_t) (y_step_1[0] * theta_1 * cos(theta_1)-1);
      sine[2*i+1] = (int16_t) (y_step_2[0] * theta_2 * cos(theta_2)-1);
      y_step_1[2] = y_step_1[1];
      y_step_1[1] = y_step_1[0];
      y_step_2[2] = y_step_2[1];
      y_step_2[1] = y_step_2[0];
    }
  }

  if(type == 4) {
    // TODO Phasor
  }
  return &sine;
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
