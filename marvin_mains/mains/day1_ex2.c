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
#include "i2c_hal.h"
#include "wm8731.h"
#include <math.h>
#include "arm_math.h"

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BUF_SIZE 256;
#define HALF_BUF_SIZE 128;
#define LUT_SIZE 480;
#define QLUT_SIZE 120;
#define MAX_AMP 32767;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static enum{SINEFUNC, LUT, IIR, PHASOR} subtasks;

static subtask = SINFUNC;

static const uint16_t fs = 48000;
static const uint16_t f1 = 800;
static const uint16_t f2 = 2000;
static const uint16_t fdivisor = (fs/f1)*(fs/f2); // kgV
static uint16_t period_offset = 0;

static const float64_t amplitude = 0.1;

// BUFFER
static int16_t* sine_buf;

// LUT
static const uint16_t fmin = 100;
static const uint16_t sine_lut[LUT_SIZE] = {    0,   427,   856,  1285,  1713,  2142,  2569,  2997,  3424,  3850,
   4276,  4700,  5125,  5548,  5970,  6391,  6811,  7230,  7648,  8064,
   8479,  8893,  9305,  9716, 10124, 10531, 10937, 11340, 11742, 12141,
  12538, 12933, 13326, 13717, 14105, 14491, 14875, 15256, 15634, 16010,
  16382, 16753, 17120, 17484, 17845, 18203, 18558, 18910, 19259, 19604,
  19946, 20285, 20620, 20952, 21280, 21604, 21925, 22241, 22555, 22864,
  23169, 23470, 23768, 24061, 24350, 24635, 24915, 25192, 25464, 25732,
  25995, 26254, 26508, 26758, 27003, 27244, 27480, 27711, 27938, 28160,
  28376, 28588, 28796, 28998, 29195, 29387, 29574, 29757, 29934, 30105,
  30272, 30434, 30590, 30741, 30887, 31028, 31163, 31293, 31417, 31536,
  31650, 31758, 31861, 31959, 32050, 32137, 32218, 32293, 32363, 32427,
  32486, 32539, 32587, 32629, 32665, 32696, 32722, 32741, 32755, 32764,
  32767, 32764, 32755, 32741, 32722, 32696, 32665, 32629, 32587, 32539,
  32486, 32427, 32363, 32293, 32218, 32137, 32050, 31959, 31861, 31758,
  31650, 31536, 31417, 31293, 31163, 31028, 30887, 30741, 30590, 30434,
  30272, 30105, 29934, 29757, 29574, 29387, 29195, 28998, 28796, 28588,
  28376, 28160, 27938, 27711, 27480, 27244, 27003, 26758, 26508, 26254,
  25995, 25732, 25464, 25192, 24915, 24635, 24350, 24061, 23768, 23470,
  23169, 22864, 22555, 22241, 21925, 21604, 21280, 20952, 20620, 20285,
  19946, 19604, 19259, 18910, 18558, 18203, 17845, 17484, 17120, 16753,
  16383, 16010, 15634, 15256, 14875, 14491, 14105, 13717, 13326, 12933,
  12538, 12141, 11742, 11340, 10937, 10531, 10124,  9716,  9305,  8893,
   8479,  8064,  7648,  7230,  6811,  6391,  5970,  5548,  5125,  4700,
   4276,  3850,  3424,  2997,  2569,  2142,  1713,  1285,   856,   427,
      0,  -429,  -858, -1287, -1715, -2144, -2571, -2999, -3426, -3852,
  -4278, -4702, -5127, -5550, -5972, -6393, -6813, -7232, -7650, -8066,
  -8481, -8895, -9307, -9718,-10126,-10533,-10939,-11342,-11744,-12143,
 -12540,-12935,-13328,-13719,-14107,-14493,-14877,-15258,-15636,-16012,
 -16384,-16755,-17122,-17486,-17847,-18205,-18560,-18912,-19261,-19606,
 -19948,-20287,-20622,-20954,-21282,-21606,-21927,-22243,-22557,-22866,
 -23171,-23472,-23770,-24063,-24352,-24637,-24917,-25194,-25466,-25734,
 -25997,-26256,-26510,-26760,-27005,-27246,-27482,-27713,-27940,-28162,
 -28378,-28590,-28798,-29000,-29197,-29389,-29576,-29759,-29936,-30107,
 -30274,-30436,-30592,-30743,-30889,-31030,-31165,-31295,-31419,-31538,
 -31652,-31760,-31863,-31961,-32052,-32139,-32220,-32295,-32365,-32429,
 -32488,-32541,-32589,-32631,-32667,-32698,-32724,-32743,-32757,-32766,
 -32767,-32766,-32757,-32743,-32724,-32698,-32667,-32631,-32589,-32541,
 -32488,-32429,-32365,-32295,-32220,-32139,-32052,-31961,-31863,-31760,
 -31652,-31538,-31419,-31295,-31165,-31030,-30889,-30743,-30592,-30436,
 -30274,-30107,-29936,-29759,-29576,-29389,-29197,-29000,-28798,-28590,
 -28378,-28162,-27940,-27713,-27482,-27246,-27005,-26760,-26510,-26256,
 -25997,-25734,-25466,-25194,-24917,-24637,-24352,-24063,-23770,-23472,
 -23171,-22866,-22557,-22243,-21927,-21606,-21282,-20954,-20622,-20287,
 -19948,-19606,-19261,-18912,-18560,-18205,-17847,-17486,-17122,-16755,
 -16385,-16012,-15636,-15258,-14877,-14493,-14107,-13719,-13328,-12935,
 -12540,-12143,-11744,-11342,-10939,-10533,-10126, -9718, -9307, -8895,
  -8481, -8066, -7650, -7232, -6813, -6393, -5972, -5550, -5127, -4702,
  -4278, -3852, -3426, -2999, -2571, -2144, -1715, -1287,  -858,  -429};
static const uint16_t sine_lut_quarter[QLUT_SIZE] = { 0,   427,   856,  1285,  1713,  2142,  2569,  2997,  3424,  3850,
   4276,  4700,  5125,  5548,  5970,  6391,  6811,  7230,  7648,  8064,
   8479,  8893,  9305,  9716, 10124, 10531, 10937, 11340, 11742, 12141,
  12538, 12933, 13326, 13717, 14105, 14491, 14875, 15256, 15634, 16010,
  16382, 16753, 17120, 17484, 17845, 18203, 18558, 18910, 19259, 19604,
  19946, 20285, 20620, 20952, 21280, 21604, 21925, 22241, 22555, 22864,
  23169, 23470, 23768, 24061, 24350, 24635, 24915, 25192, 25464, 25732,
  25995, 26254, 26508, 26758, 27003, 27244, 27480, 27711, 27938, 28160,
  28376, 28588, 28796, 28998, 29195, 29387, 29574, 29757, 29934, 30105,
  30272, 30434, 30590, 30741, 30887, 31028, 31163, 31293, 31417, 31536,
  31650, 31758, 31861, 31959, 32050, 32137, 32218, 32293, 32363, 32427,
  32486, 32539, 32587, 32629, 32665, 32696, 32722, 32741, 32755, 32764,
  32767, 32764, 32755, 32741, 32722, 32696, 32665, 32629, 32587, 32539};

// IIR
static float64_t theta_1 = 2 * PI * 100 / fs;
static float64_t y_step_1[3] = {0, 0, 0};
static float64_t theta_2 = 2 * PI * 300 / fs;
static float64_t y_step_2[3] = {0, 0, 0};
static float64_t r = 1; // --> r^x = 1

// Phasor
static float64_t re_d_Phi1;
static float64_t im_d_Phi1;
static float64_t re_Phi1 = 0.0;
static float64_t im_Phi1 = 1.0;

static float64_t re_d_Phi2;
static float64_t im_d_Phi2;
static float64_t re_Phi2 = 0.0;
static float64_t im_Phi2 = 1.0;

static float64_t re_Phi1_old;
static float64_t im_Phi1_old;
static float64_t re_Phi2_old;
static float64_t im_Phi2_old;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void sine_func(void);
void sine_lut(int quarter);
void sine_iir(void);
void sine_phasor(void);

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

  // init buffer
  sine_buf = (uint16_t *)calloc(BUF_SIZE, sizeof(uint16_t));

  // pre-calc phasor values 
  if(subtask == PHASOR){
    re_d_Phi1 = cos(2*PI*f1/fs);
    im_d_Phi1 = sin(2*PI*f1/fs);
    re_d_Phi2 = cos(2*PI*f2/fs);
    im_d_Phi2 = sin(2*PI*f2/fs);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    switch(subtask){ // calc sine in buffer
      case SINEFUNC:
        sine_func();
      case LUT:
        sine_lut(0); // 0 --> full LUT, 1 --> 1/4th LUT
      case IIR:
        sine_iir();
      case PHASOR:
        sine_phasor();
    }

    wm8731_waitOutBuf(&wm8731_dev);
    wm8731_putOutBuf(&wm8731_dev, &sine_buf); // put buffer

    period_offset = ++period_offset % fdivisor; // calc period offset

    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }

  free(sine_buf);
  
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

void sine_func(void){
  for(int16_t i=0; i < HALF_BUF_SIZE; i++) {
    sine_buf[i*2] = (int16_t) (sin(2 * PI * (float32_t)(f1)/fs * (i + period_offset*HALF_BUF_SIZE)) * (amplitude * MAX_AMP)); // left channel
    sine_buf[i*2+1] = (int16_t) (sin(2 * PI * (float32_t)(f2)/fs * (i + period_offset*HALF_BUF_SIZE)) * (amplitude * MAX_AMP)); // right channel
  }
}

void sine_lut(int quarter){
  if(quarter){ // only 1/4th of the period saved in LUT
    uint16_t idx_left;
    uint16_t idx_right;

    for(int16_t i=0; i < HALF_BUF_SIZE; i++) {
      idx_left = (i + period_offset*HALF_BUF_SIZE)*(f1/fmin) % LUT_SIZE; // f1
      idx_right = (i + period_offset*HALF_BUF_SIZE)*(f2/fmin) % LUT_SIZE; // f2

      switch(idx_left / QLUT_SIZE){
        case 0: sine_buf[2*i] = (int16_t) (sine_lut_quarter[idx_left]);
        case 1: sine_buf[2*i] = (int16_t) (sine_lut_quarter[2*QLUT_SIZE-1-idx_left]);
        case 2: sine_buf[2*i] = -(int16_t) (sine_lut_quarter[-2*QLUT_SIZE+idx_left]);
        case 3: sine_buf[2*i] = -(int16_t) (sine_lut_quarter[4*QLUT_SIZE-1-idx_left]);
      }
      switch(idx_right / QLUT_SIZE){
        case 0: sine_buf[2*i+1] = (int16_t) (sine_lut_quarter[idx_right]);
        case 1: sine_buf[2*i+1] = (int16_t) (sine_lut_quarter[2*QLUT_SIZE-1-idx_right]);
        case 2: sine_buf[2*i+1] = -(int16_t) (sine_lut_quarter[-2*QLUT_SIZE+idx_right]);
        case 3: sine_buf[2*i+1] = -(int16_t) (sine_lut_quarter[4*QLUT_SIZE-1-idx_right]);
      }
    }
  } else { // complete period in LUT
    for(int16_t i=0; i < HALF_BUF_SIZE; i++) {
      sine_buf[2*i] = (int16_t) sine_lut[((i + period_offset*HALF_BUF_SIZE)*(f1/fmin)) % LUT_SIZE];
      sine_buf[2*i+1] = (int16_t) sine_lut[(i + period_offset*HALF_BUF_SIZE)*(f2/fmin) % LUT_SIZE];
    }
  }
}

void sine_iir(void){
  for(int i=0; i < HALF_BUF_SIZE; i++) {
      y_step_1[0] = 2 * 1 * (float64_t) cos(theta_1) * y_step_1[1] - 1 * y_step_1[2] + 1;
      y_step_2[0] = 2 * 1 * (float64_t) cos(theta_2) * y_step_2[1] - 1 * y_step_2[2] + 1;
      sine_buf[2*i] = (int16_t) (y_step_1[0] * theta_1 * cos(theta_1)-1);
      sine_buf[2*i+1] = (int16_t) (y_step_2[0] * theta_2 * cos(theta_2)-1);
      y_step_1[2] = y_step_1[1];
      y_step_1[1] = y_step_1[0];
      y_step_2[2] = y_step_2[1];
      y_step_2[1] = y_step_2[0];
    }
}

void sine_phasor(void){
  // PHASOR, new, to be testet
  if (period_offset == 0) {
      re_Phi1 = 0.0;
      im_Phi1 = 1.0;
      re_Phi2 = 0.0;
      re_Phi2 = 1.0;
  }
  for(int i=0; i < HALF_BUF_SIZE; i++) {
    sine_buf[2*i] = (int16_t) (re_Phi1 * MAX_AMP);
    sine_buf[2*i+1] = (int16_t) (re_Phi2 * MAX_AMP);
    re_Phi1_old = re_Phi1;
    im_Phi1_old = im_Phi1;
    re_Phi2_old = re_Phi2;
    im_Phi2_old = im_Phi2;
    re_Phi1 = re_d_Phi1 * re_Phi1_old - im_Phi1_old * im_d_Phi1;
    im_Phi1 = im_d_Phi1 * re_Phi1_old + re_d_Phi1 * im_Phi1_old;
    re_Phi2 = re_d_Phi2 * re_Phi2_old - im_Phi2_old * im_d_Phi2;
    im_Phi2 = im_d_Phi2 * re_Phi2_old + re_d_Phi2 * im_Phi2_old;
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
