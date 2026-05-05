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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_platform_drive.h"
#include "TB6612.h"
#include "camera_uart.h"
#include "encoder.h"
#include "icm42688p.h"
#include "screen_uart.h"
#include "task3.h"
#include "vision_app.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_GOTO_PWM_ABS              ((int16_t)360)
#define APP_GOTO_PWM_SLOW_ABS         ((int16_t)260)
#define APP_GOTO_COUNT_TOLERANCE      ((int32_t)8)
#define APP_GOTO_COUNT_NEAR_THRESHOLD ((int32_t)80)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static app_platform_drive_t s_platform_drive;
static app_platform_drive_config_t s_platform_config;
static uint32_t s_last_drive_update_ms = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void App_MainProcess(void);
static void App_HandleScreenCommands(void);
static void App_PrintHelp(void);
static uint8_t App_ReadAllEncoderCounts(int32_t counts[ENCODER_COUNT]);
static void App_PrintEncoderCounts(const char *prefix, const int32_t counts[ENCODER_COUNT]);
static void App_PrintPlatformState(void);
static const char *App_PointName(uint8_t point_index);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void App_PrintHelp(void)
{
  (void)ScreenUart_Printf("point map: 1=LT 2=RT 3=RB 4=LB 5=C\r\n");
  (void)ScreenUart_Printf("cmd: zero, 1, 2, 3, 4, 5, enc, stat, help\r\n");
}

static uint8_t App_ReadAllEncoderCounts(int32_t counts[ENCODER_COUNT])
{
  uint32_t index;

  if (counts == NULL)
  {
    return 0U;
  }

  for (index = 0U; index < (uint32_t)ENCODER_COUNT; index++)
  {
    if (Encoder_GetCount((encoder_id_t)index, &counts[index]) != ENCODER_STATUS_OK)
    {
      return 0U;
    }
  }

  return 1U;
}

static void App_PrintEncoderCounts(const char *prefix, const int32_t counts[ENCODER_COUNT])
{
  if ((prefix == NULL) || (counts == NULL))
  {
    return;
  }

  (void)ScreenUart_Printf("%s m1=%ld m2=%ld m3=%ld m4=%ld\r\n",
                          prefix,
                          (long)counts[0],
                          (long)counts[1],
                          (long)counts[2],
                          (long)counts[3]);
}

static void App_PrintPlatformState(void)
{
  const rope_platform_solver_t *solver = AppPlatformDrive_GetSolver(&s_platform_drive);

  if ((solver == NULL) || (solver->state_valid == 0U))
  {
    (void)ScreenUart_Printf("pose invalid\r\n");
    return;
  }

  (void)ScreenUart_Printf("pose x=%.4f y=%.4f vx=%.4f vy=%.4f rms=%.5f zero=%u goto=%u\r\n",
                          (double)solver->position_x_m,
                          (double)solver->position_y_m,
                          (double)solver->velocity_x_mps,
                          (double)solver->velocity_y_mps,
                          (double)solver->residual_rms_m,
                          (unsigned int)Task3_IsCenterZeroReady(),
                          (unsigned int)Task3_IsBusy());
}

static const char *App_PointName(uint8_t point_index)
{
  switch (point_index)
  {
    case 1U:
      return "LT";
    case 2U:
      return "RT";
    case 3U:
      return "RB";
    case 4U:
      return "LB";
    case 5U:
      return "C";
    default:
      return "?";
  }
}

static void App_HandleScreenCommands(void)
{
  char line[SCREEN_UART_LINE_BUFFER_SIZE];
  int32_t counts[ENCODER_COUNT];

  while (ScreenUart_ReadLine(line, sizeof(line)) != 0U)
  {
    if (strcmp(line, "enc") == 0)
    {
      if (App_ReadAllEncoderCounts(counts) == 0U)
      {
        (void)ScreenUart_Printf("enc read failed\r\n");
      }
      else
      {
        App_PrintEncoderCounts("enc", counts);
      }
      continue;
    }

    if (strcmp(line, "zero") == 0)
    {
      if (Encoder_ResetAll() != ENCODER_STATUS_OK)
      {
        (void)ScreenUart_Printf("enc zero failed\r\n");
      }
      else if (Task3_SetCenterZero() == 0U)
      {
        (void)ScreenUart_Printf("center zero failed\r\n");
      }
      else
      {
        (void)ScreenUart_Printf("center zero ok\r\n");
      }
      continue;
    }

    if ((strlen(line) == 1U) &&
        (line[0] >= '1') &&
        (line[0] <= '5'))
    {
      const uint8_t target = (uint8_t)(line[0] - '0');

      if (Task3_IsCenterZeroReady() == 0U)
      {
        (void)ScreenUart_Printf("please zero at center first\r\n");
      }
      else
      {
        if (Task3_StartGoto((task3_point_id_t)target) != 0U)
        {
          (void)ScreenUart_Printf("goto p%u(%s) start\r\n",
                                  (unsigned int)target,
                                  App_PointName(target));
        }
        else
        {
          (void)ScreenUart_Printf("goto p%u(%s) failed\r\n",
                                  (unsigned int)target,
                                  App_PointName(target));
        }
      }
      continue;
    }

    if (strcmp(line, "stat") == 0)
    {
      if (App_ReadAllEncoderCounts(counts) == 0U)
      {
        (void)ScreenUart_Printf("stat read failed\r\n");
      }
      else
      {
        App_PrintEncoderCounts("stat", counts);
        App_PrintPlatformState();
      }
      continue;
    }

    if (strcmp(line, "help") == 0)
    {
      App_PrintHelp();
      continue;
    }

    (void)ScreenUart_Printf("unknown: %s\r\n", line);
  }
}

static void App_MainProcess(void)
{
  const uint32_t now_ms = HAL_GetTick();
  static task3_state_t s_last_task3_state = TASK3_STATE_IDLE;

  App_HandleScreenCommands();

  if ((now_ms - s_last_drive_update_ms) >= 10U)
  {
    s_last_drive_update_ms = now_ms;
    TASK3();
    if (Task3_IsBusy() == 0U)
    {
      (void)AppPlatformDrive_UpdateStateOnly(&s_platform_drive, 0.010f);
    }
  }

  if ((s_last_task3_state == TASK3_STATE_RUNNING) && (Task3_GetState() == TASK3_STATE_FINISHED))
  {
    (void)ScreenUart_Printf("goto p%u(%s) reached\r\n",
                            (unsigned int)Task3_GetCurrentTarget(),
                            App_PointName((uint8_t)Task3_GetCurrentTarget()));
  }

  if ((s_last_task3_state == TASK3_STATE_RUNNING) && (Task3_GetState() == TASK3_STATE_ERROR))
  {
    (void)ScreenUart_Printf("goto p%u(%s) failed\r\n",
                            (unsigned int)Task3_GetCurrentTarget(),
                            App_PointName((uint8_t)Task3_GetCurrentTarget()));
  }

  s_last_task3_state = Task3_GetState();
}
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  (void)CameraUart_Init();
  (void)ScreenUart_Init();
  (void)ScreenUart_SendString("boot uart5 ok\r\n");
  AppPlatformDrive_GetDefaultConfig(&s_platform_config);
  if (AppPlatformDrive_Init(&s_platform_drive, &s_platform_config) != APP_PLATFORM_DRIVE_STATUS_OK)
  {
    (void)ScreenUart_Printf("platform init failed\r\n");
  }
  else
  {
    Task3_Init(&s_platform_drive);
    (void)ScreenUart_Printf("platform init ok\r\n");
    App_PrintHelp();
  }
  s_last_drive_update_ms = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_MainProcess();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart5)
  {
    ScreenUart_RxCpltCallback(huart);
    return;
  }

  VisionApp_RxCpltCallback(huart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  CameraUart_RxEventCallback(huart, Size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  CameraUart_ErrorCallback(huart);
  ScreenUart_ErrorCallback(huart);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ICM42688P_INT1_Pin)
  {
    ICM42688_Int1IrqHandler();
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
