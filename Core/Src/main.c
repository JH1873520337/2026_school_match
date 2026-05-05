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
#include "icm42688p.h"
#include "screen_uart.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
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
static app_platform_drive_t s_platform_drive;
static app_platform_drive_config_t s_platform_config;
static uint32_t s_last_drive_update_ms = 0U;
static uint32_t s_last_status_print_ms = 0U;
static uint8_t s_all_motor_test_enable = 0U;
static int16_t s_all_motor_test_pwm = 0;
static uint8_t s_position_loop_enable = 0U;
static float s_position_target_x_m = 0.0f;
static float s_position_target_y_m = 0.0f;
static float s_position_kp = 1.0f;
static float s_position_deadband_m = 0.005f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void App_MainProcess(void);
static void App_HandleScreenCommands(void);
static void App_PrintHelp(void);
static void App_PrintPlatformStatus(void);
static uint8_t App_ParseInt16Arg(const char *line, const char *prefix, int16_t *value);
static uint8_t App_ParseFloatArg(const char *line, const char *prefix, float *value);
static uint8_t App_ParseTargetXY(const char *line, float *target_x_m, float *target_y_m);
static void App_UpdatePositionLoop(void);
static float App_ToUserFrameX(float solver_x_m);
static float App_ToUserFrameY(float solver_y_m);
static float App_ToUserFrameVx(float solver_vx_mps);
static float App_ToUserFrameVy(float solver_vy_mps);
static float App_ToUserFrameCmdX(float command_vx_mps);
static float App_ToUserFrameCmdY(float command_vy_mps);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void App_PrintHelp(void)
{
  (void)ScreenUart_Printf("cmd: cal, stop, fwd, back, left, right, all=PWM, pos=x,y, pkp=, ptol=, stat, help\r\n");
}

static float App_ToUserFrameX(float solver_x_m)
{
  return -solver_x_m;
}

static float App_ToUserFrameY(float solver_y_m)
{
  return -solver_y_m;
}

static float App_ToUserFrameVx(float solver_vx_mps)
{
  return -solver_vx_mps;
}

static float App_ToUserFrameVy(float solver_vy_mps)
{
  return -solver_vy_mps;
}

static float App_ToUserFrameCmdX(float command_vx_mps)
{
  return -command_vx_mps;
}

static float App_ToUserFrameCmdY(float command_vy_mps)
{
  return -command_vy_mps;
}

static uint8_t App_ParseInt16Arg(const char *line, const char *prefix, int16_t *value)
{
  long parsed;
  char *end_ptr;

  if ((line == NULL) || (prefix == NULL) || (value == NULL))
  {
    return 0U;
  }

  if (strncmp(line, prefix, strlen(prefix)) != 0)
  {
    return 0U;
  }

  parsed = strtol(line + strlen(prefix), &end_ptr, 10);
  if ((end_ptr == (line + strlen(prefix))) || (*end_ptr != '\0') ||
      (parsed < -1000L) || (parsed > 1000L))
  {
    return 0U;
  }

  *value = (int16_t)parsed;
  return 1U;
}

static uint8_t App_ParseFloatArg(const char *line, const char *prefix, float *value)
{
  char *end_ptr;

  if ((line == NULL) || (prefix == NULL) || (value == NULL))
  {
    return 0U;
  }

  if (strncmp(line, prefix, strlen(prefix)) != 0)
  {
    return 0U;
  }

  *value = strtof(line + strlen(prefix), &end_ptr);
  if ((end_ptr == (line + strlen(prefix))) || (*end_ptr != '\0'))
  {
    return 0U;
  }

  return 1U;
}

static uint8_t App_ParseTargetXY(const char *line, float *target_x_m, float *target_y_m)
{
  char *comma_ptr;
  char *end_ptr;

  if ((line == NULL) || (target_x_m == NULL) || (target_y_m == NULL))
  {
    return 0U;
  }

  if (strncmp(line, "pos=", 4U) != 0)
  {
    return 0U;
  }

  comma_ptr = strchr(line + 4, ',');
  if (comma_ptr == NULL)
  {
    return 0U;
  }

  *target_x_m = strtof(line + 4, &end_ptr);
  if (end_ptr != comma_ptr)
  {
    return 0U;
  }

  *target_y_m = strtof(comma_ptr + 1, &end_ptr);
  if (*end_ptr != '\0')
  {
    return 0U;
  }

  return 1U;
}

static void App_PrintPlatformStatus(void)
{
  const rope_platform_solver_t *solver = AppPlatformDrive_GetSolver(&s_platform_drive);
  float user_x_m;
  float user_y_m;
  float user_vx_mps;
  float user_vy_mps;
  float user_cmd_x_mps;
  float user_cmd_y_mps;

  if (solver == NULL)
  {
    (void)ScreenUart_Printf("solver=null\r\n");
    return;
  }

  user_x_m = App_ToUserFrameX(solver->position_x_m);
  user_y_m = App_ToUserFrameY(solver->position_y_m);
  user_vx_mps = App_ToUserFrameVx(solver->velocity_x_mps);
  user_vy_mps = App_ToUserFrameVy(solver->velocity_y_mps);
  user_cmd_x_mps = App_ToUserFrameCmdX(s_platform_drive.command_vx_mps);
  user_cmd_y_mps = App_ToUserFrameCmdY(s_platform_drive.command_vy_mps);

  (void)ScreenUart_Printf("pos=(%.3f,%.3f) vel=(%.3f,%.3f) rms=%.5f cmd=(%.3f,%.3f) tgt=(%.3f,%.3f) ploop=%u\r\n",
                          (double)user_x_m,
                          (double)user_y_m,
                          (double)user_vx_mps,
                          (double)user_vy_mps,
                          (double)solver->residual_rms_m,
                          (double)user_cmd_x_mps,
                          (double)user_cmd_y_mps,
                          (double)s_position_target_x_m,
                          (double)s_position_target_y_m,
                          (unsigned int)s_position_loop_enable);
}

static void App_UpdatePositionLoop(void)
{
  const rope_platform_solver_t *solver = AppPlatformDrive_GetSolver(&s_platform_drive);
  float user_x_m;
  float user_y_m;
  float error_x_m;
  float error_y_m;
  float cmd_vx_mps;
  float cmd_vy_mps;

  if ((s_position_loop_enable == 0U) || (solver == NULL) || (solver->state_valid == 0U))
  {
    return;
  }

  user_x_m = App_ToUserFrameX(solver->position_x_m);
  user_y_m = App_ToUserFrameY(solver->position_y_m);
  error_x_m = s_position_target_x_m - user_x_m;
  error_y_m = s_position_target_y_m - user_y_m;

  if ((fabsf(error_x_m) <= s_position_deadband_m) &&
      (fabsf(error_y_m) <= s_position_deadband_m))
  {
    (void)AppPlatformDrive_Stop(&s_platform_drive);
    s_position_loop_enable = 0U;
    (void)ScreenUart_Printf("pos target reached\r\n");
    return;
  }

  cmd_vx_mps = s_position_kp * error_x_m;
  cmd_vy_mps = s_position_kp * error_y_m;
  (void)AppPlatformDrive_SetCommandVelocity(&s_platform_drive, cmd_vx_mps, cmd_vy_mps);
}

static void App_HandleScreenCommands(void)
{
  char line[SCREEN_UART_LINE_BUFFER_SIZE];

  while (ScreenUart_ReadLine(line, sizeof(line)) != 0U)
  {
    if (strcmp(line, "cal") == 0)
    {
      if (AppPlatformDrive_SetCurrentPoseAsOrigin(&s_platform_drive, 0.0f, 0.0f) ==
          APP_PLATFORM_DRIVE_STATUS_OK)
      {
        (void)ScreenUart_Printf("cal ok\r\n");
      }
      else
      {
        (void)ScreenUart_Printf("cal failed\r\n");
      }
      continue;
    }

    if (strcmp(line, "stop") == 0)
    {
      s_all_motor_test_enable = 0U;
      s_all_motor_test_pwm = 0;
      s_position_loop_enable = 0U;
      (void)AppPlatformDrive_Stop(&s_platform_drive);
      (void)ScreenUart_Printf("stop ok\r\n");
      continue;
    }

    if (strcmp(line, "fwd") == 0)
    {
      s_all_motor_test_enable = 0U;
      s_position_loop_enable = 0U;
      (void)AppPlatformDrive_MoveForward(&s_platform_drive, 0.05f);
      (void)ScreenUart_Printf("move fwd\r\n");
      continue;
    }

    if (strcmp(line, "back") == 0)
    {
      s_all_motor_test_enable = 0U;
      s_position_loop_enable = 0U;
      (void)AppPlatformDrive_MoveBackward(&s_platform_drive, 0.05f);
      (void)ScreenUart_Printf("move back\r\n");
      continue;
    }

    if (strcmp(line, "left") == 0)
    {
      s_all_motor_test_enable = 0U;
      s_position_loop_enable = 0U;
      (void)AppPlatformDrive_MoveLeft(&s_platform_drive, 0.05f);
      (void)ScreenUart_Printf("move left\r\n");
      continue;
    }

    if (strcmp(line, "right") == 0)
    {
      s_all_motor_test_enable = 0U;
      s_position_loop_enable = 0U;
      (void)AppPlatformDrive_MoveRight(&s_platform_drive, 0.05f);
      (void)ScreenUart_Printf("move right\r\n");
      continue;
    }

    if (App_ParseInt16Arg(line, "all=", &s_all_motor_test_pwm) != 0U)
    {
      s_all_motor_test_enable = 1U;
      s_position_loop_enable = 0U;
      (void)AppPlatformDrive_Stop(&s_platform_drive);
      (void)ScreenUart_Printf("all motor pwm=%d\r\n", (int)s_all_motor_test_pwm);
      continue;
    }

    if (App_ParseTargetXY(line, &s_position_target_x_m, &s_position_target_y_m) != 0U)
    {
      const rope_platform_solver_t *solver = AppPlatformDrive_GetSolver(&s_platform_drive);

      if ((solver == NULL) || (solver->state_valid == 0U))
      {
        (void)ScreenUart_Printf("pos loop needs cal first\r\n");
      }
      else
      {
        s_all_motor_test_enable = 0U;
        s_position_loop_enable = 1U;
        (void)ScreenUart_Printf("pos target=(%.3f,%.3f)\r\n",
                                (double)s_position_target_x_m,
                                (double)s_position_target_y_m);
      }
      continue;
    }

    if (App_ParseFloatArg(line, "pkp=", &s_position_kp) != 0U)
    {
      (void)ScreenUart_Printf("pos kp=%.3f\r\n", (double)s_position_kp);
      continue;
    }

    if (App_ParseFloatArg(line, "ptol=", &s_position_deadband_m) != 0U)
    {
      (void)ScreenUart_Printf("pos tol=%.3f\r\n", (double)s_position_deadband_m);
      continue;
    }

    if (strcmp(line, "stat") == 0)
    {
      App_PrintPlatformStatus();
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

  App_HandleScreenCommands();

  if ((now_ms - s_last_drive_update_ms) >= 10U)
  {
    s_last_drive_update_ms = now_ms;
    if (s_all_motor_test_enable != 0U)
    {
      (void)TB6612_SetMotorSpeed(TB6612_MOTOR_1, s_all_motor_test_pwm);
      (void)TB6612_SetMotorSpeed(TB6612_MOTOR_2, s_all_motor_test_pwm);
      (void)TB6612_SetMotorSpeed(TB6612_MOTOR_3, s_all_motor_test_pwm);
      (void)TB6612_SetMotorSpeed(TB6612_MOTOR_4, s_all_motor_test_pwm);
      (void)AppPlatformDrive_UpdateStateOnly(&s_platform_drive, 0.010f);
    }
    else
    {
      App_UpdatePositionLoop();
      (void)AppPlatformDrive_Update(&s_platform_drive, 0.010f);
    }
  }

  if ((now_ms - s_last_status_print_ms) >= 200U)
  {
    s_last_status_print_ms = now_ms;
    App_PrintPlatformStatus();
  }
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
  AppPlatformDrive_GetDefaultConfig(&s_platform_config);
  s_platform_config.max_platform_speed_mps = 0.05f;
  if (AppPlatformDrive_Init(&s_platform_drive, &s_platform_config) != APP_PLATFORM_DRIVE_STATUS_OK)
  {
    (void)ScreenUart_Printf("platform init failed\r\n");
  }
  else
  {
    (void)ScreenUart_Printf("platform init ok\r\n");
    App_PrintHelp();
  }
  s_last_drive_update_ms = HAL_GetTick();
  s_last_status_print_ms = s_last_drive_update_ms;
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
  /* User can add his own implementation to report the HAL error return state */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
