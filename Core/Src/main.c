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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control.h"
#include "telemetry.h"
#include "flight_log.h"
#include "telem_playback.h"
#include "coast_sil.h"
#include "physics.h"
#include <string.h>

/* 1 = embedded CSV playback, 0 = USART2 flight telemetry */
#ifndef TELEM_SOURCE_PLAYBACK
#define TELEM_SOURCE_PLAYBACK  1
#endif
/* 1 = CSV to burnout then closed-loop coast, 0 = full CSV open-loop */
#ifndef TELEM_PLAYBACK_CLOSED_LOOP
#define TELEM_PLAYBACK_CLOSED_LOOP  1
#endif
/* 6 discrete output levels in Servo_SetAngleDeg (0 = off); bench softening only */
#ifndef SERVO_DISCRETE_LEVELS
#define SERVO_DISCRETE_LEVELS  6U
#endif
/* 1 = boot sweep 90 -> 78 -> ... -> 30 -> 90 deg before main loop */
#ifndef SERVO_BOOT_SWEEP
#define SERVO_BOOT_SWEEP  1
#endif
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

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */
#define huart_fc huart2

static control_t g_control;
static stub_telemetry_frame_t g_telem;
static stub_telemetry_frame_t g_telem_work;
static volatile uint8_t g_telem_ready;
#if TELEM_SOURCE_PLAYBACK
typedef enum {
    PLAYBACK_BOOST = 0,
    PLAYBACK_COAST,
    PLAYBACK_DONE
} playback_phase_t;

static uint8_t g_playback_finished;
static playback_phase_t g_playback_phase;
static uint32_t g_burnout_idx;
static coast_sil_t g_coast_sil;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern TIM_HandleTypeDef htim2;
_Static_assert(sizeof(stub_telemetry_frame_t) == 21u, "Telemetry frame must be 21 bytes");

/*
 * Servo mapping (sync with core/physics_utils.py):
 *   u=0 -> 90 deg (brakes closed, safe)
 *   u=1 -> 30 deg (max brake)
 * PWM: SERVO_PWM_MIN @ 30 deg, SERVO_PWM_MAX @ 90 deg.
 */
#ifndef SERVO_PWM_MIN
#define SERVO_PWM_MIN  (1000u)
#endif
#ifndef SERVO_PWM_MAX
#define SERVO_PWM_MAX  (2000u)
#endif
#ifndef SERVO_TIM_CHANNEL
#define SERVO_TIM_CHANNEL  TIM_CHANNEL_3
#endif

/** Round to nearest of SERVO_DISCRETE_LEVELS steps on [OPEN, SAFE]. */
static float Servo_DiscretizeAngleDeg(float angle_deg)
{
#if SERVO_DISCRETE_LEVELS <= 1U
    return angle_deg;
#else
    const float span = SERVO_ANGLE_SAFE_DEG - SERVO_ANGLE_OPEN_DEG;
    const float step = span / (float)(SERVO_DISCRETE_LEVELS - 1U);
    uint32_t idx = (uint32_t)((angle_deg - SERVO_ANGLE_OPEN_DEG) / step + 0.5f);

    if (idx >= SERVO_DISCRETE_LEVELS) {
        idx = SERVO_DISCRETE_LEVELS - 1U;
    }
    return SERVO_ANGLE_OPEN_DEG + (float)idx * step;
#endif
}

static float Servo_SetAngleDeg(float angle_deg)
{
    if (angle_deg != angle_deg) { /* NaN */
        angle_deg = SERVO_ANGLE_SAFE_DEG;
    }
    angle_deg = clamp_f(angle_deg, SERVO_ANGLE_OPEN_DEG, SERVO_ANGLE_SAFE_DEG);
#if SERVO_DISCRETE_LEVELS > 1U
    angle_deg = Servo_DiscretizeAngleDeg(angle_deg);
#endif

    const float span = SERVO_ANGLE_SAFE_DEG - SERVO_ANGLE_OPEN_DEG;
    const float frac = (angle_deg - SERVO_ANGLE_OPEN_DEG) / span;
    uint32_t pulse = (uint32_t)clamp_f(
        (float)SERVO_PWM_MIN + frac * (float)(SERVO_PWM_MAX - SERVO_PWM_MIN),
        (float)SERVO_PWM_MIN,
        (float)SERVO_PWM_MAX);
    __HAL_TIM_SET_COMPARE(&htim2, SERVO_TIM_CHANNEL, pulse);
    return angle_deg;
}

#if !TELEM_SOURCE_PLAYBACK
static void Telemetry_ClearRxErrors(void)
{
    __HAL_UART_FLUSH_DRREGISTER(&huart_fc);
    __HAL_UART_CLEAR_OREFLAG(&huart_fc);
    __HAL_UART_CLEAR_NEFLAG(&huart_fc);
    __HAL_UART_CLEAR_FEFLAG(&huart_fc);
    huart_fc.ErrorCode = HAL_UART_ERROR_NONE;
}

static void Telemetry_DisableRx(void)
{
    (void)HAL_UART_AbortReceive_IT(&huart_fc);
    CLEAR_BIT(huart_fc.Instance->CR1, USART_CR1_RE);
    Telemetry_ClearRxErrors();
}

static void Telemetry_StartRx(stub_telemetry_frame_t *dst)
{
    (void)HAL_UART_AbortReceive_IT(&huart_fc);
    Telemetry_ClearRxErrors();
    SET_BIT(huart_fc.Instance->CR1, USART_CR1_RE);
    if (HAL_UART_Receive_IT(&huart_fc, (uint8_t *)dst, TELEMETRY_FRAME_SIZE) != HAL_OK) {
        Error_Handler();
    }
}

static void OnTelemetryFrameReceived(void)
{
    g_telem_ready = 1;
}
#else
static void Playback_Finish(void)
{
    g_playback_finished = 1U;
    g_playback_phase = PLAYBACK_DONE;
    if (flight_log_flush() != 0) {
        BSP_LED_On(LED_RED);
    } else {
        BSP_LED_On(LED_GREEN);
    }
}

static void PlantState_ToTelem(const flight_state_t *st, stub_telemetry_frame_t *frame)
{
    frame->state = (uint8_t)TELEM_STATE_FREE_FLIGHT;
    frame->timeMs = (uint32_t)(st->time_s * 1000.0f);
    frame->posZ = st->position_z;
    frame->velZ = st->velocity_z;
    frame->gpsSpeed = st->velocity_lateral;
    frame->gpsCourse = 0.0f;
}

static void Telemetry_PlaybackFeed(void)
{
    if (g_playback_finished) {
        return;
    }

#if TELEM_PLAYBACK_CLOSED_LOOP
    if (g_playback_phase == PLAYBACK_BOOST) {
        if (telem_playback_index() > g_burnout_idx) {
            return;
        }
        if (telem_playback_next(&g_telem) == 0U) {
            Playback_Finish();
            return;
        }
        g_telem_ready = 1;
        return;
    }

    if (g_playback_phase == PLAYBACK_COAST) {
        flight_state_t st;
        control_output_t cmd;
        if (coast_sil_step(&g_coast_sil, &g_control, &st, &cmd) == 0U) {
            Playback_Finish();
            return;
        }
        PlantState_ToTelem(&st, &g_telem_work);
        cmd.servo_angle_deg = Servo_SetAngleDeg(
            cmd.active ? cmd.servo_angle_deg : SERVO_ANGLE_SAFE_DEG);
        (void)flight_log_append(&g_telem_work, &st, &cmd);
        if (flight_log_is_full()) {
            BSP_LED_On(LED_RED);
        }
        return;
    }
#else
    if (telem_playback_next(&g_telem) == 0U) {
        Playback_Finish();
        return;
    }
    g_telem_ready = 1;
#endif
}
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
#if !TELEM_SOURCE_PLAYBACK
  Telemetry_DisableRx();
#endif
  HAL_TIM_PWM_Start(&htim2, SERVO_TIM_CHANNEL);

  control_init(&g_control);
  /* Model params: model_params_default() in physics.c (mass 12.57 kg, Cd 0.42,
   * BODY_CROSS_SECTION_M2). Override here only for experiments. */
  g_control.pd.params.target_apogee = 2200.0f;
  g_control.pd.params.kp = 0.08f;
  g_control.pd.params.kd = 0.0005f;
  g_control.pd.params.error_deadband = 10.0f;
  g_control.predictor.solver = SOLVER_EULER;
  g_control.predictor.dt = 0.01f;

  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

#if TELEM_SOURCE_PLAYBACK
  if (flight_log_begin_staging() != 0) {
    Error_Handler();
  }
  telem_playback_reset();
  g_playback_finished = 0U;
  g_playback_phase = PLAYBACK_BOOST;
  g_burnout_idx = telem_playback_burnout_index();
#if TELEM_PLAYBACK_CLOSED_LOOP
  if (g_burnout_idx >= TELEM_PLAYBACK_FRAME_COUNT) {
    Error_Handler();
  }
#endif
#else
  if (flight_log_boot() != 0) {
    Error_Handler();
  }
  Telemetry_StartRx(&g_telem);
#endif
  /* USER CODE END 2 */

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#if SERVO_BOOT_SWEEP
  Servo_SetAngleDeg(SERVO_ANGLE_SAFE_DEG);
  BSP_LED_Toggle(LED_YELLOW);
  HAL_Delay(1000);
  Servo_SetAngleDeg(78.0f);
  BSP_LED_Toggle(LED_YELLOW);
  HAL_Delay(1000);
  Servo_SetAngleDeg(66.0f);
  BSP_LED_Toggle(LED_YELLOW);
  HAL_Delay(1000);
  Servo_SetAngleDeg(54.0f);
  BSP_LED_Toggle(LED_YELLOW);
  HAL_Delay(1000);
  Servo_SetAngleDeg(42.0f);
  BSP_LED_Toggle(LED_YELLOW);
  HAL_Delay(1000);
  Servo_SetAngleDeg(SERVO_ANGLE_OPEN_DEG);
  BSP_LED_Toggle(LED_YELLOW);
  HAL_Delay(1000);
  Servo_SetAngleDeg(SERVO_ANGLE_SAFE_DEG);
#else
  Servo_SetAngleDeg(SERVO_ANGLE_SAFE_DEG);
#endif

  BSP_LED_On(LED_YELLOW);

  while (1)
  {
#if TELEM_SOURCE_PLAYBACK
    Telemetry_PlaybackFeed();
#endif

    if (g_telem_ready) {
        g_telem_ready = 0;
        memcpy(&g_telem_work, &g_telem, TELEMETRY_FRAME_SIZE);
#if !TELEM_SOURCE_PLAYBACK
        BSP_LED_Toggle(LED_GREEN);
        Telemetry_StartRx(&g_telem);
#endif

        flight_state_t st;
        control_output_t cmd = telemetry_run_control(&g_control, &g_telem_work, &st);
        cmd.servo_angle_deg = Servo_SetAngleDeg(
            cmd.active ? cmd.servo_angle_deg : SERVO_ANGLE_SAFE_DEG);

        (void)flight_log_append(&g_telem_work, &st, &cmd);
        if (flight_log_is_full()) {
            BSP_LED_On(LED_RED);
        }

#if TELEM_SOURCE_PLAYBACK && TELEM_PLAYBACK_CLOSED_LOOP
        if (g_playback_phase == PLAYBACK_BOOST &&
            telem_playback_index() == (g_burnout_idx + 1U)) {
            coast_sil_init(&g_coast_sil, &st, cmd.control_u, COAST_SIL_MAX_STEPS_DEFAULT);
            control_arm(&g_control);
            g_playback_phase = PLAYBACK_COAST;
        }
#endif
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 18;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#if !TELEM_SOURCE_PLAYBACK
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart_fc) {
        OnTelemetryFrameReceived();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart_fc) {
        Telemetry_StartRx(&g_telem);
    }
}
#endif

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
