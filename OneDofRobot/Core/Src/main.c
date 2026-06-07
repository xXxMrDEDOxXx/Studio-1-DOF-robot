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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>            /* snprintf — USART1 stream → MATLAB              */
#include "base_system.h"      /* Register map, SystemMode_t, modbus_registers[] */
#include "cascade_control.h"
#include "trajectory.h"
#include "encoder.h"
#include "dashboard.h"        /* Manual Mode Dashboard (Modbus ↔ PC)           */
#include "auto_mission.h"     /* Pick & Place Auto Mission (MODE_AUTO)          */
#include "homing.h"           /* Force-Homing state machine (MODE_HOMING)       */
#include "gripper.h"          /* Gripper: PC4=arm PC10=jaw + reed feedback      */
#include "can_gripper.h"      /* CAN relay node backend for gripper outputs     */
#include "test_mode.h"        /* Performance & Precision test (MODE_TEST)       */
#include "joystick.h"         /* Funduino Joystick Shield (MODE_MANUAL only)    */
#include "volt_test.h"        /* Open-loop voltage excitation (Lab 1 ID)        */
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
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int reed_up = 0;
int reed_down = 0;
int reed_close_open = 0;
/* ── Internal Temperature Sensor ─────────────────────────────────────────
 * ดูค่าได้ใน Live Expressions: chip_temp_C
 * G474 internal sensor: ADC1 CH16, cal @ VDDA=3.0V
 * ───────────────────────────────────────────────────────────────────────*/
volatile float chip_temp_C = 0.0f;


/* ── Live Expressions: operating mode & emergency ────────────────────────
 * ดูค่าใน Live Expressions:
 *   debug_op_mode  → 0=Homing 1=Auto 2=Manual 3=Test
 *   debug_selector → 0=Manual 1=Auto  (physical switch)
 *   debug_estop    → 0=OK  1=Emergency active
 * ───────────────────────────────────────────────────────────────────────*/
volatile uint8_t debug_op_mode  = 0;
volatile uint8_t debug_selector = 0;
volatile uint8_t debug_estop    = 0;

Encoder_t henc2;

/* SystemMode_t typedef อยู่ใน base_system.h
 * ประกาศ definition ที่นี่ — base_system.h มี extern declaration แล้ว */
volatile SelectorMode_t selector_mode      = SELECTOR_MANUAL; /* physical switch */
volatile SystemMode_t   current_system_mode = MODE_HOMING;    /* actual mode     */

uint8_t rx_buffer[128];

DMA_HandleTypeDef hdma_usart1_tx;   /* USART1 TX DMA (legacy, ไม่ใช้แล้ว — ย้ายไป USART2) */

/* ── Analysis sub-mode (แก้ผ่าน CubeIDE debugger / Live Expressions เท่านั้น) ──
 *  0 = DASHBOARD (default) → MANUAL ใช้ Modbus ปกติ (dashboard.py)
 *  1 = ANALYSIS           → MANUAL stream CSV ออก USART2 แทน Modbus (เก็บค่า sys ID)
 *  ทำงานเฉพาะ MODE_MANUAL — โหมดอื่น (AUTO/HOMING) ไม่สน                            */
volatile uint8_t analysis_mode = 0;

/* ── สั่ง move ใน analysis mode ผ่าน Live Expressions ────────────────────────
 *  เขียน cmd_deg = องศาเป้าหมาย → หุ่นวิ่งไป (Septic move + cascade AUTO gains)
 *  ใช้สำหรับเก็บ step/move response ทำ sys ID                                   */
volatile float cmd_deg = 0.0f;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
//Update_Telemetry_To_PC(void);
static void Stream_Update(void);   /* USART1 stream → MATLAB (นิยามใน USER CODE 4) */
static void Stream_DMA_Init(void); /* USART1 TX DMA init */
static void Analysis_Control(void); /* analysis mode: สั่ง move ตาม cmd_deg */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Internal Temperature Sensor — ADC1 CH16 + DMA1 CH1 (bare-metal)
 *
 *  Mode: Continuous conversion + DMA Circular
 *        ADC แปลงตลอดเวลา, DMA เขียนลง adc_dma_buf อัตโนมัติ
 *        TempSensor_Update() ไม่ block — แค่อ่าน buffer แล้วคำนวณ
 *
 *  Clock:    HCLK/4 = 170/4 = 42.5 MHz  (≤ 52 MHz ✓)
 *  Sampling: 640.5 cycles ≈ 15 µs  (datasheet req ≥ 5 µs ✓)
 *  DMA:      DMA1_Channel1, DMAMUX request = 5 (ADC1)
 *  Cal addr: TS_CAL1@30°C = 0x1FFF75A8, TS_CAL2@130°C = 0x1FFF75CA
 * ═══════════════════════════════════════════════════════════════════════════ */
static volatile uint16_t adc_dma_buf = 0;  /* DMA เขียนค่า raw ลงตรงนี้ */

static void TempSensor_Init(void)
{
    /* ── 1. เปิด Clock: ADC12, DMA1, DMAMUX1 ─────────────────────────────── */
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;

    /* ── 2. ADC clock = HCLK/4 (CKMODE=11) ──────────────────────────────── */
    ADC12_COMMON->CCR = (3UL << ADC_CCR_CKMODE_Pos);

    /* ── 3. ออกจาก Deep Power-Down + เปิด Voltage Regulator ─────────────── */
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |=  ADC_CR_ADVREGEN;
    for (volatile uint32_t i = 0; i < 5000; i++);      /* ≥ 20 µs @ 170 MHz */

    /* ── 4. เปิด Temperature Sensor + รอ stable ─────────────────────────── */
    /* ADC_CCR_TSEN = bit 23 (ไม่มีใน CMSIS header เพราะ HAL_ADC ไม่ได้ enable) */
    ADC12_COMMON->CCR |= (1UL << 23U);
    for (volatile uint32_t i = 0; i < 30000; i++);     /* ≥ 120 µs stabilize */

    /* ── 5. Calibration (single-ended) ──────────────────────────────────── */
    ADC1->CR &= ~ADC_CR_ADCALDIF;
    ADC1->CR |=  ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL);

    /* ── 6. Config DMA1_Channel1 → ADC1 (DMAMUX request = 5) ─────────────
     *   Peripheral → Memory, 16-bit, Circular, ไม่ increment memory
     * ─────────────────────────────────────────────────────────────────────*/
    DMAMUX1_Channel0->CCR = 5UL;                        /* ADC1 request */

    DMA1_Channel1->CCR   = 0;
    DMA1_Channel1->CNDTR = 1;                           /* 1 element */
    DMA1_Channel1->CPAR  = (uint32_t)&ADC1->DR;        /* source: ADC data reg */
    DMA1_Channel1->CMAR  = (uint32_t)&adc_dma_buf;     /* dest: our buffer */
    DMA1_Channel1->CCR   = DMA_CCR_CIRC                 /* circular mode */
                          | DMA_CCR_PSIZE_0              /* peripheral 16-bit */
                          | DMA_CCR_MSIZE_0              /* memory 16-bit */
                          | DMA_CCR_EN;                  /* enable DMA */

    /* ── 7. Enable ADC ───────────────────────────────────────────────────── */
    ADC1->ISR |= ADC_ISR_ADRDY;
    ADC1->CR  |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY));

    /* ── 8. Config ADC: Continuous + DMA Circular ───────────────────────── */
    ADC1->CFGR  = ADC_CFGR_CONT      /* continuous conversion */
                | ADC_CFGR_DMAEN     /* DMA enable */
                | ADC_CFGR_DMACFG;   /* DMA circular (ไม่หยุดหลัง 1 conv) */
    ADC1->CFGR2 = 0;

    /* ── 9. Sequence: 1 conv, Channel 16 (VSENSE), sampling 640.5 cyc ───── */
    ADC1->SQR1  = (16UL << ADC_SQR1_SQ1_Pos) | (0UL << ADC_SQR1_L_Pos);
    ADC1->SMPR2 = (7UL << ADC_SMPR2_SMP16_Pos);

    /* ── 10. Start — ADC วิ่งต่อเนื่อง, DMA อัปเดต adc_dma_buf เองตลอด ── */
    ADC1->CR |= ADC_CR_ADSTART;
}

static void TempSensor_Update(void)
{
    /* ไม่ block! — แค่อ่าน buffer ที่ DMA อัปเดตไว้ แล้วคำนวณ */
    uint16_t raw = adc_dma_buf;

    int32_t cal1 = (int32_t)(*(uint16_t*)0x1FFF75A8UL);   /* raw @ 30°C  */
    int32_t cal2 = (int32_t)(*(uint16_t*)0x1FFF75CAUL);   /* raw @ 130°C */

    chip_temp_C = (float)(130 - 30) * (float)((int32_t)raw - cal1)
                / (float)(cal2 - cal1) + 30.0f;
}

/* ── Modbus RTU Echo Filter (Content-based) ─────────────────────────────────
 *
 *  VCP loopback ทำให้ STM32 TX response วนกลับเข้า RX ทุกครั้งที่ส่ง
 *  ใช้ content-based filter แทน timing guard เพื่อความถูกต้องตามมาตรฐาน
 *
 *  ┌──────┬────────────────────────────────────────────────────────────────┐
 *  │ FC   │ วิธีกรอง echo                                                  │
 *  ├──────┼────────────────────────────────────────────────────────────────┤
 *  │ 0x03 │ Size-based: response มีขนาด ≠ 8  → drop                       │
 *  │ 0x06 │ Content-based: เปรียบ frame กับ modbus_echo_buf (one-shot)     │
 *  │ 0x10 │ Size-based: response มีขนาด == 8 → drop                       │
 *  └──────┴────────────────────────────────────────────────────────────────┘
 * ─────────────────────────────────────────────────────────────────────────*/
#include <string.h>   /* memcmp(), memcpy() */
static uint8_t frame_copy[128];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART2) return;

    if (Size < 8 || rx_buffer[0] != SLAVE_ID) {
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_buffer, sizeof(rx_buffer));
        return;
    }

    uint8_t fc = rx_buffer[1];

    /* ── Echo filter ── */
    uint8_t is_echo = 0;

    if (fc == 0x03 && Size != 8) {
        /* FC03 response: ขนาด 5+2N ≠ 8 → echo ของ response */
        is_echo = 1;
    }
    else if (fc == 0x10 && Size == 8) {
        /* FC16 response: ขนาดคงที่ 8 bytes → echo ของ response */
        is_echo = 1;
    }
    else if (fc == 0x06 && Size == 8) {
        /* FC06: request กับ response มีขนาดเท่ากัน (8 bytes)
         * ใช้ content + time window (50 ms) — ถ้า echo ไม่มาใน 50 ms
         * echo_valid expire แล้ว → frame นี้คือ request จริง ไม่ใช่ echo */
        if (modbus_echo_valid &&
            (HAL_GetTick() - modbus_echo_time) < 50U &&
            memcmp(rx_buffer, modbus_echo_buf, 8) == 0) {
            modbus_echo_valid = 0;   /* บริโภค echo แล้ว — request ถัดไปผ่านได้ */
            is_echo = 1;
        }
    }

    if (is_echo) {
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_buffer, sizeof(rx_buffer));
        return;
    }

    /* ── Process real request ── */
    uint16_t len = (Size > sizeof(frame_copy)) ? sizeof(frame_copy) : Size;
    memcpy(frame_copy, rx_buffer, len);

    HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_buffer, sizeof(rx_buffer));
    Modbus_Parse_Frame(frame_copy, len);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        /* clear error flags ทั้งหมด: ORE, FE, NE, PE */
        __HAL_UART_CLEAR_FLAG(&huart2,
            UART_CLEAR_OREF | UART_CLEAR_NEF |
            UART_CLEAR_FEF  | UART_CLEAR_PEF);
        HAL_UART_AbortReceive(&huart2);  /* reset HAL RX state */
        /* echo ที่กำลังรอรับถูก abort → clear echo_valid ป้องกัน
         * real request ถัดไปถูก drop ว่าเป็น echo ผิดๆ */
        modbus_echo_valid = 0;
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_buffer, sizeof(rx_buffer));
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* ── Heartbeat/Modbus ขึ้นก่อนเพื่อนใน USER CODE 2 → หลัง NVIC reset base system
   *    เห็น "alive" ไวสุด (USART2 init แล้วใน auto region ด้านบน) ──────────────*/
  Heartbeat_Init();
  HAL_UART_Transmit(&huart1, (uint8_t*)"UART1 OK\r\n", 10, 100);
  HAL_UARTEx_EnableFifoMode(&huart2);  /* เปิด RX FIFO 8 byte — margin กัน overrun */
  __HAL_UART_CLEAR_OREFLAG(&huart2);   /* เคลียร์ error flags ก่อน */
  HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_buffer, sizeof(rx_buffer));

  TempSensor_Init();        /* ADC1 CH16 — อ่าน chip_temp_C ใน Live Expressions */
  henc2.htim = &htim2;

  Encoder_Init(&henc2, &htim2);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  Cascade_Control_Init();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_MOE_ENABLE(&htim1);

  Dashboard_Init();
  AutoMission_Init();
  Homing_Init();
  CanGripper_Init();
  Gripper_Init();
  TestMode_Init();
  Joystick_Init();
  VoltTest_Init();
  Stream_DMA_Init();        /* USART1 TX DMA — stream 1kHz → MATLAB */

  current_system_mode            = MODE_HOMING;
  modbus_registers[REG_SYS_MODE] = MODE_HOMING;
  modbus_registers[REG_BS_TASK]  = TASK_HOMING;
  Homing_Start();
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      Heartbeat_Update();
      CanGripper_Update();
      Stream_Update();          /* USART1 → MATLAB (~500 Hz) */

      /* ── Selector Switch — อ่านตลอดเวลา, priority สูงสุด ──────────────
       *  TIM6 ISR จะบังคับ mode ตาม selector_mode ทุก tick
       * ─────────────────────────────────────────────────────────────────── */
      selector_mode = (HAL_GPIO_ReadPin(Manual_mode_GPIO_Port, Manual_mode_Pin)
                       == GPIO_PIN_RESET)
                      ? SELECTOR_MANUAL : SELECTOR_AUTO;

      /* อ่านอุณหภูมิ chip ทุก 500 ms — ดูใน Live Expressions: chip_temp_C */
      static uint32_t last_temp_ms = 0;
      uint32_t now_ms = HAL_GetTick();
      if (now_ms - last_temp_ms >= 500U) {
          last_temp_ms = now_ms;
          TempSensor_Update();
      }
      reed_up =  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7);
      reed_down = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9);
      reed_close_open = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 34;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 7;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 5;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 9;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 169;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
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
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart2) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Motor_Dir_GPIO_Port, Motor_Dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, gripper_u_d_Pin|gripper_o_c_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : E_stop_Pin */
  GPIO_InitStruct.Pin = E_stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(E_stop_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor_Dir_Pin gripper_u_d_Pin gripper_o_c_Pin */
  GPIO_InitStruct.Pin = Motor_Dir_Pin|gripper_u_d_Pin|gripper_o_c_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Homing_signal_Pin A_Pin B_Pin C_Pin */
  GPIO_InitStruct.Pin = Homing_signal_Pin|A_Pin|B_Pin|C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Manual_mode_Pin Auto_Mode_Pin reed_open_close_Pin */
  GPIO_InitStruct.Pin = Manual_mode_Pin|Auto_Mode_Pin|reed_open_close_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : K_Pin D_Pin */
  GPIO_InitStruct.Pin = K_Pin|D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : reed_up_Pin */
  GPIO_InitStruct.Pin = reed_up_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(reed_up_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : reed_down_Pin */
  GPIO_InitStruct.Pin = reed_down_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(reed_down_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ── USART1 stream → MATLAB (PB6=TX, DMA) ───────────────────────────────────
 *  ส่ง CSV 8 ช่อง int×1000 ที่ 1 kHz ผ่าน DMA (non-blocking — ไม่กิน main loop)
 *  คอลัมน์: q_ref,qd_ref,qdd_ref,jerk_ref,q_out,qd_out,qdd_out,V  (MATLAB หาร 1000)
 *  DMA1_Channel2 + DMA_REQUEST_USART1_TX (ADC ใช้ Ch1 อยู่ → ใช้ Ch2)            */
extern volatile float ref_j;   /* jerk-ref (auto_mission.c) */

/* ── Analysis mode controller: สั่ง move ตาม cmd_deg (Live Expressions) ──────
 *  เขียน cmd_deg → Septic move ไปองศานั้น + cascade (AUTO gains). เก็บ response sys ID */
/* =================================================================
 * กำหนด Trajectory ที่ต้องการจะทดสอบ (สำหรับ Lab 3)
 * 1 = Trapezoidal (ความเร่งเป็นขั้นบันได, Jerk กระชาก)
 * 2 = S-Curve     (ความเร่งโค้ง, Jerk คงที่)
 * 3 = Quintic     (S-Curve แบบฟิกเวลา 5th-order)
 * 4 = Septic      (นุ่มที่สุด Jerk ไม่กระชากเลย 7th-order)
 * ================================================================= */
#define TRAJ_SELECT 4   // <--- เปลี่ยนตัวเลขตรงนี้เพื่อสลับโพรไฟล์

static void Analysis_Control(void)
{
    static float   an_prev = 0.0f;
    static uint8_t an_init = 0;

    /* ตัวแปรรับค่าจากโพรไฟล์ */
    float rq = 0.0f, rqd = 0.0f, rqdd = 0.0f, rj = 0.0f;

/* --------------------------------------------------
 * 1. Trapezoidal Profile
 * -------------------------------------------------- */
#if TRAJ_SELECT == 1
    static Trapz_Profile_t prof;
    if (!an_init) { Trapz_Init(&prof); an_prev = cmd_deg; an_init = 1; }

    if (cmd_deg != an_prev) {
        Trapz_MoveTo(&prof, q_out, cmd_deg * (3.14159265f / 180.0f));
        an_prev = cmd_deg;
    }
    Trapz_Update(&prof, &rq, &rqd, &rqdd, &rj);

/* --------------------------------------------------
 * 2. S-Curve Profile
 * -------------------------------------------------- */
#elif TRAJ_SELECT == 2
    static SCurve_Profile_t prof;
    if (!an_init) { SCurve_Init(&prof); an_prev = cmd_deg; an_init = 1; }

    if (cmd_deg != an_prev) {
        SCurve_MoveTo(&prof, q_out, cmd_deg * (3.14159265f / 180.0f));
        an_prev = cmd_deg;
    }
    SCurve_Update(&prof, &rq, &rqd, &rqdd, &rj);

/* --------------------------------------------------
 * 3. Quintic Profile
 * -------------------------------------------------- */
#elif TRAJ_SELECT == 3
    static Quintic_Profile_t prof;
    if (!an_init) { Quintic_Init(&prof); an_prev = cmd_deg; an_init = 1; }

    if (cmd_deg != an_prev) {
        Quintic_MoveTo(&prof, q_out, cmd_deg * (3.14159265f / 180.0f), TRAJ_MOVE_TIME);
        an_prev = cmd_deg;
    }
    Quintic_Update(&prof, &rq, &rqd, &rqdd, &rj);

/* --------------------------------------------------
 * 4. Septic Profile
 * -------------------------------------------------- */
#elif TRAJ_SELECT == 4
    static Septic_Profile_t prof;
    if (!an_init) { Septic_Init(&prof); an_prev = cmd_deg; an_init = 1; }

    if (cmd_deg != an_prev) {
        Septic_MoveTo(&prof, q_out, cmd_deg * (3.14159265f / 180.0f), TRAJ_MOVE_TIME);
        an_prev = cmd_deg;
    }
    Septic_Update(&prof, &rq, &rqd, &rqdd, &rj);

#endif

    // ส่งค่า reference (Pos, Vel, Accel) เข้าสู่ระบบควบคุม
    Cascade_Control_Update_FF(rq, rqd, rqdd);
}

static void Stream_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    hdma_usart1_tx.Instance                 = DMA1_Channel2;
    hdma_usart1_tx.Init.Request             = DMA_REQUEST_USART1_TX;
    hdma_usart1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode                = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority            = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_usart1_tx);

    __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

    /* IRQ — priority ต่ำกว่า TIM6 control (เลขมาก = ด่วนน้อย) ไม่กวน 1kHz loop */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* IRQ handlers (override weak symbol จาก startup) */
void DMA1_Channel2_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart1_tx); }
void USART1_IRQHandler(void)        { HAL_UART_IRQHandler(&huart1); }

/* ── ANALYSIS stream → USART2 (COM9 ST-Link VCP, 230400 8-E-1) — BINARY 8 ช่อง @ 1kHz
 *  MANUAL + analysis_mode==1 → frame 18 byte = [0xAA 0x55] + 8×int16 LE (signed = มีทิศ)
 *  8 ช่อง ASCII ไม่พอ bandwidth 230400 → ใช้ binary (18 byte × 1000 = 18kB/s พอดี)
 *  Simulink: Serial Configuration (Parity=Even) + Serial Receive (Header [170 85], int16, 8)
 *  คอลัมน์: q_ref qd_ref qdd_ref jerk q_out qd_out qdd_out V
 *  scale: q,qd,V ×1000 | qdd ×100 | jerk ×10  (Simulink หารกลับ)                  */
static inline int16_t sat16(float x) {
    if (x >  32767.0f) return  32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)x;
}

static void Stream_Update(void)
{
    if (current_system_mode != MODE_MANUAL || analysis_mode != 1) return;

    static uint32_t last = 0;
    if (HAL_GetTick() - last < 1U) return;              /* 1 kHz (Ts = 0.001) */
    last = HAL_GetTick();
    if (huart2.gState != HAL_UART_STATE_READY) return;  /* กันชน Modbus/stream TX ก่อนหน้า */

    /* ขยาย Array เป็น 17 ช่อง (14 เดิม + 3 ของ Lab 3) */
    int16_t v[17];
    v[0] = sat16(mon_q_ref   * 1000.0f);
    v[1] = sat16(mon_qd_ref  * 1000.0f);
    v[2] = sat16(mon_qdd_ref *  100.0f);
    v[3] = sat16(mon_j_out   *   10.0f);  /* เปลี่ยนจาก ref_j เป็น mon_j_out (actual jerk) */
    v[4] = sat16(mon_q_out   * 1000.0f);
    v[5] = sat16(mon_qd_out  * 1000.0f);
    v[6] = sat16(mon_qdd_out *  100.0f);
    v[7] = sat16(mon_v_in    * 1000.0f);

    /* ชุดตัวแปร PID debug */
    v[8]  = sat16(mon_u_pos  * 1000.0f);  /* u หลังออกจาก pos controller (ก่อนบวก ref_qd) */
    v[9]  = sat16(mon_u_vel  * 1000.0f);  /* u ของ velo controller (V_VEL) */
    v[10] = sat16(mon_v_ff   * 1000.0f);  /* Feedforward velocity */
    v[11] = sat16(mon_v_acc  * 1000.0f);  /* Feedforward acceleration */
    v[12] = sat16(mon_v_fric * 1000.0f);  /* Coulomb / Friction comp */
    v[13] = sat16(mon_v_dist * 1000.0f);  /* Disturbance comp */

    /* ── ชุด Lab 3 (Kalman) ── */
    v[14] = sat16(mon_qd_fd  * 1000.0f);  /* finite-diff velocity ดิบ (เทียบ KF, item 5) */
    v[15] = sat16(mon_tau_d  * 1000.0f);  /* KF disturbance τ_d [N.m]  (item 2) */
    v[16] = sat16(mon_i_est  * 1000.0f);  /* KF current i      [A]     (item 2) */

    /* ขนาด Frame = Header(2) + (17 ข้อมูล * 2 ไบต์) = 36 ไบต์ */
    static uint8_t fr[36];
    fr[0] = 0xAA; fr[1] = 0x55;
    for (int i = 0; i < 17; i++) {
        fr[2 + i*2] = (uint8_t)(v[i] & 0xFF);           /* little-endian */
        fr[3 + i*2] = (uint8_t)((v[i] >> 8) & 0xFF);
    }
    HAL_UART_Transmit_IT(&huart2, fr, sizeof(fr));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13) // ตรวจสอบว่าใช่ขา PC13 หรือไม่
    {
    	// อ่านค่าสถานะปัจจุบันของปุ่ม (PC13 Pull-up: กด=0, ปล่อย=1)
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
		{
			/* ── กดปุ่ม E-Stop ─────────────────────────────────────── */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_MOE_DISABLE(&htim1);
			/* 1. เริ่มเบรกนุ่ม (hybrid) — ยังไม่ตัด MOE
			 *    TIM6 ISR จะ decel ~150ms แล้วค่อยตัดไฟ. ไม่ Cascade_Control_Reset
			 *    ที่นี่ เพราะต้องคง velocity estimate (KF) ไว้ให้เบรกได้             */
			modbus_registers[REG_ESTOP]  = 1;   /* บอก PC: E-Stop active */
			modbus_registers[REG_RUN]    = 0;   /* หยุด Dashboard loop      */
			modbus_registers[REG_VT_MODE] = 0;  /* หยุด open-loop volt test */

			/* 2. clear mission state (hold ปลอดภัย + gripper abort) — ไม่แตะ KF */
			AutoMission_Reset();
			TestMode_Reset();
		}
		else
		{
			/* ── ปุ่มตู้ถูกปล่อย → clear ESTOP + เปิด MOE + กลับไปทำ HOMING ใหม่ ──
			 * สเปก: ปลด emergency แล้วหุ่นต้อง reset homing (หา home ด้วย sensor)
			 * ใหม่ทุกครั้ง — ไม่ resume ตำแหน่งเดิม (ตำแหน่ง encoder อาจเพี้ยน
			 * ระหว่างที่ตัดไฟ motor drive)                                         */
			if (modbus_registers[REG_ESTOP] == 1) {
				NVIC_SystemReset();
			}
		}
    }
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) return;  /* รันทุกๆ 1 ms (DT_VEL) */

    /* ── Pending start flags (ต้องเป็น static ให้จำค่าได้) ── */
    static uint8_t pending_auto_ticks = 0;
    static uint8_t pending_test_ticks = 0;
    static uint8_t auto_start_pending = 0;

    /* ── Debug counter (อ่านได้จาก REG_ISR_CNT) ── */
    modbus_registers[REG_ISR_CNT]++;

    /* ── Live Expressions monitor ── */
    debug_op_mode  = (uint8_t)current_system_mode;
    debug_selector = (uint8_t)selector_mode;
    debug_estop    = (uint8_t)modbus_registers[REG_ESTOP];


    /* ══════════════════════════════════════════════════════════════════════
     *  Priority 1 (สูงสุด): Homing กำลังทำงานอยู่ — ห้ามแทรก
     * ═════════════════════════════════════════════════════════════════════*/
    if (current_system_mode == MODE_HOMING) {
        Homing_Update();
        modbus_registers[REG_SYS_MODE] = MODE_HOMING;
        return;
    }

    /* ══════════════════════════════════════════════════════════════════════
     *  GLOBAL SOFT STOP (ปุ่ม STOP บน base system → REG_BS_SOFT_STOP 0x25 = 1)
     *  หยุดทุกอย่างทันทีในทุกโหมด (MANUAL/AUTO/TEST + joystick) — มอเตอร์ดับ
     *  ต่างจาก E-Stop: ไม่ตัด MOE, ไม่ latch ESTOP → base เคลียร์ 0x25=0 แล้วสั่งใหม่ได้
     * ═════════════════════════════════════════════════════════════════════*/
    if (modbus_registers[REG_BS_SOFT_STOP] & 0x0001) {
        AutoMission_Reset();      /* clear P&P state machine                 */
        TestMode_Reset();         /* clear test state machine                */
        modbus_registers[REG_RUN]     = 0;          /* หยุด dashboard loop    */
        modbus_registers[REG_BS_MODE] = 0;          /* ทิ้ง mode command ค้าง */
        modbus_registers[REG_BS_TASK] = TASK_IDLE;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);   /* มอเตอร์ดับ (override hold) */
        Gripper_Update();         /* reed telemetry ยัง update ได้           */
        return;
    }

    /* ══════════════════════════════════════════════════════════════════════
     *  Priority 2: Selector switch หน้าตู้ = MANUAL → บังคับ MODE_MANUAL
     *  ★ joystick + dashboard + gripper manual ทำงานเฉพาะตรงนี้ ★
     *  base system mode command (0x01) ถูกทิ้งทั้งหมด → joystick ทำงาน "เหมือนเดิม"
     * ═════════════════════════════════════════════════════════════════════*/
    if (selector_mode == SELECTOR_MANUAL) {
        /* เพิ่งสลับเข้า MANUAL → clear auto/test state ครั้งเดียว (กัน reset รัวๆ) */
        if (current_system_mode != MODE_MANUAL) {
            AutoMission_Reset();
            TestMode_Reset();
            Cascade_Control_Reset();
            current_system_mode = MODE_MANUAL;
        }
        modbus_registers[REG_BS_MODE]  = 0;          /* ทิ้ง mode command จาก base */
        modbus_registers[REG_SYS_MODE] = MODE_MANUAL;

        pending_auto_ticks = 0;
        pending_test_ticks = 0;
        auto_start_pending = 0;

        /* ── Open-loop voltage test (Lab 1 ID) — bypass controller ──────────
         * ทำงานเมื่อ REG_VT_MODE != 0 และไม่ ESTOP. จ่าย V ตรง + KF + 1kHz logger
         * (joystick e-stop ปุ่ม A ไม่ทำงานช่วงนี้ — ใช้ปุ่มตู้ หรือ STOP บน dashboard) */
        if (modbus_registers[REG_VT_MODE] != 0 && modbus_registers[REG_ESTOP] == 0) {
            VoltTest_Update();
            return;
        }

        /* ── Analysis mode: สั่ง move ผ่าน cmd_deg (Live Expressions) → sys ID ── */
        if (analysis_mode == 1 && modbus_registers[REG_ESTOP] == 0) {
            Analysis_Control();
            return;
        }

        /* Joystick ก่อน — คืน 1 = joystick กำลัง drive motor → ข้าม Dashboard
         * (กัน joystick กับ dashboard/jog แย่งสั่งมอเตอร์)                 */
        uint8_t joy_active = Joystick_Update();

        /* ── E-Stop ค้าง (joystick A / ปุ่มตู้ PC13) → freeze ──────────────────
         * เรียก Joystick_Update ด้านบนแล้ว (รับปุ่ม A กดซ้ำเพื่อ reset ได้) แต่
         * ห้าม drive มอเตอร์/กระตุ้น gripper ต่อ — ตัด compare ไว้ (กันมอเตอร์ขยับ
         * แม้ MOE หลุด) ส่วน gripper ถูก Abort → safe ตั้งแต่ตอน latch แล้ว ค้างนิ่ง */
        if (modbus_registers[REG_ESTOP]) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        } else {
            if (!joy_active) {
                Dashboard_Update();   /* base jog (0x05) + telemetry */
            }
            Gripper_Update();         /* manual gripper (0x02/0x03) + reed */
        }

        /* ── เขียน base telemetry (0x28/0x29/0x30) ใน MANUAL ───────────────────
         * joystick (free/point) bypass cascade → KF ไม่อัปเดต ต้องคำนวณ vel/acc
         * จาก encoder เอง. POS = deg ×10, VEL/ACC = rad/s,rad/s² ×10 (ตรงป้าย UI)
         *
         * ★ vel/acc วัดบน window 10ms ไม่ใช่ทุก 1ms:
         *   encoder 1 count = 2π/8192 = 7.67e-4 rad. ถ้า diff ทุก 1ms, 1 count/tick
         *   = 0.767 rad/s, accel พุ่งสูง → ×10 เสี่ยงทะลุ int16 wrap.
         *   window 10ms ลด quantization step 10× + EMA + clamp                      */
        Encoder_Update(&henc2);

        static float   bs_win_prev_rad = 0.0f;   /* pos ตอนต้น window 10ms [rad] */
        static float   bs_vel_ema      = 0.0f;   /* [rad/s]  */
        static float   bs_prev_vel     = 0.0f;
        static float   bs_acc_ema      = 0.0f;   /* [rad/s²] */
        static uint8_t bs_win_tick     = 0;

        /* POS — ทุก tick [deg ×10] */
        modbus_registers[REG_BS_POS] = (uint16_t)(int16_t)(henc2.position_deg * 10.0f * BS_DIR_SIGN);

        /* VEL/ACC — ทุก 10ms (window finite-diff, หน่วย rad ให้ตรงป้าย UI) */
        if (++bs_win_tick >= 10U) {
            bs_win_tick = 0;

            float pos_rad   = henc2.position_rad;
            float vel_raw   = (pos_rad - bs_win_prev_rad) * 100.0f;  /* rad/s (÷0.01) */
            bs_win_prev_rad = pos_rad;
            bs_vel_ema     += 0.3f * (vel_raw - bs_vel_ema);

            float acc_raw   = (bs_vel_ema - bs_prev_vel) * 100.0f;   /* rad/s² (÷0.01) */
            bs_prev_vel     = bs_vel_ema;
            bs_acc_ema     += 0.3f * (acc_raw - bs_acc_ema);

            /* ×10 ให้ตรง decode ฝั่ง BS (÷10); clamp กัน int16 overflow */
            float vel_q = bs_vel_ema * 10.0f * BS_DIR_SIGN;
            float acc_q = bs_acc_ema * 10.0f * BS_DIR_SIGN;
            if      (vel_q >  32767.0f) vel_q =  32767.0f;
            else if (vel_q < -32768.0f) vel_q = -32768.0f;
            if      (acc_q >  32767.0f) acc_q =  32767.0f;
            else if (acc_q < -32768.0f) acc_q = -32768.0f;

            modbus_registers[REG_BS_VEL] = (uint16_t)(int16_t)vel_q;
            modbus_registers[REG_BS_ACC] = (uint16_t)(int16_t)acc_q;
        }

        return;
    }

    /* ══════════════════════════════════════════════════════════════════════
     *  Priority 3: Selector switch = AUTO → base system ควบคุมผ่าน REG_BS_MODE
     *  ★ base auto ทำงานได้ "ก็ต่อเมื่อ selector = AUTO" เท่านั้น ★
     *  (joystick ไม่ทำงานฝั่งนี้ — manual ใช้สวิตช์หน้าตู้อย่างเดียว)
     *    0x01 bit0 HOME → go-home | bit1 JOG → PP_JOG | bit2 AUTO → Pick&Place
     *    bit3 SET_HOME → zero encoder | bit4 TEST → Performance/Precision
     * ═════════════════════════════════════════════════════════════════════*/

    /* selector เพิ่งโยก MANUAL → AUTO แต่ mode ยังค้าง MANUAL → ออกมาเป็น AUTO idle */
    if (current_system_mode == MODE_MANUAL) {
        Cascade_Control_Reset();
        AutoMission_Reset();

        pending_auto_ticks = 0;
        pending_test_ticks = 0;
        auto_start_pending = 0;

        current_system_mode            = MODE_AUTO;
        modbus_registers[REG_SYS_MODE] = MODE_AUTO;
    }

    uint16_t bs_cmd = modbus_registers[REG_BS_MODE];

    if (bs_cmd != 0 && modbus_registers[REG_ESTOP] == 0) {

        /* command register เป็น pulse → clear ทันที */
        modbus_registers[REG_BS_MODE] = 0;

        if (bs_cmd & REG_BS_MODE_HOME) {

            /* BS "Go Home" = controlled move กลับ position 0 (ใช้ cascade control)
             * ต่างจาก force homing ตอน boot (raw-seek หา flag) — ไม่ re-home flag */
            AutoMission_Reset();

            pending_auto_ticks = 0;
            pending_test_ticks = 0;
            auto_start_pending = 0;

            modbus_registers[REG_RUN]      = 0;

            AutoMission_GoHome();          /* Septic move → 0 (controlled) */

            current_system_mode            = MODE_AUTO;
            modbus_registers[REG_BS_TASK]  = TASK_HOMING;
        }

        else if (bs_cmd & REG_BS_MODE_JOG) {

            /* base MANUAL/Jog tab → MODE_AUTO + PP_JOG_IDLE (jog ผ่าน 0x05)
             * NB: MODE_MANUAL จริงสงวนไว้ให้สวิตช์หน้าตู้เท่านั้น                */
            Cascade_Control_Reset();
            AutoMission_Reset();

            pending_auto_ticks = 0;
            pending_test_ticks = 0;
            auto_start_pending = 0;

            modbus_registers[REG_RUN] = 0;

            AutoMission_StartJog();        /* → PP_JOG_IDLE */

            current_system_mode = MODE_AUTO;
        }

        else if (bs_cmd & REG_BS_MODE_AUTO) {

            /* กัน BS refresh AUTO command ซ้ำ */
            if (!auto_start_pending) {
                Cascade_Control_Reset();
                AutoMission_Reset();

                pending_test_ticks = 0;
                pending_auto_ticks = 150;
                auto_start_pending = 1;
            }

            current_system_mode = MODE_AUTO;
        }

        else if (bs_cmd & REG_BS_MODE_SET_HOME) {
            /* zero encoder ที่ตำแหน่งปัจจุบัน — ไม่เปลี่ยนโหมด */
            Homing_SetHome();
        }

        else if (bs_cmd & REG_BS_MODE_TEST) {

            /* guard: เริ่ม test ใหม่เฉพาะตอนเปลี่ยนเข้า TEST (กัน re-send รีสตาร์ทรัวๆ) */
            if (current_system_mode != MODE_TEST) {
                Cascade_Control_Reset();
                AutoMission_Reset();

                pending_auto_ticks = 0;
                pending_test_ticks = 150;
                auto_start_pending = 0;

                current_system_mode = MODE_TEST;
            }
        }
    }

    /* ── Pending start countdown (รอ ~150ms ให้ params ลงครบก่อน start) ── */
    if (pending_auto_ticks > 0) {
        if (--pending_auto_ticks == 0) {
            auto_start_pending = 0;
            AutoMission_StartAuto();
        }
    }

    if (pending_test_ticks > 0) {
        if (--pending_test_ticks == 0) {
            TestMode_Start();
        }
    }

    /* ══════════════════════════════════════════════════════════════════════
     *  Operating mode dispatch
     * ═════════════════════════════════════════════════════════════════════*/
    switch (current_system_mode) {
        /* MODE_MANUAL จัดการใน Priority 2 (selector=MANUAL) แล้ว return ก่อนถึงนี่
         * → ที่นี่เหลือแค่ AUTO / TEST (selector=AUTO)                        */
        case MODE_AUTO:
            AutoMission_Update();
            break;

        case MODE_TEST:
            TestMode_Update();
            break;

        default:
            break;
    }

    /* ── Report current mode (0x32) ── */
    modbus_registers[REG_SYS_MODE] = (uint16_t)current_system_mode;
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
