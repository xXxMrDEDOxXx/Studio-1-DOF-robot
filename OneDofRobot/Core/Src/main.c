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
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "base_system.h"      /* Register map, SystemMode_t, modbus_registers[] */
#include "cascade_control.h"
#include "trajectory.h"
#include "encoder.h"
#include "dashboard.h"        /* Manual Mode Dashboard (Modbus ↔ PC)           */
#include "auto_mission.h"     /* Pick & Place Auto Mission (MODE_AUTO)          */
#include "homing.h"           /* Force-Homing state machine (MODE_HOMING)       */
#include "gripper.h"          /* Gripper: PC4=arm PC10=jaw + reed feedback      */
#include "test_mode.h"        /* Performance & Precision test (MODE_TEST)       */
#include "joystick.h"         /* Funduino Joystick Shield (MODE_MANUAL only)    */
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

/* ── Internal Temperature Sensor ─────────────────────────────────────────
 * ดูค่าได้ใน Live Expressions: chip_temp_C
 * G474 internal sensor: ADC1 CH16, cal @ VDDA=3.0V
 * ───────────────────────────────────────────────────────────────────────*/
volatile float chip_temp_C = 0.0f;
volatile uint32_t debug_uart_rx_count = 0;   /* นับทุกครั้งที่ UART3 ได้รับ frame */
volatile uint32_t debug_modbus_ok_count = 0; /* นับทุกครั้งที่ Modbus_Parse_Frame ถูกเรียก */

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
SCurve_Profile_t my_profile;

/* SystemMode_t typedef อยู่ใน base_system.h
 * ประกาศ definition ที่นี่ — base_system.h มี extern declaration แล้ว */
volatile SelectorMode_t selector_mode      = SELECTOR_MANUAL; /* physical switch */
volatile SystemMode_t   current_system_mode = MODE_HOMING;    /* actual mode     */

typedef enum {
    STATE_IDLE,    // สถานะว่าง (รอคนกดปุ่ม)
    STATE_MOVING,  // กำลังหมุนมอเตอร์ไปเป้าหมาย
    STATE_WAITING  // ถึงเป้าหมายแล้ว (กำลังนับเวลาหน่วง 2 วินาที)
} SequenceState_t;
SequenceState_t seq_state = STATE_IDLE; // เริ่มต้นที่สถานะว่าง

const float WAIT_TIME = 2;           // กำหนดเวลาหน่วง (2 วินาที)
float waypoints_deg[] = {90.0f, 0.0f, 180.0f, 0.0f, 360.0f, 0.0f};
float current_degree_out;
float wait_timer = 0.0f;                // ตัวจับเวลารอ
float ref_q = 0.0f;
float ref_qd = 0.0f;
float ref_qdd = 0.0f;
float ref_j = 0.0f;


int signal= 0;
int current_wp_index = 0;               // จุดที่กำลังจะไป


const uint8_t total_waypoints = 6;
volatile uint8_t debug_pb0_state;
volatile uint8_t debug_pb1_state;

uint8_t rx_buffer[128];


/* tune variables ย้ายไป velocity_tune.c แล้ว */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//Update_Telemetry_To_PC(void);
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

    debug_uart_rx_count++;   /* ทุก frame ที่เข้ามา (รวม echo) */

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
    debug_modbus_ok_count++;   /* frame ผ่าน echo filter แล้ว → parse */
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
  /* USER CODE BEGIN 2 */

  TempSensor_Init();        /* ADC1 CH16 — อ่าน chip_temp_C ใน Live Expressions */
  Heartbeat_Init();
  HAL_UARTEx_EnableFifoMode(&huart2);  /* เปิด RX FIFO 8 byte — margin กัน overrun
                                        * (override DisableFifoMode ใน usart.c) */
  __HAL_UART_CLEAR_OREFLAG(&huart2);   /* เคลียร์ error flags ก่อน */
  HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_buffer, sizeof(rx_buffer));
  henc2.htim = &htim2;

  Encoder_Init(&henc2, &htim2);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  SCurve_Init(&my_profile);
  Cascade_Control_Init();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_MOE_ENABLE(&htim1);

  Dashboard_Init();
  AutoMission_Init();
  Homing_Init();
  Gripper_Init();
  TestMode_Init();
  Joystick_Init();

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

/* USER CODE BEGIN 4 */
/* VelocityTune_Update() ย้ายไป Core/Src/velocity_tune.c แล้ว */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13) // ตรวจสอบว่าใช่ขา PC13 หรือไม่
    {
    	// อ่านค่าสถานะปัจจุบันของปุ่ม (PC13 Pull-up: กด=0, ปล่อย=1)
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
		{
			/* ── กดปุ่ม E-Stop ─────────────────────────────────────── */

			/* 1. หยุด Hardware ทันที */
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_MOE_DISABLE(&htim1);

			/* 2. ล็อกสถานะ Software */
			seq_state             = STATE_IDLE;
			my_profile.is_running = 0;
			modbus_registers[REG_ESTOP] = 1;   /* บอก PC: E-Stop active */

			/* 3. หยุด Dashboard ISR loop
			 *    ป้องกัน Dashboard_Update() ส่ง voltage ต่อหลัง e-stop */
			modbus_registers[REG_RUN] = 0;

			/* 4. Reset KF + PID integrators
			 *    ป้องกัน integrator windup และ KF state ผิดพลาดหลัง resume */
			Cascade_Control_Reset();

			/* 5. Reset Auto Mission state machine (ถ้าอยู่ใน MODE_AUTO) */
			AutoMission_Reset();

			/* 6. Reset Test Mode */
			TestMode_Reset();
		}
		else
		{
			/* ปุ่มที่ตู้ถูกปล่อย → clear ESTOP และ re-enable motor drive
			 * (ไม่ reset MCU — ระบบทำงานต่อจากเดิมทันที)              */
			if (modbus_registers[REG_ESTOP] == 1) {
				modbus_registers[REG_ESTOP] = 0;
				__HAL_TIM_MOE_ENABLE(&htim1);
				Cascade_Control_Reset();   /* sync KF + clear integrators */
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
     *  Priority 2: Selector switch หน้าตู้ — MANUAL บังคับ MODE_MANUAL
     * ═════════════════════════════════════════════════════════════════════*/
    if (selector_mode == SELECTOR_MANUAL) {
        current_system_mode            = MODE_MANUAL;
        modbus_registers[REG_BS_MODE]  = 0;
        modbus_registers[REG_SYS_MODE] = MODE_MANUAL;

        /* clear pending ทุกอย่าง */
        pending_auto_ticks = 0;
        pending_test_ticks = 0;
        auto_start_pending = 0;

        Dashboard_Update();
        Gripper_Update();      /* manual gripper (REG_BS_GRIPPER_MAN) + reed telemetry */
        return;
    }

    /* ══════════════════════════════════════════════════════════════════════
     *  Priority 3: selector = AUTO → base system ควบคุม
     * ═════════════════════════════════════════════════════════════════════*/

    /* selector โยกมา AUTO แต่ mode ค้างอยู่ MANUAL */
    if (current_system_mode == MODE_MANUAL) {
        Cascade_Control_Reset();
        AutoMission_Reset();

        pending_auto_ticks = 0;
        pending_test_ticks = 0;
        auto_start_pending = 0;

        current_system_mode            = MODE_AUTO;
        modbus_registers[REG_SYS_MODE] = MODE_AUTO;
    }

    /* ── Pending start countdown ── */
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

    /* ── Read Base System command ── */
    uint16_t bs_cmd = modbus_registers[REG_BS_MODE];

    if (bs_cmd != 0 && modbus_registers[REG_ESTOP] == 0) {

        /* command register เป็น pulse → clear ทันที */
        modbus_registers[REG_BS_MODE] = 0;

        if (bs_cmd & REG_BS_MODE_HOME) {

            /* BS "Go Home" = controlled move กลับ position 0 (ใช้ cascade control)
             * ต่างจาก force homing ตอน boot (raw-seek หา flag) — ไม่ re-home flag
             * encoder zero ถูกตั้งจาก force homing ตอนเริ่มแล้ว                    */
            AutoMission_Reset();

            pending_auto_ticks = 0;
            pending_test_ticks = 0;
            auto_start_pending = 0;

            modbus_registers[REG_RUN]      = 0;

            AutoMission_GoHome();          /* Septic move → 0 (controlled) */

            current_system_mode            = MODE_AUTO;
            modbus_registers[REG_SYS_MODE] = MODE_AUTO;
            modbus_registers[REG_BS_TASK]  = TASK_HOMING;
        }

        else if (bs_cmd & REG_BS_MODE_JOG) {

            Cascade_Control_Reset();
            AutoMission_Reset();

            pending_auto_ticks = 0;
            pending_test_ticks = 0;
            auto_start_pending = 0;

            AutoMission_StartJog();

            current_system_mode            = MODE_AUTO;
            modbus_registers[REG_SYS_MODE] = MODE_AUTO;
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

            current_system_mode            = MODE_AUTO;
            modbus_registers[REG_SYS_MODE] = MODE_AUTO;
        }

        else if (bs_cmd & REG_BS_MODE_SET_HOME) {
            Homing_SetHome();
        }

        else if (bs_cmd & REG_BS_MODE_TEST) {

            Cascade_Control_Reset();
            AutoMission_Reset();

            pending_auto_ticks = 0;
            pending_test_ticks = 150;
            auto_start_pending = 0;

            current_system_mode            = MODE_TEST;
            modbus_registers[REG_SYS_MODE] = MODE_TEST;
        }
    }

    /* ── Operating mode dispatch ── */
    switch (current_system_mode) {
        case MODE_MANUAL:
            Dashboard_Update();
            Gripper_Update();   /* manual gripper + reed telemetry */
            break;

        case MODE_AUTO:
            AutoMission_Update();
            break;

        case MODE_TEST:
            TestMode_Update();
            break;

        default:
            break;
    }

    /* ── Report current mode ── */
    modbus_registers[REG_SYS_MODE] = (uint16_t)current_system_mode;

    current_degree_out = Encoder_GetPositionDeg(&henc2);
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
