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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cascade_control.h"
#include "trajectory.h"
#include "encoder.h"
#include <math.h>
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

Encoder_t henc2;
//Trapz_Profile_t my_profile;
SCurve_Profile_t my_profile;

volatile uint32_t current_counter = 0;

float current_degree_out;
int signal= 0;

float waypoints_deg[] = {-45.0f, 0.0f};

const uint8_t total_waypoints = 2;

typedef enum {
    STATE_IDLE,    // สถานะว่าง (รอคนกดปุ่ม)
    STATE_MOVING,  // กำลังหมุนมอเตอร์ไปเป้าหมาย
    STATE_WAITING =3 // ถึงเป้าหมายแล้ว (กำลังนับเวลาหน่วง 2 วินาที)
} SequenceState_t;


SequenceState_t seq_state = STATE_IDLE; // เริ่มต้นที่สถานะว่าง
int current_wp_index = 0;               // จุดที่กำลังจะไป
float wait_timer = 0.0f;                // ตัวจับเวลารอ

const float WAIT_TIME = 2;           // กำหนดเวลาหน่วง (2 วินาที)

static uint8_t btn_prev_state = 0;


float ref_q = 0.0f;
float ref_qd = 0.0f;
float ref_qdd = 0.0f;
float ref_j = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  henc2.htim = &htim2;

  Cascade_Control_Init();

  Encoder_Init(&henc2, &htim2);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
 // HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_MOE_ENABLE(&htim1);

  HAL_TIM_Base_Start_IT(&htim6); // เปิด Interrupt เป็นลำดับสุดท้าย

  //Trapz_Init(&my_profile);
  SCurve_Init(&my_profile);

  //Homing Testing Code
//  while(!(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13))){
//	  current_counter = __HAL_TIM_GET_COUNTER(&htim2);
//	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//	  signal =HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4);
//  }
//  while(!(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4))){
//	  current_counter = __HAL_TIM_GET_COUNTER(&htim2);
//	  HAL_GPIO_WritePin(Motor_Dir_GPIO_Port, Motor_Dir_Pin, GPIO_PIN_RESET);
//	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
//  }
//  while(!(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13))){
//	  current_counter = __HAL_TIM_GET_COUNTER(&htim2);
//	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//  }
//  current_counter = __HAL_TIM_SET_COUNTER(&htim2,0);
//  while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)){
//	  current_counter = __HAL_TIM_GET_COUNTER(&htim2);
//	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
//  }
//  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  //Homing Testing Code

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) { // ตรวจสอบว่าเป็น Timer ที่คุณใช้ทำ Control Loop



        // --- ส่วนที่ 1: เช็คปุ่มกด (Trigger) เพื่อเริ่มงาน ---
        // (สมมติว่ากดปุ่มแล้วได้ Logic 1, ถ้าระบบเป็น Active Low ให้แก้เป็น 0)
        uint8_t btn_current_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
        if (btn_current_state == 1 && btn_prev_state == 0) {
            // ถ้าระบบว่างอยู่ ให้เริ่มวิ่งไปจุดแรก
            if (seq_state == STATE_IDLE) {
                seq_state = STATE_MOVING;
                current_wp_index = 0; // รีเซ็ตให้ไปจุดเริ่มต้น (Index 0)

                // ดึงตำแหน่งปัจจุบัน และคำนวณตำแหน่งเป้าหมาย
                float start_q = Encoder_GetPositionRad(&henc2);
                float target_q = waypoints_deg[current_wp_index] * (PI / 180.0f);

                // สั่งเดินเครื่อง
                //Trapz_MoveTo(&my_profile, start_q, target_q);
                SCurve_MoveTo(&my_profile, start_q, target_q);
            }
        }
        btn_prev_state = btn_current_state; // จำสถานะปุ่มไว้ใช้รอบหน้า

        // --- ส่วนที่ 2: ระบบ State Machine (จัดการลำดับอัตโนมัติ) ---
        switch (seq_state) {
            case STATE_IDLE:
                // ไม่ต้องทำอะไร รอจนกว่าจะมีการกดปุ่ม
                break;

            case STATE_MOVING:
                // ถ้ามอเตอร์วิ่งไปถึงเป้าหมายแล้ว (is_running เปลี่ยนเป็น 0)
                if (my_profile.is_running == 0) {
                    seq_state = STATE_WAITING; // สลับไปสถานะรอ 2 วิ
                    wait_timer = 0.0f;         // รีเซ็ตตัวนับเวลารอ
                }
                break;

            case STATE_WAITING:
                wait_timer += 0.001f; // นับเวลาไปเรื่อยๆ (DT คือเวลาต่อลูปจาก cascade_control.h)

                // ถ้านับเวลาครบ 2 วินาทีแล้ว
                if (wait_timer >= WAIT_TIME) {
                    current_wp_index++; // เลื่อน Index ไปจุดถัดไป

                    // เช็คว่ายังมีจุดเหลือใน Array ให้วิ่งต่อไหม?
                    if (current_wp_index < total_waypoints) {
                        //float start_q = Encoder_GetPositionRad(&henc2);
                        float start_q = ref_q;
                        float target_q = waypoints_deg[current_wp_index] * (PI / 180.0f);

                        //Trapz_MoveTo(&my_profile, start_q, target_q); // สั่งวิ่งต่อ
                        SCurve_MoveTo(&my_profile, start_q, target_q);
                        seq_state = STATE_MOVING; // เปลี่ยนสถานะกลับไปเป็น MOVING
                    } else {
                        // ถ้าวิ่งครบทุกจุดใน Array แล้ว
                        seq_state = STATE_IDLE; // หยุดและกลับไปรอคนกดปุ่มเริ่มรอบใหม่
                    }
                }
                break;
        }

        // --- ส่วนที่ 3: อัปเดตการควบคุมมอเตอร์ ---
        // (ฟังก์ชันนี้ไม่ต้องส่ง DT ไปแล้ว เพราะโค้ดในไลบรารีไปดึงมาจาก cascade_control.h ให้เอง)
        //Trapz_Update(&my_profile, &ref_q, &ref_qd, &ref_qdd,&ref_j);
        SCurve_Update(&my_profile, &ref_q, &ref_qd, &ref_qdd, &ref_j);

        // ส่งต่อเป้าหมายเข้า Cascade Controller
        Cascade_Control_Update(ref_q, ref_qd);

        // ดึงองศาปัจจุบันมาไว้ดูผลลัพธ์ผ่าน Live Watch
        //Encoder_Update(&henc2);
        current_degree_out = Encoder_GetPositionRad(&henc2);
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
