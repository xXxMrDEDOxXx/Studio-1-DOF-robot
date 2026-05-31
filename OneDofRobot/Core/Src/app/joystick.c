/*
 * joystick.c
 *
 *  Funduino Joystick Shield — Manual Mode Interface
 *  เรียก Joystick_Update() จาก TIM6 ISR ทุก 1 ms (ใน MODE_MANUAL เท่านั้น)
 */

#include "joystick.h"
#include "main.h"          /* peripheral handles (htim1) ประกาศ extern ที่นี่ */
#include "base_system.h"
#include "cascade_control.h"
#include "gripper.h"
#include "homing.h"
#include "auto_mission.h"
#include "test_mode.h"
#include "trajectory.h"   /* Septic (S-curve) trajectory สำหรับ point-mode move */
#include <math.h>

#define DEG2RAD  (3.14159265f / 180.0f)

/* ── External state ──────────────────────────────────────────────────────── */
extern volatile float q_out;   /* actual position [rad] — from cascade_control.c */
extern volatile SystemMode_t current_system_mode;

/* ══════════════════════════════════════════════════════════════════════════
 *  ADC2 bare-metal helpers (PC3 = ADC2_IN9)
 * ══════════════════════════════════════════════════════════════════════════*/

static void ADC2_JoyInit(void)
{
    /* ADC12 clock (shared ADC1/ADC2) — may already be enabled by TempSensor.
     * ADC12_COMMON->CCR (CKMODE = HCLK/4) ถูกตั้งโดย TempSensor_Init() ก่อนแล้ว.
     * PC3 → Analog mode ถูกตั้งโดย HAL_GPIO_Init() ใน Joystick_Init() ก่อนเรียกฟังก์ชันนี้ */
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;

    /* Exit Deep Power-Down + enable voltage regulator */
    ADC2->CR &= ~ADC_CR_DEEPPWD;
    ADC2->CR |=  ADC_CR_ADVREGEN;
    for (volatile uint32_t i = 0; i < 5000U; i++);   /* ≥ 20 µs @ 170 MHz */

    /* Calibration (single-ended) */
    ADC2->CR &= ~ADC_CR_ADCALDIF;
    ADC2->CR |=  ADC_CR_ADCAL;
    while (ADC2->CR & ADC_CR_ADCAL);

    /* Enable ADC2 */
    ADC2->ISR |= ADC_ISR_ADRDY;
    ADC2->CR  |= ADC_CR_ADEN;
    while (!(ADC2->ISR & ADC_ISR_ADRDY));

    /* Single conversion, software trigger, 12-bit, right-aligned */
    ADC2->CFGR  = 0;
    ADC2->CFGR2 = 0;

    /* Sequence: 1 conversion, channel 9 (PC3 = ADC2_IN9) */
    ADC2->SQR1  = (9UL << ADC_SQR1_SQ1_Pos) | (0UL << ADC_SQR1_L_Pos);

    /* Channel 9 sampling: 12.5 cycles (code 2) — SMP9 = bits [29:27] of SMPR1 */
    ADC2->SMPR1 = (2UL << 27U);
}

/* Blocking single conversion — takes ~0.6 µs @ ADC clock 42.5 MHz */
static uint16_t ADC2_ReadJoyX(void)
{
    ADC2->ISR |= ADC_ISR_EOC;          /* clear EOC */
    ADC2->CR  |= ADC_CR_ADSTART;       /* start conversion */
    while (!(ADC2->ISR & ADC_ISR_EOC));
    return (uint16_t)(ADC2->DR & 0x0FFFU);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Button debounce
 * ══════════════════════════════════════════════════════════════════════════*/

#define BTN_A  0
#define BTN_B  1
#define BTN_C  2
#define BTN_D  3
#define BTN_K  4
#define BTN_COUNT 5

static uint8_t btn_raw_prev[BTN_COUNT] = {0};
static uint8_t btn_cnt[BTN_COUNT]      = {0};
static uint8_t btn_stable[BTN_COUNT]   = {0};   /* current stable state: 1=pressed */
static uint8_t btn_edge[BTN_COUNT]     = {0};   /* 1 = press-edge this tick (one-shot) */

static void buttons_update(void)
{
    uint8_t raw[BTN_COUNT];
    raw[BTN_A] = (HAL_GPIO_ReadPin(JOY_BTN_A_GPIO_Port, JOY_BTN_A_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    raw[BTN_B] = (HAL_GPIO_ReadPin(JOY_BTN_B_GPIO_Port, JOY_BTN_B_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    raw[BTN_C] = (HAL_GPIO_ReadPin(JOY_BTN_C_GPIO_Port, JOY_BTN_C_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    raw[BTN_D] = (HAL_GPIO_ReadPin(JOY_BTN_D_GPIO_Port, JOY_BTN_D_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    raw[BTN_K] = (HAL_GPIO_ReadPin(JOY_BTN_K_GPIO_Port, JOY_BTN_K_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        btn_edge[i] = 0;
        if (raw[i] == btn_raw_prev[i]) {
            if (btn_cnt[i] < JOY_DEBOUNCE_MS) {
                btn_cnt[i]++;
            } else if (raw[i] != btn_stable[i]) {
                btn_stable[i] = raw[i];
                if (raw[i]) btn_edge[i] = 1;   /* press edge (0→1) */
            }
        } else {
            btn_cnt[i] = 0;
        }
        btn_raw_prev[i] = raw[i];
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Internal state
 * ══════════════════════════════════════════════════════════════════════════*/

static JoyMode_t joy_mode       = JOY_MODE_FREE;
static float     joy_target_rad = 0.0f;   /* point mode: current target [rad] */
static uint8_t   grip_toggle    = 0;      /* 0=next btn_B does Pick, 1=Place   */
static uint8_t   arm_toggle     = 0;      /* 0=next btn_D goes Down, 1=Up      */
static uint8_t   joy_was_neutral = 1;     /* point mode: joystick was at neutral */
static uint8_t   joy_prev_driving = 0;   /* free mode: was directly driving motor */
static Septic_Profile_t joy_septic;       /* point mode: S-curve move profile      */

/* ══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════════════════════════*/

void Joystick_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* Buttons A, B, C (GPIOA) — input pull-up */
    gpio.Pin  = JOY_BTN_A_Pin | JOY_BTN_B_Pin | JOY_BTN_C_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* Buttons K, D (GPIOB) — input pull-up */
    gpio.Pin  = JOY_BTN_K_Pin | JOY_BTN_D_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* ADC input (GPIOC PC2) */
    gpio.Pin  = JOY_ADC_Pin;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(JOY_ADC_GPIO_Port, &gpio);

    ADC2_JoyInit();

    joy_mode         = JOY_MODE_FREE;
    joy_target_rad   = 0.0f;
    grip_toggle      = 0;
    arm_toggle       = 0;
    joy_was_neutral  = 1;
    joy_prev_driving = 0;
    Septic_Init(&joy_septic);
}

JoyMode_t Joystick_GetMode(void) { return joy_mode; }

/* ──────────────────────────────────────────────────────────────────────────
 *  Joystick_Update — เรียกทุก 1 ms จาก TIM6 ISR ใน MODE_MANUAL
 *  คืนค่า 1 = joystick กำลัง drive motor (caller ควร skip Dashboard_Update)
 * ─────────────────────────────────────────────────────────────────────────*/
uint8_t Joystick_Update(void)
{
    if (current_system_mode != MODE_MANUAL) return 0U;

    buttons_update();
    uint16_t adc = ADC2_ReadJoyX();

    /* ── Button A: Emergency stop / Reset (TOGGLE) ────────────────────────
     * กดครั้งที่ 1 (ยังไม่ ESTOP) → ตัดไฟ motor drive ทันที + latch ESTOP
     * กดครั้งที่ 2 (ESTOP อยู่)   → clear ESTOP + เปิด MOE + กลับไปทำ HOMING ใหม่
     *                              (sensor-based — เหมือนเปิดเครื่อง)
     * (ปุ่มตู้ PC13 ปล่อย ก็ re-home เหมือนกัน — สองทางอิสระต่อกัน)            */
    if (btn_edge[BTN_A]) {
        if (modbus_registers[REG_ESTOP] == 0) {
            /* → Emergency STOP */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
            modbus_registers[REG_ESTOP] = 1;
            modbus_registers[REG_RUN]   = 0;
            Cascade_Control_Reset();
            AutoMission_Reset();
            TestMode_Reset();
            joy_prev_driving = 0;
        } else {
            /* → RESET: เปิด MOE + กลับไปทำ homing (sensor) ใหม่ */
            modbus_registers[REG_ESTOP]    = 0;
            __HAL_TIM_MOE_ENABLE(&htim1);
            Cascade_Control_Reset();
            AutoMission_Reset();
            TestMode_Reset();
            Homing_Start();                       /* sensor-based homing */
            current_system_mode            = MODE_HOMING;
            modbus_registers[REG_SYS_MODE] = MODE_HOMING;
            modbus_registers[REG_BS_TASK]  = TASK_HOMING;
            joy_prev_driving = 0;
            return 0U;   /* ออกทันที → Priority 1 (HOMING) คุมตั้งแต่ tick หน้า */
        }
    }

    /* หยุดทำงานทั้งหมดถ้า ESTOP active (รอกด A ซ้ำ หรือปุ่มตู้ PC13 เพื่อ clear) */
    if (modbus_registers[REG_ESTOP]) return 0U;

    /* ── Button B: Gripper Pick / Place toggle ────────────────────────────*/
    if (btn_edge[BTN_B]) {
        if (grip_toggle == 0U) {
            modbus_registers[REG_BS_GRIPPER_SEQ] = 1;  /* Pick */
            grip_toggle = 1;
        } else {
            modbus_registers[REG_BS_GRIPPER_SEQ] = 2;  /* Place */
            grip_toggle = 0;
        }
    }

    /* ── Button C: Set Home (ตั้ง home เฉยๆ — หุ่นต้องไม่ขยับ) ──────────────
     * zero encoder + sync KF + re-arm Septic ให้ hold ที่ home ใหม่
     * (ไม่งั้น point-mode Septic ยังถือ target เก่า → หุ่นวิ่งหลังกด C)        */
    if (btn_edge[BTN_C]) {
        Homing_SetHome();               /* zero TIM2 encoder ที่ตำแหน่งปัจจุบัน */
        Cascade_Control_Reset();        /* sync KF กับ encoder ที่เพิ่ง zero     */
        joy_target_rad  = 0.0f;
        joy_was_neutral = 1;
        Septic_MoveTo(&joy_septic, 0.0f, 0.0f, JOY_POINT_MOVE_TIME);  /* hold ที่ 0 */
    }

    /* ── Button D: Gripper arm Up / Down toggle ───────────────────────────
     * คุม arm ตรงๆ ผ่าน Gripper_SetArm() — ไม่เขียน REG_BS_GRIPPER_MAN (0x02)
     * → ไม่ชนกับ jaw command (OPEN/CLOSE) ที่ base ส่งมาทาง register เดียวกัน */
    if (btn_edge[BTN_D]) {
        if (arm_toggle == 0U) {
            Gripper_SetArm(1);   /* down */
            arm_toggle = 1;
        } else {
            Gripper_SetArm(0);   /* up */
            arm_toggle = 0;
        }
    }

    /* ── Button K: Toggle movement mode (Free ↔ Point) ────────────────────*/
    if (btn_edge[BTN_K]) {
        if (joy_mode == JOY_MODE_FREE) {
            joy_mode = JOY_MODE_POINT;
            joy_target_rad  = q_out;   /* ตั้ง target เป็นตำแหน่งปัจจุบัน */
            joy_was_neutral = 1;
            /* hold ที่ตำแหน่งปัจจุบัน: dist=0 → is_running=0, q_end=q_out
             * → Septic_Update จะคืน q_out ค้างไว้จนกว่าจะมีคลิกใหม่           */
            Septic_MoveTo(&joy_septic, q_out, q_out, JOY_POINT_MOVE_TIME);
            /* เปิด position loop: ตั้ง pos gains (×100) ให้ cascade คุมตำแหน่งได้
             * ค่าตรงกับ POS_*_AUTO ใน cascade_control.c (15.5 / 1.2 / 0)        */
            modbus_registers[REG_POS_KP] = 1550;
            modbus_registers[REG_POS_KI] = 120;
            modbus_registers[REG_POS_KD] = 0;
            modbus_registers[REG_DRIVE_MODE] = 0;   /* cascade (pos→vel→V) */
        } else {
            joy_mode = JOY_MODE_FREE;
            /* ปิด position loop กลับเป็น velocity-only (default dashboard) */
            modbus_registers[REG_POS_KP] = 0;
            modbus_registers[REG_POS_KI] = 0;
            modbus_registers[REG_POS_KD] = 0;
        }
        /* Reset cascade เมื่อสลับ mode เพื่อล้าง integrator */
        Cascade_Control_Reset();
        modbus_registers[REG_RUN] = 0;
        joy_prev_driving = 0;
    }

    /* ══════════════════════════════════════════════════════════════════════
     *  ADC movement control
     * ══════════════════════════════════════════════════════════════════════*/
    uint8_t adc_ccw = (adc < JOY_ADC_LOW)  ? 1U : 0U;
    uint8_t adc_cw  = (adc > JOY_ADC_HIGH) ? 1U : 0U;
    uint8_t adc_active = adc_ccw || adc_cw;

    if (joy_mode == JOY_MODE_FREE) {
        /* ── Free Movement Mode: direct PWM at 15% duty ─────────────────── */
        if (adc_active) {
            /* หยุด dashboard จากการ override */
            modbus_registers[REG_RUN] = 0;

            /* ตั้งทิศและ duty โดยตรง (bypass cascade) */
            GPIO_PinState dir = adc_ccw ? JOY_DIR_CCW : JOY_DIR_CW;
            HAL_GPIO_WritePin(Motor_Dir_GPIO_Port, Motor_Dir_Pin, dir);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, JOY_FREE_DUTY);

            joy_prev_driving = 1;
            return 1U;   /* แจ้ง caller ว่า joystick กำลัง drive → skip Dashboard */
        } else {
            /* Neutral → หยุดมอเตอร์, reset cascade ถ้าเพิ่งหยุด drive */
            if (joy_prev_driving) {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                Cascade_Control_Reset();
                joy_prev_driving = 0;
            }
            return 0U;   /* ให้ Dashboard ควบคุมต่อ */
        }

    } else {
        /* ── Point-to-Point Mode: ±5° ต่อคลิก (S-curve / Septic เนียน) ───── */

        if (!adc_active) {
            /* Joystick กลับมา neutral → อนุญาตให้ trigger ครั้งถัดไปได้ */
            joy_was_neutral = 1;
        }

        if (adc_active && joy_was_neutral) {
            /* Trigger การเคลื่อนที่ใหม่ (เฉพาะครั้งแรกที่กดหลังปล่อย) */
            joy_was_neutral = 0;

            /* CCW (joystick ซ้าย) = firmware ทิศลบ (encoder ลด); CW = ทิศบวก
             * (firmware + = CW ตามที่วัดจริง) */
            float step = JOY_STEP_DEG * DEG2RAD;
            joy_target_rad = q_out + (adc_ccw ? -step : step);

            /* สร้าง S-curve trajectory จากตำแหน่งจริง → target (jerk-continuous)
             * เนียน ไม่กระชาก, ไม่กระตุก backlash เหมือน step เดิม                */
            Septic_MoveTo(&joy_septic, q_out, joy_target_rad, JOY_POINT_MOVE_TIME);
        }

        /* รัน trajectory ทุก tick → ป้อน cascade แบบ feedforward (q, qd, qdd)
         * จบ move แล้ว Septic_Update คืน q_end ค้าง (is_running=0) → hold ตำแหน่ง */
        float ref_q, ref_qd, ref_qdd, ref_j;
        Septic_Update(&joy_septic, &ref_q, &ref_qd, &ref_qdd, &ref_j);

        modbus_registers[REG_RUN] = 0;   /* ป้องกัน Dashboard override */
        Cascade_Control_Update_FF(ref_q, ref_qd, ref_qdd);
        return 1U;
    }
}
