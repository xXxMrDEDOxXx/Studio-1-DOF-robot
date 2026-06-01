/*
 * test_mode.c
 *
 *  Performance & Precision test mode (MODE_TEST)
 */

#include "test_mode.h"
#include "base_system.h"
#include "cascade_control.h"
#include "trajectory.h"
#include "auto_mission.h"    /* HOLE_STEP_DEG */
#include "main.h"
#include <math.h>

#define DEG2RAD  (3.14159265f / 180.0f)
#define RAD2DEG  (180.0f / 3.14159265f)
#define TM_DWELL_MS  500U

/* ── Legacy S-Curve params — ไม่ได้ใช้แล้ว ──────────────────────────────────
 *  ปัจจุบัน Precision ใช้ Septic time-scaled (Septic_MoveTo + TRAJ_MOVE_TIME)
 *  ซึ่งไม่กินค่า V/A/J เหล่านี้ (เก็บไว้เผื่อย้อนกลับไปใช้ S-Curve)            */
#define TM_PREC_V_MAX   TRAJ_V_MAX   /* rad/s  */
#define TM_PREC_A_MAX   15.0f        /* rad/s² */
#define TM_PREC_J_MAX   80.0f        /* rad/s³ */

/* ── States ──────────────────────────────────────────────────────────────── */
#define TM_IDLE    0
#define TM_MOVING  1
#define TM_HOLD    2
#define TM_DONE    3

/* ── Extern (cascade_control.c) ──────────────────────────────────────────── */
extern volatile float q_out;
extern volatile float qd_out;
extern          float qdd_out;

/* ── Private state ───────────────────────────────────────────────────────── */
static uint8_t         tm_state      = TM_IDLE;
static uint8_t         tm_type       = 0;      /* 0=precision 1=performance  */
static float           tm_pos_a      = 0.0f;   /* position A [rad]           */
static float           tm_pos_b      = 0.0f;   /* position B [rad]           */
static float           tm_cur_end    = 0.0f;   /* current move destination   */
static uint8_t         tm_init_done  = 0;      /* 0=positioning to A, 1=testing */
static uint8_t         tm_trips      = 0;      /* one-way trips completed    */
static uint8_t         tm_trips_max  = 0;      /* 2 × repeat_count           */
static uint32_t        tm_hold_start = 0;

static Trapz_Profile_t   tm_trapz;
static Septic_Profile_t  tm_septic;  /* Precision mode ใช้ Septic (jerk-continuous) */
static float             tm_ref_q = 0.0f, tm_ref_qd = 0.0f;
static float             tm_ref_qdd = 0.0f, tm_ref_j = 0.0f;

/* ── Private helpers ─────────────────────────────────────────────────────── */
static void _tm_telemetry(uint16_t task_bits)
{
    modbus_registers[REG_BS_POS]  = (uint16_t)(int16_t)(q_out   * RAD2DEG * 10.0f * BS_DIR_SIGN); /* deg ×10   */
    modbus_registers[REG_BS_VEL]  = (uint16_t)(int16_t)(qd_out            * 10.0f * BS_DIR_SIGN); /* rad/s ×10 */
    modbus_registers[REG_BS_ACC]  = (uint16_t)(int16_t)(qdd_out           * 10.0f * BS_DIR_SIGN); /* rad/s² ×10*/
    modbus_registers[REG_BS_TASK] = task_bits;
}

static void _tm_start_move(float from_rad, float to_rad)
{
    tm_cur_end = to_rad;
    if (tm_type == 1) {
        /* Performance: Trapz, custom vel/acc จาก registers */
        float v = (float)(int16_t)modbus_registers[REG_BS_PERF_VEL] * DEG2RAD;
        float a = (float)(int16_t)modbus_registers[REG_BS_PERF_ACC] * DEG2RAD;
        if (v <= 0.0f) v = TRAJ_V_MAX;
        if (a <= 0.0f) a = TRAJ_V_MAX / TRAJ_ACCEL_TIME;
        Trapz_MoveToFull(&tm_trapz, from_rad, to_rad, v, a);
    } else {
        /* Precision: Septic time-scaled (jerk-continuous, ถึงเป้าเป๊ะ) */
        Septic_MoveTo(&tm_septic, from_rad, to_rad, TRAJ_MOVE_TIME);
    }
    tm_state = TM_MOVING;
}

/* ── Public API ──────────────────────────────────────────────────────────── */
void TestMode_Init(void)
{
    Trapz_Init(&tm_trapz);
    Septic_Init(&tm_septic);
    tm_state = TM_IDLE;
}

void TestMode_Reset(void)
{
    tm_trapz.is_running  = 0;
    tm_septic.is_running = 0;
    tm_state = TM_IDLE;
    _tm_telemetry(TASK_IDLE);
    Cascade_Control_Update(q_out, 0.0f);
}

void TestMode_Start(void)
{
    tm_type = (uint8_t)(modbus_registers[REG_BS_TEST_TYPE] & 0x0001);

    int16_t reg_init  = (int16_t)modbus_registers[REG_BS_PREC_INIT];
    int16_t reg_final = (int16_t)modbus_registers[REG_BS_PREC_FINAL];
    int16_t reg_rpt   = (int16_t)modbus_registers[REG_BS_PREC_RPT];

    uint8_t use_index = (reg_rpt < 0) ? 1U : 0U;
    uint8_t rep_count = (uint8_t)(reg_rpt < 0 ? -reg_rpt : reg_rpt);
    if (rep_count == 0) rep_count = 1;

    if (use_index) {
        float mag_a = (float)(reg_init  < 0 ? -reg_init  : reg_init)  * HOLE_STEP_DEG;
        float mag_b = (float)(reg_final < 0 ? -reg_final : reg_final) * HOLE_STEP_DEG;
        tm_pos_a = mag_a * DEG2RAD * (reg_init  < 0 ? -1.0f : 1.0f);
        tm_pos_b = mag_b * DEG2RAD * (reg_final < 0 ? -1.0f : 1.0f);
    } else {
        tm_pos_a = (float)reg_init  * DEG2RAD;
        tm_pos_b = (float)reg_final * DEG2RAD;
    }

    /* base "+" = CCW → กลับทิศให้ตรง firmware (BS_DIR_SIGN) */
    tm_pos_a *= BS_DIR_SIGN;
    tm_pos_b *= BS_DIR_SIGN;

    tm_trips     = 0;
    tm_trips_max = (uint8_t)(rep_count * 2U);
    tm_init_done = 0;

    _tm_start_move(q_out, tm_pos_a);
}

void TestMode_Update(void)
{
    if (modbus_registers[REG_BS_SOFT_STOP] & 0x0001) {
        TestMode_Reset();
        return;
    }

    switch (tm_state) {

        case TM_IDLE:
        case TM_DONE:
            _tm_telemetry(TASK_IDLE);
            Cascade_Control_Update(q_out, 0.0f);
            break;

        case TM_MOVING:
            if (tm_type == 1) {
                Trapz_Update(&tm_trapz,  &tm_ref_q, &tm_ref_qd, &tm_ref_qdd, &tm_ref_j);
            } else {
                Septic_Update(&tm_septic, &tm_ref_q, &tm_ref_qd, &tm_ref_qdd, &tm_ref_j);
            }
            Cascade_Control_Update_FF(tm_ref_q, tm_ref_qd, tm_ref_qdd);
            _tm_telemetry(TASK_GO_POINT);

            if (!(tm_type == 1 ? tm_trapz.is_running : tm_septic.is_running)) {
                Cascade_Flush_VelIntegral();
                if (!tm_init_done) {
                    /* เพิ่งถึง pos_a (positioning move) → เริ่ม trip แรก */
                    tm_init_done = 1;
                    _tm_start_move(tm_pos_a, tm_pos_b);
                    break;
                }

                tm_trips++;

                if (tm_type == 1) {
                    /* Performance: วิ่งต่อจน soft stop */
                    float next = (fabsf(tm_cur_end - tm_pos_a) < fabsf(tm_cur_end - tm_pos_b))
                                 ? tm_pos_b : tm_pos_a;
                    _tm_start_move(tm_cur_end, next);
                } else {
                    /* Precision: เช็ค done */
                    if (tm_trips >= tm_trips_max) {
                        tm_state = TM_DONE;
                    } else {
                        tm_hold_start = HAL_GetTick();
                        tm_state = TM_HOLD;
                    }
                }
            }
            break;

        case TM_HOLD:
            _tm_telemetry(TASK_IDLE);
            Cascade_Control_Update(tm_cur_end, 0.0f);

            if (HAL_GetTick() - tm_hold_start >= TM_DWELL_MS) {
                float next = (fabsf(tm_cur_end - tm_pos_a) < fabsf(tm_cur_end - tm_pos_b))
                             ? tm_pos_b : tm_pos_a;
                _tm_start_move(tm_cur_end, next);
            }
            break;

        default:
            tm_state = TM_IDLE;
            break;
    }
}
