/*
 * dashboard.c
 *
 * Manual Mode Dashboard Module
 * ─────────────────────────────────────────────────────────────────────────────
 * จัดการ Modbus Dashboard เมื่ออยู่ใน MODE_MANUAL:
 *
 *   Velocity Mode (REG_POS_KP == 0):
 *     สร้าง reference waveform → ส่งเข้า Cascade_Control_Update(0, ref_qd)
 *
 *   Position Mode (REG_POS_KP != 0):
 *     รับ target จาก REG_TARGET_POS (deg × 10, int16 signed)
 *     → ส่งเข้า Cascade_Control_Update(ref_q, 0)
 *     Drive Mode กำหนดจาก REG_DRIVE_MODE (0=Cascade, 1=Direct)
 *
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include "dashboard.h"
#include "base_system.h"       /* REG_xxx defines, modbus_registers[], current_system_mode */
#include "cascade_control.h"   /* Cascade_Control_Update(), q_out, qd_out, monitor_V_in    */
#include "kalman_filter.h"     /* KalmanFilter_t                                           */
#include "trajectory.h"        /* Septic_Profile_t — smooth position targeting             */
#include <math.h>              /* sinf(), fabsf()                                          */

/* ─── External state ────────────────────────────────────────────────────────── */
extern KalmanFilter_t  hkf;           /* defined in cascade_control.c */
extern volatile float  q_out;         /* actual position  [rad]        */
extern volatile float  qd_out;        /* actual velocity  [rad/s]      */
extern volatile float  monitor_V_in;  /* motor voltage magnitude [V]   */

/* ─── Default values (fallback เมื่อ PC ยังไม่ได้เขียน register) ─────────────── */
#define DASH_DEFAULT_SPEED      3.0f   /* rad/s          (velocity mode) */
#define DASH_DEFAULT_HALF_T     3.0f   /* seconds                        */

/* ─── Waveform codes (ตรงกับ REG_WAVEFORM) ──────────────────────────────────── */
#define WAVE_SQUARE  0
#define WAVE_SINE    1
#define WAVE_STEP    2

/* ─── Unit conversion ────────────────────────────────────────────────────────── */
#define DEG2RAD  (3.14159265f / 180.0f)

/* ─── Internal state ─────────────────────────────────────────────────────────── */
static float dash_ref_qd  = 0.0f;   /* velocity reference [rad/s]                */
static float dash_ref_q   = 0.0f;   /* position reference [rad]                  */
static float dash_t       = 0.0f;   /* internal waveform timer [s]               */

/* ─── Position mode: Septic trajectory (jerk-continuous, ตรงกับ auto mission) ── */
static Septic_Profile_t dash_pos_septic;        /* smooth trajectory to target    */
static float            dash_pos_tgt_prev = 1e10f; /* last target (sentinel init) */


/* ─────────────────────────────────────────────────────────────────────────────
 *  Dashboard_Init
 *  เรียกครั้งเดียวใน main() → USER CODE BEGIN 2  (หลัง Cascade_Control_Init)
 * ───────────────────────────────────────────────────────────────────────────*/
void Dashboard_Init(void)
{
    dash_ref_qd      = 0.0f;
    dash_ref_q       = 0.0f;
    dash_t           = 0.0f;
    dash_pos_tgt_prev = 1e10f;    /* sentinel — ทุก target จะ trigger Septic ครั้งแรก */
    Septic_Init(&dash_pos_septic);

    /* ตั้งค่า default: Stop ก่อน — ผู้ใช้กด Start เองจาก Dashboard */
    modbus_registers[REG_RUN]      = 0;   /* run flag  = Stop   */
    modbus_registers[REG_WAVEFORM] = 0;   /* waveform  = Square */
    modbus_registers[REG_DRIVE_MODE] = 0; /* drive     = Cascade */

    /* ── Pre-initialize Velocity PID registers ด้วยค่า default ──────────────
     * ป้องกัน ISR อ่านค่า 0 แล้ว guard != 0 ไม่ update → gain ค้างที่ผิด
     * ค่าต้องตรงกับ cascade_control.c: vel_ctrl = {Kp=2.5, Ki=0.5, Kd=0.0}
     * ────────────────────────────────────────────────────────────────────── */
    modbus_registers[REG_VEL_KP] = (uint16_t)(int16_t)(2.5f * 100.0f);   /* 250 */
    modbus_registers[REG_VEL_KI] = (uint16_t)(int16_t)(0.5f * 100.0f);   /* 50  */
    modbus_registers[REG_VEL_KD] = 0;                                      /* 0   */

    /* REG_POS_KP = 0 → velocity-only mode on boot (pos loop disabled)
     * ผู้ใช้ Apply Pos PID จาก Dashboard เพื่อเปิด position loop          */
    modbus_registers[REG_POS_KP] = 0;
    modbus_registers[REG_POS_KI] = 0;
    modbus_registers[REG_POS_KD] = 0;
}


/* ─────────────────────────────────────────────────────────────────────────────
 *  _dash_stop_motor  (private helper)
 *  หยุดมอเตอร์และรีเซ็ต state ภายใน — เรียกเมื่อ Stop หรือออกจาก Manual mode
 * ───────────────────────────────────────────────────────────────────────────*/
static void _dash_stop_motor(void)
{
    dash_ref_qd       = 0.0f;
    dash_ref_q        = 0.0f;
    dash_t            = 0.0f;
    dash_pos_septic.is_running = 0;
    dash_pos_tgt_prev = 1e10f;   /* reset sentinel → ทุก target ใหม่จะ trigger Septic */
    Cascade_Control_Update(0.0f, 0.0f);

    /* ── Telemetry: ส่งข้อมูลปัจจุบันแม้จะหยุดแล้ว ── */
    modbus_registers[REG_REF_QD] = 0;
    modbus_registers[REG_QD_OUT] = (uint16_t)(int16_t)(qd_out          * 100.0f);
    modbus_registers[REG_V_IN]   = (uint16_t)(int16_t)(monitor_V_in    * 100.0f);
    modbus_registers[REG_Q_OUT]  = (uint16_t)(int16_t)(q_out           * 100.0f);
    modbus_registers[REG_EST_I]  = (uint16_t)(int16_t)(hkf.est_current * 1000.0f);
    modbus_registers[REG_REF_Q]  = 0;

    modbus_registers[REG_BS_POS]  = (uint16_t)(int16_t)(q_out  * (180.0f / 3.14159265f) * 10.0f);
    modbus_registers[REG_BS_VEL]  = (uint16_t)(int16_t)(qd_out * (180.0f / 3.14159265f) * 10.0f);
    modbus_registers[REG_BS_ACC]  = 0;
    modbus_registers[REG_BS_TASK] = TASK_IDLE;
}


/* ─────────────────────────────────────────────────────────────────────────────
 *  Dashboard_Update
 *  เรียกทุก 1 ms จาก HAL_TIM_PeriodElapsedCallback (TIM6)
 *  ── ทำงานเฉพาะเมื่อ current_system_mode == MODE_MANUAL ──
 * ───────────────────────────────────────────────────────────────────────────*/
void Dashboard_Update(void)
{
    /* ── 0. Mode guard ── */
    if (current_system_mode != MODE_MANUAL) {
        _dash_stop_motor();
        return;
    }

    /* ── 1. อ่านพารามิเตอร์จาก Modbus ── */
    uint16_t wave     = modbus_registers[REG_WAVEFORM];   /* 0=Square 1=Sine 2=Step */
    uint16_t run      = modbus_registers[REG_RUN];        /* 0=Stop   1=Run         */
    uint8_t  pos_mode = (modbus_registers[REG_POS_KP] != 0);

    float half_t = (modbus_registers[REG_HALF_PERIOD] != 0)
                   ? (float)modbus_registers[REG_HALF_PERIOD] / 1000.0f
                   : DASH_DEFAULT_HALF_T;

    /* ── 2. Stop: หยุดมอเตอร์ทันที รีเซ็ต timer ── */
    if (!run) {
        _dash_stop_motor();
        return;
    }

    /* ── 3. สร้าง Reference ──
     * หมายเหตุ: PID gain update (REG_VEL_KP … REG_POS_KD) ทำภายใน
     * Cascade_Control_Update() อัตโนมัติ — ไม่ต้องทำซ้ำที่นี่ */
    if (pos_mode) {
        /* ════ POSITION MODE ════
         * REG_TARGET_POS = deg × 10  (int16 signed)
         *   e.g.  90.0° → 900,  -45.5° → -455
         * ใช้ Septic trajectory (jerk-continuous) แทน step command
         */
        float target_deg = (float)(int16_t)modbus_registers[REG_TARGET_POS] / 10.0f;
        float target_rad = target_deg * DEG2RAD;

        /* เมื่อ target เปลี่ยน → เริ่ม Septic ใหม่จากตำแหน่งปัจจุบัน */
        if (fabsf(target_rad - dash_pos_tgt_prev) > 0.001f) {
            Septic_MoveTo(&dash_pos_septic, q_out, target_rad, TRAJ_MOVE_TIME);
            dash_pos_tgt_prev = target_rad;
        }

        /* อัปเดต trajectory ทุก tick */
        float traj_qd = 0.0f, traj_qdd = 0.0f, traj_j;
        if (dash_pos_septic.is_running) {
            Septic_Update(&dash_pos_septic, &dash_ref_q, &traj_qd, &traj_qdd, &traj_j);
            dash_ref_qd = traj_qd;   /* feedforward velocity จาก trajectory */
        } else {
            dash_ref_q  = target_rad;
            dash_ref_qd = 0.0f;      /* ถึง target แล้ว: หยุด feedforward   */
        }

        /* 2-DOF: ส่ง q̈_ref เข้า acceleration feedforward (ตรงกับ auto mission) */
        Cascade_Control_Update_FF(dash_ref_q, dash_ref_qd, traj_qdd);

    } else {
        /* ════ VELOCITY MODE ════
         * REG_SPEED = rad/s × 10  (e.g. 3 rad/s → 30)
         */

        /* Advance waveform timer */
        float period = half_t * 2.0f;
        dash_t += 0.001f;
        if (dash_t >= period) dash_t -= period;

        float speed = (modbus_registers[REG_SPEED] != 0)
                      ? (float)(int16_t)modbus_registers[REG_SPEED] / 10.0f
                      : DASH_DEFAULT_SPEED;

        switch (wave) {
            case WAVE_SINE:
                dash_ref_qd = speed * sinf(2.0f * 3.14159265f * dash_t / period);
                break;
            case WAVE_STEP:
                dash_ref_qd = speed;
                break;
            case WAVE_SQUARE:
            default:
                dash_ref_qd = (dash_t < half_t) ? speed : -speed;
                break;
        }
        dash_ref_q = 0.0f;

        Cascade_Control_Update(0.0f, dash_ref_qd);
    }

    /* ── 6. Telemetry → Modbus ──────────────────────────────────────────────
     * REG_REF_QD  ref_qd      × 100   [int16, rad/s]  velocity reference
     * REG_QD_OUT  qd_out      × 100   [int16, rad/s]  actual velocity
     * REG_V_IN    V_in (mag)  × 100   [int16, V]      motor voltage
     * REG_Q_OUT   q_out       × 100   [int16, rad]    actual position
     * REG_EST_I   est_current × 1000  [int16, A]      estimated current
     * REG_REF_Q   ref_q       × 100   [int16, rad]    position reference
     * ─────────────────────────────────────────────────────────────────────*/
    modbus_registers[REG_REF_QD] = (uint16_t)(int16_t)(dash_ref_qd        * 100.0f);
    modbus_registers[REG_QD_OUT] = (uint16_t)(int16_t)(qd_out             * 100.0f);
    modbus_registers[REG_V_IN]   = (uint16_t)(int16_t)(monitor_V_in       * 100.0f);
    modbus_registers[REG_Q_OUT]  = (uint16_t)(int16_t)(q_out              * 100.0f);
    modbus_registers[REG_EST_I]  = (uint16_t)(int16_t)(hkf.est_current    * 1000.0f);
    modbus_registers[REG_REF_Q]  = (uint16_t)(int16_t)(dash_ref_q         * 100.0f);

    modbus_registers[REG_BS_POS]  = (uint16_t)(int16_t)(q_out  * (180.0f / 3.14159265f) * 10.0f);
    modbus_registers[REG_BS_VEL]  = (uint16_t)(int16_t)(qd_out * (180.0f / 3.14159265f) * 10.0f);
    modbus_registers[REG_BS_ACC]  = 0;
    modbus_registers[REG_BS_TASK] = TASK_IDLE;
}
