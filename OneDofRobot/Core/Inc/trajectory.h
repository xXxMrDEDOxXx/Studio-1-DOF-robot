/*
 * trajectory.h
 *
 *  Created on: 29 เม.ย. 2569
 *      Author: POND
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "main.h"
#include "encoder.h"
#include <stdint.h>

// ─────────────────────────────────────────────────────────────────────────────
//  Trajectory Parameters  (θ_mission = 5° = 0.0873 rad)
//
//  หมายเหตุ: Auto Pick&Place + Test Precision ใช้ Septic time-scaled (TRAJ_MOVE_TIME)
//  ค่าด้านล่างใช้โดย S-Curve และ Trapz (เช่น Test Performance):
//
//  S-Curve  (smooth, jerk-limited):
//    Tj = A_MAX / J_MAX      = 6.4/15  = 0.427s
//    Ta = V_MAX / A_MAX − Tj = 3.2/6.4 − 0.427 = 0.073s
//
//  Trapz    (faster, no load):
//    a_calc  = V_MAX / ACCEL_TIME = 3.2/0.45 ≈ 7.11 rad/s²
// ─────────────────────────────────────────────────────────────────────────────
#define TRAJ_V_MAX      3.2f    /* ความเร็วสูงสุด [rad/s]  (hardware limit 4.2) */
#define TRAJ_A_MAX      6.4f    /* ความเร่งสูงสุด [rad/s²] ใช้โดย S-Curve       */
#define TRAJ_J_MAX      15.0f   /* ความกระชากสูงสุด [rad/s³] ใช้โดย S-Curve     */
#define TRAJ_ACCEL_TIME 0.45f    /* เวลาเร่ง 0→V_MAX [s]    ใช้โดย Trapz เท่านั้น */

/* ── Quintic time-scaled profile (Option B: per-move time-scaling) ───────────
 *  q(t) = q0 + d·(10τ³−15τ⁴+6τ⁵),  τ=t/T   (minimum-jerk, ถึงเป้าเป๊ะที่ t=T)
 *  - ทุก move ตั้งเป้าจบใน TRAJ_MOVE_TIME (ถ้าระยะสั้นพอ)
 *  - feasibility cap: ถ้า v_peak/a_peak เกิน hardware → ยืด T อัตโนมัติ
 *      v_peak = 1.875·d/T      a_peak = 5.7735·d/T²
 *  - ถึง q_end พอดีที่ t=T → ไม่มี undershoot (ต่างจาก S-Curve)
 * ─────────────────────────────────────────────────────────────────────────── */
#define TRAJ_QV_MAX     3.5f    /* เพดานความเร็ว [rad/s] — จำกัดด้วย V_FF!
                                 *   V_FF = K_ff·qd = 5.4·qd; ที่ 3.5 = 18.9V (เหลือ
                                 *   headroom ~5V ให้ feedback). เดิม 4.5 → V_FF=24V
                                 *   = saturate เต็ม → control พัง → กระตุก move ใหญ่ */
#define TRAJ_QA_MAX     12.0f   /* เพดานความเร่ง [rad/s²] — V_acc=0.768·a; 12 = 9.2V
                                 *   (เดิม 15 = 11.5V) กัน accel saturate รวมกับ V_FF */
#define TRAJ_MOVE_TIME  0.5f    /* เวลาเป้าหมายต่อ move [s] (settling spec)         */


// =================================================================
// 1. Trapezoidal Profile
// =================================================================
typedef struct {
    float q_start;
    float q_end;
    float v_cruise;
    float a_calc;       // อัตราเร่งที่คำนวณจากเวลา
    float t_a;
    float t_c;
    float t_f;
    float sign;

    float current_t;
    uint8_t is_running;
} Trapz_Profile_t;

void Trapz_Init(Trapz_Profile_t *prof);
void Trapz_MoveTo(Trapz_Profile_t *prof, float start_pos, float target_pos);
void Trapz_MoveToFull(Trapz_Profile_t *prof, float start_pos, float target_pos,
                      float v_max, float accel);
void Trapz_Update(Trapz_Profile_t *prof, float *out_q, float *out_qd, float *out_qdd, float *out_j);


// =================================================================
// 2. S-Curve Profile
// =================================================================
typedef struct {
    float q_start, q_end, sign;
    float current_t;
    int is_running;

    // ช่วงเวลาทั้ง 7 (Time Segments)
    float t1, t2, t3, t4, t5, t6, t7;

    // ตัวแปรสะสมค่าแบบ Real-time
    float q_cmd, v_cmd, a_cmd;
    float prev_a_cmd; // ไว้ใช้หาค่า Jerk จากการ Diff

    float j_max_eff;  /* jerk สูงสุดที่ใช้จริง — SCurve_MoveTo ตั้ง TRAJ_J_MAX */
} SCurve_Profile_t;

void SCurve_Init(SCurve_Profile_t *prof);
void SCurve_MoveTo(SCurve_Profile_t *prof, float start_pos, float target_pos);
void SCurve_MoveToFull(SCurve_Profile_t *prof, float start_pos, float target_pos,
                       float v_max, float a_max, float j_max);
void SCurve_Update(SCurve_Profile_t *prof, float *out_q, float *out_qd, float *out_qdd, float *out_j);


// =================================================================
// 3. Quintic Time-Scaled Profile  (Option B — per-move time-scaling)
// =================================================================
typedef struct {
    float   q_start;
    float   q_end;
    float   d;            /* q_end − q_start (signed)        */
    float   T;            /* duration จริงหลัง feasibility cap [s] */
    float   current_t;
    uint8_t is_running;
} Quintic_Profile_t;

void Quintic_Init(Quintic_Profile_t *prof);
/* MoveTo: ตั้งเป้าจบใน T_target (จะถูกยืดถ้า v/a เกิน hardware) */
void Quintic_MoveTo(Quintic_Profile_t *prof, float start_pos, float target_pos,
                    float T_target);
void Quintic_Update(Quintic_Profile_t *prof, float *out_q, float *out_qd,
                    float *out_qdd, float *out_j);


// =================================================================
// 4. Septic (7th-order) Time-Scaled Profile  — jerk-continuous
// =================================================================
//  q(t) = q0 + d·(35τ⁴ − 84τ⁵ + 70τ⁶ − 20τ⁷),  τ = t/T
//   - q, q̇, q̈, q⃛ = 0 ที่ปลายทั้งสองด้าน → jerk ไม่กระโดด (เนียนสุด)
//   - เหมาะกับ carrying rod / ลดการกระตุ้น backlash
//   - peak: v=2.1875·d/T   a=7.5117·d/T²   (สูงกว่า Quintic เล็กน้อย)
//   - ถึง q_end พอดีที่ t=T → ไม่มี undershoot
typedef struct {
    float   q_start;
    float   q_end;
    float   d;
    float   T;
    float   current_t;
    uint8_t is_running;
} Septic_Profile_t;

void Septic_Init(Septic_Profile_t *prof);
void Septic_MoveTo(Septic_Profile_t *prof, float start_pos, float target_pos,
                   float T_target);
void Septic_Update(Septic_Profile_t *prof, float *out_q, float *out_qd,
                   float *out_qdd, float *out_j);

#endif /* INC_TRAJECTORY_H_ */
