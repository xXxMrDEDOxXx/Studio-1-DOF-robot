/*
 * trajectory.c
 *
 *  Created on: 29 เม.ย. 2569
 *      Author: POND
 */


#include "trajectory.h"
#include "cascade_control.h" // จำเป็นต้อง Include เพื่อดึงค่า PI และ q_out มาใช้งาน
#include <math.h>

// =================================================================
// 1. Trapezoidal Profile
// =================================================================

void Trapz_Init(Trapz_Profile_t *prof) {
    prof->is_running = 0;
}

void Trapz_MoveTo(Trapz_Profile_t *prof, float start_pos, float target_pos) {
    prof->q_start = start_pos;
    prof->q_end = target_pos;
    prof->current_t = 0.0f;

    float distance = fabs(target_pos - start_pos);
    prof->sign = (target_pos > start_pos) ? 1.0f : -1.0f;

    if (distance == 0.0f) {
        prof->is_running = 0;
        return;
    }

    // แปลง "เวลา (ACCEL_TIME)" เป็น "อัตราเร่ง"
    prof->a_calc = TRAJ_V_MAX / TRAJ_ACCEL_TIME;

    float a_max = prof->a_calc;
    float d_a = 0.5f * (TRAJ_V_MAX * TRAJ_V_MAX) / a_max;

    if (2.0f * d_a >= distance) { // Triangle (วิ่งไม่ถึง V_max)
        float calc_t_a = sqrtf(distance / a_max);
        prof->v_cruise = a_max * calc_t_a;
        prof->t_a = calc_t_a;
        prof->t_c = 0.0f;
    } else { // Trapezoid (วิ่งถึง V_max)
        prof->v_cruise = TRAJ_V_MAX;
        prof->t_a = TRAJ_ACCEL_TIME; // ใช้เวลาที่กำหนดได้เลย
        prof->t_c = (distance - (2.0f * d_a)) / TRAJ_V_MAX;
    }

    prof->t_f = (2.0f * prof->t_a) + prof->t_c;
    prof->is_running = 1;
}

void Trapz_MoveToFull(Trapz_Profile_t *prof, float start_pos, float target_pos,
                      float v_max, float accel)
{
    if (v_max <= 0.0f || accel <= 0.0f) {
        Trapz_MoveTo(prof, start_pos, target_pos);
        return;
    }
    prof->q_start   = start_pos;
    prof->q_end     = target_pos;
    prof->current_t = 0.0f;

    float distance = fabs(target_pos - start_pos);
    prof->sign = (target_pos > start_pos) ? 1.0f : -1.0f;

    if (distance == 0.0f) { prof->is_running = 0; return; }

    prof->a_calc = accel;
    float d_a    = 0.5f * (v_max * v_max) / accel;

    if (2.0f * d_a >= distance) {
        float calc_t_a  = sqrtf(distance / accel);
        prof->v_cruise  = accel * calc_t_a;
        prof->t_a       = calc_t_a;
        prof->t_c       = 0.0f;
    } else {
        prof->v_cruise  = v_max;
        prof->t_a       = v_max / accel;
        prof->t_c       = (distance - 2.0f * d_a) / v_max;
    }
    prof->t_f       = 2.0f * prof->t_a + prof->t_c;
    prof->is_running = 1;
}

void Trapz_Update(Trapz_Profile_t *prof, float *out_q, float *out_qd, float *out_qdd, float *out_j) {
    if (!prof->is_running) {
        *out_q = prof->q_end;
        *out_qd = 0.0f;
        *out_qdd = 0.0f;
        return;
    }

    prof->current_t += 0.001f;
    float t = prof->current_t;
    float a_max = prof->a_calc;

    if (t <= prof->t_a) {
        *out_q = prof->q_start + prof->sign * (0.5f * a_max * t * t);
        *out_qd = prof->sign * (a_max * t);
        *out_qdd = prof->sign * a_max;
    }
    else if (t <= (prof->t_a + prof->t_c)) {
        float t_cruise = t - prof->t_a;
        float d_a = 0.5f * prof->v_cruise * prof->t_a;
        *out_q = prof->q_start + prof->sign * (d_a + (prof->v_cruise * t_cruise));
        *out_qd = prof->sign * prof->v_cruise;
        *out_qdd = 0.0f;
    }
    else if (t < prof->t_f) {
        float t_dec = t - (prof->t_a + prof->t_c);
        float d_a = 0.5f * prof->v_cruise * prof->t_a;
        float d_c = prof->v_cruise * prof->t_c;
        *out_q = prof->q_start + prof->sign * (d_a + d_c + (prof->v_cruise * t_dec) - (0.5f * a_max * t_dec * t_dec));
        *out_qd = prof->sign * (prof->v_cruise - (a_max * t_dec));
        *out_qdd = prof->sign * -a_max;
    }
    else {
        *out_q = prof->q_end;
        *out_qd = 0.0f;
        *out_qdd = 0.0f;
        prof->is_running = 0;
    }
}


// =================================================================
// 2. S-Curve Profile
// =================================================================

void SCurve_Init(SCurve_Profile_t *prof) {
    prof->is_running = 0;
    prof->current_t = 0.0f;
    prof->q_start = 0.0f;
    prof->q_end = 0.0f;
    prof->q_cmd = 0.0f;
    prof->v_cmd = 0.0f;
    prof->a_cmd = 0.0f;
    prof->prev_a_cmd = 0.0f;
}

/* ── helper: คำนวณ t1..t7 จาก v_max, a_max, j_max และระยะทาง ── */
static void _scurve_compute_segments(SCurve_Profile_t *prof,
                                     float distance,
                                     float v_max, float a_max, float j_max)
{
    prof->j_max_eff = j_max;

    float Tj = a_max / j_max;
    float Ta = (v_max / a_max) - Tj;
    float Tv = (distance / v_max) - (Ta + 2.0f * Tj);

    if (Tv < 0.0f) {
        Tv = 0.0f;
        float a_quad = 1.0f / a_max;
        float b_quad = Tj;
        float c_quad = -distance;
        float V_new  = (-b_quad + sqrtf(b_quad*b_quad - 4.0f*a_quad*c_quad)) / (2.0f*a_quad);
        Ta = (V_new / a_max) - Tj;
        if (Ta < 0.0f) {
            Ta = 0.0f;
            Tj = cbrtf(distance / (2.0f * j_max));
            prof->j_max_eff = j_max;  /* Tj updated แต่ j_max ยังเดิม */
        }
    }

    prof->t1 = Tj;
    prof->t2 = Tj + Ta;
    prof->t3 = Tj + Ta + Tj;
    prof->t4 = prof->t3 + Tv;
    prof->t5 = prof->t4 + Tj;
    prof->t6 = prof->t5 + Ta;
    prof->t7 = prof->t6 + Tj;
}

void SCurve_MoveTo(SCurve_Profile_t *prof, float start_pos, float target_pos) {
    prof->q_start    = start_pos;
    prof->q_end      = target_pos;
    prof->current_t  = 0.0f;
    prof->is_running = 1;
    prof->q_cmd      = start_pos;
    prof->v_cmd      = 0.0f;
    prof->a_cmd      = 0.0f;
    prof->prev_a_cmd = 0.0f;

    float distance = fabs(target_pos - start_pos);
    prof->sign = (target_pos >= start_pos) ? 1.0f : -1.0f;

    if (distance == 0.0f) { prof->is_running = 0; return; }

    _scurve_compute_segments(prof, distance, TRAJ_V_MAX, TRAJ_A_MAX, TRAJ_J_MAX);
}

void SCurve_MoveToFull(SCurve_Profile_t *prof, float start_pos, float target_pos,
                       float v_max, float a_max, float j_max)
{
    if (v_max <= 0.0f || a_max <= 0.0f || j_max <= 0.0f) {
        SCurve_MoveTo(prof, start_pos, target_pos);
        return;
    }
    prof->q_start    = start_pos;
    prof->q_end      = target_pos;
    prof->current_t  = 0.0f;
    prof->is_running = 1;
    prof->q_cmd      = start_pos;
    prof->v_cmd      = 0.0f;
    prof->a_cmd      = 0.0f;
    prof->prev_a_cmd = 0.0f;

    float distance = fabs(target_pos - start_pos);
    prof->sign = (target_pos >= start_pos) ? 1.0f : -1.0f;

    if (distance == 0.0f) { prof->is_running = 0; return; }

    _scurve_compute_segments(prof, distance, v_max, a_max, j_max);
}

void SCurve_Update(SCurve_Profile_t *prof, float *out_q, float *out_qd, float *out_qdd, float *out_j) {
    // 1. ถ้าระบบไม่ได้รัน ให้ล็อกค่าเป้าหมายสุดท้ายไว้นิ่งๆ (ป้องกัน Stick-slip)
	if (!prof->is_running) {
	        // ให้พิกัดเป้าหมายค่อยๆ ไหลเข้าหาจุดจบแบบล่องหน (ป้องกันการวาร์ปข้ามทศนิยม)
	        float q_err = prof->q_end - prof->q_cmd;
	        if (fabs(q_err) > 0.0001f) {
	            prof->q_cmd += 0.01f * q_err; // ค่อยๆ ซึมเข้าหาเป้า
	        } else {
	            prof->q_cmd = prof->q_end; // ล็อกเมื่อสนิทแล้ว
	        }

	        *out_q = prof->q_cmd;
	        *out_qd = 0.0f;
	        *out_qdd = 0.0f;
	        *out_j = 0.0f;
	        return;
	    }

    prof->current_t += 0.001f; // DT = 1ms
    float t   = prof->current_t;
    float Jm  = prof->j_max_eff;   /* ใช้ค่าจาก instance ไม่ใช่ hardcode */
    float J   = 0.0f;

    // 2. กำหนดค่า Jerk ตามช่วงเวลา
    if      (t <= prof->t1) J =  Jm;
    else if (t <= prof->t2) J =  0.0f;
    else if (t <= prof->t3) J = -Jm;
    else if (t <= prof->t4) J =  0.0f;
    else if (t <= prof->t5) J = -Jm;
    else if (t <= prof->t6) J =  0.0f;
    else if (t <= prof->t7) J =  Jm;

    // 3. กระบวนการ Integrate และระบบ ป้องกันอาการกระตุกตอนจบ (Anti-Snap)
    if (t <= prof->t7) {
        float signed_J = J * prof->sign;
        prof->a_cmd += signed_J * 0.001f;
        prof->v_cmd += prof->a_cmd * 0.001f;

        // ✨ THE MAGIC FIX: Zero-Crossing Detection ✨
        // ในช่วงเบรกสุดท้าย (t > t6) ความเร็วจะวิ่งลงมาหา 0
        // ถ้าทศนิยมพาความเร็วทะลุ 0 ไปอีกฝั่ง ให้ตัดจบโปรไฟล์อย่างนิ่มนวลทันที!
        if (t > prof->t6) {
            if ((prof->sign > 0.0f && prof->v_cmd <= 0.0f) ||
                (prof->sign < 0.0f && prof->v_cmd >= 0.0f)) {

                prof->is_running = 0;
                /* ไม่ jump q_cmd ไป q_end ทันที — ปล่อยให้ drift code
                 * ค่อยๆ พา q_cmd → q_end ใน not-running section
                 * เพื่อกำจัด position reference spike ที่ทำให้กระตุก */
                prof->v_cmd = 0.0f;
                prof->a_cmd = 0.0f;
            }
        }

        // ถ้ายังไม่โดนตัดจบ ก็สะสมตำแหน่งต่อไป
        if (prof->is_running) {
            prof->q_cmd += prof->v_cmd * 0.001f;
        }

    } else {
        // กรณีเผื่อเหลือเผื่อขาด ถ้าเวลาเกิน t7 ไปแล้ว
        prof->is_running = 0;
        prof->q_cmd = prof->q_end;
        prof->v_cmd = 0.0f;
        prof->a_cmd = 0.0f;
    }

    // 4. บันทึกค่า a เพื่อคำนวณ Jerk ออกไปพล็อตกราฟ
    float derived_jerk = (prof->a_cmd - prof->prev_a_cmd) / 0.001f;
    prof->prev_a_cmd = prof->a_cmd;

    // 5. ส่งค่าออก
    *out_q = prof->q_cmd;
    *out_qd = prof->v_cmd;
    *out_qdd = prof->a_cmd;
    *out_j = derived_jerk;
}


// =================================================================
// 3. Quintic Time-Scaled Profile  (Option B — per-move time-scaling)
// =================================================================
//
//  s(τ)    = 10τ³ − 15τ⁴ + 6τ⁵          (τ = t/T)
//  s'(τ)   = 30τ² − 60τ³ + 30τ⁴   → v = d·s'/T
//  s''(τ)  = 60τ − 180τ² + 120τ³  → a = d·s''/T²
//  s'''(τ) = 60 − 360τ + 360τ²    → j = d·s'''/T³
//
//  peak: v=1.875·d/T  a=5.7735·d/T²  (ใช้ตั้ง feasibility cap)
//  q(0)=q0, q(T)=q_end เป๊ะ — ไม่มี undershoot
// =================================================================

#define _Q_KV   1.875f      /* v_peak = KV·d/T   */
#define _Q_KA   5.7735f     /* a_peak = KA·d/T²  */

void Quintic_Init(Quintic_Profile_t *prof) {
    prof->is_running = 0;
    prof->current_t  = 0.0f;
    prof->q_start = prof->q_end = prof->d = prof->T = 0.0f;
}

void Quintic_MoveTo(Quintic_Profile_t *prof, float start_pos, float target_pos,
                    float T_target)
{
    prof->q_start   = start_pos;
    prof->q_end     = target_pos;
    prof->d         = target_pos - start_pos;   /* signed */
    prof->current_t = 0.0f;

    float dist = fabsf(prof->d);
    if (dist < 1e-6f) {       /* ไม่ขยับ */
        prof->T = 0.0f;
        prof->is_running = 0;
        return;
    }

    /* ── feasibility cap: ยืด T ถ้า v_peak / a_peak เกิน hardware ── */
    float T  = (T_target > 0.001f) ? T_target : TRAJ_MOVE_TIME;
    float Tv = _Q_KV * dist / TRAJ_QV_MAX;             /* กัน v เกิน  */
    float Ta = sqrtf(_Q_KA * dist / TRAJ_QA_MAX);      /* กัน a เกิน  */
    if (Tv > T) T = Tv;
    if (Ta > T) T = Ta;

    prof->T = T;
    prof->is_running = 1;
}

void Quintic_Update(Quintic_Profile_t *prof, float *out_q, float *out_qd,
                    float *out_qdd, float *out_j)
{
    if (!prof->is_running) {
        *out_q   = prof->q_end;
        *out_qd  = 0.0f;
        *out_qdd = 0.0f;
        *out_j   = 0.0f;
        return;
    }

    prof->current_t += 0.001f;          /* DT = 1 ms */
    float T = prof->T;
    float t = prof->current_t;

    if (t >= T) {                        /* ถึงเป้าพอดี → จบ */
        prof->is_running = 0;
        *out_q   = prof->q_end;
        *out_qd  = 0.0f;
        *out_qdd = 0.0f;
        *out_j   = 0.0f;
        return;
    }

    float d   = prof->d;                 /* signed distance */
    float invT = 1.0f / T;
    float tau  = t * invT;
    float tau2 = tau  * tau;
    float tau3 = tau2 * tau;
    float tau4 = tau3 * tau;
    float tau5 = tau4 * tau;

    float s    = 10.0f*tau3 - 15.0f*tau4 + 6.0f*tau5;
    float sd   = (30.0f*tau2 - 60.0f*tau3 + 30.0f*tau4) * invT;
    float sdd  = (60.0f*tau  - 180.0f*tau2 + 120.0f*tau3) * invT * invT;
    float sddd = (60.0f - 360.0f*tau + 360.0f*tau2) * invT * invT * invT;

    *out_q   = prof->q_start + d * s;
    *out_qd  = d * sd;
    *out_qdd = d * sdd;
    *out_j   = d * sddd;
}


// =================================================================
// 4. Septic (7th-order) Time-Scaled Profile  — jerk-continuous
// =================================================================
//  s(τ)    = 35τ⁴ − 84τ⁵ + 70τ⁶ − 20τ⁷
//  s'(τ)   = 140τ³ − 420τ⁴ + 420τ⁵ − 140τ⁶
//  s''(τ)  = 420τ² − 1680τ³ + 2100τ⁴ − 840τ⁵
//  s'''(τ) = 840τ − 5040τ² + 8400τ³ − 4200τ⁴
//  peak: v=2.1875·d/T  a=7.5117·d/T²   (q⃛=0 ที่ปลาย → jerk ไม่กระโดด)
// =================================================================

#define _S_KV   2.1875f     /* v_peak = KV·d/T   */
#define _S_KA   7.5117f     /* a_peak = KA·d/T²  */

void Septic_Init(Septic_Profile_t *prof) {
    prof->is_running = 0;
    prof->current_t  = 0.0f;
    prof->q_start = prof->q_end = prof->d = prof->T = 0.0f;
}

void Septic_MoveTo(Septic_Profile_t *prof, float start_pos, float target_pos,
                   float T_target)
{
    prof->q_start   = start_pos;
    prof->q_end     = target_pos;
    prof->d         = target_pos - start_pos;
    prof->current_t = 0.0f;

    float dist = fabsf(prof->d);
    if (dist < 1e-6f) {
        prof->T = 0.0f;
        prof->is_running = 0;
        return;
    }

    /* feasibility cap: ยืด T ถ้า v_peak/a_peak เกิน hardware */
    float T  = (T_target > 0.001f) ? T_target : TRAJ_MOVE_TIME;
    float Tv = _S_KV * dist / TRAJ_QV_MAX;
    float Ta = sqrtf(_S_KA * dist / TRAJ_QA_MAX);
    if (Tv > T) T = Tv;
    if (Ta > T) T = Ta;

    prof->T = T;
    prof->is_running = 1;
}

void Septic_Update(Septic_Profile_t *prof, float *out_q, float *out_qd,
                   float *out_qdd, float *out_j)
{
    if (!prof->is_running) {
        *out_q = prof->q_end; *out_qd = 0.0f; *out_qdd = 0.0f; *out_j = 0.0f;
        return;
    }

    prof->current_t += 0.001f;
    float T = prof->T;
    float t = prof->current_t;

    if (t >= T) {
        prof->is_running = 0;
        *out_q = prof->q_end; *out_qd = 0.0f; *out_qdd = 0.0f; *out_j = 0.0f;
        return;
    }

    float d    = prof->d;
    float invT = 1.0f / T;
    float x    = t * invT;
    float x2=x*x, x3=x2*x, x4=x3*x, x5=x4*x, x6=x5*x, x7=x6*x;

    float s    = 35.0f*x4 - 84.0f*x5 + 70.0f*x6 - 20.0f*x7;
    float sd   = (140.0f*x3 - 420.0f*x4 + 420.0f*x5 - 140.0f*x6) * invT;
    float sdd  = (420.0f*x2 - 1680.0f*x3 + 2100.0f*x4 - 840.0f*x5) * invT * invT;
    float sddd = (840.0f*x - 5040.0f*x2 + 8400.0f*x3 - 4200.0f*x4) * invT * invT * invT;

    *out_q   = prof->q_start + d * s;
    *out_qd  = d * sd;
    *out_qdd = d * sdd;
    *out_j   = d * sddd;
}
