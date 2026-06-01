/*
 * kalman_filter.c
 *
 *  Created on: 18 พ.ค. 2569
 *      Author: POND
 *
 * =======================================================================
 *  Full-order Discrete Kalman Filter -- 1-DOF Robot
 * =======================================================================
 *
 *  CONTINUOUS-TIME STATE SPACE  (ตรงกับ state space ในโน้ต)
 *  ──────────────────────────────────────────────────────────
 *  x = [q, q_dot, tau_d, i]^T       u = V (motor voltage)
 *
 *  Motor shaft: Jm=0.0036, Bm=0.0243  →  Joint-side: J=Jm·N²=0.0144, B=Bm·N²=0.0972
 *  θ = joint angle (encoder on joint shaft),  ω_motor = N·θ̇
 *
 *         θ       θ̇          τ_d        i
 *  Ac = [  0,      1,          0,          0     ]   dθ/dt  = θ̇
 *       [  0,  -Bj/Jj,     -1/Jj,  η·Ke/(N·Jm) ]   dθ̇/dt = −6.750·θ̇ − 69.44·τ_d + 2.753·i
 *       [  0,      0,          0,          0     ]   dτ_d/dt= 0  (random walk)
 *       [  0, -N·Ke/L,         0,        -R/L   ]   di/dt  = −78.44·θ̇ − 793.1·i + 375.1·V
 *
 *  Bc = [0,  0,  0,  1/L]^T       (Vin → di/dt เท่านั้น)
 *  C  = [1,  0,  0,    0]         (วัดตำแหน่งอย่างเดียว)
 *
 *  DISCRETISATION (Euler forward, DT = 1 ms)   Ad = I + Ac·DT,  Bd = Bc·DT
 *  ──────────────────────────────────────────────────────────────────────────
 *         θ          θ̇           τ_d           i
 *  Ad = [ 1,      +0.001,          0,           0       ]
 *       [ 0,    +0.99325,      -0.06944,    +0.002753   ]
 *       [ 0,          0,          1,           0       ]
 *       [ 0,     -0.07844,        0,        +0.2069     ]
 *
 *  Bd = [0,  0,  0,  0.3751]^T
 *
 *  KF CYCLE (every 1 ms)
 *  ─────────────────────
 *  --- PREDICT ---
 *    x_p = Ad*x + Bd*u
 *    P_p = Ad*P*Ad^T + Q
 *
 *  --- UPDATE  (C=[1,0,0,0] simplifies everything to scalars) ---
 *    S   = P_p[0][0] + R_meas          (scalar)
 *    K   = P_p[:,0]  / S               (4x1 vector, = P_p*C^T/S)
 *    nu  = z - x_p[0]                  (innovation)
 *    x   = x_p + K*nu
 *    P   = (I - K*C) * P_p
 * =======================================================================
 */

#include "kalman_filter.h"
#include <string.h>
#include <math.h>

/* ================================================================== */
/*  Internal matrix helpers (4x4, optimised for no heap allocation)   */
/* ================================================================== */

/* out = A * B  (4x4) */
static void mat4_mul(float out[4][4],
                     const float A[4][4],
                     const float B[4][4])
{
    float tmp[4][4];
    memset(tmp, 0, sizeof(tmp));
    for (int i = 0; i < 4; i++)
        for (int k = 0; k < 4; k++) {
            if (A[i][k] == 0.0f) continue;
            for (int j = 0; j < 4; j++)
                tmp[i][j] += A[i][k] * B[k][j];
        }
    memcpy(out, tmp, sizeof(tmp));
}

/* out = A * B^T  (4x4) -- used for Ad*P*Ad^T in two steps */
static void mat4_mul_BT(float out[4][4],
                        const float A[4][4],
                        const float B[4][4])
{
    float tmp[4][4];
    memset(tmp, 0, sizeof(tmp));
    /* out[i][j] = sum_k A[i][k] * B[j][k]  (B transposed) */
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                tmp[i][j] += A[i][k] * B[j][k];
    memcpy(out, tmp, sizeof(tmp));
}

/* ================================================================== */
/*  KF_Init                                                            */
/* ================================================================== */
void KF_Init(KalmanFilter_t *kf, float q0)
{
    memset(kf, 0, sizeof(KalmanFilter_t));

    /* ---------- Initial state  x = [q, q_dot, tau_d, i] ---------- */
    kf->x[0] = q0;    /* q      position    [rad]   */
    kf->x[1] = 0.0f;  /* q_dot  velocity    [rad/s] */
    kf->x[2] = 0.0f;  /* tau_d  disturbance [N.m]   */
    kf->x[3] = 0.0f;  /* i      current     [A]     */

    /* ---------- Derive continuous-time Ac entries ----------
     *  Motor 2 รอบ : Joint 1 รอบ  ->  N = 2
     *  All quantities in joint-side coordinates.
     *
     *  Row 1 (θ̈):   Ac[1][1] = -B/J,  Ac[1][2] = +1/J,  Ac[1][3] = N·η·Ke/J
     *  Row 3 (İ) :   Ac[3][1] = -Ke/(N·L),               Ac[3][3] = -R/L
     */
    /* J = Jm·N² = 0.0144,  B = Bm·N² = 0.0972  (joint-side, encoder on joint shaft)
     * ω_motor = N·ω_joint  →  back-EMF = Ke·N·ω_joint */
    const float a11 = -KF_B  / KF_J;                    /* -6.750  (-Bj/Jj)          */
    const float a12 = -1.0f  / KF_J;                    /* -69.44  (-1/Jj, τ_d ต้าน) */
    const float a13 =  KF_EEFF * KF_KE / (KF_N * KF_JM); /* +2.753  (η·Ke/(N·Jm))   */
    const float a31 = -KF_N  * KF_KE  / KF_L;           /* -78.44  (-N·Ke/L)         */
    const float a33 = -KF_R  / KF_L;                    /* -793.1  (-R/L)            */
    const float b3  =  1.0f  / KF_L;                    /* +375.1  (1/L)             */

    /* ---------- Build Ad = I + Ac*DT (Euler forward) ---------- */

    /* Row 0: dθ/dt = θ̇ */
    kf->Ad[0][0] = 1.0f;
    kf->Ad[0][1] = KF_DT;        /* 0.001            */
    kf->Ad[0][2] = 0.0f;
    kf->Ad[0][3] = 0.0f;

    /* Row 1: dθ̇/dt = a11·θ̇ + a12·tau_d + a13·i */
    kf->Ad[1][0] = 0.0f;
    kf->Ad[1][1] = 1.0f + a11 * KF_DT;   /*  1 - 6.750e-3  =  0.99325   */
    kf->Ad[1][2] = a12  * KF_DT;          /* -69.44e-3       = -0.06944  */
    kf->Ad[1][3] = a13  * KF_DT;          /* +2.753e-3       = +0.002753 */

    /* Row 2: dtau_d/dt = 0  (random walk — driven by process noise only) */
    kf->Ad[2][0] = 0.0f;
    kf->Ad[2][1] = 0.0f;
    kf->Ad[2][2] = 1.0f;          /* ไม่มี decay */
    kf->Ad[2][3] = 0.0f;

    /* Row 3: dI/dt = a31·θ̇ + a33·I + b3·V */
    kf->Ad[3][0] = 0.0f;
    kf->Ad[3][1] = a31 * KF_DT;           /* -78.44e-3       = -0.07844  */
    kf->Ad[3][2] = 0.0f;
    kf->Ad[3][3] = 1.0f + a33 * KF_DT;   /*  1 - 793.1e-3   =  0.2069   */

    /* ---------- Build Bd = Bc*DT  (Bc = [0, 0, 0, 1/L]^T) ---------- */
    kf->Bd[0] = 0.0f;
    kf->Bd[1] = 0.0f;
    kf->Bd[2] = 0.0f;
    kf->Bd[3] = b3 * KF_DT;    /* 375.1e-3 = 0.3751  (Vin → İ) */

    /* ---------- Process noise Q (diagonal) ----------
     *  x = [q, q_dot, tau_d, i]
     */
    kf->Q[0][0] = KF_Q_POS;    /* q     */
    kf->Q[1][1] = KF_Q_VEL;    /* q_dot */
    kf->Q[2][2] = KF_Q_TAUD;   /* tau_d (เล็ก = estimate แทบนิ่ง, V_dist ปิดอยู่) */
    kf->Q[3][3] = KF_Q_CUR;    /* i     */

    /* ---------- Measurement noise ---------- */
    kf->R_meas = KF_R_MEAS;    /* 1.471e-7 rad^2 */

    /* ---------- Initial covariance P0 = diag([1, 10, 1, 1]) ----------
     *  ตั้งสูงเผื่อความไม่แน่ใจเริ่มต้น โดยเฉพาะ velocity           */
    kf->P[0][0] = 1.0f;
    kf->P[1][1] = 10.0f;
    kf->P[2][2] = 1.0f;
    kf->P[3][3] = 1.0f;

    /* ---------- Output convenience fields ---------- */
    kf->est_position    = q0;
    kf->est_velocity    = 0.0f;
    kf->est_disturbance = 0.0f;
    kf->est_current     = 0.0f;
}

/* ================================================================== */
/*  KF_Reset                                                           */
/* ================================================================== */
void KF_Reset(KalmanFilter_t *kf, float q0)
{
    /* Keep Ad, Bd, Q, R_meas intact -- only reset state & covariance
     * x = [q, q_dot, tau_d, i] */
    kf->x[0] = q0;    /* q      */
    kf->x[1] = 0.0f;  /* q_dot  */
    kf->x[2] = 0.0f;  /* tau_d  */
    kf->x[3] = 0.0f;  /* i      */

    memset(kf->P, 0, sizeof(kf->P));
    kf->P[0][0] = 1.0f;
    kf->P[1][1] = 10.0f;
    kf->P[2][2] = 1.0f;
    kf->P[3][3] = 1.0f;

    kf->est_position    = q0;
    kf->est_velocity    = 0.0f;
    kf->est_disturbance = 0.0f;
    kf->est_current     = 0.0f;
}

/* ================================================================== */
/*  KF_Update  -- call every 1 ms                                      */
/* ================================================================== */
void KF_Update(KalmanFilter_t *kf, float V_in, float z_pos)
{
    int i, j;

    /* ==============================================================
     *  SANITY GUARD: ถ้า z_pos หรือ V_in เป็น NaN/Inf → reset แล้วออก
     * ============================================================== */
    if (!isfinite(z_pos) || !isfinite(V_in)) {
        KF_Reset(kf, kf->x[0]);
        return;
    }

    /* ==============================================================
     *  STEP 1: PREDICT STATE
     *  x_p = Ad * x  +  Bd * u
     * ============================================================== */
    float xp[4];
    for (i = 0; i < 4; i++) {
        xp[i] = kf->Bd[i] * V_in;
        for (j = 0; j < 4; j++)
            xp[i] += kf->Ad[i][j] * kf->x[j];
    }

    /* ── NaN guard หลัง predict ── ถ้า state ระเบิด → reset KF */
    for (i = 0; i < 4; i++) {
        if (!isfinite(xp[i])) {
            KF_Reset(kf, z_pos);   /* ใช้ encoder ที่วัดได้เป็น q0 */
            return;
        }
    }

    /* Clamp current estimate (physical sanity: |i| < V_max/R)
     * i อยู่ที่ index 3 ใน state x = [q, q_dot, tau_d, i] */
    const float i_max = KF_VMAX / KF_R;   /* ~11.35 A */
    if      (xp[3] >  i_max) xp[3] =  i_max;
    else if (xp[3] < -i_max) xp[3] = -i_max;

    /* Clamp velocity (joint-side, physical limit ~50 rad/s = ~480 rpm at motor) */
    const float qd_max = 50.0f;
    if      (xp[1] >  qd_max) xp[1] =  qd_max;
    else if (xp[1] < -qd_max) xp[1] = -qd_max;

    /* Clamp disturbance (ไม่ควรเกิน max torque ของระบบ: Kt*i_max*N*η) */
    const float taud_max = KF_KT * i_max * KF_N * KF_EEFF;  /* ~4.5 N.m */
    if      (xp[2] >  taud_max) xp[2] =  taud_max;
    else if (xp[2] < -taud_max) xp[2] = -taud_max;

    /* ==============================================================
     *  STEP 2: PREDICT COVARIANCE
     *  P_p = Ad * P * Ad^T  +  Q
     *
     *  Compute in two steps to avoid a 3rd temporary:
     *    tmp  = Ad * P
     *    P_p  = tmp * Ad^T   (= mat4_mul_BT(tmp, Ad))
     * ============================================================== */
    float tmp[4][4];
    float Pp[4][4];

    mat4_mul(tmp, kf->Ad, kf->P);    /* tmp = Ad * P    */
    mat4_mul_BT(Pp, tmp, kf->Ad);    /* Pp  = tmp * Ad^T */

    /* Add Q */
    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++)
            Pp[i][j] += kf->Q[i][j];

    /* ==============================================================
     *  STEP 2b: Clamp P diagonal ป้องกัน covariance โต unbounded
     *  (defensive: กัน P[2][2] โตเร็วถ้าตั้ง Q_TAUD ใหญ่ → coupling ผ่าน Ad → P[1][1] โต)
     * ============================================================== */
    {
        const float P_max[4] = { 1e3f, 1e4f, 1e4f, 1e3f };
        for (i = 0; i < 4; i++) {
            if (!isfinite(Pp[i][i]) || Pp[i][i] > P_max[i])
                Pp[i][i] = P_max[i];
            if (Pp[i][i] < 0.0f)
                Pp[i][i] = 0.0f;
        }
    }

    /* ==============================================================
     *  STEP 3: KALMAN GAIN
     *  C = [1, 0, 0, 0]
     *  S   = C * P_p * C^T + R  =  P_p[0][0] + R_meas
     *  K   = P_p * C^T / S      =  P_p[:, 0] / S
     * ============================================================== */
    float S = Pp[0][0] + kf->R_meas;
    if (S < kf->R_meas) S = kf->R_meas;   /* ป้องกัน S <= 0 (กรณี Pp หลุดเป็น negative) */
    float Sinv = 1.0f / S;

    for (i = 0; i < 4; i++)
        kf->K[i] = Pp[i][0] * Sinv;

    /* ==============================================================
     *  STEP 4: UPDATE STATE
     *  innovation nu = z - C * x_p  =  z - x_p[0]
     *  x = x_p + K * nu
     * ============================================================== */
    float nu = z_pos - xp[0];

    for (i = 0; i < 4; i++)
        kf->x[i] = xp[i] + kf->K[i] * nu;

    /* ==============================================================
     *  STEP 5: UPDATE COVARIANCE  (Joseph form for numerical stability)
     *  P = (I - K*C) * P_p * (I - K*C)^T  +  K * R * K^T
     *
     *  For scalar R and C=[1,0,0,0]:
     *    IKC[i][j] = delta(i,j) - K[i] * C[j]
     *             = delta(i,j) - K[i] * (j==0)
     *
     *  Joseph form:
     *    P = IKC * Pp * IKC^T  +  R * K * K^T
     * ============================================================== */
    float IKC[4][4];
    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++)
            IKC[i][j] = (float)(i == j) - kf->K[i] * (float)(j == 0);

    float IKC_Pp[4][4];
    mat4_mul(IKC_Pp, IKC, Pp);             /* IKC_Pp = IKC * Pp         */
    mat4_mul_BT(kf->P, IKC_Pp, IKC);      /* P = IKC_Pp * IKC^T        */

    /* Add  R * K * K^T  (rank-1 update) */
    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++)
            kf->P[i][j] += kf->R_meas * kf->K[i] * kf->K[j];

    /* Force P symmetric (fix floating-point drift) */
    for (i = 0; i < 4; i++)
        for (j = i + 1; j < 4; j++) {
            float avg = 0.5f * (kf->P[i][j] + kf->P[j][i]);
            kf->P[i][j] = avg;
            kf->P[j][i] = avg;
        }

    /* ==============================================================
     *  FINAL SANITY: ถ้า state ใดกลาย NaN/Inf หลัง update → reset
     *  (ป้องกัน HardFault ที่ ISR ถัดไป)
     * ============================================================== */
    for (i = 0; i < 4; i++) {
        if (!isfinite(kf->x[i])) {
            KF_Reset(kf, z_pos);
            return;
        }
    }

    /* ==============================================================
     *  Mirror to convenience output fields
     *  x = [q, q_dot, tau_d, i]
     * ============================================================== */
    kf->est_position    = kf->x[0];   /* index 0 */
    kf->est_velocity    = kf->x[1];   /* index 1 */
    kf->est_disturbance = kf->x[2];   /* index 2  tau_d [N.m] */
    kf->est_current     = kf->x[3];   /* index 3  i     [A]   */
}
