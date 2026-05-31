/*
 * kalman_filter.h
 *
 *  Created on: 18 พ.ค. 2569
 *      Author: POND
 *
 * =======================================================================
 *  Full-order Kalman Filter -- 1-DOF Robot (DC Motor + Pulley)
 *  State vector: x = [q, q_dot, tau_d, i]^T   (ลำดับตรงกับ state space ในโน้ต)
 *
 *  JOINT-SIDE EQUATIONS OF MOTION
 *  ───────────────────────────────
 *    J * q_ddot = N * Kt * e_eff * i  -  B * q_dot  -  tau_d
 *    L * i_dot  = V  -  R * i  -  Ke * N * q_dot
 *    tau_d_dot  = 0     (slow random-walk disturbance)
 *
 *  GEAR (Pulley)
 *  ─────────────
 *    Motor 2 รอบ : Joint 1 รอบ
 *      => N = omega_motor / omega_joint = 2.0
 *      => omega_motor = N * q_dot_joint
 *      => tau_joint   = tau_motor * N * e_eff   (torque ขยาย 2x)
 *    Ke, Kt อ้างอิงที่ motor shaft:
 *      back-EMF = Ke * omega_motor = Ke * N * q_dot_joint
 *
 *  ENCODER (on JOINT shaft)
 *  ────────────────────────
 *    CPR        = 8192 counts/rev
 *    resolution = 2*PI / 8192 = 7.669e-4 rad/count
 *    sigma      = 0.5 count  => 3.835e-4 rad
 *    R_meas     = sigma^2    = 1.47e-7 rad^2
 *
 *  OPERATING LIMITS
 *  ────────────────
 *    V_max = 24 V,  DT = 1 ms
 *
 *  MEASUREMENT OUTPUT
 *  ──────────────────
 *    y = q   (position only, C = [1, 0, 0, 0])
 * =======================================================================
 */

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  Motor / Plant Parameters                                           */
/* ------------------------------------------------------------------ */
#define KF_R        2.1142f       /* Winding resistance              [Ohm]        */
#define KF_L        0.0026657f    /* Winding inductance              [H]          */
#define KF_KE       0.1045f       /* Back-EMF constant (motor shaft) [V.s/rad_m]  */
#define KF_KT       0.1045f       /* Torque constant   (motor shaft) [N.m/A]      */
#define KF_JM       0.0036f       /* Motor shaft inertia  (identified with pulley) [kg.m^2]    */
#define KF_BM       0.0243f       /* Motor shaft damping  (identified with pulley) [N.m.s/rad] */
#define KF_J        (KF_JM * KF_N * KF_N)   /* Joint-side: Jm·N² = 0.0144  [kg.m^2]    */
#define KF_B        (KF_BM * KF_N * KF_N)   /* Joint-side: Bm·N² = 0.0972  [N.m.s/rad] */
#define KF_EEFF     0.1897f       /* Drivetrain efficiency           [-]           */
#define KF_N        2.0f          /* Gear ratio: omega_motor/omega_joint           */
#define KF_VMAX     24.0f         /* Max motor voltage               [V]           */
#define KF_DT       0.001f        /* Sample period                   [s]           */

/*
 *  Derived Ac-matrix entries  (state x = [q, qd, tau_d, i]):
 *
 *     col:  q      qd       tau_d    i
 *  row qd : [0,  a11,       a12,    a13]
 *  row i  : [0,  a31,         0,    a33]
 *
 *  θ = joint angle (encoder on joint shaft),  ω_motor = N·θ̇_joint
 *  Jm=0.0036, Bm=0.0243  →  J=Jm·N²=0.0144, B=Bm·N²=0.0972
 *
 *    a11 = -Bj / Jj                          = -0.0972/0.0144             = -6.750
 *    a12 = -1.0 / Jj                         = -1/0.0144                  = -69.44  (τ_d ต้านการเคลื่อนที่)
 *    a13 =  η·Ke / (N·Jm)                   =  0.1897*0.1045/(2*0.0036)  = +2.753
 *    a31 = -N·Ke / L                         = -2*0.1045/0.0026657        = -78.44  (ω_motor = N·ω_joint)
 *    a33 = -R  / L                           = -2.1142/0.0026657          = -793.1
 *    b3  =  1.0 / L                          =  1/0.0026657               = +375.1
 *
 *  Disturbance feed-forward (disabled — gain ≈ 53x, active only after re-ID with arm):
 *    V_dist = (Rm / (N·η·Kt)) · tau_d
 *           = est_disturbance * KF_R / (KF_N * KF_EEFF * KF_KT)
 */

/* ------------------------------------------------------------------ */
/*  Encoder                                                            */
/* ------------------------------------------------------------------ */
#define KF_ENC_CPR      8192
/* resolution = 2*PI / CPR = 7.669e-4 rad,  sigma = 0.5 count = 3.835e-4 rad */

/* ------------------------------------------------------------------ */
/*  Noise Tuning                                                       */
/*                                                                     */
/*  Q[i][i]  = process noise  (ยิ่งมาก = เชื่อ model น้อยลง)         */
/*  R_meas   = measurement noise variance (จาก encoder resolution)   */
/*                                                                     */
/*  แนวทางปรับ Q:                                                      */
/*   - velocity ยัง oscillate มาก  -> เพิ่ม KF_Q_VEL                 */
/*   - filter ตาม encoder เร็วเกิน -> ลด KF_Q_VEL หรือเพิ่ม R_MEAS  */
/*   - disturbance ตอบสนองช้า      -> เพิ่ม KF_Q_TAUD                */
/* ------------------------------------------------------------------ */
/*
 *  Q TUNING GUIDE (อ้างอิงจากระบบจริง):
 *  ──────────────────────────────────────────────────────────────────
 *  max accel = (Kt * i_max) / J = (0.1045 * 11.35) / 0.0036 ≈ 330 rad/s²
 *  dv_max per step (1ms) = 330 * 0.001 = 0.33 rad/s
 *  Q_VEL ควร ≥ (dv_max)² ≈ 0.11  ให้ filter ตามทันระบบจริงได้
 *
 *  ถ้า qd_out ขึ้นน้อยเกิน → เพิ่ม Q_VEL
 *  ถ้า qd_out กระตุกเยอะ  → ลด Q_VEL หรือเพิ่ม R_MEAS
 */
#define KF_Q_POS    1e-6f    /* q     : position process noise         */
/*
 *  Tuning philosophy:
 *    Model = motor + pulley only  (J = Jm·N², B = Bm·N²)
 *    τ_d absorbs arm + gravity + rope + friction
 *
 *    Q_TAUD ต้องใหญ่พอให้ τ_d วิ่งตาม J_arm·q̈ ได้ทัน
 *    → model error ถูก absorb → velocity estimate สะอาด
 *
 *    ถ้า qd_out ยัง noisy  → เพิ่ม Q_TAUD อีก (1e-1, 1e0)
 *    ถ้า τ_d oscillate เกิน → ลด Q_TAUD ลง
 */
#define KF_Q_VEL    1e-6f    /* q_dot : เล็กมาก → velocity smooth สุด
                              *   tradeoff: ตาม velocity จริงได้ช้าลง
                              *   tune: 1e-6 → 1e-5 → 1e-4 ถ้าต้องการ response เร็วขึ้น */
#define KF_Q_CUR    1e-2f    /* i     : current process noise                              */
#define KF_Q_TAUD   1e-8f    /* tau_d : ลดลงจาก 1e0 → tau_d estimate smooth ขึ้น ~10×
                              *   KF bandwidth ของ tau_d ≈ sqrt(Q/R_meas) / dt
                              *   1e0  → tau_d ตาม noise เร็ว → V_dist กระตุก
                              *   1e-2 → tau_d smooth กว่า   → V_dist นิ่งพอควบคุมได้
                              *   ถ้ายัง noisy ให้ลดเป็น 1e-3
                              *   ถ้า disturbance ตาม load ไม่ทัน ให้เพิ่มกลับมา */

#define KF_R_MEAS   1.471e-7f  /* encoder (0.5 count)²  — ค่า theoretical
                                *   อย่าลดต่ำกว่านี้มาก: R→0 ทำให้ K[1] ขยาย
                                *   quantization noise → velocity noisy       */

/* ------------------------------------------------------------------ */
/*  Dimensions                                                         */
/* ------------------------------------------------------------------ */
#define KF_N_STATES  4       /* [q, q_dot, tau_d, i]  ← ลำดับตาม state space โน้ต */
#define KF_N_OBS     1       /* [q]                   */

/* ------------------------------------------------------------------ */
/*  Kalman Filter Structure                                            */
/* ------------------------------------------------------------------ */
typedef struct {

    /* State estimate: x = [q, q_dot, tau_d, i]
     *   x[0] = q      position     [rad]
     *   x[1] = q_dot  velocity     [rad/s]
     *   x[2] = tau_d  disturbance  [N.m]   ← joint-side load torque
     *   x[3] = i      current      [A]
     */
    float x[KF_N_STATES];

    /* Error covariance matrix 4x4 (row-major) */
    float P[KF_N_STATES][KF_N_STATES];

    /* Discrete-time system matrices (built once in KF_Init) */
    float Ad[KF_N_STATES][KF_N_STATES]; /* Ad = I + Ac*dt  (Euler forward) */
    float Bd[KF_N_STATES];              /* Bd = Bc*dt,  Bc=[0,0,0,1/L]^T  */

    /* Noise covariance */
    float Q[KF_N_STATES][KF_N_STATES];  /* Process noise (diagonal)         */
    float R_meas;                        /* Measurement noise variance       */

    /* Kalman gain vector (4x1, exploits C = [1,0,0,0]) */
    float K[KF_N_STATES];

    /* ---- Convenience output fields (always mirror x[]) ---- */
    float est_position;     /* x[0]  q       [rad]   */
    float est_velocity;     /* x[1]  q_dot   [rad/s] */
    float est_disturbance;  /* x[2]  tau_d   [N.m]   */
    float est_current;      /* x[3]  i       [A]     */

} KalmanFilter_t;

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief  Initialise: build Ad/Bd from motor params, set Q, R, P0.
 * @param  kf   Pointer to filter instance.
 * @param  q0   Initial joint position [rad]  (read from encoder after homing).
 */
void KF_Init(KalmanFilter_t *kf, float q0);

/**
 * @brief  One filter cycle (predict + update). Call every KF_DT = 1 ms.
 *
 *         Recommended call order inside TIM ISR:
 *           1. Encoder_Update(&henc2);
 *           2. z = Encoder_GetPositionRad(&henc2);
 *           3. KF_Update(&hkf, monitor_V_in, z);      // <-- here
 *           4. Cascade_Control_Update(ref_q, ref_qd);
 *
 * @param  kf     Filter instance.
 * @param  V_in   Voltage commanded in the PREVIOUS cycle [V].
 *                Use monitor_V_in (set by Motor_Drive in last cycle).
 *                One-step lag is negligible at 1 ms.
 * @param  z_pos  Encoder measurement this cycle [rad].
 */
void KF_Update(KalmanFilter_t *kf, float V_in, float z_pos);

/**
 * @brief  Reset state and covariance (call after e-stop or re-homing).
 * @param  kf   Filter instance.
 * @param  q0   Current joint position [rad].
 */
void KF_Reset(KalmanFilter_t *kf, float q0);

#endif /* KALMAN_FILTER_H_ */
