/*
 * cascade_control.h
 *
 *  Created on: 29 เม.ย. 2569
 *      Author: POND
 */

#ifndef CASCADE_CONTROL_H_
#define CASCADE_CONTROL_H_

#include "main.h"

// ---------------- Hardware Parameters ----------------
#define DT_VEL 0.001f   // 1 ms   — vel loop (1 kHz)
#define DT_POS 0.001f   // 1 ms — pos loop ทุก tick (1 kHz) = เท่า vel loop
#define POS_DIV  1U     // pos loop ทุก 1 tick → ไม่มี staircase ใน vel setpoint
                        //   (เดิม 5 → pos_div_out กระโดดทุก 5ms × Kp สูง = กระตุก)
#define MAX_VOLTAGE 24.0f   // Motor Maximum Voltage
#define PWM_ARR_MAX 9999.0f // TIM1 PSC=9, ARR=9999 → 1700 Hz PWM
#define GEAR_RATIO 2.0f

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float integral_limit;
} PID_Controller;

extern float          target_q;       // ตำแหน่งเป้าหมาย (Radian)
extern float          target_qd;      // ความเร็วเป้าหมาย (Radian/s) สำหรับ Trajectory
extern volatile float q_out;          // ตำแหน่งจริง (volatile สำหรับ CubeMonitor)
extern volatile float qd_out;         // ความเร็วจริง จาก KF (volatile สำหรับ CubeMonitor)
extern volatile float monitor_V_in;     // แรงดัน magnitude (display)
extern volatile float monitor_V_signed;  // แรงดัน signed clamped (telemetry/analysis)
extern volatile float g_traj_span_rad;   // ระยะ move ปัจจุบัน |target−start| [rad]
                                         //   ตั้งโดย *_MoveTo() — ใช้ gate backlash comp
/* Real-time monitor mirrors (CubeMonitor) — jerk-ref ดู ref_j ใน auto_mission.c */
extern volatile float mon_q_ref, mon_qd_ref, mon_qdd_ref;
extern volatile float mon_q_out, mon_qd_out, mon_qdd_out, mon_j_out;
extern volatile float mon_v_in;

// ---------------- Public Function Prototypes ----------------
void Cascade_Control_Init(void);
void Cascade_Control_Update(float ref_q, float ref_qd);
/* 3-arg: เพิ่ม q̈_ref สำหรับ acceleration feedforward (inverse dynamics) */
void Cascade_Control_Update_FF(float ref_q, float ref_qd, float ref_qdd);

/**
 * @brief  Reset KF state + PID integrators (เรียกหลัง e-stop หรือ homing)
 *         ต้องเรียกจาก context ที่ ISR ไม่ preempt (เช่นใน EXTI callback หรือ main loop)
 */
void Cascade_Control_Reset(void);
void Cascade_Flush_VelIntegral(void);

/* Open-loop voltage drive (bypass controller) — Lab 1 parameter estimation */
void Cascade_OpenLoopVolt(float V);

#endif /* CASCADE_CONTROL_H_ */
