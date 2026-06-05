/*
 * cascade_control.c
 *
 *  Created on: 29 เม.ย. 2569
 *      Author: POND
 */


#include "cascade_control.h"
#include "encoder.h"
#include "trajectory.h"
#include "kalman_filter.h"
#include "base_system.h"   /* modbus_registers[] */
#include <math.h>

// ---------------- ดึงค่าฮาร์ดแวร์จาก main.c ----------------
extern TIM_HandleTypeDef htim1; // PWM PA8
extern TIM_HandleTypeDef htim2; // QEI
extern Encoder_t henc2;

// ลบคำว่า extern ออก แล้วกำหนดค่าเริ่มต้นได้ตามปกติเลยครับ
volatile float monitor_V_in     = 0.0f;  /* magnitude สำหรับ display/telemetry (≥ 0)   */
volatile float monitor_V_signed = 0.0f;  /* SIGNED clamped V สำหรับ telemetry/analysis */
static   float kf_V_in          = 0.0f;  /* SIGNED สำหรับ Kalman Filter                */

/* ระยะ move ปัจจุบัน |target−start| [rad] — *_MoveTo() เป็นคนตั้ง (ดู trajectory.c)
 * ใช้ gate backlash comp: move สั้น (<120°) ปิด comp ป้องกันสั่น/ไม่ smooth */
volatile float g_traj_span_rad  = 0.0f;

/* ── Real-time monitor mirrors (STM32CubeMonitor) ────────────────────────────
 *  global volatile, อัปเดตทุก tick ทุกโหมด (AUTO/MANUAL/TEST) → เลือกใน CubeMonitor ได้เลย
 *  jerk-ref ดูที่ ref_j (global volatile ใน auto_mission.c)                     */
volatile float mon_q_ref = 0.0f, mon_qd_ref = 0.0f, mon_qdd_ref = 0.0f;
volatile float mon_q_out = 0.0f, mon_qd_out = 0.0f, mon_qdd_out = 0.0f;
volatile float mon_j_out = 0.0f;   /* jerk จริง (measured, จาก j_out) */
volatile float mon_v_in  = 0.0f;

// ---------------- Private Variables (ตัวแปรภายใน) ----------------
volatile float q_out = 0.0f;       // ตำแหน่งจริง
static float prev_q_out = 0.0f;
volatile float qd_out = 0.0f;      // ความเร็วจริง


// ---------------- Public Variables (กำหนดค่าเริ่มต้น) ----------------
float target_q = 0.0f;
float target_qd = 0.0f;
float qdd_out = 0.0f;
float j_out = 0.0f;

KalmanFilter_t hkf;

/* ── AUTO/TEST fixed PID gains ───────────────────────────────────────────────
 *  ใช้ตอน MODE_AUTO/TEST (dashboard เขียน Modbus ทับไม่ได้)
 *  MODE_MANUAL ใช้ค่าจาก Modbus แทน (สำหรับ tune)
 * ─────────────────────────────────────────────────────────────────────────── */
#define POS_KP_AUTO  22.0f
#define POS_KI_AUTO  8.6f
#define POS_KD_AUTO  0.0f
/* gain-schedule POS_KI: hold (ref นิ่ง) ใช้ค่าต่ำ + clamp integral → กัน windup สั่นที่ปลายทาง
 * (move ใช้ POS_KI_AUTO เต็มเพื่อความแม่น) — แก้ AUTO สั่นโดยไม่ลด accuracy ตอน move */
#define POS_KI_HOLD   1.0f
#define POS_HOLD_ILIM 1.0f
#define VEL_KP_AUTO  8.5f
#define VEL_KI_AUTO  0.5f
#define VEL_KD_AUTO  0.0f    /* Kd=0: ตัด derivative kick — backlash step × Kd/dt
                              * = 27V spike ทุกครั้งเปลี่ยนทิศ (ดู troubleshoot) */

// ---------------- Setup PID Controllers ----------------
PID_Controller pos_ctrl = { .integral = 0.0f, .prev_error = 0.0f, .integral_limit = 15.0f };
/* integral_limit = 10 ใช้ได้ทั้ง cascade mode (output เป็น rad/s clamp ที่ ±10 อยู่แล้ว)
 * และ direct drive mode (output เป็น V — 10V ต่ำกว่า MAX_VOLTAGE 24V อย่างปลอดภัย)              */
PID_Controller vel_ctrl = { .integral = 0.0f, .prev_error = 0.0f, .integral_limit = 8.0f };
/* integral_limit = 6.0f (ไม่ใช่ MAX_VOLTAGE):
 *   จำกัด Ki × integral ≤ ±6V → ป้องกัน integral windup ที่ทำให้ bang-bang saturate
 *   Anti-windup check อยู่ใน calculate_pid() → ไม่ต้อง flush integral ด้วยตนเอง
 *
 * V_FF = K_ff * ref_qd  (ไม่ต้องคูณ GEAR_RATIO แล้ว — K_ff รวมทุกอย่างไว้แล้ว)
 * V_ss ที่ ω_joint = 1 rad/s:
 *   V_back_EMF = Ke * N * ω  = 0.1045*2*1 = 0.209 V
 *   V_friction  = R * Bj*ω/(N*η*Ke) = 2.1142*0.0972/(2*0.1897*0.1045) = 5.20 V
 *   V_ss ≈ 5.41 V/rad/s   (tune ลงได้ถ้า overshoot)
 */
static float K_ff = 5.4f;

/* ── Acceleration Feedforward (inverse dynamics) ────────────────────────────
 *  V_acc = R·J/(N·η·Kt) · q̈_ref   — แรงดันที่ต้องใช้เร่ง inertia ตาม trajectory
 *  คำนวณจาก reference (Quintic ให้ q̈_ref) → สะอาด ไม่มี noise
 *  K_ACC = 2.1142·0.0144/(2·0.1897·0.1045) ≈ 0.768 V/(rad/s²)
 *  ช่วย feedback ไม่ต้องสร้างแรงเร่งเองผ่าน error → lag ลด, แม่นขึ้น, gain เบาลงได้ */
#define K_ACC  (KF_R * KF_J / (KF_N * KF_EEFF * KF_KT))

/* ── Deadband Compensation with Hysteresis ──────────────────────────────────
 *  มอเตอร์ไม่ตอบสนองเมื่อ duty < ~4.5% (วัดจริง) = 450 counts
 *
 *  Zone 1:  duty == 0                    → 0  (ปกติ)
 *  Zone 2:  0 < duty < HYST  (< 2.25%)  → 0  ตัดเป็น 0 (noise floor — ป้องกัน chattering)
 *  Zone 3:  HYST ≤ duty < MIN (2.25–4.5%) → kick ขึ้น MIN  (เพียงพอจะเคลื่อน)
 *  Zone 4:  duty ≥ MIN                   → pass-through ปกติ
 *
 *  Hysteresis = 50% × MIN = 225 counts ≈ 0.54V
 *  → ป้องกัน limit-cycle เมื่อ pos_error < ~0.5°
 * ─────────────────────────────────────────────────────────────────────────── */
#define PWM_DEADBAND_MIN   450U   /* 4.5% × 9999 — motor เริ่มขยับ          */
#define PWM_DEADBAND_HYST  225U   /* 2.25% × 9999 — kick threshold (50% MIN)
                                   * error < ~1° → cut to 0 → passive settle  */

/* ── Coulomb / Deadband Feedforward (noise-free friction comp) ──────────────
 *  ปัญหา: motor ไม่ขยับใต้ deadband (~4.5% = 1.08V) ตอนความเร็วต่ำ/กลับทิศ
 *  วิธี observer (tau_d เร็ว / DOB) → ขยาย measurement noise → หุ่นสั่น
 *
 *  วิธีนี้ใช้ sign(ref_qd) [reference จาก trajectory = สะอาด ไม่มี noise]:
 *     V_fric = V_COULOMB · sat(ref_qd / QD_EPS)
 *       - เคลื่อน → ดัน motor ถึงขอบ deadband → PID เล็กๆ ก็ทะลุได้
 *       - หยุด (ref_qd≈0) → 0 → ไม่ chatter ตอน holding
 *  ปลอด noise เพราะไม่พึ่ง measured velocity → ไม่สั่น
 *
 *  จูน: V_COULOMB เริ่ม ~deadband voltage (1.08V). ถ้ายังไม่ทะลุ → เพิ่ม;
 *       ถ้า overshoot/กระตุกตอนเริ่ม → ลด หรือเพิ่ม QD_EPS ให้ ramp นุ่มขึ้น
 * ─────────────────────────────────────────────────────────────────────────── */
#define V_COULOMB   0.5f    /* ขนาด FF [V] ≈ deadband voltage (จูน 0.8–1.5)   */
#define QD_EPS      0.3f   /* smooth sign ใต้ความเร็วนี้ [rad/s]              */

/* ── Backlash Inverse Compensation ─────────────────────────────────────────
 *  อ้างอิง: Tao & Kokotovic, "Adaptive control of systems with backlash",
 *            Automatica 29(2), 1993.
 *
 *  BL_RAD  = mean(45,46,53,50,40,45 pulse) × (2π/8192)
 *           = 46.5 × 7.669e-4  =  0.03566 rad  =  2.04°   (วัดจริง n=6)
 *
 *  BL_VEL_THR  ใช้ตรวจว่า ref_qd เปลี่ยนทิศ และ joint เริ่มเคลื่อนจริงแล้ว
 * ─────────────────────────────────────────────────────────────────────────── */
#define BL_RAD       0.03566f   /* backlash width [rad]   — hardcoded จากการวัด */
#define BL_VEL_THR   0.05f      /* velocity threshold [rad/s] to detect direction */
#define BL_MIN_MOVE_RAD  2.0944f /* 120° — move สั้นกว่านี้ "ปิด" backlash comp
                                  * (comp ทำให้ near move สั่นแรง/ไม่ smooth) */
/* ── Take-up แบบ ramp (กัน near move กระตุก) ─────────────────────────────────
 *  เดิม inject/clear bl_comp เป็น step 2° ทันที → pos loop เห็น error กระโดด →
 *  velocity setpoint พุ่ง ~0.55 rad/s = kick แรง (เด่นชัดตอน move สั้น).
 *  ramp 2° ภายใน BL_TAKEUP_MS แทน → error ค่อยๆ โต → setpoint นุ่ม ไม่กระชาก    */
#define BL_TAKEUP_MS  1000.0f     /* เวลาค่อยๆ ใส่/ถอน backlash [ms] (ใหญ่=นุ่มแต่ช้า) */
#define BL_STEP       (BL_RAD / BL_TAKEUP_MS)   /* rad ต่อ tick (1 tick = 1 ms) */

static float  bl_comp     = 0.0f;  /* offset ที่ inject จริง (ramp เข้าหา target) [rad] */
static float  bl_target   = 0.0f;  /* เป้าหมายของ bl_comp: ±BL_RAD หรือ 0          */
static int8_t bl_last_dir = 0;     /* ทิศล่าสุด: +1 / -1 / 0=unknown */

/* ── Pos-loop divider state (file-scope เพื่อให้ Reset() เข้าถึงได้) ── */
static uint8_t pos_div_tick  = 0;
static float   pos_div_out   = 0.0f;

// ---------------- Private Functions (ฟังก์ชันซ่อนภายใน) ----------------
// ใส่ static เพื่อไม่ให้ไฟล์อื่นเรียกใช้ได้โดยตรง
static float calculate_pid(PID_Controller *pid, float error, float dt) {
    /* Proportional + Derivative */
    float P = pid->Kp * error;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    float D = pid->Kd * derivative;

    /* Tentative integral update (clamped) */
    float i_new = pid->integral + error * dt;
    if      (i_new >  pid->integral_limit) i_new =  pid->integral_limit;
    else if (i_new < -pid->integral_limit) i_new = -pid->integral_limit;
    float I = pid->Ki * i_new;

    float output = P + I + D;

    /* ── Anti-windup (conditional integration) ─────────────────────────────
     *  อย่า update integral เมื่อ output กำลัง saturate ไปในทิศเดียวกับ error
     *  (windup condition): ป้องกัน integral สะสมจนเกิด bang-bang limit cycle
     *  ─────────────────────────────────────────────────────────────────────*/
    if (!((output >  pid->integral_limit && error > 0.0f) ||
          (output < -pid->integral_limit && error < 0.0f))) {
        pid->integral = i_new;
    }

    return output;
}

static void Motor_Drive(float V_in) {

    /* บันทึก signed voltage ก่อน strip sign — ใช้ใน KF_Update รอบถัดไป */
    kf_V_in = V_in;

    if (V_in >= 0.0f) {
        HAL_GPIO_WritePin(Motor_Dir_GPIO_Port, Motor_Dir_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(Motor_Dir_GPIO_Port, Motor_Dir_Pin, GPIO_PIN_SET);
        V_in = -V_in;
    }

    if (V_in > MAX_VOLTAGE) V_in = MAX_VOLTAGE;
    monitor_V_in     = V_in;                                /* magnitude (display)         */
    monitor_V_signed = (kf_V_in >= 0.0f) ? V_in : -V_in;    /* signed clamped (telemetry)  */
    uint32_t duty = (uint32_t)((V_in / MAX_VOLTAGE) * PWM_ARR_MAX);

    /* ── Deadband hysteresis ────────────────────────────────────────────────
     *  Zone 2 (noise floor)  → ตัด 0  ป้องกัน limit-cycle ที่ steady-state
     *  Zone 3 (meaningful)   → kick ขึ้น MIN                               */
//    if (duty > 0U && duty < PWM_DEADBAND_MIN) {
//        duty = (duty >= PWM_DEADBAND_HYST) ? PWM_DEADBAND_MIN : 0U;
//    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
}

// ---------------- Public Functions ----------------
// ฟังก์ชันล้างค่า เผื่อใช้สำหรับรีเซ็ตระบบ
void Cascade_Control_Init(void) {
    q_out = 0.0f;
    prev_q_out = 0.0f;
    qd_out = 0.0f;
    target_q = 0.0f;
    target_qd = 0.0f;

    KF_Init(&hkf, 0.0f);
}

void Cascade_Flush_VelIntegral(void)
{
    vel_ctrl.integral   = 0.0f;
    vel_ctrl.prev_error = 0.0f;
}

// รีเซ็ตระบบควบคุมทั้งหมด (เรียกหลัง e-stop / homing)
void Cascade_Control_Reset(void)
{
    /* 1. Reset KF ด้วยตำแหน่งปัจจุบันจาก encoder */
    Encoder_Update(&henc2);
    float q_now = Encoder_GetPositionRad(&henc2);
    KF_Reset(&hkf, q_now);

    /* 2. Reset PID integrators + prev_error */
    vel_ctrl.integral   = 0.0f;
    vel_ctrl.prev_error = 0.0f;
    pos_ctrl.integral   = 0.0f;
    pos_ctrl.prev_error = 0.0f;

    /* 3. Clear signed voltage (ไม่ให้ KF รอบแรกหลัง reset ใช้ค่าเก่า) */
    kf_V_in      = 0.0f;
    monitor_V_in = 0.0f;

    /* 5. Reset backlash compensator */
    bl_comp     = 0.0f;
    bl_target   = 0.0f;
    bl_last_dir = 0;

    /* 6. Reset pos-loop divider */
    pos_div_tick = 0;
    pos_div_out  = 0.0f;

    /* 4. Reset state outputs */
    q_out  = q_now;
    qd_out = 0.0f;
    qdd_out = 0.0f;
    j_out   = 0.0f;
}

// ลูปคำนวณหลัก ที่จะถูกเรียกใน Timer Interrupt
/* wrapper เดิม (2-arg) — ใช้ตอน hold/velocity mode ที่ไม่มี q̈_ref */
void Cascade_Control_Update(float ref_q, float ref_qd)
{
    Cascade_Control_Update_FF(ref_q, ref_qd, 0.0f);
}

/* 3-arg เต็ม — รับ q̈_ref สำหรับ acceleration feedforward */
void Cascade_Control_Update_FF(float ref_q, float ref_qd, float ref_qdd)
{
    float          current_q_dot_ref  = 0.0f;
    static float   prev_qd_kf         = 0.0f;
    static float   prev_qdd_out       = 0.0f;

    /* ── Pos-loop divider: รัน pos loop ทุก POS_DIV ticks (ปัจจุบัน 1 = ทุก 1ms)
     * ใช้ file-scope vars (pos_div_tick / pos_div_out) เพื่อให้ Reset() reset ได้ */

    /* ── [เดิม] อ่าน encoder ── */
    Encoder_Update(&henc2);
    float z_enc = Encoder_GetPositionRad(&henc2);

    /* ── [ใหม่] รัน Kalman Filter ──
     *   monitor_V_in ถูก set ใน Motor_Drive() รอบก่อนหน้า (1-step lag ยอมรับได้)
     */
    KF_Update(&hkf, kf_V_in, z_enc);   /* signed voltage — ทิศทางถูกต้อง */

    /* ── รับค่า state จาก KF แทนการคำนวณแบบเดิม ── */
    q_out  = hkf.est_position;
    qd_out = hkf.est_velocity;

    /* ── qdd, jerk: คำนวณจาก qd_out ที่ smooth แล้วของ KF ── */
    float raw_qdd = (qd_out - prev_qd_kf) / KF_DT;
    float alpha_a = 0.02f;
    qdd_out = (alpha_a * raw_qdd) + ((1.0f - alpha_a) * qdd_out);
    prev_qd_kf = qd_out;

    float raw_j   = (qdd_out - prev_qdd_out) / KF_DT;
    float alpha_j = 0.005f;
    j_out = (alpha_j * raw_j) + ((1.0f - alpha_j) * j_out);
    prev_qdd_out = qdd_out;

    /* ── mirror สัญญาณให้ CubeMonitor (ก่อน early-return ของ direct-drive) ── */
    mon_q_ref = ref_q;   mon_qd_ref = ref_qd;   mon_qdd_ref = ref_qdd;
    mon_q_out = q_out;   mon_qd_out = qd_out;    mon_qdd_out = qdd_out;   mon_j_out = j_out;
    mon_v_in  = monitor_V_signed;   /* 1-tick lag — พอสำหรับ monitor */

    /* ════════════════════════════════════════════════════════════
     *  PID gain source แยกตาม mode:
     *    MODE_MANUAL → live-update จาก Modbus (dashboard tuning)
     *    อื่นๆ (AUTO/TEST) → ค่า fixed ในโค้ด (จูนมาแล้ว, dashboard ทับไม่ได้)
     *  → ป้องกัน gain ที่ tune ใน MANUAL หลุดไปใช้ตอน AUTO
     * ════════════════════════════════════════════════════════════ */
    if (current_system_mode == MODE_MANUAL) {
        vel_ctrl.Kp = (float)(int16_t)modbus_registers[REG_VEL_KP] / 100.0f;
        vel_ctrl.Ki = (float)(int16_t)modbus_registers[REG_VEL_KI] / 100.0f;
        vel_ctrl.Kd = (float)(int16_t)modbus_registers[REG_VEL_KD] / 100.0f;
        /* REG_POS_KP == 0 → pos_ctrl.Kp = 0 → disable position loop (velocity-only) */
        pos_ctrl.Kp = (float)(int16_t)modbus_registers[REG_POS_KP] / 100.0f;
        pos_ctrl.Ki = (float)(int16_t)modbus_registers[REG_POS_KI] / 100.0f;
        pos_ctrl.Kd = (float)(int16_t)modbus_registers[REG_POS_KD] / 100.0f;
    } else {
        /* AUTO / TEST: ใช้ค่า fixed (จูนแล้ว) */
        pos_ctrl.Kp = POS_KP_AUTO; pos_ctrl.Kd = POS_KD_AUTO;
        vel_ctrl.Kp = VEL_KP_AUTO; vel_ctrl.Ki = VEL_KI_AUTO; vel_ctrl.Kd = VEL_KD_AUTO;
        /* ── POS_KI gain-schedule (แก้ AUTO สั่นที่ปลายทาง) ──────────────────────
         *  move (|ref_qd|>0.05) → KI เต็ม 8.6 = track แม่น
         *  hold (ref นิ่ง)       → KI ต่ำ + clamp integral = ไม่ wind-up = ไม่สั่น     */
        if (fabsf(ref_qd) > 0.05f) {
            pos_ctrl.Ki = POS_KI_AUTO;
        } else {
            pos_ctrl.Ki = POS_KI_HOLD;
            if      (pos_ctrl.integral >  POS_HOLD_ILIM) pos_ctrl.integral =  POS_HOLD_ILIM;
            else if (pos_ctrl.integral < -POS_HOLD_ILIM) pos_ctrl.integral = -POS_HOLD_ILIM;
        }
    }

    /* ════════════════════════════════════════════════════════════
     *  BACKLASH INVERSE COMPENSATION  (Tao & Kokotovic, 1993)
     *  ────────────────────────────────────────────────────────
     *  ทฤษฎี:  u_c = y_d + (BL/2)·sign(ẏ_d)
     *    y_d   = ref_q      (desired joint position)
     *    ẏ_d   = ref_qd     (desired joint velocity — ใช้ตรวจทิศ)
     *    u_c   = ref_q_comp (compensated reference เข้า pos loop)
     *
     *  เมื่อ ref_qd เปลี่ยนทิศ:
     *    → inject bl_comp = BL_RAD × new_dir เข้า ref_q
     *    → pos loop เห็น error ใหญ่ขึ้น BL_RAD ทันที
     *    → แรงดันเพิ่ม → motor ข้าม dead zone เร็วขึ้น โดยไม่รอ integral wind-up
     *
     *  Clear condition:
     *    qd_out ยืนยันว่า joint เริ่มเคลื่อนในทิศใหม่แล้ว
     *    → backlash ถูก take up แล้ว → reset offset → feedback ทำงานตามปกติ
     * ════════════════════════════════════════════════════════════ */
    {
        int8_t bl_cur_dir = (ref_qd >  BL_VEL_THR) ?  1 :
                            (ref_qd < -BL_VEL_THR) ? -1 : 0;

        if (bl_cur_dir != 0) {
            /* ── ตรวจ direction reversal → ตั้ง target (ramp เข้าหา ไม่ step) ──
             *  gate: inject เฉพาะ move ยาว ≥ 120° (BL_MIN_MOVE_RAD)
             *  move สั้น → ข้าม (bl_target คง 0) → ไม่กระตุก/ไม่สั่น  */
            if (bl_last_dir != 0 && bl_cur_dir != bl_last_dir &&
                g_traj_span_rad >= BL_MIN_MOVE_RAD) {
                bl_target = BL_RAD * (float)bl_cur_dir;
            }
            /* ── joint ยืนยันเคลื่อนในทิศใหม่แล้ว → ตั้ง target กลับ 0 (ramp ถอน) ── */
            if ((bl_target > 0.0f && qd_out >  BL_VEL_THR) ||
                (bl_target < 0.0f && qd_out < -BL_VEL_THR)) {
                bl_target = 0.0f;
            }
            bl_last_dir = bl_cur_dir;
        }

        /* ── ramp bl_comp → bl_target ด้วย rate จำกัด (ทำทุก tick) ── */
        if      (bl_comp < bl_target - BL_STEP) bl_comp += BL_STEP;
        else if (bl_comp > bl_target + BL_STEP) bl_comp -= BL_STEP;
        else                                    bl_comp  = bl_target;
    }
    float ref_q_comp = ref_q + bl_comp;

    /* ════════════════════════════════════════════════════════════
     *  OUTER LOOP: Position  →  generates velocity setpoint
     *  ถ้า pos_ctrl.Kp == 0  →  velocity-only mode (bypass)
     * ════════════════════════════════════════════════════════════ */
    if (pos_ctrl.Kp != 0.0f) {
        /* ── Pos loop divider: คำนวณใหม่ทุก POS_DIV ticks (1 = ทุก 1ms, ไม่ staircase) ── */
        if (++pos_div_tick >= POS_DIV) {
            pos_div_tick = 0;
            float q_error = ref_q_comp - q_out;
            float V_or_qd = calculate_pid(&pos_ctrl, q_error, DT_POS);

            if      (V_or_qd >  10.0f) V_or_qd =  10.0f;
            else if (V_or_qd < -10.0f) V_or_qd = -10.0f;
            pos_div_out = V_or_qd;
        }

        if (modbus_registers[REG_DRIVE_MODE] != 0) {
            /* ════════════════════════════════════════════════════════════
             *  DIRECT POSITION DRIVE  (REG_DRIVE_MODE = 0x3D = 1)
             *  pos_ctrl output → Motor voltage โดยตรง (cached, update 100ms)
             *  ข้าม velocity loop + feed-forward ทั้งหมด
             *  pos_ctrl.Kp หน่วย [V/rad]   (เช่น Kp=10 → ±10V ต่อ 1 rad error)
             * ════════════════════════════════════════════════════════════ */
            Motor_Drive(pos_div_out);
            return;   /* ← ออกจาก function ทันที ไม่รัน velocity loop */
        }

        /* ── Cascade mode: ใช้ cached vel setpoint ── */
        current_q_dot_ref = ref_qd + pos_div_out;

    } else {
        /* ── Velocity-only: reset pos integrator + divider ── */
        pos_ctrl.integral   = 0.0f;
        pos_ctrl.prev_error = 0.0f;
        pos_div_tick        = 0;
        pos_div_out         = 0.0f;
        current_q_dot_ref   = ref_qd;
    }

    /* ════════════════════════════════════════════════════════════
     *  INNER LOOP: Velocity (ทุก 1 ms)
     * ════════════════════════════════════════════════════════════ */
    float qd_error = current_q_dot_ref - qd_out;
    float V_VEL    = calculate_pid(&vel_ctrl, qd_error, DT_VEL);

    /* Feed Forward (velocity) */
    float V_FF = K_ff * ref_qd;

    /* Acceleration Feedforward (inverse dynamics) — pre-supply แรงเร่ง inertia
     * จาก q̈_ref (Quintic) → feedback ไม่ต้องแบก → lag ลด แม่นขึ้น */
    float V_acc = K_ACC * ref_qdd;

    /* ── Coulomb / Deadband Feedforward (noise-free) ────────────────────────
     *  ดัน bias ตามทิศ ref_qd ให้ทะลุ deadband โดยไม่ขยาย measurement noise
     *  ใช้ ref_qd (trajectory, สะอาด) ผ่าน saturated-sign เพื่อ ramp นุ่มใกล้ 0 */
    float qd_n = ref_qd / QD_EPS;
    if      (qd_n >  1.0f) qd_n =  1.0f;
    else if (qd_n < -1.0f) qd_n = -1.0f;
    float V_fric = V_COULOMB * qd_n;

    /*
     *  Disturbance Feed-Forward จาก KF
     *  ────────────────────────────────────────────
     *  tau_d [N.m joint] -> V_dist = tau_d * R / (N * e_eff * Kt)
     *  gain ≈ 53×
     *
     *  KF_Q_TAUD = 1e-8 → tau_d smooth มาก (bandwidth ต่ำ)
     *  ช่วย compensate backlash + friction ที่เปลี่ยนช้า
     *  Safety clamp ±6V ป้องกัน saturate
     */
    float V_dist = hkf.est_disturbance * KF_R / (KF_N * KF_EEFF * KF_KT);
    if      (V_dist >  6.0f) V_dist =  6.0f;
    else if (V_dist < -6.0f) V_dist = -6.0f;

    /* Fade V_dist → 0 เมื่อ velocity ต่ำ (เก็บไว้เผื่อใช้) */
    float vel_fade = fabsf(qd_out) / 1.0f;
    if (vel_fade > 1.0f) vel_fade = 1.0f;
    V_dist *= vel_fade;
    (void)V_dist;   /* ── ปิดใช้งาน V_dist ──
                     *  แกนหมุนระนาบแนวนอน → ไม่มี gravity torque → V_dist
                     *  (KF tau_d observer) ไม่มีอะไรให้ compensate แต่เอา noise
                     *  มาใส่ motor → กระตุกตอน move ใหญ่ (|qd|>1, fade=1)
                     *  V_fric จัดการ deadband แล้ว → ตัด V_dist ออก = เนียน
                     *  ถ้าต้องใช้แกนตั้ง (มี gravity) ค่อยเอา V_dist กลับ        */

    /* 2-DOF: feedforward ล้วน (V_FF+V_acc+V_fric จาก reference สะอาด) + feedback (V_VEL) */

    Motor_Drive(V_VEL + V_FF + V_acc + V_fric);
}

/* ── Open-loop voltage drive (Lab 1 system ID) ───────────────────────────────
 *  จ่าย V ตรงเข้ามอเตอร์ (ไม่มี control loop) + รัน KF observer ด้วย V ที่จ่าย
 *  → ได้ q,qd estimate, เขียน dashboard telemetry, feed 1kHz logger
 *    (logger: input=V[ch5], output q[ch1]/qd[ch3], ref=0)
 * ─────────────────────────────────────────────────────────────────────────── */
void Cascade_OpenLoopVolt(float V)
{
    Encoder_Update(&henc2);
    float z = Encoder_GetPositionRad(&henc2);
    KF_Update(&hkf, V, z);            /* observer ด้วย input ที่รู้ค่า (signed) */
    q_out  = hkf.est_position;
    qd_out = hkf.est_velocity;

    Motor_Drive(V);                   /* apply (dir/clamp) → set monitor_V_signed */

    /* dashboard telemetry (ให้กราฟ live อัปเดตระหว่างทดสอบ) */
    modbus_registers[REG_REF_QD] = 0;
    modbus_registers[REG_QD_OUT] = (uint16_t)(int16_t)(qd_out           * 100.0f);
    modbus_registers[REG_V_IN]   = (uint16_t)(int16_t)(monitor_V_signed * 100.0f);
    modbus_registers[REG_Q_OUT]  = (uint16_t)(int16_t)(q_out            * 100.0f);
    modbus_registers[REG_EST_I]  = (uint16_t)(int16_t)(hkf.est_current  * 1000.0f);
    modbus_registers[REG_REF_Q]  = 0;
}



/* ── 5. E-STOP / RE-HOMING ───────────────────────────────────────────── */
/*
 *  เมื่อ homing เสร็จหรือ e-stop clear -> reset KF ด้วย
 *
 *  ตัวอย่างการเรียกใน main.c หรือ interrupt handler:
 *
 *    extern KalmanFilter_t hkf;
 *    extern Encoder_t      henc2;
 *    Encoder_Update(&henc2);
 *    KF_Reset(&hkf, Encoder_GetPositionRad(&henc2));
 */


/* ── 6. MODBUS MONITOR (ตัวเลือก) ───────────────────────────────────── */
/*
 *  Register map (ตัวอย่าง) -- encode float เป็น int16 (*100 หรือ *1000)
 *
 *  #define REG_KF_POS    10    //  q       *100  [0.01 rad/bit]
 *  #define REG_KF_VEL    11    //  q_dot   *100  [0.01 rad/s per bit]
 *  #define REG_KF_CUR    12    //  i       *1000 [0.001 A per bit]
 *  #define REG_KF_TAUD   13    //  tau_d   *1000 [0.001 N.m per bit]
 *
 *  // เรียกใน while(1) หลัง Heartbeat_Update():
 *  modbus_registers[REG_KF_POS]  = (int16_t)(hkf.est_position    * 100.0f);
 *  modbus_registers[REG_KF_VEL]  = (int16_t)(hkf.est_velocity    * 100.0f);
 *  modbus_registers[REG_KF_CUR]  = (int16_t)(hkf.est_current     * 1000.0f);
 *  modbus_registers[REG_KF_TAUD] = (int16_t)(hkf.est_disturbance * 1000.0f);
 */
