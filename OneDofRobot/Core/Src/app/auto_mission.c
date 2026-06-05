/*
 * auto_mission.c
 *
 *  Created on: 22 พ.ค. 2569
 *      Author: POND
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  Pick & Place Auto Mission — Firmware State Machine
 *
 *  อ่าน command จาก Base System registers (เหมือนกับที่ main.exe ส่งมา)
 *  → firmware นี้ทำงานได้กับทั้ง Dashboard (simulation) และ Base System จริงๆ
 *    โดยไม่ต้องแก้โค้ด
 *
 *  Register interface (Base System compatible):
 *  ──────────────────────────────────────────────────────────────────────────
 *  PC/Dashboard → firmware (WRITE):
 *    0x01  REG_BS_MODE      bit 2 = 4  → start Auto P&P
 *                           bit 0 = 1  → go home
 *    0x12  REG_BS_PP_SEQ   sequence[0…9]  hole index (int16, signed)
 *                           slot[2i]   = pick  index for pair i
 *                           slot[2i+1] = place index for pair i
 *                           sign: +CCW / −CW (ignored — trajectory handles it)
 *    0x22  REG_BS_PAIR_COUNT  number of pick-place pairs (1–5)
 *    0x25  REG_BS_SOFT_STOP   bit 0 = 1 → abort immediately
 *
 *  firmware → PC/Dashboard (READ):
 *    0x27  REG_BS_TASK      bit 1 = GoPick, bit 2 = GoPlace, 0 = Idle
 *    0x28  REG_BS_POS       position  × 10  [int16, deg]
 *    0x29  REG_BS_VEL       velocity  × 10  [int16, rad/s]
 *    0x30  REG_BS_ACC       accel     × 10  [int16, rad/s²]
 *
 *  hole-index → angle:
 *    angle_deg = |index| × HOLE_STEP_DEG   (default 5°/index, ดู auto_mission.h)
 *
 *  Profile ที่ใช้:
 *    ทุก move : Septic (7th-order) time-scaled — jerk-continuous (เนียนสุด)
 *               ถึงเป้าเป๊ะที่ t=T, q⃛=0 ที่ปลาย → ไม่กระชากตอนออก/จอด
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include "auto_mission.h"
#include "base_system.h"        /* REG_BS_*, TASK_*, modbus_registers[]          */
#include "cascade_control.h"    /* Cascade_Control_Update(), q_out, qd_out, qdd_out */
#include "trajectory.h"         /* Septic_Profile_t (time-scaled ทุก move)       */
#include "gripper.h"            /* Gripper_Pick/Place/Update/IsDone — reed feedback */
#include <math.h>               /* fabsf()                                       */

/* ─── External state (cascade_control.c) ────────────────────────────────── */
extern volatile float q_out;    /* actual position [rad]   */
extern volatile float qd_out;   /* actual velocity [rad/s] */
extern          float qdd_out;  /* actual accel    [rad/s²]*/

/* ─── Unit conversions ───────────────────────────────────────────────────── */
#define DEG2RAD  (3.14159265f / 180.0f)
#define RAD2DEG  (180.0f / 3.14159265f)

/* ─── Anti-swing dwell: ถึง place แล้วรอ rod หยุดเหวี่ยงก่อนลงวาง [ms] ──────── */
#define ANTI_SWING_MS  2000U

/* ─── Private trajectory instance ───────────────────────────────────────────
 *  ใช้ Septic (7th-order) กับทุก move — jerk-continuous เนียนสุด
 *  state machine ทำทีละ move → instance เดียวพอ                              */
static Septic_Profile_t pp_septic;   /* time-scaled, จบใน TRAJ_MOVE_TIME */

/* ─── State machine ──────────────────────────────────────────────────────── */
static uint8_t   pp_state       = PP_IDLE;
static uint8_t   pp_pair_idx    = 0;     /* current pair (0-based)          */
static uint8_t   pp_pair_count  = 0;
static uint32_t  pp_settle_t0   = 0;     /* timestamp เริ่ม anti-swing dwell */
/* pp_dwell_start เลิกใช้ — dwell ใช้ Gripper_IsDone() (reed) แทน timer */

/* ─── Decoded sequence (radians, continuous-accumulated targets) ──────────────
 *  pp_pick_rad/pp_place_rad = ตำแหน่งสะสมจริง (อาจ > 2π) → Septic ได้ d ทิศแน่นอน */
#define MAX_PAIRS  8     /* slots 0x12–0x21 = 16 reg = 8 คู่ (pick+place)        */
static float  pp_pick_rad [MAX_PAIRS];
static float  pp_place_rad[MAX_PAIRS];
static float  pp_goto_target = 0.0f;  /* GoPoint / Jog destination [rad] */
static int8_t pp_last_fw_dir = -1;    /* ทิศ firmware leg ล่าสุด (carry สำหรับ hole 0)
                                       *  default −1 = CW (firmware+); ปรับ carry ทิศต่อเนื่อง */

/* ─── Trajectory ref outputs ─────────────────────────────────────────────── */
static float ref_q   = 0.0f;
static float ref_qd  = 0.0f;
static float ref_qdd = 0.0f;
volatile float ref_j = 0.0f;   /* jerk-ref — global volatile สำหรับ CubeMonitor (live ตอน AUTO) */

/* ─────────────────────────────────────────────────────────────────────────────
 *  Private helpers
 * ─────────────────────────────────────────────────────────────────────────────*/

/* ── Directional continuous target (แก้ทิศ P&P มั่ว) ─────────────────────────
 *  base spec 0x12–0x21: magnitude = ตำแหน่งรูสัมบูรณ์ (×5°), sign = ทิศบังคับ
 *    + = CCW, − = CW (README 3.8)
 *
 *  ปัญหาเดิม: Septic_MoveTo(q_out, target_absolute) หมุน "ทางสั้นสุด" (target−q_out)
 *  → ไม่สน sign; ที่ move 180° พอดี ทิศ flip ตาม noise → บางรอบ CW บางรอบ CCW
 *
 *  แก้: คำนวณ target แบบ "continuous-accumulated" — หมุนจาก `from` (ค่า target สะอาด
 *  ของ leg ก่อนหน้า) ไป "ทิศที่สั่ง" จนเจอรู |idx| (mod 360°). Septic จึงได้ d ที่มี
 *  เครื่องหมายแน่นอน (ไม่ flip). idx=0 (home ไม่มี sign) → ใช้ทิศ leg ก่อนหน้า (carry)
 *
 *  ทิศ firmware: base "+"(CCW) → firmware "−"; base "−"(CW) → firmware "+"
 *               (BS_DIR_SIGN=−1; firmware "+" = CW ตามที่วัดจริง)               */
#define PP_TWO_PI   6.2831853f
#define PP_DIR_EPS  0.02f   /* ~1.1° — holes ห่าง 5° → ถ้าใกล้รูในระยะนี้ถือว่า "ถึงแล้ว" */

static float _dir_move_target(float from, int16_t idx)
{
    float mag_rad = (float)(idx < 0 ? -idx : idx) * HOLE_STEP_DEG * DEG2RAD;
    float A       = mag_rad * BS_DIR_SIGN;   /* firmware representative angle ของรู (wrapped) */

    int8_t fw_dir;
    if      (idx > 0) fw_dir = (BS_DIR_SIGN < 0.0f) ? -1 : +1;   /* base CCW */
    else if (idx < 0) fw_dir = (BS_DIR_SIGN < 0.0f) ? +1 : -1;   /* base CW  */
    else              fw_dir = pp_last_fw_dir;                    /* hole 0: carry ทิศเดิม */
    pp_last_fw_dir = fw_dir;

    float k = fmodf(A - from, PP_TWO_PI);
    if (fw_dir > 0) {
        if (k < 0.0f) k += PP_TWO_PI;                       /* [0, 2π) — หมุนบวก */
        if (k < PP_DIR_EPS || k > PP_TWO_PI - PP_DIR_EPS) k = 0.0f;  /* อยู่ที่รูแล้ว → นิ่ง */
    } else {
        if (k > 0.0f) k -= PP_TWO_PI;                       /* (−2π, 0] — หมุนลบ */
        if (k > -PP_DIR_EPS || k < -(PP_TWO_PI - PP_DIR_EPS)) k = 0.0f;
    }
    return from + k;
}

/* อัปเดต REG_BS_TASK + telemetry registers ทุก tick */
static void _write_telemetry(uint16_t task_bits)
{
    modbus_registers[REG_BS_TASK] = task_bits;
    /* telemetry กลับทิศให้ตรง convention ของ base (BS_DIR_SIGN) */
    modbus_registers[REG_BS_POS]  = (uint16_t)(int16_t)(q_out   * RAD2DEG * 10.0f * BS_DIR_SIGN); /* deg ×10   */
    modbus_registers[REG_BS_VEL]  = (uint16_t)(int16_t)(qd_out            * 10.0f * BS_DIR_SIGN); /* rad/s ×10 */
    modbus_registers[REG_BS_ACC]  = (uint16_t)(int16_t)(qdd_out           * 10.0f * BS_DIR_SIGN); /* rad/s² ×10*/
}

static void _set_state(uint8_t new_state)
{
    pp_state = new_state;
}

/* โหลด sequence จาก Modbus (0x12–0x21) → pp_pick_rad[], pp_place_rad[]
 *  สร้าง target แบบ "continuous-accumulated" (chain) จาก anchor (ตำแหน่งจริงตอนเริ่ม)
 *  → แต่ละ leg หมุนตามทิศที่ base สั่ง(sign ของ index) ไม่ใช่ทางสั้นสุด → ทิศไม่มั่ว  */
static void _load_sequence(float anchor)
{
    pp_pair_count = (uint8_t)modbus_registers[REG_BS_PAIR_COUNT];

    if (pp_pair_count > MAX_PAIRS)
        pp_pair_count = MAX_PAIRS;

    pp_last_fw_dir = -1;          /* default CW (firmware+) สำหรับ hole-0 leg แรก */
    float cursor   = anchor;      /* clean accumulation cursor (ไม่อ่าน q_out ซ้ำ → ไม่ flip) */

    for (uint8_t i = 0; i < pp_pair_count; i++) {
        int16_t pick_idx  = (int16_t)modbus_registers[REG_BS_PP_SEQ + i * 2];
        int16_t place_idx = (int16_t)modbus_registers[REG_BS_PP_SEQ + i * 2 + 1];

        pp_pick_rad[i]  = _dir_move_target(cursor, pick_idx);
        cursor          = pp_pick_rad[i];
        pp_place_rad[i] = _dir_move_target(cursor, place_idx);
        cursor          = pp_place_rad[i];
    }
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  AutoMission_Init
 * ─────────────────────────────────────────────────────────────────────────────*/
void AutoMission_Init(void)
{
    Septic_Init(&pp_septic);

    pp_state     = PP_IDLE;
    pp_pair_idx  = 0;
    ref_q = ref_qd = ref_qdd = ref_j = 0.0f;

    /* เขียน telemetry เริ่มต้น */
    _write_telemetry(TASK_IDLE);
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  AutoMission_Reset  — เรียกเมื่อ e-stop / ออกจาก MODE_AUTO
 * ─────────────────────────────────────────────────────────────────────────────*/
void AutoMission_Reset(void)
{
    pp_septic.is_running = 0;
    pp_state    = PP_IDLE;
    pp_pair_idx = 0;
    ref_q = ref_qd = ref_qdd = ref_j = 0.0f;

    Gripper_Abort();           /* arm up, jaw open — safe */
    _write_telemetry(TASK_IDLE);

    /* hold ตำแหน่งปัจจุบัน */
    Cascade_Control_Update(q_out, 0.0f);
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  AutoMission_StartAuto  — เรียกจาก main.c เมื่อรับ REG_BS_MODE_AUTO
 *  ถ้า pair_count > 0 → Pick & Place; ถ้า == 0 → GoPoint (0x23/0x24)
 * ─────────────────────────────────────────────────────────────────────────────*/
void AutoMission_StartAuto(void)
{
    /* main.c รอ 150ms ให้แล้ว → โหลด params ได้เลย */

    pp_pair_count = (uint8_t)modbus_registers[REG_BS_PAIR_COUNT];
    if (pp_pair_count > MAX_PAIRS) pp_pair_count = MAX_PAIRS;

    if (pp_pair_count > 0) {
        /* Pick & Place mode — สร้าง chain จาก anchor = ตำแหน่งจริงตอนเริ่ม */
        _load_sequence(q_out);
        pp_pair_idx = 0;

        Septic_MoveTo(&pp_septic, q_out, pp_pick_rad[0], TRAJ_MOVE_TIME);
        _set_state(PP_MOVE_PICK);
    }
    else {
        /* GoPoint mode */
        uint16_t unit = modbus_registers[REG_BS_P2P_UNIT];
        int16_t raw   = (int16_t)modbus_registers[REG_BS_P2P_TARGET];

        if (unit == 0) {
            /* degree — absolute target ตามทิศของ sign (กลับทิศ base→firmware) */
            pp_goto_target = (float)raw * DEG2RAD * BS_DIR_SIGN;
            Septic_MoveTo(&pp_septic, q_out, pp_goto_target, TRAJ_MOVE_TIME);
        } else {
            /* index — หมุนตามทิศ sign ไปถึงรู (directional, ไม่ใช่ทางสั้นสุด) */
            pp_last_fw_dir = -1;
            pp_goto_target = _dir_move_target(q_out, raw);
            Septic_MoveTo(&pp_septic, q_out, pp_goto_target, TRAJ_MOVE_TIME);
        }
        _set_state(PP_GO_POINT);
    }
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  AutoMission_StartJog  — เรียกจาก main.c เมื่อรับ REG_BS_MODE_JOG
 * ─────────────────────────────────────────────────────────────────────────────*/
void AutoMission_StartJog(void)
{
    _set_state(PP_JOG_IDLE);
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  AutoMission_GoHome  — BS "Go Home": ใช้ cascade control เคลื่อนกลับ position 0
 *  ต่างจาก force homing (boot): ไม่ raw-seek หา flag — แค่ controlled move ไป 0
 *  (encoder zero ถูกตั้งไว้แล้วจาก force homing ตอนเริ่ม)
 * ─────────────────────────────────────────────────────────────────────────────*/
void AutoMission_GoHome(void)
{
    pp_goto_target = 0.0f;
    Septic_MoveTo(&pp_septic, q_out, 0.0f, TRAJ_MOVE_TIME);
    _set_state(PP_GO_HOME);
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  AutoMission_Update  — เรียกทุก 1 ms (case MODE_AUTO ใน ISR)
 * ─────────────────────────────────────────────────────────────────────────────*/
void AutoMission_Update(void)
{
    /* ── 1. Soft Stop / E-Stop → หยุดทันที ──────────────────────────────── */
    if ((modbus_registers[REG_BS_SOFT_STOP] & 0x0001) ||
        (modbus_registers[REG_ESTOP]        != 0)) {
        AutoMission_Reset();
        return;
    }

    /* ── 2. State Machine ───────────────────────────────────────────────── */
    switch (pp_state) {

        /* ════════════════════════════════════════════════════════════════════
         *  IDLE — hold ตำแหน่ง รอ main.c เรียก StartAuto / StartJog
         * ════════════════════════════════════════════════════════════════════*/
        case PP_IDLE:
			_write_telemetry(TASK_IDLE);
			Cascade_Control_Update(q_out, 0.0f);
			Gripper_Update();   /* base MANUAL tab gripper (0x02/0x03) ทำงานตอน AUTO idle ด้วย */
			break;

        /* ════════════════════════════════════════════════════════════════════
         *  MOVE_PICK — Septic เดินไป pick position (เร็ว, ไม่มีของ)
         * ════════════════════════════════════════════════════════════════════*/
        case PP_MOVE_PICK:
            Septic_Update(&pp_septic, &ref_q, &ref_qd, &ref_qdd, &ref_j);
            Cascade_Control_Update_FF(ref_q, ref_qd, ref_qdd);
            _write_telemetry(TASK_GO_PICK);

            if (!pp_septic.is_running) {
                Cascade_Flush_VelIntegral();
                Gripper_Pick();          /* เริ่ม pick: arm↓ → jaw close → arm↑ */
                _set_state(PP_DWELL_PICK);
            }
            break;

        /* ════════════════════════════════════════════════════════════════════
         *  DWELL_PICK — hold ที่ pick, รัน gripper pick (reed-confirmed)
         * ════════════════════════════════════════════════════════════════════*/
        case PP_DWELL_PICK:
            Cascade_Control_Update(pp_pick_rad[pp_pair_idx], 0.0f);
            _write_telemetry(TASK_GO_PICK);
            Gripper_Update();

            if (Gripper_IsDone()) {
                /* gripper เสร็จ — เดินไป place */
                Septic_MoveTo(&pp_septic,
                               pp_pick_rad[pp_pair_idx],
                               pp_place_rad[pp_pair_idx], TRAJ_MOVE_TIME);
                _set_state(PP_MOVE_PLACE);
            }
            break;

        /* ════════════════════════════════════════════════════════════════════
         *  MOVE_PLACE — Septic พาของไป place (smooth)
         * ════════════════════════════════════════════════════════════════════*/
        case PP_MOVE_PLACE:
            Septic_Update(&pp_septic, &ref_q, &ref_qd, &ref_qdd, &ref_j);
            Cascade_Control_Update_FF(ref_q, ref_qd, ref_qdd);
            _write_telemetry(TASK_GO_PLACE);

            if (!pp_septic.is_running) {
                Cascade_Flush_VelIntegral();
                pp_settle_t0 = HAL_GetTick();   /* เริ่มจับเวลา anti-swing */
                _set_state(PP_SETTLE_PLACE);     /* รอ rod หยุดเหวี่ยงก่อนค่อยลงวาง */
            }
            break;

        /* ════════════════════════════════════════════════════════════════════
         *  SETTLE_PLACE — ถึง place แล้ว hold ตำแหน่ง รอ rod หยุดเหวี่ยง ~2s
         *                 (แขนยังยกถือ rod อยู่) แล้วค่อยเริ่ม place sequence
         * ════════════════════════════════════════════════════════════════════*/
        case PP_SETTLE_PLACE:
            Cascade_Control_Update(pp_place_rad[pp_pair_idx], 0.0f);
            _write_telemetry(TASK_GO_PLACE);
            if (HAL_GetTick() - pp_settle_t0 >= ANTI_SWING_MS) {
                Gripper_Place();         /* เริ่ม place: arm↓ → jaw open → arm↑ */
                _set_state(PP_DWELL_PLACE);
            }
            break;

        /* ════════════════════════════════════════════════════════════════════
         *  DWELL_PLACE — hold ที่ place, รัน gripper place (reed-confirmed)
         * ════════════════════════════════════════════════════════════════════*/
        case PP_DWELL_PLACE:
            Cascade_Control_Update(pp_place_rad[pp_pair_idx], 0.0f);
            _write_telemetry(TASK_GO_PLACE);
            Gripper_Update();

            if (Gripper_IsDone()) {
                pp_pair_idx++;

                if (pp_pair_idx < pp_pair_count) {
                    /* pair ถัดไป — กลับไป pick */
                    Septic_MoveTo(&pp_septic,
                                   pp_place_rad[pp_pair_idx - 1],
                                   pp_pick_rad[pp_pair_idx], TRAJ_MOVE_TIME);
                    _set_state(PP_MOVE_PICK);
                } else {
                    /* ครบทุก pair → กลับ home (position 0) ตามสเปก base system */
                    pp_goto_target = 0.0f;
                    Septic_MoveTo(&pp_septic,
                                   pp_place_rad[pp_pair_idx - 1],
                                   0.0f, TRAJ_MOVE_TIME);
                    _set_state(PP_GO_HOME);
                }
            }
            break;

        /* ════════════════════════════════════════════════════════════════════
         *  DONE — ภารกิจสำเร็จ, hold ตำแหน่งสุดท้าย
         *         รอ main.c เรียก StartAuto ใหม่เมื่อ Base System ส่งคำสั่ง
         * ════════════════════════════════════════════════════════════════════*/
        case PP_DONE:
            _write_telemetry(TASK_IDLE);
            Cascade_Control_Update(q_out, 0.0f);
            Gripper_Update();   /* manual gripper ยังสั่งได้หลังจบ mission */
            break;

        /* ════════════════════════════════════════════════════════════════════
         *  JOG_IDLE — รอ jog step จาก REG_BS_JOG_DEG (0x05)
         * ════════════════════════════════════════════════════════════════════*/
        case PP_JOG_IDLE:
            _write_telemetry(TASK_IDLE);
            Cascade_Control_Update(q_out, 0.0f);
            Gripper_Update();   /* base MANUAL tab: gripper 6 ปุ่ม (0x02/0x03) ทำงานในโหมด manual/jog */
            {
                int16_t jog_step = (int16_t)modbus_registers[REG_BS_JOG_DEG];
                if (jog_step != 0) {
                    modbus_registers[REG_BS_JOG_DEG] = 0;
                    pp_goto_target = q_out + (float)jog_step * DEG2RAD;
                    Septic_MoveTo(&pp_septic, q_out, pp_goto_target, TRAJ_MOVE_TIME);
                    _set_state(PP_JOG_MOVING);
                }
            }
            break;

        /* ════════════════════════════════════════════════════════════════════
         *  JOG_MOVING — Septic ไปยัง jog target แล้วกลับ JOG_IDLE
         * ════════════════════════════════════════════════════════════════════*/
        case PP_JOG_MOVING:
            Septic_Update(&pp_septic, &ref_q, &ref_qd, &ref_qdd, &ref_j);
            Cascade_Control_Update_FF(ref_q, ref_qd, ref_qdd);
            _write_telemetry(TASK_GO_POINT);
            Gripper_Update();   /* manual gripper ระหว่าง jog (base MANUAL tab) */
            if (!pp_septic.is_running) {
                Cascade_Flush_VelIntegral();
                _set_state(PP_JOG_IDLE);
            }
            break;

        /* ════════════════════════════════════════════════════════════════════
         *  GO_POINT — Septic ไปยัง P2P target
         * ════════════════════════════════════════════════════════════════════*/
        case PP_GO_POINT:
            Septic_Update(&pp_septic, &ref_q, &ref_qd, &ref_qdd, &ref_j);
            Cascade_Control_Update_FF(ref_q, ref_qd, ref_qdd);
            _write_telemetry(TASK_GO_POINT);
            if (!pp_septic.is_running) {
                Cascade_Flush_VelIntegral();
                _set_state(PP_GO_POINT_HOLD);
            }
            break;

        /* ════════════════════════════════════════════════════════════════════
         *  GO_POINT_HOLD — ถึง target แล้ว hold จนได้รับคำสั่งใหม่
         * ════════════════════════════════════════════════════════════════════*/
        case PP_GO_POINT_HOLD:
            _write_telemetry(TASK_IDLE);
            Cascade_Control_Update(pp_goto_target, 0.0f);
            Gripper_Update();   /* manual gripper ยังสั่งได้ตอน hold */
            break;

        /* ════════════════════════════════════════════════════════════════════
         *  GO_HOME — BS Go Home: control เคลื่อนกลับ 0 (Septic) แล้ว hold
         *            ต่างจาก force homing (boot) — ไม่ raw-seek หา flag
         * ════════════════════════════════════════════════════════════════════*/
        case PP_GO_HOME:
            Septic_Update(&pp_septic, &ref_q, &ref_qd, &ref_qdd, &ref_j);
            Cascade_Control_Update_FF(ref_q, ref_qd, ref_qdd);
            _write_telemetry(TASK_HOMING);
            if (!pp_septic.is_running) {
                Cascade_Flush_VelIntegral();
                _set_state(PP_GO_POINT_HOLD);   /* hold ที่ 0 (pp_goto_target=0) */
            }
            break;

        default:
            _set_state(PP_IDLE);
            break;
    }
}
