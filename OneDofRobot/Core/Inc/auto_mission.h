/*
 * auto_mission.h
 *
 *  Created on: 22 พ.ค. 2569
 *      Author: POND
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  Pick & Place Auto Mission  (MODE_AUTO)
 *
 *  State machine ฝั่ง firmware สำหรับงาน Pick & Place จริง
 *  เรียกใช้โดย HAL_TIM_PeriodElapsedCallback → case MODE_AUTO
 *
 *  Profile ที่ใช้:
 *    ทุก move : Septic (7th-order) time-scaled — jerk-continuous (เนียนสุด)
 *
 *  ── Register sources (ตรงกับ Base System จริง) ───────────────────────────
 *  COMMAND (PC/Base System → firmware):
 *    REG_BS_MODE (0x01)   bit 2 = 4 → execute auto P&P
 *    REG_BS_PP_SEQ (0x12–0x1B)  sequence slots (hole index, signed)
 *    REG_BS_PAIR_COUNT (0x22)   number of pick-place pairs
 *    REG_BS_SOFT_STOP (0x25)    bit 0 = 1 → abort
 *
 *  STATUS (firmware → PC/Base System):
 *    REG_BS_TASK (0x27)   current task bits (GoPick / GoPlace / Idle)
 *    REG_BS_POS  (0x28)   position  × 10  [deg]
 *    REG_BS_VEL  (0x29)   velocity  × 10  [rad/s]
 *    REG_BS_ACC  (0x30)   accel     × 10  [rad/s²]
 *
 *  ── Hole-index to angle conversion ──────────────────────────────────────
 *    angle_deg = |hole_index| × HOLE_STEP_DEG
 *    (sign of index encodes CCW/CW — trajectory planner handles direction)
 * ─────────────────────────────────────────────────────────────────────────────
 */

#ifndef AUTO_MISSION_H_
#define AUTO_MISSION_H_

#include <stdint.h>

/* ── ปรับให้ตรงกับ spacing ของรูในชิ้นงานจริง ── */
#define HOLE_STEP_DEG   5.0f    /* องศาต่อ 1 hole index (5° = 1 rod spacing)   */
#define PP_DWELL_MS_DEFAULT  2000U  /* dwell time [ms] ถ้า pair_count ไม่ set  */
#define PP_WAIT_PARAMS  99  /* เพิ่ม State สำหรับรอ Modbus */

/* ─────────────────────────────────────────────────────────────────────────────
 *  P&P State constants  (เขียนลง REG_BS_TASK — PC อ่านได้)
 * ─────────────────────────────────────────────────────────────────────────────*/
#define PP_IDLE           0   /* รอคำสั่ง                                         */
#define PP_MOVE_PICK      1   /* Septic: กำลังเดินไป pick position                */
#define PP_DWELL_PICK     2   /* รอ dwell (gripper ปิด)                           */
#define PP_MOVE_PLACE     3   /* Septic: กำลังพาของไป place position              */
#define PP_DWELL_PLACE    4   /* รอ dwell (gripper เปิด)                          */
#define PP_DONE           5   /* ภารกิจสำเร็จ — hold ตำแหน่งสุดท้าย              */
#define PP_JOG_IDLE       6   /* รอ jog step จาก REG_BS_JOG_DEG (0x05)          */
#define PP_JOG_MOVING     7   /* กำลัง jog (Septic)                              */
#define PP_GO_POINT       8   /* กำลังเดินไป P2P target (Septic)                 */
#define PP_GO_POINT_HOLD  9   /* ถึง target แล้ว holding                         */
#define PP_GO_HOME        10  /* BS Go Home: control กลับ position 0 (ไม่ force seek) */
#define PP_SETTLE_PLACE   11  /* ถึง place แล้ว รอ rod หยุดเหวี่ยง ~2s ก่อนลงวาง   */

/* ─────────────────────────────────────────────────────────────────────────────
 *  API
 * ─────────────────────────────────────────────────────────────────────────────*/
void AutoMission_Init(void);        /* เรียกครั้งเดียวใน main() USER CODE BEGIN 2    */
void AutoMission_Update(void);      /* เรียกทุก 1 ms ใน ISR → case MODE_AUTO        */
void AutoMission_Reset(void);       /* reset state + trajectory (e-stop / mode change)*/
void AutoMission_StartAuto(void);   /* เริ่ม Pick&Place หรือ GoPoint จาก registers  */
void AutoMission_StartJog(void);    /* เข้า Jog mode — รอ REG_BS_JOG_DEG            */
void AutoMission_GoHome(void);      /* BS Go Home: control เคลื่อนกลับ 0 (ไม่ force seek) */

#endif /* AUTO_MISSION_H_ */
