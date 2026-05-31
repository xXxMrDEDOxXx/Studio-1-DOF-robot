/*
 * gripper.h
 *
 *  Gripper Driver — pneumatic, active-LOW control + reed-switch feedback
 * ─────────────────────────────────────────────────────────────────────────────
 *  Control outputs (main.h labels, active LOW = LOW สั่งทำงาน):
 *    ARM up/down  = gripper_u_d  (PC4)
 *        LOW (RESET) = DOWN (energize) | HIGH (SET) = UP   (default/safe)
 *    JAW open/close = gripper_o_c (PC10)
 *        LOW (RESET) = CLOSE         | HIGH (SET) = OPEN  (default/safe)
 *    ⚠️ ถ้าทิศกลับ (ลงเป็นขึ้น/ปิดเป็นเปิด) สลับ GRIP_*_ACTIVE ด้านล่าง
 *
 *  Reed-switch feedback (input):
 *    reed_up   (PC7)  | reed_down (PA9) | reed_open (PB4) | reed_close (PB9)
 *    REED_ON_STATE = level ที่ถือว่า "triggered" (ปรับถ้า logic กลับ)
 *
 *  Auto sequence (ใช้ reed ยืนยัน + timeout fallback):
 *    Pick :  Arm Down → Jaw Close → Arm Up
 *    Place:  Arm Down → Jaw Open  → Arm Up
 *
 *  Modbus:
 *    REG_BS_GRIPPER_MAN (0x02) manual: 0=Up 1=Down 2=Open 4=Close
 *    REG_BS_GRIPPER_EN  (0x04) 0=disable in AUTO  1=enable
 *    REG_BS_REED        (0x26) reed state bits (firmware เขียน)
 * ─────────────────────────────────────────────────────────────────────────────
 */

#ifndef GRIPPER_H_
#define GRIPPER_H_

#include "main.h"
#include <stdint.h>

/* ── Active levels (สลับได้ถ้าทิศกลับ) ───────────────────────────────────── */
#define GRIP_ARM_DOWN_LVL   GPIO_PIN_RESET   /* LOW = down */
#define GRIP_ARM_UP_LVL     GPIO_PIN_SET     /* HIGH = up  */
#define GRIP_JAW_CLOSE_LVL  GPIO_PIN_RESET   /* LOW = close */
#define GRIP_JAW_OPEN_LVL   GPIO_PIN_SET     /* HIGH = open */
#define REED_ON_STATE       GPIO_PIN_RESET   /* reed triggered = LOW (สลับถ้ากลับ) */

/* ── Timing: timeout fallback ถ้า reed ไม่ trigger [ms] ──────────────────── */
#define GRIP_ARM_MS   800U   /* เวลาสูงสุดรอแขนถึงที่ (down/up)  */
#define GRIP_ACT_MS   600U   /* เวลาสูงสุดรอ jaw (close/open)    */

/* ── Manual command codes (REG_BS_GRIPPER_MAN 0x02) ──────────────────────── */
#define GRIP_MAN_UP     0U
#define GRIP_MAN_DOWN   1U
#define GRIP_MAN_OPEN   2U
#define GRIP_MAN_CLOSE  4U

/* ── REG_BS_REED bit map (อ่านจาก PC) ───────────────────────────────────── */
#define GRIP_REED_UP     0x0001U
#define GRIP_REED_DOWN   0x0002U
#define GRIP_REED_CLOSED 0x0004U
#define GRIP_REED_OPEN   0x0008U

/* ── Sequence states ─────────────────────────────────────────────────────── */
typedef enum {
    G_IDLE = 0,
    G_SEQ_DOWN,   /* แขนลง — รอ reed_down  */
    G_SEQ_ACT,    /* jaw close/open — รอ reed */
    G_SEQ_UP,     /* แขนขึ้น — รอ reed_up  */
    G_SEQ_DONE,
} GripperSeqState_t;

/* ── Public API ──────────────────────────────────────────────────────────── */
void    Gripper_Init(void);
void    Gripper_Update(void);    /* เรียกทุก 1 ms (จาก dwell ใน auto / manual) */
void    Gripper_Pick(void);      /* เริ่ม pick: down→close→up                 */
void    Gripper_Place(void);     /* เริ่ม place: down→open→up                 */
uint8_t Gripper_IsDone(void);    /* 1 = sequence เสร็จ                        */
void    Gripper_Abort(void);     /* หยุด → arm up, jaw open (safe)            */

/* Direct control (manual) */
void    Gripper_JawOpen(void);
void    Gripper_JawClose(void);
void    Gripper_ArmUp(void);
void    Gripper_ArmDown(void);

/* Reed read helpers (1 = triggered) */
uint8_t Gripper_ReedUp(void);
uint8_t Gripper_ReedDown(void);
uint8_t Gripper_ReedOpen(void);
uint8_t Gripper_ReedClose(void);

#endif /* GRIPPER_H_ */
