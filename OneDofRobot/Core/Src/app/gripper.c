/*
 * gripper.c
 *
 *  Gripper Driver — pneumatic, active-LOW + reed-switch feedback
 *  ARM = gripper_u_d (PC4), JAW = gripper_o_c (PC10)
 *  reed: reed_up(PC7) reed_down(PA9) reed_open(PB4) reed_close(PB9)
 *
 *  Sequence (reed-confirmed + timeout fallback):
 *    Pick : arm↓ (reed_down) → jaw close (reed_close) → arm↑ (reed_up)
 *    Place: arm↓ (reed_down) → jaw open  (reed_open)  → arm↑ (reed_up)
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include "gripper.h"
#include "base_system.h"   /* REG_BS_GRIPPER_EN, REG_BS_GRIPPER_MAN, REG_BS_REED */
#if GRIPPER_OUTPUT_BACKEND_CAN
#include "can_gripper.h"
#endif

#ifndef REG_BS_GRIPPER_MAN
#define REG_BS_GRIPPER_MAN  0x02
#endif

/* ── Private state ───────────────────────────────────────────────────────── */
static GripperSeqState_t g_state     = G_IDLE;
static uint8_t           g_pick_mode = 0;     /* 1=pick(close) 0=place(open) */
static uint32_t          g_deadline  = 0;     /* HAL_GetTick() step start    */

/* ── Latched manual states — arm กับ jaw แยกอิสระจากกัน ──────────────────────
 *  แก้ปัญหา: joystick ปุ่ม D (arm) กับ base gripper (jaw) เคยใช้ REG 0x02 ร่วมกัน
 *  → สลับค่ากันจน jaw ทำงานตอนกด D. ตอนนี้ latch แยก + apply ทั้งคู่ทุก tick
 *  arm: 0=up 1=down | jaw: 0=open 1=close                                      */
static uint8_t  g_arm_down  = 0;
static uint8_t  g_jaw_close = 0;
static uint16_t g_man_prev  = 0xFFFF;   /* prev REG_BS_GRIPPER_MAN (edge detect) */

/* ── Public setters — joystick/อื่นๆ คุม arm/jaw ตรงๆ ไม่ผ่าน REG 0x02 ── */
void Gripper_SetArm(uint8_t down)  { g_arm_down  = down  ? 1U : 0U; }
void Gripper_SetJaw(uint8_t close) { g_jaw_close = close ? 1U : 0U; }

/* ── Low-level control ───────────────────────────────────────────────────── */
#if GRIPPER_OUTPUT_BACKEND_CAN
void Gripper_ArmDown(void) { CanGripper_SetArmDown(1U); }
void Gripper_ArmUp(void)   { CanGripper_SetArmDown(0U); }
void Gripper_JawClose(void){ CanGripper_SetJawClose(1U); }
void Gripper_JawOpen(void) { CanGripper_SetJawClose(0U); }
#else
void Gripper_ArmDown(void) { HAL_GPIO_WritePin(gripper_u_d_GPIO_Port, gripper_u_d_Pin, GRIP_ARM_DOWN_LVL); }
void Gripper_ArmUp(void)   { HAL_GPIO_WritePin(gripper_u_d_GPIO_Port, gripper_u_d_Pin, GRIP_ARM_UP_LVL);   }
void Gripper_JawClose(void){ HAL_GPIO_WritePin(gripper_o_c_GPIO_Port, gripper_o_c_Pin, GRIP_JAW_CLOSE_LVL);}
void Gripper_JawOpen(void) { HAL_GPIO_WritePin(gripper_o_c_GPIO_Port, gripper_o_c_Pin, GRIP_JAW_OPEN_LVL); }
#endif

/* ── Reed read (1 = triggered) ───────────────────────────────────────────── */
uint8_t Gripper_ReedUp(void)    { return (HAL_GPIO_ReadPin(reed_up_GPIO_Port,    reed_up_Pin)    == REED_ON_STATE) ? 1U : 0U; }
uint8_t Gripper_ReedDown(void)  { return (HAL_GPIO_ReadPin(reed_down_GPIO_Port,  reed_down_Pin)  == REED_ON_STATE) ? 1U : 0U; }
/* reed_open + reed_close รวมเป็น pin เดียว (reed_open_close PB4) — open=active, close=inactive */
uint8_t Gripper_ReedOpen(void)  { return (HAL_GPIO_ReadPin(reed_open_close_GPIO_Port, reed_open_close_Pin) == REED_ON_STATE) ? 1U : 0U; }
uint8_t Gripper_ReedClose(void) { return (HAL_GPIO_ReadPin(reed_open_close_GPIO_Port, reed_open_close_Pin) != REED_ON_STATE) ? 1U : 0U; }

/* ── Update REG_BS_REED (firmware → PC) ──────────────────────────────────── */
static void _update_reed_reg(void)
{
    uint16_t reed = 0;
    if (Gripper_ReedUp())    reed |= GRIP_REED_UP;
    if (Gripper_ReedDown())  reed |= GRIP_REED_DOWN;
    if (Gripper_ReedClose()) reed |= GRIP_REED_CLOSED;
    if (Gripper_ReedOpen())  reed |= GRIP_REED_OPEN;
    modbus_registers[REG_BS_REED] = reed;
}

/* ─────────────────────────────────────────────────────────────────────────── */
void Gripper_Init(void)
{
    g_state     = G_IDLE;
    g_pick_mode = 0;
    g_deadline  = 0;
    g_arm_down  = 0;        /* latch: arm up */
    g_jaw_close = 0;        /* latch: jaw open */
    g_man_prev  = 0xFFFF;
    Gripper_ArmUp();        /* safe: arm up, jaw open */
    Gripper_JawOpen();
    _update_reed_reg();
}

void Gripper_Pick(void)
{
    if (!modbus_registers[REG_BS_GRIPPER_EN]) { g_state = G_SEQ_DONE; return; }
    g_pick_mode = 1;
    g_deadline  = HAL_GetTick();
    g_state     = G_SEQ_DOWN;
}

void Gripper_Place(void)
{
    if (!modbus_registers[REG_BS_GRIPPER_EN]) { g_state = G_SEQ_DONE; return; }
    g_pick_mode = 0;
    g_deadline  = HAL_GetTick();
    g_state     = G_SEQ_DOWN;
}

uint8_t Gripper_IsDone(void)
{
    return (g_state == G_SEQ_DONE || g_state == G_IDLE) ? 1U : 0U;
}

void Gripper_Abort(void)
{
    Gripper_ArmUp();
    Gripper_JawOpen();
    g_arm_down  = 0;        /* latch: arm up, jaw open (safe) */
    g_jaw_close = 0;
    g_state = G_IDLE;
    _update_reed_reg();
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  Gripper_Update — เรียกทุก 1 ms (จาก dwell ใน auto, หรือ manual)
 *    sequence: ยืนยันด้วย reed; ถ้า reed ไม่ trigger ใน timeout → ไปต่อ (fallback)
 * ───────────────────────────────────────────────────────────────────────────*/
void Gripper_Update(void)
{
    uint32_t now = HAL_GetTick();

    switch (g_state) {

        /* ── แขนลง — ค้าง GRIP_DOWN_MS (0.5s) ก่อนค่อยคีบ (กัน "ลงพร้อมหยิบ") ── */
        case G_SEQ_DOWN:
            Gripper_ArmDown();
            if (now - g_deadline >= GRIP_DOWN_MS) {
                g_deadline = now;
                g_state    = G_SEQ_ACT;
            }
            break;

        /* ── jaw close/open — ค้าง GRIP_ACT_MS (0.5s) ให้คีบ/ปล่อยสุดก่อนยกขึ้น ── */
        case G_SEQ_ACT:
            if (g_pick_mode) Gripper_JawClose();
            else             Gripper_JawOpen();
            if (now - g_deadline >= GRIP_ACT_MS) {
                g_deadline = now;
                g_state    = G_SEQ_UP;
            }
            break;

        /* ── แขนขึ้น — ค้าง GRIP_UP_MS (0.5s) ── */
        case G_SEQ_UP:
            Gripper_ArmUp();
            if (now - g_deadline >= GRIP_UP_MS) {
                /* sync latch กับผลลัพธ์ sequence: arm ขึ้น, jaw ตาม pick/place
                 * → IDLE จะ hold สถานะนี้ ไม่ดีดกลับ (เช่น pick แล้วไม่ปล่อย rod) */
                g_arm_down  = 0;
                g_jaw_close = g_pick_mode ? 1U : 0U;
                g_state = G_SEQ_DONE;
            }
            break;

        /* ── เสร็จ / idle — ตรวจ manual command ── */
        case G_SEQ_DONE:
            g_state = G_IDLE;
            /* fall-through */
        case G_IDLE: {
            /* ตรวจ Gripper Sequence (0x03): 1=Pick, 2=Place — BS/joystick ส่งมา */
            uint16_t seq = modbus_registers[REG_BS_GRIPPER_SEQ];
            if (seq == 1U) {
                modbus_registers[REG_BS_GRIPPER_SEQ] = 0;  /* clear pulse */
                modbus_registers[REG_BS_GRIPPER_EN]  = 1;  /* force enable */
                Gripper_Pick();
            } else if (seq == 2U) {
                modbus_registers[REG_BS_GRIPPER_SEQ] = 0;
                modbus_registers[REG_BS_GRIPPER_EN]  = 1;
                Gripper_Place();
            } else {
                /* ── base manual gripper (REG_BS_GRIPPER_MAN 0x02) — edge-triggered ──
                 *  อัปเดต latch แยก arm/jaw (OPEN/CLOSE แตะ jaw เท่านั้น,
                 *  UP/DOWN แตะ arm เท่านั้น) → ไม่ก้าวก่ายกัน
                 *  edge detect: act เฉพาะตอนค่าเปลี่ยน → ไม่ทับ latch ที่ joystick ตั้ง */
                uint16_t man = modbus_registers[REG_BS_GRIPPER_MAN];
                if (man != g_man_prev) {
                    g_man_prev = man;
                    switch (man) {
                        case GRIP_MAN_DOWN:  g_arm_down  = 1U; break;
                        case GRIP_MAN_OPEN:  g_jaw_close = 0U; break;
                        case GRIP_MAN_CLOSE: g_jaw_close = 1U; break;
                        case GRIP_MAN_UP:    g_arm_down  = 0U; break;  /* 0 */
                        default: break;
                    }
                }
                /* apply ทั้ง arm และ jaw จาก latch ทุก tick (อิสระต่อกัน) */
                if (g_arm_down)  Gripper_ArmDown();  else Gripper_ArmUp();
                if (g_jaw_close) Gripper_JawClose(); else Gripper_JawOpen();
            }
            break;
        }

        default:
            g_state = G_IDLE;
            break;
    }

    _update_reed_reg();
}
