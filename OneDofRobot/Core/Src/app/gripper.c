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

#ifndef REG_BS_GRIPPER_MAN
#define REG_BS_GRIPPER_MAN  0x02
#endif

/* ── Private state ───────────────────────────────────────────────────────── */
static GripperSeqState_t g_state     = G_IDLE;
static uint8_t           g_pick_mode = 0;     /* 1=pick(close) 0=place(open) */
static uint32_t          g_deadline  = 0;     /* HAL_GetTick() step start    */

/* ── Low-level control (active LOW) ──────────────────────────────────────── */
void Gripper_ArmDown(void) { HAL_GPIO_WritePin(gripper_u_d_GPIO_Port, gripper_u_d_Pin, GRIP_ARM_DOWN_LVL); }
void Gripper_ArmUp(void)   { HAL_GPIO_WritePin(gripper_u_d_GPIO_Port, gripper_u_d_Pin, GRIP_ARM_UP_LVL);   }
void Gripper_JawClose(void){ HAL_GPIO_WritePin(gripper_o_c_GPIO_Port, gripper_o_c_Pin, GRIP_JAW_CLOSE_LVL);}
void Gripper_JawOpen(void) { HAL_GPIO_WritePin(gripper_o_c_GPIO_Port, gripper_o_c_Pin, GRIP_JAW_OPEN_LVL); }

/* ── Reed read (1 = triggered) ───────────────────────────────────────────── */
uint8_t Gripper_ReedUp(void)    { return (HAL_GPIO_ReadPin(reed_up_GPIO_Port,    reed_up_Pin)    == REED_ON_STATE) ? 1U : 0U; }
uint8_t Gripper_ReedDown(void)  { return (HAL_GPIO_ReadPin(reed_down_GPIO_Port,  reed_down_Pin)  == REED_ON_STATE) ? 1U : 0U; }
uint8_t Gripper_ReedOpen(void)  { return (HAL_GPIO_ReadPin(reed_open_GPIO_Port,  reed_open_Pin)  == REED_ON_STATE) ? 1U : 0U; }
uint8_t Gripper_ReedClose(void) { return (HAL_GPIO_ReadPin(reed_close_GPIO_Port, reed_close_Pin) == REED_ON_STATE) ? 1U : 0U; }

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

        /* ── แขนลง — รอ reed_down ── */
        case G_SEQ_DOWN:
            Gripper_ArmDown();
            if (Gripper_ReedDown() || (now - g_deadline >= GRIP_ARM_MS)) {
                g_deadline = now;
                g_state    = G_SEQ_ACT;
            }
            break;

        /* ── jaw close/open — รอ reed ── */
        case G_SEQ_ACT:
            if (g_pick_mode) {
                Gripper_JawClose();
                if (Gripper_ReedClose() || (now - g_deadline >= GRIP_ACT_MS)) {
                    g_deadline = now;  g_state = G_SEQ_UP;
                }
            } else {
                Gripper_JawOpen();
                if (Gripper_ReedOpen() || (now - g_deadline >= GRIP_ACT_MS)) {
                    g_deadline = now;  g_state = G_SEQ_UP;
                }
            }
            break;

        /* ── แขนขึ้น — รอ reed_up ── */
        case G_SEQ_UP:
            Gripper_ArmUp();
            if (Gripper_ReedUp() || (now - g_deadline >= GRIP_ARM_MS)) {
                g_state = G_SEQ_DONE;
            }
            break;

        /* ── เสร็จ / idle — ตรวจ manual command ── */
        case G_SEQ_DONE:
            g_state = G_IDLE;
            /* fall-through */
        case G_IDLE: {
            switch (modbus_registers[REG_BS_GRIPPER_MAN]) {
                case GRIP_MAN_DOWN:  Gripper_ArmDown();  break;
                case GRIP_MAN_OPEN:  Gripper_JawOpen();  break;
                case GRIP_MAN_CLOSE: Gripper_JawClose(); break;
                case GRIP_MAN_UP:
                default:             Gripper_ArmUp();    break;
            }
            break;
        }

        default:
            g_state = G_IDLE;
            break;
    }

    _update_reed_reg();
}
