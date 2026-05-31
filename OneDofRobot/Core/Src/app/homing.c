/*
 * homing.c
 *  State machine: [H_LEAVE →] H_SEEK → H_COUNT → H_RETURN → H_DONE
 *  เรียกจาก TIM6 ISR ทุก 1 ms (case MODE_HOMING)
 */

#include "homing.h"
#include "main.h"          /* peripheral handles (htim1/htim2) ประกาศ extern ที่นี่ */
#include "encoder.h"
#include "base_system.h"
#include "cascade_control.h"   /* Cascade_Control_Update_FF/Reset — H_UNWIND controlled */
#include "trajectory.h"        /* Septic — unwind 360° ด้วย trajectory                 */

/* ── Tuning ─────────────────────────────────────────────────────────────── */
#define HOMING_SEEK_DUTY    800U    /* 5.5% duty — seek (วิ่งต่อเนื่อง พอ)    */
#define HOMING_LEAVE_DUTY   800U    /* 5.5% duty — ออกจาก zone ก่อน seek     */
#define HOMING_RETURN_DUTY  800U    /* 8.0% duty — กลับทิศจากนิ่ง ต้องสู้
                                     * static friction + เก็บ backlash 2°
                                     * (550 อ่อนไป → ค้าง)                   */
#define HOMING_LEAVE_TICKS  10000U  /* 10 s timeout สำหรับ H_LEAVE           */
#define HOMING_SEEK_TICKS   30000U  /* 20 s timeout                          */
#define HOMING_COUNT_TICKS  15000U  /* 15 s timeout — กันค้างใน H_COUNT      */
#define HOMING_RETURN_TICKS 15000U  /* 15 s timeout — กันค้างใน H_RETURN     */
#define HOMING_SETTLE_TICKS  1000U  /* 1 s รอมอเตอร์หยุดนิ่งก่อน H_COUNT   */
#define HOMING_HALF_REV      4096   /* CPR/2 (8192/2) — seek เกินนี้ = หมุนผิดฝั่ง */
#define HOMING_FULL_REV      8192   /* CPR (360°) — ระยะ unwind                  */
#define HOMING_UNWIND_TICKS  30000U /* 30 s timeout สำหรับ H_UNWIND             */

/* ── Externals ───────────────────────────────────────────────────────────── */
extern Encoder_t henc2;
extern volatile SystemMode_t current_system_mode;

/* ── Module-private state ────────────────────────────────────────────────── */
static HomingState_t hom_state    = H_IDLE;
static uint32_t      hom_ticks    = 0;
static int32_t       zone_count   = 0;
static int32_t       target_count = 0;
static uint8_t       dir_pin        = GPIO_PIN_RESET;
static uint32_t      settle_ticks   = 0;

/* ── Wrong-side detection / 360° unwind ──────────────────────────────────── */
static uint8_t       wrong_side     = 0;   /* 1 = seek หมุนเกิน CPR/2 (ผิดฝั่ง) */
static uint8_t       seek_armed     = 0;   /* 1 = จับ seek_start_pos แล้ว        */
static int32_t       seek_start_pos = 0;   /* ตำแหน่ง encoder ตอนเริ่ม seek      */
static int8_t        seek_dir       = 1;   /* ทิศ encoder ตอน raw_drive (seek): +1/-1 */
static Septic_Profile_t hom_unwind;        /* trajectory สำหรับ unwind 360° (controlled) */

/* ── Debug ───────────────────────────────────────────────────────────────── */
volatile HomingState_t debug_hom_state  = H_IDLE;
volatile uint32_t      debug_hom_ticks  = 0;
volatile uint8_t       debug_hom_signal = 0;
volatile int32_t       debug_zone_count = 0;

/* ── Helpers ─────────────────────────────────────────────────────────────── */
static void raw_drive(uint32_t duty)
{
    HAL_GPIO_WritePin(Motor_Dir_GPIO_Port, Motor_Dir_Pin,
                      (GPIO_PinState)dir_pin);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
}

static void raw_drive_opposite(uint32_t duty)
{
    HAL_GPIO_WritePin(Motor_Dir_GPIO_Port, Motor_Dir_Pin,
                      (dir_pin == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
}

static void raw_stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}

static void zero_encoder(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    henc2.last_counter = 0;
    henc2.position_raw = 0;
    henc2.position_rad = 0.0f;
    henc2.position_deg = 0.0f;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void Homing_Init(void)
{
    hom_state    = H_IDLE;
    hom_ticks    = 0;
    zone_count   = 0;
    target_count = 0;
    dir_pin = GPIO_PIN_RESET;
    Septic_Init(&hom_unwind);
}

void Homing_Start(void)
{
    hom_ticks    = 0;
    zone_count   = 0;
    target_count = 0;
    settle_ticks = 0;
    wrong_side   = 0;
    seek_armed   = 0;
    dir_pin      = GPIO_PIN_RESET;

    /* ถ้า sensor HIGH อยู่แล้ว → ต้องออกจาก zone ก่อน (H_LEAVE)
     * ถ้า LOW → เริ่ม seek ปกติ */
    if (HAL_GPIO_ReadPin(Homing_signal_GPIO_Port, Homing_signal_Pin)
            == GPIO_PIN_SET) {
        hom_state = H_LEAVE;
    } else {
        hom_state = H_SEEK;
    }
}

void Homing_Update(void)
{
    debug_hom_state  = hom_state;
    debug_hom_ticks  = hom_ticks;
    debug_hom_signal = (uint8_t)HAL_GPIO_ReadPin(Homing_signal_GPIO_Port,
                                                   Homing_signal_Pin);
    debug_zone_count = zone_count;

    hom_ticks++;

    switch (hom_state) {

        /* ── H_LEAVE ─────────────────────────────────────────────────────── */
        /* sensor HIGH ตอน start → ขับออกทิศตรงข้ามจน LOW แล้วเข้า H_SEEK   */
        case H_LEAVE:
            raw_drive_opposite(HOMING_LEAVE_DUTY);

            if (HAL_GPIO_ReadPin(Homing_signal_GPIO_Port, Homing_signal_Pin)
                    == GPIO_PIN_RESET) {
                raw_stop();
                hom_ticks = 0;
                settle_ticks = 0;
                hom_state = H_SEEK;
                break;
            }

            if (hom_ticks >= HOMING_LEAVE_TICKS) {
                raw_stop();
                modbus_registers[REG_ESTOP]    = 1;
                current_system_mode            = MODE_MANUAL;
                modbus_registers[REG_SYS_MODE] = MODE_MANUAL;
                hom_state = H_IDLE;
            }
            break;

        /* ── H_SEEK ──────────────────────────────────────────────────────── */
        case H_SEEK:
        	if (settle_ticks < HOMING_SETTLE_TICKS) {
				settle_ticks++;
				break;   /* รอนิ่งก่อน ไม่ขับมอเตอร์ */
			}
            Encoder_Update(&henc2);
            /* จับตำแหน่งเริ่ม seek ครั้งเดียว → วัดระยะหมุนจนเจอ sensor */
            if (!seek_armed) {
                seek_start_pos = henc2.position_raw;
                seek_armed     = 1;
            }
            raw_drive(HOMING_SEEK_DUTY);

            if (HAL_GPIO_ReadPin(Homing_signal_GPIO_Port, Homing_signal_Pin)
                    == GPIO_PIN_SET) {
                raw_stop();
                /* seek travel จาก reference — เกิน CPR/2 = หมุนผิดฝั่ง */
                int32_t travel = henc2.position_raw - seek_start_pos;
                seek_dir = (travel >= 0) ? 1 : -1;   /* ทิศ encoder ของ raw_drive */
                if (travel < 0) travel = -travel;
                wrong_side = (travel > HOMING_HALF_REV) ? 1U : 0U;

                zero_encoder();
                hom_ticks    = 0;
                settle_ticks = 0;
                hom_state    = H_COUNT;
                break;
            }

            if (hom_ticks >= HOMING_SEEK_TICKS) {
                raw_stop();
                modbus_registers[REG_ESTOP]    = 1;
                current_system_mode            = MODE_MANUAL;
                modbus_registers[REG_SYS_MODE] = MODE_MANUAL;
                hom_state = H_IDLE;
            }
            break;

        /* ── H_COUNT ─────────────────────────────────────────────────────── */
        case H_COUNT:
            if (settle_ticks < HOMING_SETTLE_TICKS) {
                settle_ticks++;
                break;   /* รอนิ่งก่อน ไม่ขับมอเตอร์ */
            }
            raw_drive(HOMING_LEAVE_DUTY);
            Encoder_Update(&henc2);

            if (HAL_GPIO_ReadPin(Homing_signal_GPIO_Port, Homing_signal_Pin)
                    == GPIO_PIN_RESET) {
                raw_stop();
                zone_count       = henc2.position_raw;
                target_count     = zone_count / 2;
                debug_zone_count = zone_count;
                hom_ticks = 0;
                settle_ticks = 0;
                hom_state = H_RETURN;
                break;
            }

            if (hom_ticks >= HOMING_COUNT_TICKS) {
                raw_stop();
                modbus_registers[REG_ESTOP]    = 1;
                current_system_mode            = MODE_MANUAL;
                modbus_registers[REG_SYS_MODE] = MODE_MANUAL;
                hom_state = H_IDLE;
            }
            break;

        /* ── H_RETURN ────────────────────────────────────────────────────── */
        case H_RETURN: {

        	if (settle_ticks < HOMING_SETTLE_TICKS) {
				settle_ticks++;
				break;   /* รอนิ่งก่อน ไม่ขับมอเตอร์ */
        	}
			raw_drive_opposite(HOMING_RETURN_DUTY);
            Encoder_Update(&henc2);

            int32_t pos  = henc2.position_raw;
            uint8_t done = (zone_count >= 0)
                           ? (pos <= target_count)
                           : (pos >= target_count);

            if (done) {
                raw_stop();
                if (wrong_side) {
                    /* ถึง flag center แต่หมุนผิดฝั่ง → หมุนกลับ 360° ด้วย trajectory */
                    zero_encoder();            /* encoder = 0 ที่ flag center      */
                    Cascade_Control_Reset();   /* sync KF → 0, clear integrators   */
                    /* target = ทิศตรงข้าม seek, 360° = 2π rad (joint)             */
                    float tgt = -(float)seek_dir * 6.2831853f;
                    Septic_MoveTo(&hom_unwind, 0.0f, tgt, TRAJ_MOVE_TIME);
                    hom_ticks    = 0;
                    settle_ticks = 0;
                    hom_state    = H_UNWIND;
                } else {
                    hom_state = H_DONE;
                }
                break;
            }

            if (hom_ticks >= HOMING_RETURN_TICKS) {
                raw_stop();
                modbus_registers[REG_ESTOP]    = 1;
                current_system_mode            = MODE_MANUAL;
                modbus_registers[REG_SYS_MODE] = MODE_MANUAL;
                hom_state = H_IDLE;
            }
            break;
        }

        /* ── H_UNWIND ────────────────────────────────────────────────────── */
        /* หมุนผิดฝั่ง → หมุนกลับ 360° ด้วย Septic trajectory + cascade control
         * (controlled, เนียน — ไม่ใช่ raw duty) แล้ว set home ที่ปลายทาง        */
        case H_UNWIND: {
        	if (settle_ticks < HOMING_SETTLE_TICKS) {
				settle_ticks++;
				break;   /* รอนิ่งก่อน ไม่ขับมอเตอร์ */
			}
            float rq, rqd, rqdd, rj;
            Septic_Update(&hom_unwind, &rq, &rqd, &rqdd, &rj);
            Cascade_Control_Update_FF(rq, rqd, rqdd);   /* คุมตาม trajectory */

            if (!hom_unwind.is_running) {               /* ครบ 360° */
                zero_encoder();           /* ← set home ที่นี่ */
                Cascade_Control_Reset();  /* sync KF → home = 0 (กัน jump ตอนเข้า AUTO) */
                hom_state = H_DONE;
                break;
            }

            if (hom_ticks >= HOMING_UNWIND_TICKS) {
                raw_stop();
                modbus_registers[REG_ESTOP]    = 1;
                current_system_mode            = MODE_MANUAL;
                modbus_registers[REG_SYS_MODE] = MODE_MANUAL;
                hom_state = H_IDLE;
            }
            break;
        }

        /* ── H_DONE ──────────────────────────────────────────────────────── */
        case H_DONE: {
            zero_encoder();
            /* ออกจาก MODE_HOMING → ตั้ง AUTO เป็น default
             * ISR Priority 2 จะ override เป็น MANUAL เองถ้า selector อยู่ MANUAL */
            current_system_mode            = MODE_AUTO;
            modbus_registers[REG_SYS_MODE] = MODE_AUTO;
            modbus_registers[REG_BS_TASK]  = TASK_IDLE;
            hom_state = H_IDLE;
            break;
        }

        case H_IDLE:
        default:
            break;
    }
}

void Homing_SetHome(void)
{
    zero_encoder();
}

uint8_t Homing_IsDone(void)
{
    return (hom_state == H_IDLE && hom_ticks > 0) ? 1U : 0U;
}
