/*
 * base_system.h
 *
 *  Created on: 15 พ.ค. 2569
 *      Author: POND
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  Shared system definitions:
 *    - Modbus parameters & register map
 *    - System mode enum (Homing / Auto / Manual)
 *    - Heartbeat API
 * ─────────────────────────────────────────────────────────────────────────────
 */

#ifndef BASE_SYSTEM_H_
#define BASE_SYSTEM_H_

#include <stdint.h>
#include <stdbool.h>

/* ─────────────────────────────────────────────────────────────────────────────
 *  Modbus RTU configuration
 * ─────────────────────────────────────────────────────────────────────────────*/
#define SLAVE_ID            21
#define HEARTBEAT_REG_ADDR  0x00    /* PC ← STM32: ROBOT_SAYS_YA / PC_SAYS_HI */
#define ROBOT_SAYS_YA       22881
#define PC_SAYS_HI          18537
#define HEARTBEAT_TIMEOUT   2000    /* ms */

/* ─────────────────────────────────────────────────────────────────────────────
 *  Modbus Register Map
 *
 *  BASE SYSTEM (0x00–0x31) — ใช้โดย main.exe ใน MODE_AUTO  [ห้ามแตะ]
 *    WRITE: 0x00 heartbeat, 0x01 BS_mode (1=Home,2=Jog,4=Auto,8=SetHome,16=Test)
 *           0x02–0x04 gripper, 0x05 jog_deg, 0x06 test_type
 *           0x07 perf_vel, 0x08 perf_acc, 0x09 prec_init
 *           0x10 prec_final, 0x11 prec_repeat
 *           0x12–0x21 pick&place sequence (16 slots)
 *           0x22 pair_count, 0x23 P2P_unit, 0x24 P2P_target, 0x25 soft_stop
 *    READ:  0x26 reed_sensors, 0x27 current_task
 *           0x28 pos÷10, 0x29 vel÷10, 0x30 acc÷10, 0x31 emergency
 *
 *  SHARED (0x31–0x32)
 *    0x31 ESTOP — ใช้ร่วมกัน (Base System = emergency, Dashboard = e-stop)
 *    0x32 SYS_MODE — physical selector switch mode (STM32 เขียนฝ่ายเดียว)
 *
 *  DASHBOARD WRITE (0x33–0x3E) — PC→STM32, ใช้เฉพาะ MODE_MANUAL
 *  DASHBOARD READ  (0x3F–0x44) — STM32→PC, ใช้เฉพาะ MODE_MANUAL
 *  STATUS          (0x45)       — ISR counter
 *
 *  modbus_registers[] size = 0x46 = 70  (ครอบคลุมทุก address)
 * ─────────────────────────────────────────────────────────────────────────────*/

/* ── BASE SYSTEM WRITE (PC → STM32) ─────────────────────────────────────────
 *  ใช้ใน auto_mission.c เพื่ออ่าน command จาก Base System / Dashboard
 * ─────────────────────────────────────────────────────────────────────────── */
#define REG_HEARTBEAT       0x00  /* PC↔STM32 handshake                        */
#define REG_BS_MODE           0x01  /* Mode cmd (one-hot)                       */
#define REG_BS_MODE_HOME      0x0001  /* bit 0: Go home (full sequence)          */
#define REG_BS_MODE_JOG       0x0002  /* bit 1: Manual / Jog                     */
#define REG_BS_MODE_AUTO      0x0004  /* bit 2: Auto mission                     */
#define REG_BS_MODE_SET_HOME  0x0008  /* bit 3: Zero encoder at current pos      */
#define REG_BS_MODE_TEST      0x0010  /* bit 4: Test mode                        */
#define REG_BS_GRIPPER_MAN  0x02  /* Manual gripper: 0=Up 1=Down 2=Open 4=Close*/
#define REG_BS_GRIPPER_EN   0x04  /* 0=gripper off  1=gripper on (in AUTO)     */
#define REG_BS_JOG_DEG      0x05  /* Jog step  [deg, int16 signed]             */
#define REG_BS_TEST_TYPE    0x06  /* 0=Precision  1=Performance                */
#define REG_BS_PERF_VEL     0x07  /* Performance: desired vel   [deg/s, int16] */
#define REG_BS_PERF_ACC     0x08  /* Performance: desired accel [deg/s², int16]*/
#define REG_BS_PREC_INIT    0x09  /* Precision: init position   [deg, int16]   */
#define REG_BS_PREC_FINAL   0x10  /* Precision: final position  [deg, int16]   */
#define REG_BS_PREC_RPT     0x11  /* Precision: repeat count; >0=deg, <0=index */
#define REG_BS_PP_SEQ       0x12  /* P&P sequence start (10 slots: 0x12–0x1B)  */
                                  /*   slot[2i]   = pick  hole-index (signed)  */
                                  /*   slot[2i+1] = place hole-index (signed)  */
                                  /*   sign: + = CCW,  − = CW                 */
#define REG_BS_PAIR_COUNT   0x22  /* Number of pick-place pairs (uint16)       */
#define REG_BS_P2P_UNIT     0x23  /* P2P unit: 0=degree  1=index               */
#define REG_BS_P2P_TARGET   0x24  /* P2P target [deg or index, int16 signed]   */
#define REG_BS_SOFT_STOP    0x25  /* bit 0 = 1 → soft stop                    */

/* ── BASE SYSTEM READ (STM32 → PC) ──────────────────────────────────────────
 *  firmware เขียนค่าเหล่านี้เพื่อให้ Base System / Dashboard อ่าน
 * ─────────────────────────────────────────────────────────────────────────── */
#define REG_BS_REED         0x26  /* Reed/limit sensors (bits)                 */
#define REG_BS_TASK         0x27  /* Current task bits: 1=Homing 2=GoPick      */
                                  /*                    4=GoPlace  8=GoPoint   */
#define REG_BS_POS          0x28  /* Position  × 10  [int16, deg]              */
#define REG_BS_VEL          0x29  /* Velocity  × 10  [int16, deg/s]            */
#define REG_BS_ACC          0x30  /* Accel     × 10  [int16, deg/s²]           */

/* task bit masks สำหรับ REG_BS_TASK */
#define TASK_HOMING         0x0001
#define TASK_GO_PICK        0x0002
#define TASK_GO_PLACE       0x0004
#define TASK_GO_POINT       0x0008
#define TASK_IDLE           0x0000

/* ── SHARED STATUS ──────────────────────────────────────────────────────────── */
#define REG_ESTOP           0x31  /* E-Stop flag: 0=OK  1=Active  (shared)    */
#define REG_SYS_MODE        0x32  /* Physical mode: 0=Homing 1=Auto 2=Manual  */

/* ── DASHBOARD WRITE (PC → STM32, MODE_MANUAL only) ────────────────────────── */
#define REG_VEL_KP          0x33  /* Velocity Kp × 100                        */
#define REG_VEL_KI          0x34  /* Velocity Ki × 100                        */
#define REG_VEL_KD          0x35  /* Velocity Kd × 100                        */
#define REG_SPEED           0x36  /* Speed [rad/s × 10]                       */
#define REG_HALF_PERIOD     0x37  /* Half-period of waveform [ms]             */
#define REG_WAVEFORM        0x38  /* Waveform: 0=Square  1=Sine  2=Step       */
#define REG_RUN             0x39  /* Run flag: 0=Stop  1=Run                  */
#define REG_POS_KP          0x3A  /* Position Kp × 100  (0 = disable pos loop)*/
#define REG_POS_KI          0x3B  /* Position Ki × 100                        */
#define REG_POS_KD          0x3C  /* Position Kd × 100                        */
#define REG_DRIVE_MODE      0x3D  /* Drive: 0=Cascade(pos→vel→V) 1=Direct(pos→V)*/
#define REG_TARGET_POS      0x3E  /* Target position [deg × 10, int16 signed] */

/* ── DASHBOARD READ (STM32 → PC, MODE_MANUAL only) ─────────────────────────── */
#define REG_REF_QD          0x3F  /* Velocity reference  × 100  [int16, rad/s]*/
#define REG_QD_OUT          0x40  /* Actual velocity     × 100  [int16, rad/s]*/
#define REG_V_IN            0x41  /* Motor voltage (mag) × 100  [int16, V]    */
#define REG_Q_OUT           0x42  /* Actual position     × 100  [int16, rad]  */
#define REG_EST_I           0x43  /* Estimated current   × 1000 [int16, A]    */
#define REG_REF_Q           0x44  /* Position reference  × 100  [int16, rad]  */

/* ── STATUS ─────────────────────────────────────────────────────────────────── */
#define REG_ISR_CNT         0x45  /* ISR call counter   [uint16, wraps at 65535]*/

/* ── AUTO MISSION / PICK & PLACE (0x46–0x50) ────────────────────────────────── */
/*    WRITE (PC → STM32):                                                         */
#define REG_PP_CMD          0x46  /* Command: 0=idle  1=start  2=stop            */
#define REG_PP_PICK         0x47  /* Pick  position [deg × 10, int16 signed]     */
#define REG_PP_PLACE1       0x48  /* Place 1 pos    [deg × 10, int16 signed]     */
#define REG_PP_PLACE2       0x49  /* Place 2 pos    [deg × 10, int16 signed]     */
#define REG_PP_PLACE3       0x4A  /* Place 3 pos    [deg × 10, int16 signed]     */
#define REG_PP_PLACE4       0x4B  /* Place 4 pos    [deg × 10, int16 signed]     */
#define REG_PP_N_RODS       0x4C  /* Number of rods 1–4                          */
#define REG_PP_DWELL_MS     0x4D  /* Dwell time [ms] (grip open/close wait)      */
#define REG_PP_THRESH       0x4E  /* Arrival threshold [deg × 10, uint16]        */
#define REG_PP_REPEAT       0x4F  /* 0=one-shot  1=repeat cycle                  */
/*    READ (STM32 → PC):                                                           */
#define REG_PP_STATE        0x50  /* Current state (PP_IDLE…PP_DONE read-only)   */

/* ── Array size (highest addr + 1 = 0x50+1 = 0x51 = 81) ────────────────────── */
#define MODBUS_REG_COUNT    81

/* ─────────────────────────────────────────────────────────────────────────────
 *  System Mode
 *  ประกาศ typedef ที่นี่เพื่อให้ทุก module ดึงไปใช้ผ่าน base_system.h
 *  Definition (+ initial value) อยู่ใน main.
 *  +6+6666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666 * ─────────────────────────────────────────────────────────────────────────────*/
/* Physical selector switch on panel (always read, highest priority) */
typedef enum {
    SELECTOR_MANUAL = 0,
    SELECTOR_AUTO   = 1,
} SelectorMode_t;

/* Operating mode — actual running mode decided by priority logic */
typedef enum {
    MODE_HOMING = 0,
    MODE_AUTO   = 1,
    MODE_MANUAL = 2,
    MODE_TEST   = 3,
} SystemMode_t;

extern volatile SelectorMode_t selector_mode;       /* from physical switch  */
extern volatile SystemMode_t   current_system_mode; /* actual running mode   */

/* ─────────────────────────────────────────────────────────────────────────────
 *  Modbus register array  (defined in base_system.c)
 * ─────────────────────────────────────────────────────────────────────────────*/
extern uint16_t modbus_registers[MODBUS_REG_COUNT];

/* ─────────────────────────────────────────────────────────────────────────────
 *  FC06 Echo Detection Buffer  (defined in base_system.c)
 *  RX callback ใน main.c ใช้ทำ content-based echo filter แทน timing guard
 * ─────────────────────────────────────────────────────────────────────────────*/
extern uint8_t  modbus_echo_buf[8];   /* สำเนา FC06 response ที่เพิ่งส่งออก */
extern uint8_t  modbus_echo_valid;    /* 1 = กำลังรอ echo, 0 = ไม่มี         */
extern uint32_t modbus_echo_time;     /* HAL_GetTick() ขณะ set echo_valid=1   */

/* ─────────────────────────────────────────────────────────────────────────────
 *  API
 * ─────────────────────────────────────────────────────────────────────────────*/
void Modbus_Parse_Frame(uint8_t *frame, uint16_t length);
void Heartbeat_Init(void);
void Heartbeat_Update(void);
bool Heartbeat_IsConnected(void);

#endif /* BASE_SYSTEM_H_ */
