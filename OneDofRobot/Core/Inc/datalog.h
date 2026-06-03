/*
 * datalog.h — 1 kHz RAM data logger (Lab 4 burst capture)
 *
 *  เก็บ snapshot ของ control signals ทุก tick (1 kHz) ลง RAM แล้วให้ PC
 *  อ่านออกผ่าน Modbus FC03 ที่ address ≥ LOG_BASE_ADDR (buffer แยกจาก
 *  modbus_registers[]). ใช้คู่กับ tools/pid_dashboard.py ปุ่ม "⚡ 1kHz".
 *
 *  Channels (LOG_C ตัว ต่อ 1 sample) — int16 scaled:
 *    [0] ref_q    × 1000   [rad]
 *    [1] q        × 1000   [rad]
 *    [2] ref_qd   × 1000   [rad/s]
 *    [3] qd       × 1000   [rad/s]
 *    [4] ref_qdd  × 100    [rad/s^2]
 *    [5] V        × 1000   [V signed]
 *    [6] i_est    × 1000   [A]
 *  time = sample_index × 0.001 s
 */
#ifndef DATALOG_H_
#define DATALOG_H_

#include <stdint.h>

/* ── โหมด full (7 ch) : 2 s @ 1kHz ── */
#define LOG_C          7         /* ref_q,q,ref_qd,qd,ref_qdd,V,i_est           */
#define LOG_N          2000      /* max samples (2.0 s)                          */
/* ── โหมด long (2 ch = V,qd) : 20 s @ 1kHz ── */
#define LOG_C_LONG     2         /* [0]=V ×1000  [1]=qd ×1000                   */
#define LOG_N_LONG     20000     /* max samples (20 s)                           */
/* buffer ก้อนเดียวใช้ร่วมสองโหมด — ขนาดใหญ่สุด = 20000×2 = 40000 int16 = 80 KB */
#define LOG_FLAT       (LOG_N_LONG * LOG_C_LONG)
#define LOG_BASE_ADDR  0x4000U   /* Modbus addr ฐานสำหรับอ่าน buffer (FC03)     */

void     DataLog_Init(void);
void     DataLog_Arm(void);                  /* เริ่มเก็บใหม่ (idx=0, capturing) */
void     DataLog_Service(void);              /* เรียกทุก tick: handle ctrl + sync status regs */
void     DataLog_Sample(float ref_q, float q, float ref_qd, float qd,
                        float ref_qdd, float V, float i);
uint16_t DataLog_Total(void);                /* LOG_N × LOG_C                  */
uint16_t DataLog_ReadFlat(uint32_t idx);     /* อ่าน 1 register จาก flat buffer */

#endif /* DATALOG_H_ */
