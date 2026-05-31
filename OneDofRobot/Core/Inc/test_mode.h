/*
 * test_mode.h
 *
 *  Performance & Precision test mode (MODE_TEST)
 *  เรียกจาก HAL_TIM_PeriodElapsedCallback ทุก 1 ms
 *
 *  Performance (REG_BS_TEST_TYPE = 1):
 *    วิ่งระหว่าง init_pos ↔ final_pos ด้วย vel/accel จาก 0x07/0x08
 *    ต่อเนื่องจน soft stop
 *
 *  Precision (REG_BS_TEST_TYPE = 0):
 *    วิ่ง init_pos → final_pos → init_pos × repeat_count ครั้ง
 *    หยุด 500 ms ที่แต่ละปลาย
 *
 *  Registers ที่ใช้:
 *    0x06 REG_BS_TEST_TYPE   0=Precision  1=Performance
 *    0x07 REG_BS_PERF_VEL   velocity  [deg/s, int16]
 *    0x08 REG_BS_PERF_ACC   accel     [deg/s², int16]
 *    0x09 REG_BS_PREC_INIT  init pos  [deg, int16]
 *    0x10 REG_BS_PREC_FINAL final pos [deg, int16]
 *    0x11 REG_BS_PREC_RPT   repeat count; >0=deg unit, <0=index unit
 */

#ifndef TEST_MODE_H_
#define TEST_MODE_H_

#include <stdint.h>

void TestMode_Init(void);    /* เรียกครั้งเดียวใน BEGIN2                        */
void TestMode_Start(void);   /* เรียกจาก main.c เมื่อรับ REG_BS_MODE_TEST       */
void TestMode_Reset(void);   /* เรียกเมื่อ mode เปลี่ยนหรือ e-stop              */
void TestMode_Update(void);  /* เรียกทุก 1 ms ใน ISR (case MODE_TEST)           */

#endif /* TEST_MODE_H_ */
