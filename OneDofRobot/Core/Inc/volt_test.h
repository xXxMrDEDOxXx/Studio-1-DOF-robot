/*
 * volt_test.h — Open-loop voltage excitation (Lab 1 parameter estimation)
 *
 *  จ่ายแรงดันตรงเข้ามอเตอร์ (bypass cascade/PID) เป็นสัญญาณทดสอบ เพื่อเก็บ
 *  input(V) → output(q, qd) ไปทำ system identification (J, B, …)
 *  ทำงานเฉพาะ MODE_MANUAL (selector หน้าตู้ = MANUAL) และเมื่อ REG_VT_MODE != 0
 *
 *  Waveforms (REG_VT_MODE):
 *    1 STEP   : V = offset + amp           (คงที่)
 *    2 SINE   : V = offset + amp·sin(2π f0 t)
 *    3 CHIRP  : sine กวาดความถี่ f0 → f1 ใน dur วินาที แล้วหยุด
 *    4 STAIR  : ไล่ระดับ offset → offset+amp เป็น nstep ขั้น ใน dur วินาที แล้วหยุด
 *
 *  ทุก waveform clamp ที่ ±MAX_VOLTAGE (24 V) — ใช้คู่ปุ่ม ⚡ 1kHz Capture
 */
#ifndef VOLT_TEST_H_
#define VOLT_TEST_H_

#include <stdint.h>

#define VT_OFF    0
#define VT_STEP   1
#define VT_SINE   2
#define VT_CHIRP  3
#define VT_STAIR  4

void VoltTest_Init(void);
void VoltTest_Update(void);   /* เรียกทุก 1 ms ใน MANUAL เมื่อ REG_VT_MODE != 0 */

#endif /* VOLT_TEST_H_ */
