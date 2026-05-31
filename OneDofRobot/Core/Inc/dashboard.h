/*
 * dashboard.h
 *
 * Manual Mode Dashboard Module
 * ─────────────────────────────────────────────────────────────────────────────
 * Module นี้จัดการการสื่อสารกับ PC Dashboard ผ่าน Modbus เมื่ออยู่ใน MODE_MANUAL
 *
 * หน้าที่:
 *   - รับค่า PID gains / target จาก PC และส่งเข้า Cascade_Control
 *   - สร้าง reference waveform (Square / Sine / Step) สำหรับ Velocity tune
 *   - รับ target position (องศา) สำหรับ Position tune
 *   - ส่งข้อมูล telemetry กลับ PC (position, velocity, voltage, current)
 *
 * เงื่อนไขการทำงาน:
 *   Dashboard_Update() จะทำงานเฉพาะเมื่อ current_system_mode == MODE_MANUAL
 *   ถ้า mode อื่น → หยุดมอเตอร์ทันทีและ return
 *
 * Register Map (ดูรายละเอียดใน base_system.h):
 *   WRITE (PC → STM32): REG_VEL_KP … REG_TARGET_POS  (0x10 – 0x1B)
 *   READ  (STM32 → PC): REG_REF_QD … REG_REF_Q       (0x20 – 0x25)
 *   STATUS:             REG_ISR_CNT, REG_ESTOP        (0x30 – 0x31)
 * ─────────────────────────────────────────────────────────────────────────────
 */

#ifndef DASHBOARD_H_
#define DASHBOARD_H_

/* เรียกครั้งเดียวใน USER CODE BEGIN 2 (หลัง Cascade_Control_Init) */
void Dashboard_Init(void);

/* เรียกใน TIM6 PeriodElapsedCallback ทุก 1 ms
 * ── จะ skip อัตโนมัติถ้า current_system_mode != MODE_MANUAL ── */
void Dashboard_Update(void);

#endif /* DASHBOARD_H_ */
