# Context — OneDofRobot (สถานะปัจจุบัน)

> ไฟล์นี้สรุป **"ตอนนี้กำลังทำอะไรอยู่"** — อ่านก่อนเริ่มงานต่อทุกครั้ง
> อัปเดตล่าสุด: 2026-05-29

---

## ภาพรวมโปรเจกต์

หุ่นยนต์ **1-DOF (แขนหมุน 1 แกน)** ควบคุมด้วย STM32G474RE
- ควบคุมแบบ **Cascade**: position loop (นอก) → velocity loop (ใน) → motor voltage
- มี **Kalman Filter** ประมาณ position / velocity / disturbance
- สื่อสารกับ PC ผ่าน **Modbus RTU** (USART2, 19200 8E1, slave ID=21)
- ควบคุมได้ 2 ทาง: **Base System** (PC app จริง) และ **pid_dashboard.py** (เครื่องมือ tune)

## โหมดการทำงาน (current_system_mode)
| Mode | ทำงานที่ | ไฟล์ |
|---|---|---|
| MODE_HOMING | หา home ด้วย prox sensor | `homing.c` |
| MODE_AUTO | Pick & Place / GoPoint / Jog | `auto_mission.c` |
| MODE_MANUAL | Dashboard tune (velocity/position) | `dashboard.c` |

ทุก loop รันใน **TIM6 ISR ทุก 1 ms**

---

## งานที่เพิ่งทำเสร็จ (รอบล่าสุด)

### 1. แก้ Steady-State Oscillation ✅
**อาการ:** หุ่น oscillate ±2° ที่ setpoint, voltage bang-bang ±20V
**สาเหตุ:** integral windup ใน velocity loop (integral_limit=24V สูงเกิน)
**แก้:** (ใน `cascade_control.c`)
- เพิ่ม anti-windup (conditional integration) ใน `calculate_pid()`
- ลด `vel_ctrl.integral_limit`: 24V → **6.0V**
- เพิ่ม vel_fade threshold: 0.3 → **1.0 rad/s** + re-enable V_dist

### 2. เปลี่ยน Position Loop เป็น 100 ms ✅
เดิม pos loop รันที่ 1ms (เหมือน vel loop) — เปลี่ยนให้รันที่ **100ms (10 Hz)**
- `cascade_control.h`: `DT_POS=0.100f`, `POS_DIV=100`
- `cascade_control.c`: เพิ่ม divider counter (`pos_div_tick`, `pos_div_out`)
- vel loop ใช้ cached vel setpoint ระหว่างที่ pos loop ยังไม่ update (sample-and-hold)

### 3. Gripper Module — สร้างแล้วแต่ยัง **ไม่ integrate** ⏸️
- สร้าง `gripper.h` + `gripper.c` (PC4=jaw, PC10=arm, active LOW)
- **revert การ integrate ออกหมดแล้ว** ตามคำสั่งผู้ใช้ ("เอา gripper ออกก่อน")
- ไฟล์ยังอยู่ในโปรเจกต์ + มี fallback defines → compile ผ่าน แต่ไม่มีใครเรียก

---

## ที่ต้องทำต่อ (ดู info.md สำหรับรายละเอียด)

1. **Re-tune Ki ของ pos loop** — เพราะ DT_POS เปลี่ยน 1ms→100ms (Ki scale ต่างไป ~100×)
2. **ลด SS error** (ตอนนี้ 0.54°–1.00° เพราะ Pos Ki=0) → เพิ่ม Pos Ki ~0.5
3. **Verify บน hardware** ว่า oscillation หายจริง
4. **Gripper** — รอผู้ใช้สั่งค่อย integrate กลับ

---

## ⚠️ กฎที่ต้องจำ
- อ่าน `docs/base_system.md` ทุกครั้งที่อ้างถึง base system
- อ่าน + อัปเดต `docs/troubleshoot.md` ทุกครั้งที่แก้ปัญหา
- ตอบเป็นภาษาไทยเสมอ
