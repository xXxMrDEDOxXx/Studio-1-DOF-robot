# Context — OneDofRobot (สถานะปัจจุบัน)

> ไฟล์นี้สรุป **"ตอนนี้กำลังทำอะไรอยู่"** — อ่านก่อนเริ่มงานต่อทุกครั้ง
> อัปเดตล่าสุด: 2026-06-01

---

## ภาพรวมโปรเจกต์

หุ่นยนต์ **1-DOF (แขนหมุน 1 แกน)** ควบคุมด้วย STM32G474RE
- ควบคุมแบบ **Cascade**: position loop (นอก) → velocity loop (ใน) → motor voltage
- มี **Kalman Filter** ประมาณ position / velocity / disturbance
- สื่อสารกับ PC ผ่าน **Modbus RTU** (USART2, 230400 8E1, slave ID=21)
- ควบคุมได้ 2 ทาง: **Base System** (PC app จริง) และ **pid_dashboard.py** (เครื่องมือ tune)

## โหมดการทำงาน (current_system_mode)
| Mode | ทำงานที่ | ไฟล์ |
|---|---|---|
| MODE_HOMING | หา home ด้วย prox sensor | `homing.c` |
| MODE_AUTO | Pick & Place / GoPoint / Jog | `auto_mission.c` |
| MODE_MANUAL | Joystick + Dashboard tune + Gripper manual | `joystick.c` / `dashboard.c` / `gripper.c` |
| MODE_TEST | Performance / Precision | `test_mode.c` |

ทุก loop รันใน **TIM6 ISR ทุก 1 ms**

## ⭐ Mode Arbitration (main.c HAL_TIM_PeriodElapsedCallback) — สำคัญสุด
ลำดับ priority ใน TIM6 ISR (อัปเดต 2026-06-01 — คืน selector-switch logic):
1. **MODE_HOMING** → Homing_Update() (boot ทำก่อนเสมอ)
2. **GLOBAL SOFT STOP** (0x25=1) → มอเตอร์ดับทุกโหมด (ไม่ latch)
3. **selector = MANUAL** → บังคับ `MODE_MANUAL` → Joystick + Dashboard + Gripper → `return`
   (base mode command ถูกทิ้ง — **joystick ทำงานเฉพาะตรงนี้**)
4. **selector = AUTO** → อ่าน REG_BS_MODE (0x01): Home/Jog/Auto/SetHome/Test
   (**base auto ทำงานก็ต่อเมื่อ selector = AUTO เท่านั้น**; base JOG → MODE_AUTO PP_JOG)
> สวิตช์ = `Manual_mode_Pin` (PB0). MANUAL = PIN_RESET

---

## งานที่เพิ่งทำเสร็จ (2026-06-01)

### A. คืน selector-switch arbitration ✅ (main.c)
- selector=MANUAL → MODE_MANUAL (joystick), selector=AUTO → base ควบคุม
- แก้ปัญหา joystick ตาย (หุ่นค้าง AUTO หลัง boot) + base auto สั่งได้ตลอดเวลา
- base JOG (0x01=2) → MODE_AUTO PP_JOG (MODE_MANUAL จริงสงวนให้สวิตช์เท่านั้น)

### B. Joystick (joystick.c) ✅
- **ปุ่ม A = EMER toggle**: กด=หยุด+latch ESTOP, **กดอีก=re-home (sensor-based)**
- **ปุ่ม B = gripper pick/place** | **D = arm ขึ้น/ลง อย่างเดียว** (latch แยก ไม่มี grab)
- **Point-mode = S-curve (Septic)**: ±5°/คลิก เคลื่อนเนียน, `JOY_POINT_MOVE_TIME=0.5s`
- **ปุ่ม C = Set Home ไม่ขยับ**: zero encoder + Cascade_Reset + re-arm Septic hold ที่ home ใหม่
- joystick ทำงาน **MANUAL only** (guard `current_system_mode != MODE_MANUAL → return 0`)

### C. Emergency → re-homing ✅ (main.c EXTI + joystick.c)
- **ปุ่มตู้ PC13 กด** → ตัดไฟ motor drive + ESTOP; **ปล่อย → Homing_Start + MODE_HOMING**
- ปุ่ม A joystick = mirror พฤติกรรม PC13 (stop ↔ re-home)
- ต่างจาก Set Home (ปุ่ม C) ที่แค่ zero encoder ไม่หา sensor

### D. Base system 3 โหมดครบ ✅
- **MANUAL tab**: เพิ่ม `Gripper_Update()` ใน PP_JOG → gripper 6 ปุ่ม + jog ทำงาน (auto_mission.c)
- **TEST**: ตรวจแล้ว Performance ใช้ vel/acc จริง (Trapz_MoveToFull) + Precision (Septic) — ถูกต้องอยู่แล้ว
- ตรวจ comms/register map/P&P slot ตรง README v1.2; index→deg ×5° (72 holes)

### E. Telemetry ใน MANUAL ✅ (main.c Priority 2) — แก้ base UI แขนไม่ขยับตาม joystick
- **ปัญหา:** MANUAL dispatch return ก่อนเขียน telemetry + joystick free-mode bypass cascade
  → 0x28/0x29/0x30 ค้าง → รูปแขนใน base UI ไม่ขยับ
- **แก้:** เขียน base telemetry ทุก tick หลัง Gripper_Update — `Encoder_Update(henc2)` (idempotent)
  + pos จาก henc2.position_deg, vel/acc = finite-diff(dt=1ms)+EMA (α 0.1/0.05). ไม่แตะ control path

---

## งานที่เพิ่งทำเสร็จ (รอบ 2026-05-29)

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
