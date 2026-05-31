# Info — โค้ดที่เขียน / ค่าที่ตั้ง / สิ่งที่ต้องทำต่อ

> รายละเอียดทางเทคนิคของการเปลี่ยนแปลงทั้งหมด
> อัปเดตล่าสุด: 2026-06-01

---

## 0. Mode Arbitration + Joystick (2026-06-01) ⭐ ล่าสุด

### Mode arbitration (`main.c` HAL_TIM_PeriodElapsedCallback) — selector-switch based
ลำดับ priority ใน TIM6 ISR:
1. `MODE_HOMING` → Homing_Update() + return
2. GLOBAL SOFT STOP (REG_BS_SOFT_STOP 0x25=1) → มอเตอร์ดับทุกโหมด + return (ไม่ latch)
3. **selector = MANUAL** (`Manual_mode_Pin` PB0 = RESET) → บังคับ `MODE_MANUAL`:
   - `Joystick_Update()` → ถ้าคืน 0 → `Dashboard_Update()` → `Gripper_Update()` → **return**
   - ทิ้ง REG_BS_MODE (base mode command ไม่มีผลใน MANUAL)
4. **selector = AUTO** → transition MANUAL→AUTO (ถ้าค้าง) → อ่าน REG_BS_MODE (0x01):
   - HOME → AutoMission_GoHome (MODE_AUTO) | JOG → AutoMission_StartJog (MODE_AUTO PP_JOG)
   - AUTO → pending 150ms → AutoMission_StartAuto (P&P/GoPoint) | SET_HOME → Homing_SetHome
   - TEST → pending 150ms → TestMode (MODE_TEST)
   - bottom switch: AUTO → AutoMission_Update, TEST → TestMode_Update

> ⚠ **base auto/P&P ทำงานก็ต่อเมื่อ selector = AUTO เท่านั้น** | **joystick = MANUAL เท่านั้น**
> เดิม (commit bc4d6c8) "base คุมเต็มตัวไม่สน selector" → ทำ joystick ตาย → revert แล้ว

### Joystick (`joystick.c` / `joystick.h`)
- ปุ่ม: A=PA5 B=PA6 C=PA7 D=PB11 K=PB10 (active-LOW pull-up) | ADC X = PC3 (ADC2_IN9, bare-metal)
- **A = Emergency / Reset (TOGGLE)**: กด1=ตัด PWM+MOE disable+REG_ESTOP=1;
  **กด2=clear ESTOP+MOE enable+`Homing_Start()`+MODE_HOMING (re-home sensor-based)**
  → กลับไป homing ใหม่เหมือนเปิดเครื่อง (ไม่ใช่ resume). ปุ่มตู้ PC13 ปล่อย → re-home เช่นกัน
- **B = Gripper Pick/Place toggle** (REG_BS_GRIPPER_SEQ)
- **C = Set Home** (zero encoder, **ไม่ขยับ**: Homing_SetHome + Cascade_Reset + Septic hold ที่ 0
  — คนละอย่างกับ re-homing ตอนเปิดเครื่อง/ปล่อย emer)
- **D = Arm Up/Down อย่างเดียว** (`Gripper_SetArm` → latch g_arm_down แยกจาก jaw, ไม่มี grab)
- **K = สลับ Free ↔ Point**
- **Free mode**: ADC <800 CCW / >3500 CW ที่ duty 15% (1500/9999, bypass cascade)
- **Point mode**: ±5°/คลิก ด้วย **Septic S-curve** (`Septic_MoveTo`+`Septic_Update`→
  `Cascade_Control_Update_FF`), `JOY_POINT_MOVE_TIME=0.5f`, ตั้ง pos gains (REG_POS_KP=1550)
  ตอนเข้า Point, ต้องปล่อยกลับ neutral ก่อนคลิกถัดไป

### Emergency / Homing flow (สำคัญ)
- **boot** → MODE_HOMING → Homing_Start (sensor: H_LEAVE→SEEK→COUNT→RETURN→DONE) → MODE_AUTO
- **ปุ่มตู้ PC13 กด** (EXTI) → ตัด PWM + MOE disable + REG_ESTOP=1 + reset auto/test
- **ปุ่มตู้ PC13 ปล่อย** → clear ESTOP + MOE enable + **Homing_Start + MODE_HOMING (re-home)**
- **joystick A** = mirror PC13 (toggle: stop ↔ re-home)
- **E-Stop / re-home ต่างจาก Set Home (ปุ่ม C)**: C แค่ zero encoder ไม่ขยับ ไม่หา sensor

### Base telemetry ใน MANUAL (main.c Priority 2) — แก้ base UI แขนไม่ขยับตาม joystick
- **ปัญหา:** MODE_MANUAL dispatch `return` ก่อนเขียน base block; joystick free-mode bypass
  cascade → Dashboard ถูก skip + cascade ไม่ถูกเรียก → 0x28/0x29/0x30 ค้าง → แขนใน base UI นิ่ง
- **แก้:** เขียน base telemetry ทุก tick **หลัง Gripper_Update** ใน Priority 2:
  - `Encoder_Update(&henc2)` — idempotent (ถ้า cascade เรียกไปแล้ว diff=0 ไม่นับซ้ำ)
  - **pos** = `henc2.position_deg ×10 ×BS_DIR_SIGN` (scale เดียวกับ q_out)
  - **vel/acc** = finite-diff (dt=1ms) + EMA (vel α=0.1, acc α=0.05) → static bs_prev_deg/
    bs_vel_ema/bs_prev_vel/bs_acc_ema ใน block. กระตุก→ลด α / lag→เพิ่ม α
  - ไม่แตะ control path / joystick.c (เขียน register อย่างเดียว)

### Base system 3 โหมด (selector = AUTO) — ตรวจตรง README v1.2
- **AUTO**: 0x01=4 → P&P (slots 0x12–0x21, N_pair 0x22; index→deg ×5° / 72 holes) หรือ
  GoPoint (0x23 unit, 0x24 target). gripper actuate เมื่อ 0x04=1 (เช็ค Gripper box)
- **MANUAL**: 0x01=2 → MODE_AUTO PP_JOG → gripper 6 ปุ่ม (0x02 up/down/open/close +
  0x03 pick/place ผ่าน `Gripper_Update()` ที่เพิ่งเพิ่มใน PP_JOG) + jog (0x05 ±deg)
- **TEST**: 0x01=16 → Performance (0x07 vel / 0x08 acc → `Trapz_MoveToFull` ใช้ค่าจริง) /
  Precision (0x09 init, 0x10 final, 0x11 repeat; sign=unit deg/index → Septic)
- **STOP**: 0x25=1 → GLOBAL SOFT STOP (มอเตอร์ดับทุกโหมด ไม่ latch)

---

## 1. Hardware / ค่าคงที่หลัก

| พารามิเตอร์ | ค่า | ไฟล์ |
|---|---|---|
| MCU | STM32G474RE | — |
| ISR | TIM6 @ 1ms | `main.c` |
| PWM | TIM1 CH1, PSC=9 ARR=9999 → ~1700Hz | `tim.c` |
| `PWM_ARR_MAX` | 9999.0f | `cascade_control.h` |
| `MAX_VOLTAGE` | 24.0V | `cascade_control.h` |
| `GEAR_RATIO` | 2.0 | `cascade_control.h` |
| Modbus | USART2 230400 8E1, slave=21 | — |

---

## 2. Cascade Control (`cascade_control.c` / `.h`)

### Sampling time
```c
#define DT_VEL  0.001f   // 1 ms — vel loop (1 kHz)
#define DT_POS  0.001f   // 1 ms — pos loop (1 kHz) ทุก tick
#define POS_DIV  1U      // pos loop ทุก 1 tick → ไม่มี staircase
```
> ประวัติ: 100ms (setpoint กระโดด 22°) → 5ms (ยัง staircase) → **1ms**
> ที่ 5ms + pos Kp=15 สูง → pos_div_out กระโดดทุก 5ms = velocity setpoint ขั้นบันได 200Hz
> → กระตุก "เหมือน sampling ต่ำ" ตอน auto. แก้: POS_DIV=1 (รันทุก tick, ไม่ staircase)
> Ki effect คงเดิม (DT_POS ปรับตาม → integral rate เท่ากัน)

### PID gains (ค่า init — ถูก override ด้วย Modbus ตอน runtime)
```c
pos_ctrl = { Kp=12.0, Ki=0.1, Kd=0.2, integral_limit=MAX_VOLTAGE }
vel_ctrl = { Kp=6.5,  Ki=1.0, Kd=0.0, integral_limit=6.0f }   ← ลดจาก 24V
```
> **runtime จริง** อ่านจาก Modbus (REG_VEL_KP… / REG_POS_KP…)
> ค่าที่ tune ล่าสุด (จาก dashboard): Pos Kp=8.0 Ki=0.0 Kd=0.0 | Vel Kp=7.2 Ki=3.0 Kd=0.09

### 2-DOF Control: Feedforward (จาก reference สะอาด) + Feedback เบา + Observer ช้า
1. **V_FF** = `K_ff × ref_qd` (K_ff=5.4) — viscous + back-EMF FF
2. **V_acc** = `K_ACC × ref_qdd` (K_ACC≈0.768) — **acceleration FF (inverse dynamics)**
   - q̈_ref จาก Quintic → pre-supply แรงเร่ง inertia → feedback ไม่ต้องแบก → แม่นขึ้น
3. **V_fric** = `V_COULOMB × sat(ref_qd/QD_EPS)` — Coulomb/deadband FF (noise-free)
   - V_COULOMB=1.2V, QD_EPS=0.15 rad/s — ใช้ ref_qd (สะอาด) ทะลุ deadband ไม่สั่น
4. ~~**V_dist** = KF tau_d~~ — **ปิดใช้งาน** (แกนนอน ไม่มี gravity + tau_d noise ทำกระตุก move ใหญ่)
5. **Backlash inverse comp** — BL_RAD=0.03566 rad (~2°), inject เมื่อเปลี่ยนทิศ
6. **Deadband hysteresis** — Zone2 (<225) ตัด 0, Zone3 (225–450) kick→450

> **Total:** `Motor_Drive(V_VEL + V_FF + V_acc + V_fric)` (V_dist ปิดแล้ว)
> **หลัก 2-DOF:** FF (V_FF+V_acc+V_fric จาก reference) แบกงานหนัก → feedback (V_VEL) แก้ residual
> → ตั้ง gain เบาได้ → ไม่ขยาย noise → ไม่สั่น + แม่น
> Interface: `Cascade_Control_Update_FF(ref_q, ref_qd, ref_qdd)` (auto_mission ใช้ตัวนี้);
> 2-arg เดิม wrap ด้วย qdd=0 (hold/velocity mode)
> ✅ gravity comp **ไม่ต้องมี** — หุ่นหมุนระนาบแนวนอน (แกนตั้งฉากพื้น) gravity ไม่สร้าง torque
> → feedforward architecture **ครบสมบูรณ์**

### Anti-windup (ใหม่) ใน `calculate_pid()`
```c
// commit integral เฉพาะเมื่อไม่ saturate ไปทิศเดียวกับ error
if (!((output >  integral_limit && error > 0) ||
      (output < -integral_limit && error < 0))) {
    pid->integral = i_new;
}
```

### Pos loop divider (ใหม่) ใน `Cascade_Control_Update()`
```c
static uint8_t pos_div_tick;  // file-scope (reset ใน Cascade_Control_Reset)
static float   pos_div_out;
if (++pos_div_tick >= POS_DIV) {
    pos_div_tick = 0;
    // คำนวณ pos PID ด้วย DT_POS → เก็บใน pos_div_out
}
current_q_dot_ref = ref_qd + pos_div_out;  // vel loop ใช้ cached value
```

---

## 3. Dashboard (`dashboard.c` + `tools/pid_dashboard.py`)

### `dashboard.c` — bug fix
- แก้ bug: `traj_qd` จาก Trapz ถูกคำนวณแต่ไม่ได้ใช้ → ref_qd=0 เสมอ → V_FF=0
- ตอนนี้ส่ง `dash_ref_qd = traj_qd` เข้า `Cascade_Control_Update()` แล้ว

### `pid_dashboard.py` — features ที่เพิ่ม
- **Ku Search (Ziegler-Nichols)** ใน Pos tab — auto increment Kp หา oscillation → รายงาน Ku/Tu
- **Cascade HW gains display** — แสดงค่า PID ที่ active บน hardware จริง (sync check)
- **ลบ hardcode PID** ตอนเข้า P&P tab (เดิม override เป็น 35,0,0.3 ทับค่าที่ tune)
- **P&P settle counter** — ต้องอยู่ในโซน 5 ticks ติด (500ms) ก่อนนับว่าถึง target

---

## 4. Gripper (`gripper.c` / `.h`) — ✅ integrate แล้ว (reed feedback)

**Pin (main.h / CubeMX):**
- **PC4 = `gripper_u_d`** (ARM up/down) — active LOW: LOW=down, HIGH=up
- **PC10 = `gripper_o_c`** (JAW open/close) — active LOW: LOW=close, HIGH=open
- Reed: `reed_up`(PC7) `reed_down`(PA9) `reed_open`(PB4) `reed_close`(PB9), REED_ON=LOW
  (ทั้ง active level + REED_ON_STATE ปรับได้ใน gripper.h ถ้าทิศ/logic กลับ)

**Sequence (reed-confirmed + timeout fallback):**
- Pick: arm↓ (รอ reed_down) → jaw close (รอ reed_close) → arm↑ (รอ reed_up)
- Place: arm↓ → jaw open (รอ reed_open) → arm↑
- timeout: `GRIP_ARM_MS=800`, `GRIP_ACT_MS=600`

**Integration:**
- `auto_mission.c` DWELL_PICK/PLACE → `Gripper_Pick/Place()` + `Gripper_Update()` จน `Gripper_IsDone()` (แทน dwell timer)
- `main.c` MANUAL mode → `Gripper_Update()` (manual REG_BS_GRIPPER_MAN 0x02 + reed telemetry 0x26)
- `Gripper_Init()` ใน main, `Gripper_Abort()` ใน AutoMission_Reset
- REG: 0x02 manual, 0x04 enable, 0x26 reed state

---

## 5. TODO — สิ่งที่ต้องทำต่อ (เรียงตามความสำคัญ)

### 🔴 สำคัญ
1. **เป้าหมาย precision 0.1°** — ⚠️ backlash 2.04° = ใหญ่กว่าเป้า 20×
   ต้องพึ่ง backlash comp แม่น + pos Ki + pos loop เร็ว (5ms)
2. **Re-tune Pos Ki** — DT_POS = 5ms (เปลี่ยนจาก 100ms) → integral scale ต่างไป
   ตอนนี้ Pos Ki=0 → SS error 0.54°–1.00° → ลองเริ่ม Ki=0.5
3. **Verify บน hardware** — ยืนยัน oscillation หาย + S-Curve เนียนหลัง flash

### Homing: wrong-side detection + 360° unwind (controlled) (`homing.c`)
- flow: `[H_LEAVE→] H_SEEK → H_COUNT → H_RETURN → [H_UNWIND→] H_DONE`
- H_SEEK วัด seek travel จาก start → ถ้า **> CPR/2 (4096=180°)** = `wrong_side` + เก็บ `seek_dir`
- H_RETURN ถึง flag center: ถ้า wrong_side → zero + Cascade_Reset → **H_UNWIND**
- **H_UNWIND ใช้ Septic trajectory + Cascade_Control_Update_FF** (controlled, เนียน — ไม่ใช่ raw duty)
  target = `-seek_dir × 2π` (ทิศตรงข้าม seek, 360°) → จบ → zero + Cascade_Reset (set home) → H_DONE
- gain ตอน unwind = AUTO fixed (MODE_HOMING → else branch)

### 🟢 เพิ่งทำเสร็จ (2026-05-29)
- **pos loop 100ms → 5ms** (POS_DIV=5) — แก้ปัญหา setpoint กระโดด 22°/tick
- **P2P → Septic (7th-order) time-scaled** (`trajectory.c/h` + `auto_mission.c`)
  - `Septic_Profile_t` + `Septic_MoveTo/Update` — q=q0+d·(35τ⁴−84τ⁵+70τ⁶−20τ⁷)
  - **jerk-continuous**: q⃛=0 ที่ปลายทั้งสองด้าน → ไม่กระชากตอนออก/จอด (เนียนสุด)
  - ถึงเป้าเป๊ะที่ t=T **ไม่ undershoot**; feasibility cap KV=2.1875, KA=7.5117 (v≤4.5,a≤15)
  - `TRAJ_MOVE_TIME=1.5s` → ทุก move ~1.5s (180°=1.53s). budget 8×1.5+16=28s<40s
  - เดิมเป็น Quintic (jerk ปลายกระโดด 372) → Septic ลดเหลือ ≈0
  - Quintic_*/SCurve_* ยังอยู่ใน trajectory.c (ไม่ได้ใช้ แต่เก็บไว้)

**พารามิเตอร์ Quintic (`trajectory.h`):**
```c
#define TRAJ_QV_MAX     4.5f    // เพดานความเร็ว (hardware)
#define TRAJ_QA_MAX     15.0f   // เพดานความเร่ง (จูนลงถ้า saturate)
#define TRAJ_MOVE_TIME  0.5f    // เวลาเป้าหมาย/move
```

### 🟡 รอง
4. ทดสอบ Ku Search บน hardware จริง (Tu detection)
5. ถ้าอยาก settle เร็วขึ้น (<0.5s) → เพิ่ม Pos Kp ทีละ 2 (8→10→12) ดู overshoot

### ⚪ รอผู้ใช้สั่ง
6. **Gripper integration** — มีไฟล์พร้อมแล้ว รอคำสั่งค่อย integrate กลับ
   - ต้องตั้ง CubeMX: PC4→"Gripper_Jaw", PC10→"Gripper_Arm" (GPIO Output, default HIGH)

---

## 6. ผลทดสอบล่าสุด (จาก dashboard P&P, 4 rods)
| Rod | Target | Overshoot | Settle | SS err |
|---|---|---|---|---|
| 1 | +45° | 0.0% | — | 1.00° |
| 2 | +135° | 0.0% | 1.08s | 0.78° |
| 3 | +225° | 0.0% | 1.15s | 0.54° |
| 4 | +315° | 0.0% | 1.08s | 0.56° |

✅ oscillation หาย, overshoot=0 | ⚠️ SS error สูง (Ki=0), settle ช้า
