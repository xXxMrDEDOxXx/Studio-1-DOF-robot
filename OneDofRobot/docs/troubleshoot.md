# Troubleshooting Log — OneDofRobot

อัปเดตไฟล์นี้ทุกครั้งที่พบปัญหาและแก้ได้

---

## [2026-05-26] UART สรุปการเปลี่ยนแปลง — กลับมาใช้ USART2

**สรุป:** USART3 (PB10/PB11) ใช้ไม่ได้เพราะ ST-Link USB เชื่อมกับ USART2 (PA2/PA3) เท่านั้น
- homing signal ย้ายจาก PA2 → PA4 (macro ชื่อ `Homing_signal_Pin` ตัว H ใหญ่)
- UART กลับมาเป็น USART2 / huart2 ทุกไฟล์

---

## [2026-05-26] USART3 NVIC ไม่ถูก enable → base system เชื่อมไม่ได้

**อาการ:** Base system เชื่อมต่อ STM32 ไม่ได้เลย ไม่มี response จาก Modbus

**สาเหตุ (ชั้นที่ 1):** เปลี่ยนจาก USART2 → USART3 แต่ไม่ได้ enable USART3 global interrupt ใน CubeMX → ไม่มี `USART3_IRQHandler`

**สาเหตุ (ชั้นที่ 2 — ลึกกว่า):** แม้จะเพิ่ม `USART3_IRQHandler` ใน stm32g4xx_it.c แล้ว แต่ usart.c MspInit ไม่มี `HAL_NVIC_EnableIRQ(USART3_IRQn)` → NVIC ไม่เคย enable → interrupt request ไม่เคยถึง CPU เลย

**แก้ไข:**
- เพิ่ม `USART3_IRQHandler` ใน stm32g4xx_it.c (USER CODE BEGIN 1) พร้อม `#include "usart.h"`
- เพิ่ม `HAL_NVIC_SetPriority(USART3_IRQn, 1, 0)` และ `HAL_NVIC_EnableIRQ(USART3_IRQn)` ใน usart.c MspInit USER CODE BEGIN 1
- ถาวร: enable ใน CubeMX → Connectivity → USART3 → NVIC Settings → tick "USART3 global interrupt"

---

## [2026-05-25] Heartbeat เดี๋ยวติดเดี๋ยวดับที่ base system

**อาการ:** Base system แสดงว่า connect แล้ว disconnect สลับกัน

**สาเหตุ:** `modbus_echo_valid` ไม่มี timeout — ถ้า VCP loopback echo หายระหว่างทาง (UART error abort) `echo_valid` ค้างที่ 1 → heartbeat write ถัดไปที่ content เดิมถูก drop ว่าเป็น echo ผิดๆ → base system ไม่เห็น YA → timeout → disconnect

**แก้ไข:**
- เพิ่ม `modbus_echo_time` timestamp และ 50ms auto-expire ใน `Heartbeat_Update()`
- เพิ่ม time-window check ใน echo filter ของ `HAL_UARTEx_RxEventCallback`
- เพิ่ม `modbus_echo_valid = 0` ใน `HAL_UART_ErrorCallback`

---

## [2026-05-26] Homing: PWM ไม่ออก, motor ไม่หมุน

**อาการ:** `debug_hom_state` = H_IDLE, `debug_hom_ticks` = 32844 (>10000 = timeout), motor ไม่ขยับเลย

**สาเหตุ 1:** CubeMX regenerate `tim.c` กลับมา `htim1.Init.Period = 9999` แต่ `PWM_ARR_MAX = 999.0f` ใน cascade_control.h ไม่ได้ถูก regenerate → ค่าไม่ตรงกัน

**สาเหตุ 2:** `HOMING_PWM_DUTY` ตั้งไว้ต่ำเกิน (80→300) แต่กับ ARR=9999 จริง = 3% เท่านั้น → motor ไม่หมุน

**สาเหตุ 3:** หลัง timeout `current_system_mode` ยังเป็น `MODE_HOMING` → TIM6 เรียก `Homing_Update()` ต่อ → `raw_stop()` ทุก ms → motor ขยับไม่ได้อีกเลย

**แก้ไข:**
- `tim.c`: `htim1.Init.Period` 9999 → 999 (ตรงกับ `PWM_ARR_MAX = 999.0f`)
- `homing.c`: `HOMING_PWM_DUTY` → 200 (20% ของ ARR=999)
- `homing.c`: timeout handler เพิ่ม `current_system_mode = MODE_MANUAL` เพื่อออกจาก MODE_HOMING

**สำคัญ:** ทุกครั้งที่ CubeMX regenerate ต้องตรวจสอบ `htim1.Init.Period` ใน tim.c ว่ายังเป็น 999

---

## [2026-05-25] Homing ไม่หยุดเมื่อ prox sensor ติด

**อาการ:** ไฟ prox ติดแต่หุ่นไม่หยุด

**สาเหตุ:** `HOMING_PWM_DUTY = 800` กับ ARR=999 = 80% duty (เร็วเกิน) และ `PWM_ARR_MAX` ยังเป็นค่าเก่า 9999 → cascade คำนวณ duty ผิด

**แก้ไข:**
- `HOMING_PWM_DUTY` 800 → 80 (8% ของ ARR=999)
- ตรวจสอบ `PWM_ARR_MAX` ใน cascade_control.h ให้ตรงกับ ARR จริง (999)

---

## [2026-05-26] Auto Mission ไม่ start เพราะ REG_BS_MODE ถูกล้างก่อนเรียก Update()

**อาการ:** กด Auto ใน Base System แล้วหุ่นไม่เริ่มวิ่ง — PP_IDLE ไม่ transition ไปไหน

**สาเหตุ:** main.c ล้าง REG_BS_MODE = 0 ก่อนเรียก AutoMission_Update() แต่ PP_IDLE เช็ค REG_BS_MODE ข้างใน → ได้ค่า 0 → ไม่เริ่ม

**แก้ไข:** แยก start logic ออกจาก AutoMission_Update() เป็น AutoMission_StartAuto() และ AutoMission_StartJog() ที่เรียกจาก main.c โดยตรงก่อนเปลี่ยน mode

---

## [2026-05-26] Homing: sensor HIGH ตอนเปิดเครื่อง → H_SEEK transition ผิด

**อาการ:** เปิดเครื่องขณะ prox sensor ถูก trigger อยู่แล้ว → H_SEEK เห็น HIGH ทันที → ข้าม zone counting → home position ผิด

**สาเหตุ:** `Homing_Start()` ตั้ง `hom_state = H_SEEK` เลยโดยไม่เช็คสถานะ sensor ปัจจุบัน

**แก้ไข:** เพิ่ม state `H_LEAVE` — ตรวจ sensor ใน `Homing_Start()`:
- ถ้า sensor HIGH → `hom_state = H_LEAVE` (ขับออกทิศตรงข้าม duty 5% จน LOW → transition to H_SEEK)
- ถ้า sensor LOW → `hom_state = H_SEEK` ตามปกติ
- H_LEAVE มี timeout 10 s → ถ้าหนีออกไม่ได้ → ESTOP + MODE_MANUAL

Flow ใหม่: `[H_LEAVE →] H_SEEK → H_COUNT → H_RETURN → H_DONE`

---

## [2026-05-25] V_dist ถูก comment out ทำให้หุ่นไม่หมุน

**อาการ:** หุ่นไม่หมุนเมื่อ cascade control ทำงาน

**สาเหตุ:** `Motor_Drive(V_VEL + V_FF)` — V_dist ถูก comment ออกโดยไม่ได้ตั้งใจ

**แก้ไข:** เปลี่ยนเป็น `Motor_Drive(V_VEL + V_FF + V_dist)` ใน `cascade_control.c`

---

## [2026-06-01] Base system คุมโหมดเต็มตัว — ตัดการบังคับด้วยสวิตช์หน้าตู้

**สเปก:** base system สั่งแล้วหุ่นต้องทำตาม **ไม่สนสวิตช์ selector หน้าตู้** (Manual_mode_Pin)

**เดิม (ผิด):** TIM6 ISR มี Priority 2 = ถ้า selector switch = MANUAL → บังคับ MODE_MANUAL
และอ่าน 0x01 (mode จาก base) เฉพาะตอน selector = AUTO → base สั่ง AUTO/TEST ไม่ได้ถ้า
สวิตช์อยู่ MANUAL, และเข้า MANUAL จาก web ไม่ได้

**แก้ (main.c HAL_TIM_PeriodElapsedCallback):** เขียน mode arbitration ใหม่
- ตัดเงื่อนไข `selector_mode` ออกจาก logic (เหลือไว้แค่ debug)
- อ่าน REG_BS_MODE (0x01) **เสมอ** (ไม่ gate ด้วยสวิตช์):
  - bit0 HOME → AutoMission_GoHome (MODE_AUTO)
  - bit1 JOG → **MODE_MANUAL** (เดิมเข้า auto-jog) — gripper/jog/joystick ทำงานในนี้
  - bit2 AUTO → Pick&Place / GoPoint (pending 150ms)
  - bit3 SET_HOME → Homing_SetHome (ไม่เปลี่ยนโหมด)
  - bit4 TEST → Performance/Precision (pending 150ms)
- dispatch: MODE_MANUAL → Joystick_Update() → (ถ้าไม่ active) Dashboard_Update() → Gripper_Update()
- **guard re-send:** JOG/TEST reset เฉพาะตอน "เปลี่ยนเข้า" โหมดนั้น (กัน base เขียน 0x01 ซ้ำ
  ทุก poll → Cascade_Control_Reset รัวๆ → มอเตอร์สะดุด); AUTO ใช้ auto_start_pending เดิม

**⚠ สมมุติฐานที่ต้องเทสบนของจริง:** base MANUAL tab ต้องเขียน **0x01 = 2 (JOG/Manual)**
เพื่อให้ firmware เข้า MODE_MANUAL ก่อน gripper (0x02/0x03) จะทำงาน. ทดสอบ: กด tab MANUAL
แล้วกด OPEN — ถ้า gripper ไม่ขยับแปลว่า base ไม่ได้ส่ง 0x01=2 → ต้องเพิ่ม auto-enter MANUAL
เมื่อมี manual command เข้ามา.

**Recheck Modbus I/O (รับ-ส่งค่าจริง):**
- RX: FC03 read block 0x00–0x31 (50 reg) → ตอบเมื่อ TX ว่าง (base retry ได้)
- RX: FC06/FC16 write → เขียน register **เสมอ** (แม้ TX busy) → command ลงแน่
- TX telemetry: 0x28/0x29/0x30 = ×10, 0x26 reed, 0x27 task, 0x31 estop, 0x32 sysmode ✓
- screenshot จริง: "Connected" + "Heartbeat: Normal" → comms layer ทำงานแล้ว

---

## [2026-05-31] จูน firmware ให้ตรง base system UI จริง (return-home / STOP / manual jog)

ดู base system UI จริงแล้วแก้ firmware ให้ตรงพฤติกรรม:

1. **Pick&Place กลับ home หลังจบ** (auto_mission.c): เดิม PP_DWELL_PLACE จบ → PP_DONE
   (ค้างจุดสุดท้าย). แก้: ครบทุก pair → `Septic_MoveTo(...→0)` + PP_GO_HOME → hold ที่ 0°
   → REG_BS_TASK = Idle. (ตรงสเปก "ขยับครบ index แล้วกลับ home")
2. **MAX_PAIRS 5 → 8** (auto_mission.c): slots 0x12–0x21 = 16 reg = 8 คู่.
3. **Global STOP** (main.c TIM6 ISR): ปุ่ม STOP บน base = REG_BS_SOFT_STOP(0x25)=1.
   เดิมเช็คเฉพาะ AUTO/TEST. เพิ่ม global check หลัง MODE_HOMING: 0x25=1 → reset auto/test +
   PWM compare=0 (มอเตอร์ดับทุกโหมด incl. MANUAL/joystick) + REG_RUN=0. ไม่ latch
   (ต่างจาก E-Stop) → base เคลียร์ 0x25=0 แล้วสั่งใหม่ได้.
4. **Base Jog ใน MANUAL** (dashboard.c): เดิม jog (0x05) ทำงานเฉพาะ selector=AUTO
   (ผ่าน AutoMission). เพิ่มใน Dashboard_Update (MODE_MANUAL): 0x05≠0 → relative Septic move
   (+deg CCW / −deg CW) เปิด pos loop ชั่วคราว (REG_POS_KP=1550) ถึงเป้าแล้ว**เคลียร์ pos gain=0**
   (กัน _dash_stop_motor สั่ง ref_q=0 → วิ่งกลับ home). ทำงานร่วม joystick ได้ (joystick
   active → skip dashboard).

**ยืนยันแล้ว (ไม่ต้องแก้):**
- **P2P เป็น absolute จาก home** ทั้ง index (×5°) และ degree → firmware GoPoint ทำถูกอยู่แล้ว
- Mode เลือกจาก web (0x01) → Priority 3 (selector=AUTO) อ่าน 0x01 ครบ (Home/Jog/Auto/SetHome/Test)
- Joystick ใช้ได้เฉพาะ MODE_MANUAL (selector switch = MANUAL) ตามสเปกเดิม

---

## [2026-05-31] CubeMX ปิด "generate peripheral init as a pair" → โค้ดหาย/compile พัง

**อาการ:** หลังกด generate (toggle "init as a pair" OFF) โค้ดหลายส่วน "หาย" → build ไม่ผ่าน

**สาเหตุ:** CubeMX ย้าย MX_*_Init ทั้งหมดกลับเข้า `main.c` (static) และ **ลบไฟล์
`gpio.c/.h`, `tim.c/.h`, `usart.c/.h`, `fdcan.c/.h` ทิ้ง**. MspInit ทุกตัวถูกรวมไว้ใน
`stm32g4xx_hal_msp.c`. handle (huart2/htim1/2/6/hfdcan1/hadc1) นิยามใน main.c ที่เดียว.
→ ไฟล์ที่ยัง `#include "tim.h"/"usart.h"` (homing.c, joystick.c, base_system.c,
stm32g4xx_it.c) compile ไม่ผ่าน ("No such file"). + object เก่า (gpio.o ฯลฯ) ค้างใน Debug/
→ ถ้า build ไม่ clean จะ duplicate symbol กับ main.o.

**แก้ไข:**
- `main.h` (USER CODE Private defines): เพิ่ม `extern` ของ hadc1/hfdcan1/htim1/2/6/huart2
  → module อื่นเข้าถึง handle ผ่าน main.h (ทนต่อ regenerate)
- เปลี่ยน include ใน homing.c / joystick.c (`tim.h`→`main.h`),
  base_system.c / stm32g4xx_it.c (`usart.h`→`main.h`)
- ลบ object เก่า gpio/tim/usart/fdcan ใน `Debug/Core/Src/`
- อัปเดต `Debug/Core/Src/subdir.mk` (เอา gpio/tim/usart/fdcan.c ออก) และ Drivers subdir.mk
  (เพิ่ม stm32g4xx_hal_adc.c/_ex.c — เพราะ CubeMX เปิด ADC1)
- CubeMX gen GPIO ของปุ่ม joystick (PA5/6/7, PB10/11) เป็น INPUT NOPULL →
  `Joystick_Init()` override เป็น PULLUP เอง (ปุ่ม active-LOW ต้องมี pull-up)

**สำคัญ:** ทุกครั้งที่ CubeMX regenerate → ทำ **Project → Clean… → Build** ใน CubeIDE
เพื่อ regenerate subdir.mk ใหม่หมด (กัน object เก่าค้าง → duplicate symbol).
ADC1 ที่ CubeMX เปิด (channel 9) ไม่เกี่ยวกับ joystick — joystick ใช้ ADC2 (PC2) แบบ
bare-metal ใน joystick.c. จะลบ ADC1 ใน .ioc ทิ้งก็ได้ถ้าไม่ใช้.

---

## [2026-05-31] เพิ่มระบบ Joystick (Funduino) + แก้ E-stop resume + heartbeat hardening

**งานที่ทำ (4 ระบบ):**

1. **Heartbeat hardening:** เพิ่ม `volatile` ให้ `modbus_registers[]`, `modbus_echo_valid`,
   `modbus_echo_time` (เขียนใน ISR/RX callback อ่านใน main loop — กัน compiler cache ค่า).
   ตรวจ config: USART2 230400, 9B+EVEN (=8E1), stop 1 → ตรงกับ base system README ✓.
   non-blocking TX fix (Transmit_IT) ยังอยู่ครบ.
   ⚠ ถ้ายัง error หลัง flash: liveness ของ base ขึ้นกับการ "อ่าน YA" (FC03) ไม่ใช่ FC06 echo
   → ถ้า read reliable ก็ alive. FC06 echo ถูก skip ตอน TX busy (reg ยังเขียนลงเสมอ) —
   ถ้า base lib ต้องการ echo confirm อาจต้องทำ TX software-queue (งานต่อ).

2. **Modbus wiring ครบ:** เพิ่ม `REG_BS_GRIPPER_SEQ (0x03)` Pick=1/Place=2 → gripper.c
   จัดการใน G_IDLE (force enable + Pick/Place). ทุก register 0x00–0x50 เชื่อมครบแล้ว.

3. **Joystick (MODE_MANUAL only):** ไฟล์ใหม่ `joystick.c/.h`. อ่านปุ่ม PA5/PA6/PA7/PB11/PB10
   (debounce 20ms, pull-up active-LOW) + ADC2_IN8 (PC2, bare-metal single-conv).
   เรียกจาก TIM6 ISR ใน selector==MANUAL: `Joystick_Update()` คืน 1 = drive motor → skip Dashboard.

4. **ปุ่ม/ADC mapping:**
   - A (PA5) = Emergency → ตัด PWM + MOE disable + REG_ESTOP=1 (clear ที่ตู้ PC13 เท่านั้น)
   - B (PA6) = Gripper Pick/Place toggle (REG_BS_GRIPPER_SEQ)
   - C (PA7) = Set Home → zero TIM2 encoder (Homing_SetHome)
   - D (PB11) = Arm Up/Down toggle (REG_BS_GRIPPER_MAN)
   - K (PB10) = สลับ Free ↔ Point (default Free); Point เปิด pos loop (ตั้ง REG_POS_KP=1550)
   - ADC X (PC2): Free → <800 CCW / >3500 CW ที่ 15% duty (1500/9999, bypass cascade);
     Point → ±5° ต่อคลิก, ต้องปล่อยกลับ neutral ก่อนสั่งครั้งถัดไป

**แก้ E-stop resume (main.c HAL_GPIO_EXTI_Callback):** เดิม `NVIC_SystemReset()` ตอนปล่อยปุ่มตู้ →
เปลี่ยนเป็น clear REG_ESTOP + `__HAL_TIM_MOE_ENABLE` + `Cascade_Control_Reset()` →
ทำงานต่อจากเดิมทันที ไม่ reset MCU (ตาม requirement).

**ขาดข้อมูล / ต้องยืนยัน:** ทิศ CCW/CW ใน free mode — ใส่ `JOY_DIR_CCW/JOY_DIR_CW` ใน joystick.h
ให้สลับเองถ้าหมุนกลับด้าน (ตอนนี้เดา CCW=RESET ตาม Motor_Drive()).

---

## [2026-05-31] Heartbeat error ที่ 230400 — write หาย (blocking TX block RX)

**อาการ:** ที่ baud 230400 base system อ่าน telemetry ได้ (read OK) แต่ **write ไม่ลง**
→ heartbeat HI (write) ไม่ถึง firmware → heartbeat error. (ที่ 19200 เคยปกติ)

**วิธี diagnose (SWD):**
- `modbus_echo_buf`(0x..294) = 0 ทั้งหมด → **FC06 write ไม่เคยถูกประมวลผล**
- rx==ok, ORE=0, rx_buffer=128 (ไม่ใช่ truncate/CRC/overrun ค้าง)
- reg0 = YA ตลอด (PC เขียน HI ไม่ลง)

**สาเหตุ:** `Modbus_Parse_Frame` ใช้ `HAL_UART_Transmit` (**blocking**) ตอบ read
ภายใน RX interrupt → block RX ~4.6ms (FIFO ปิด, buffer 1 byte) ตอนตอบ read 105 byte
→ ที่ 230400 base **pipeline เร็ว** ยิง write (HI) แทรกตอน firmware กำลัง transmit
→ RX ไม่ทันรับ → **overrun → write frame ทิ้ง** (error callback ล้าง ORE)
→ read รอด (retry) / write (HI 5Hz) โดนทิ้ง
(ที่ 19200 base poll ช้า รอ response ทีละอัน ไม่ยิงทับ → write ลงได้)

**แก้ไข:**
- `base_system.c`: `HAL_UART_Transmit` → **`HAL_UART_Transmit_IT`** (non-blocking) +
  static `mb_tx[135]` + ส่งเฉพาะตอน `gState==READY`. **FC06/FC16 เขียน register เสมอ**
  (แม้ TX busy → HI/config ลงแน่) ตอบ echo เฉพาะตอน TX ว่าง
- `main.c`: `HAL_UARTEx_EnableFifoMode(&huart2)` (RX FIFO 8 byte — margin)
- → RX ไม่ถูก block → write frame ไม่หาย → heartbeat กลับมา

---

## [2026-05-29] Thin spike ±27V ตอนเปลี่ยนทิศ — vel Kd kick บน backlash step

**อาการ:** กราฟ V_in มี spike แหลม 1-sample ±22-27V ตอนเริ่ม/เปลี่ยนทิศ move

**สาเหตุ (คำนวณตรงเป๊ะ):** backlash inverse comp inject `BL_RAD=0.03566 rad` เป็น
**step** เข้า ref_q ตอนเปลี่ยนทิศ → pos error step → ไหลผ่าน:
```
pos P (Kp 15) = 15 × 0.03566       = 0.535 rad/s  (velocity setpoint step)
vel D (Kd 0.05) = 0.05 × 0.535/0.001 = 26.7V       = spike ที่เห็น ✓
```
vel Kd ขยาย step ×50 (÷dt=1ms) → spike

**แก้ไข:** `VEL_KD_AUTO 0.05 → 0` (ตัด derivative kick)
ทางเลือกเสริม: soften backlash inject (ramp แทน step) ถ้าต้องคง Kd

---

## [2026-05-29] Move ใหญ่ saturate ±20V → กระตุก — V_FF เกินที่ velocity สูง

**อาการ:** กราฟ V_in: move ใหญ่ (250°) saturate เต็ม +20V (accel) → -20V (decel)
→ control พัง ตอน saturate → กระตุก/overshoot

**สาเหตุ:** `K_ff = 5.4 V/(rad/s)` → ที่ peak velocity 4.5 rad/s (TRAJ_QV_MAX):
`V_FF = 5.4 × 4.5 = 24.3V` = **saturate เต็มแค่ feedforward** → ไม่เหลือ headroom
ให้ feedback → มอเตอร์ทำ 4.5 rad/s ไม่ไหวจริง (ต้องใช้ 24V แค่สู้ friction)
+ V_acc (0.768×15=11.5V) ตอน accel ยิ่งซ้ำ

**แก้ไข (trajectory.h):**
- `TRAJ_QV_MAX` 4.5 → **3.5** (V_FF = 18.9V, เหลือ ~5V ให้ feedback)
- `TRAJ_QA_MAX` 15 → **12** (V_acc = 9.2V, กัน accel saturate)
- หลักการ: peak voltage (V_FF+V_acc+feedback) ต้อง < MAX_VOLTAGE 24V

---

## [2026-05-29] Auto กระตุก แต่ Dashboard ลื่น (gain เท่ากัน) — V_dist + move ใหญ่

**อาการ:** dashboard P2P (move เล็ก) ลื่นมาก แต่ auto P&P (move ใหญ่) กระตุก/staircase
ทั้งที่ trajectory (Septic) + FF + POS_DIV=1 + gain เหมือนกัน

**สาเหตุ 2 ชั้น:**
1. **POS_DIV=5 staircase** (แก้แล้ว → POS_DIV=1)
2. **vel Kd=0.05** ขยาย KF velocity noise (×50 ที่ dt=1ms) → แก้ → Kd=0
3. **V_dist (KF tau_d) + vel_fade** — fade เป็น 0 เมื่อ |qd|<1:
   - dashboard move เล็ก → |qd|<1 → V_dist faded → ลื่น
   - auto move ใหญ่ (index 50=250°) → |qd| ถึง 4.5 → **V_dist active เต็ม** → tau_d noise → กระตุก

**แก้ไข:**
- POS_DIV 5→1 (cascade_control.h), vel Kd→0
- **ปิด V_dist** — แกนหมุน**ระนาบแนวนอน → ไม่มี gravity torque** → V_dist ไม่จำเป็น
  (V_fric จัดการ deadband แล้ว). `Motor_Drive(V_VEL + V_FF + V_acc + V_fric)`
  ถ้าใช้แกนตั้ง (มี gravity) ค่อยเอา V_dist กลับ

---

## [2026-05-29] Encoder wrap ทุก 1 รอบ — ARR ไม่ใช่ 0xFFFFFFFF

**อาการ:** encoder อ่านได้ แต่พอครบรอบ position_raw กระโดด/วน (wrap)

**สาเหตุ:** delta method `(int32)(cur - last)` ถูกต้องเองเฉพาะเมื่อ TIM2 ARR =
0xFFFFFFFF (วนที่ 2^32). ถ้า build เก่า ARR เล็ก (เช่น 0xFFFF) → counter วนเร็ว →
ตอนวน cur=0,last=N-1 → diff = -(N-1) กระโดดลงแทน +1 → position วน

**เช็ค:** source tim.c + `.ioc` = `TIM2.PeriodNoDither=4294967295` (0xFFFFFFFF) ✓ ถูกแล้ว
→ wrap มาจาก firmware build เก่าที่ flash อยู่

**แก้ไข (encoder.c):** เพิ่ม wrap correction อ่าน ARR มาชดเชย — robust ทุกค่า ARR:
```c
uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
if (arr != 0xFFFFFFFF) {
    int32_t period = arr + 1;
    if      (diff >  period/2) diff -= period;
    else if (diff < -period/2) diff += period;
}
```
+ reflash build ปัจจุบัน (ARR=0xFFFFFFFF)

---

## [2026-05-29] Modbus ไม่ตอบเลย (rx_count=0) — BAUD mismatch (dashboard ยัง 19200)

**อาการ:** dashboard ต่อ COM ได้ (`OK 19200 8E1`) แต่ READ time out ทุก frame
"No communication with the instrument" — **เปลี่ยนบอร์ดก็อาการเดิม**

**วิธี diagnose (SWD + STM32_Programmer_CLI) — เทคนิคสำคัญ:**
1. ต่อ SWD hotplug → CPU 3.29V, board ตอบ → ไม่ใช่ USB หลุด
2. ICSR(0xE000ED04)=0, CFSR(0xE000ED28)=0 → **ไม่ใช่ HardFault**
3. REG_ISR_CNT (modbus_registers@0x20000204 +69×2) เพิ่ม ~1kHz → **firmware healthy**
4. reg0 = 0x5961 (YA) ค้าง → PC เขียน HI ไม่ถึง
5. `debug_uart_rx_count`(0x20000320) = **0** → UART RX ไม่เคยยิง → byte ไม่ถึง

**สาเหตุ:** baud ไม่ตรง — base system + firmware อัปเกรดเป็น **230400** แล้ว
แต่ dashboard ยังตั้ง 19200 → byte เพี้ยน → framing error → RxEventCallback ไม่ยิง → rx_count=0
(flash build เดียวกันทุกบอร์ด → อาการเหมือนกันทุกบอร์ด)

**แก้ไข:** sync ทุกฝั่งให้ baud ตรงกัน (ปัจจุบัน = **230400**)
- firmware `usart.c` / `.ioc` `USART2.BaudRate` = 230400  (เจ้าของแก้ใน MX)
- `tools/pid_dashboard.py` `BAUD = 230400`
- base system PC app = 230400
- ⚠️ ทั้ง 3 ฝั่งต้องตรงกันเสมอ ไม่งั้น rx_count=0

**บทเรียน:** Modbus เงียบ + พอร์ตเปิดได้ → เช็ค `debug_uart_rx_count` ผ่าน SWD ก่อน
ถ้า =0 = ปัญหา RX layer (baud/parity/pin) ไม่ใช่ logic ฝั่ง firmware

---

## [2026-05-29] Homing ค้างใน H_RETURN — เจอ sensor + count แต่ไม่กลับ home

**อาการ:** Homing เจอ sensor (H_SEEK ✓) เริ่มนับความกว้าง flag (H_COUNT ✓) แต่ **ไม่ return** ไปกึ่งกลาง (ค้างถาวร)

**สาเหตุ:**
1. `HOMING_RETURN_DUTY = 550` (5.5%) อ่อนเกิน — H_RETURN ต้อง **กลับทิศจากนิ่ง**:
   - เอาชนะ static friction (เริ่มจาก standstill หลัง settle 1s)
   - **เก็บ backlash 2° ก่อน** joint encoder (henc2) ถึงขยับ
   - 550 ไม่พอ → มอเตอร์ค้าง เก็บ backlash ไม่ผ่าน → `pos` ไม่เปลี่ยน → `done` ไม่จริง → ค้าง
   - (H_SEEK/H_COUNT วิ่งทิศเดียวต่อเนื่อง มี momentum → 550 พอ)
2. **H_COUNT/H_RETURN ไม่มี timeout** (ต่างจาก H_SEEK/H_LEAVE) → ค้างแล้วไม่มีทางออก

**แก้ไข (homing.c):**
- `HOMING_RETURN_DUTY` 550 → **800** (8%) — สู้ friction + backlash reversal
- เพิ่ม `HOMING_COUNT_TICKS` / `HOMING_RETURN_TICKS` = 15s → timeout → ESTOP + MANUAL
  (เหมือน H_SEEK) กันค้างถาวร

**[2026-05-29 ต่อ] อีก manifestation: H_RETURN → H_DONE ทันที ไม่หมุนกลับ**
- สาเหตุ: `zone_count ≈ 0` → target=0 → `done` (pos≤0) จริงทันทีที่เข้า H_RETURN
- zone≈0 เพราะ H_COUNT จับ "sensor LOW" ตั้งแต่ tick แรก: robot coast ทะลุ flag
  ช่วง settle 1s หรือ H_SEEK เจอ glitch HIGH ไม่ได้เข้า flag จริง
- แก้: เพิ่ม `count_armed` (ต้องเห็น sensor HIGH ใน H_COUNT ก่อน) +
  `HOMING_ZONE_MIN=30` counts (ต้องเดินทะลุ flag จริงก่อนรับ far edge)
  → กัน false-trigger zone≈0

---

## [2026-05-27] Steady-state oscillation ±2° — voltage bang-bang ~20V

**อาการ:** หุ่น oscillate ±2° รอบ setpoint ตลอดเวลา แม้ไม่มี ref command  
qd_out กระเพื่อม ±0.3 rad/s, voltage กระโดด ±~20V แบบ bang-bang

**สาเหตุ:** Integral windup ใน velocity loop (vel_ctrl)
- `vel_ctrl.integral_limit = MAX_VOLTAGE = 24V` → Ki × integral สามารถ contribute ได้ถึง ±24V
- ไม่มี anti-windup → เมื่อ output saturate ยังคง integrate ต่อ
- ทำให้เกิด limit cycle: integral ใหญ่ → overshoot → integral กลับทิศ → overshoot ซ้าย → วนไม่หยุด

**แก้ไข (cascade_control.c):**

1. **Anti-windup** ใน `calculate_pid()`: conditional integration — หยุด update integral เมื่อ output saturate ไปในทิศเดียวกับ error:
   ```c
   if (!((output >  pid->integral_limit && error > 0.0f) ||
         (output < -pid->integral_limit && error < 0.0f))) {
       pid->integral = i_new;
   }
   ```

2. **ลด vel_ctrl.integral_limit** จาก `MAX_VOLTAGE` (24V) → `6.0f`:  
   จำกัด integral contribution สูงสุด Ki × 6 = ±6V (ป้องกัน windup ที่ร้ายแรง)

3. **เพิ่ม vel_fade threshold** จาก `0.3 rad/s` → `1.0 rad/s`:  
   V_dist fade เป็น 0 เร็วขึ้น → ไม่ drive motor ตอน near-stop  
   V_dist re-enable กลับมาพร้อม threshold ใหม่นี้

---
