# Lab 4 — Control Integration : แผนทำงานเฟส 0–2

> ระบบ: 1-DOF, STM32G474RE, firmware `OneDofRobot`
> เป้าหมาย spec (studio): error ≤ ±0.10°, P.O. < 1%, settling < 0.5 s, a ≥ 2 rad/s², v ≥ 3 rad/s

---

## เฟส 0 — เตรียมระบบ + ช่องทาง logging

### 0.1 Pre-checks
- [ ] Build/flash ผ่าน (แก้ port 61234 → ใช้ 3333 ใน Debug Config ถ้าชน Docker)
- [ ] Homing ทำงาน (boot → seek flag → home)
- [ ] E-stop ตู้ + จอย ตัดมอเตอร์ได้
- [ ] AUTO หยิบ/วางจริง (เพิ่ง fix `REG_BS_GRIPPER_EN`)
- [ ] ตรวจ reed: ดู `REG_BS_REED` (0x26) ตอนหุ่นนิ่ง ต้องไม่ใช่ค่า trigger ค้าง

### 0.2 เลือกวิธี log (ต้องการ ~1 kHz ให้ resolve overshoot/settling)

**แนะนำ: Firmware RAM logger (true 1 kHz, reproducible)**
- buffer ใน RAM เก็บทุก tick ตอน move แล้ว dump เป็น CSV
- capture point ที่ดีที่สุด = ใน `Cascade_Control_Update_FF()` (เห็นทั้ง ref และ state พร้อมกัน)
- ช่อง (channels) ที่ต้องเก็บ → ดู schema 0.3
- *ยังไม่มีในโค้ด — แจ้งให้ช่วย implement ได้*

**ทางเลือกเร็ว: STM32CubeMonitor (ไม่ต้องแก้โค้ด)**
- log ตัวแปร `volatile`: `q_out, qd_out, qdd_out, monitor_V_in, hkf.est_current, hkf.est_disturbance, henc2.position_rad`
- export CSV — แต่ rate อาจ < 1 kHz (เช็คว่าพอ resolve P.O. ไหม)

### 0.3 CSV schema (มาตรฐานเดียวให้ทั้ง logger + สคริปต์วิเคราะห์)
```
t,ref_q,q,ref_qd,qd,ref_qdd,V,i_est
```
หน่วย SI: `t[s] ref_q,q[rad] ref_qd,qd[rad/s] ref_qdd[rad/s²] V[V signed] i_est[A]`
(แปลงเป็น deg ในรายงานเฉพาะตอนเทียบ spec ±0.10°)

### 0.4 นิยาม move มาตรฐานสำหรับทดสอบ
- **Step ใหญ่** (เช่น 0°→90°) : วัด P.O./settling/control effort
- **Step เล็ก** (เช่น ±5°) : วัดความแม่น ±0.10° + backlash
- ใช้ move เดียวกันทุกการเปรียบเทียบ เพื่อ apple-to-apple

---

## เฟส 1 — ตารางการทดลอง (เก็บข้อมูล)

> ใช้ **MANUAL/Dashboard mode** สั่งผ่าน Modbus register (ดูตาราง register ด้านล่าง) หรือ AUTO/TEST mode ตามเคส
> ทุกการทดลอง: ทำซ้ำ **≥5 รอบ** เพื่อทำสถิติ

| # | เกณฑ์ (Lab) | ตั้งค่า / โหมด | log สัญญาณ | output ที่ต้องได้ |
|---|---|---|---|---|
| E1 | Motor Param: Damp/Inertia (L1) | step voltage + coast-down, ≥5 รอบ | t, qd, V | Mean/Std/95%CI, residual, cross-val |
| E2 | FF on/off (L2) | move เดียวกัน, สลับ FF on↔off | t, ref_q, q | RMS error, MAE (เทียบ) |
| E3 | Disturbance comp (L2) | ใส่ load/กดต้าน, comp on↔off | t, ref_q, q, i_est, taud | error ลดลง + อธิบายปริมาณที่ชดเชย |
| E4 | Kp/Ki/Kd sweep (L2) | เปลี่ยน `REG_POS/VEL_*` ทีละค่า | t, ref_q, q, V | กราฟ response แต่ละ gain |
| E5 | Anti-windup (L2) | สั่ง step ใหญ่จน saturate, on↔off | t, ref_q, q, V | response + integral ก่อน/หลัง |
| E6 | Saturation (L2) | จำกัด/ไม่จำกัด V | t, q, V | response + ประเด็น safety |
| E7 | Cascade tuning (L2, TA) | จูน outer/inner ทีละ loop | t, ref_q, q, qd | ขั้นตอน + ผลแต่ละสเตป |
| E8 | Trapz vs S-curve (L2) | move เดียวกัน, Trapz↔Septic | t, ref_q, q, ref_qd, ref_qdd, V | jerk, smoothness, tracking err, control effort |
| E9 | Q/R trade-off (L3) | เปลี่ยน `KF_Q_VEL`/`KF_R_MEAS` (recompile หลายค่า) | t, q, qd, i_est | std(qd) vs lag → trade-off |
| E10 | KF state validation (L3) | move ปกติ | t, qd(KF) + qd(finite-diff), i_est | KF vs วิธีอื่น สมเหตุผลไหม |
| E11 | Final integrated (L4) | step ใหญ่ + step เล็ก, ≥5 รอบ | ทุกช่อง | P.O., settling, ss-error เทียบ spec |

### Register ที่ใช้สั่ง (MANUAL/Dashboard)
| reg | addr | หน้าที่ |
|---|---|---|
| REG_VEL_KP/KI/KD | 0x33/34/35 | vel gains ×100 |
| REG_POS_KP/KI/KD | 0x3A/3B/3C | pos gains ×100 (0=ปิด pos loop) |
| REG_TARGET_POS | 0x3E | เป้าหมาย deg×10 |
| REG_WAVEFORM | 0x38 | 0=square 1=sine 2=step |
| REG_SPEED / REG_HALF_PERIOD | 0x36/37 | velocity-mode waveform |
| REG_DRIVE_MODE | 0x3D | 0=cascade 1=direct |
| REG_RUN | 0x39 | 1=run 0=stop |

### ⚠️ Hook ที่ต้องเพิ่มในเฟส 0 ก่อนทดลอง (แจ้งให้ช่วยได้)
1. **Firmware logger** (E1–E11) — buffer + trigger + dump CSV
2. **FF enable/disable flag** (E2) — runtime toggle V_FF/V_acc/V_fric ผ่าน Modbus reg
3. **Disturbance comp enable flag** (E3) — เปิด `V_dist` ที่ตอนนี้ `(void)` อยู่
4. (E9) Q/R: ใช้วิธี recompile ทีละค่าได้ ไม่ต้องเพิ่ม reg

---

## เฟส 2 — วิเคราะห์ (สถิติ + กราฟ SI)

ใช้สคริปต์ `analysis/lab4_analysis.py` (อ่าน CSV ตาม schema 0.3)

### Metric ที่คำนวณ
- **Tracking:** RMS error, MAE = mean/√mean ของ (ref_q − q)
- **Step response:** P.O.% = (q_max − q_ss)/(q_ss − q_0)×100 ; settling = เวลาที่ |q−q_ss| เข้าแบนด์ (ทั้ง 2% และ ±0.10°) ค้าง ; rise time ; steady-state error
- **Control effort:** ∫|V|dt และ ∫V²dt , peak V
- **Jerk/smoothness (E8):** max|jerk|, RMS jerk
- **Param ID (E1):** mean, std, 95% CI (t-dist), residual + normality (Shapiro), cross-validation RMSE
- **Q/R trade-off (E9):** std(qd) (noise) vs phase lag (responsiveness)

### Output ต่อเกณฑ์
- ทุกกราฟ: แกนหน่วย SI ครบ, legend, เทียบ case
- ตารางสรุป: ค่า metric + Mean±Std (CI) + p-value ที่เกี่ยวข้อง
- E11: ตารางเทียบ spec (ผ่าน/ไม่ผ่าน): P.O.<1%, settling<0.5s, error≤0.10°

### การใช้สคริปต์
```bash
python analysis/lab4_analysis.py step   data/E11_step90.csv          # P.O./settling/ss-error
python analysis/lab4_analysis.py track  data/E2_ff_on.csv            # RMS/MAE
python analysis/lab4_analysis.py compare data/E8_trapz.csv data/E8_scurve.csv
python analysis/lab4_analysis.py paramid data/E1_trial*.csv          # mean/std/CI/residual
```
