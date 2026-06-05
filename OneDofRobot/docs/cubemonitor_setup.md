# STM32CubeMonitor — Real-time monitor 9 ช่อง (q/qd/qdd ref+out, jerk ref+out, Vin)

อ่านตัวแปรตรงจาก RAM ผ่าน ST-Link (SWD) แบบ live ~kHz **โดยไม่หยุด CPU** + **export CSV ได้ในตัว**
ไม่กิน UART → รัน**คู่กับ Python dashboard ได้** (dashboard ใช้ VCP, CubeMonitor ใช้ SWD คนละช่อง)

---

## 0) เตรียม (ครั้งเดียว)
1. ติดตั้ง **STM32CubeMonitor** (ฟรีจาก ST — st.com)
2. Build โปรเจกต์ **Debug** config (มี `-g`, `-O0`) → ได้ไฟล์ symbol:
   `OneDofRobot/Debug/OneDofRobot.elf`
3. **Flash Debug build ตัวล่าสุด** ลงบอร์ด (โค้ดมี global `mon_*` + `ref_j` ครบแล้ว)
4. ปิด **debug session ใน CubeIDE** (อย่าค้าง debug ไว้ — ST-Link ใช้ได้ทีละตัว)
   แค่ Flash แล้วกด Run/รีเซ็ตให้แอปวิ่งอิสระ — ไม่ต้องอยู่ในโหมด debug

---

## 1) ตัวแปร 9 ช่อง (พิมพ์ชื่อนี้ตอน Add Variable)
| # | ชื่อ symbol | สัญญาณ | หน่วย | หมายเหตุ |
|---|---|---|---|---|
| 1 | `mon_q_ref`   | q reference     | rad    | ทุกโหมด |
| 2 | `mon_qd_ref`  | q̇ reference     | rad/s  | ทุกโหมด |
| 3 | `mon_qdd_ref` | q̈ reference     | rad/s² | ทุกโหมด |
| 4 | `ref_j`       | jerk reference  | rad/s³ | **live เฉพาะ AUTO** |
| 5 | `mon_q_out`   | q measured (KF) | rad    | ทุกโหมด |
| 6 | `mon_qd_out`  | q̇ measured (KF) | rad/s  | ทุกโหมด |
| 7 | `mon_qdd_out` | q̈ measured      | rad/s² | ทุกโหมด |
| 8 | `mon_j_out`   | **jerk จริง (measured)** | rad/s³ | ทุกโหมด (filter หนัก → laggy/นุ่ม) |
| 9 | `mon_v_in`    | Vin (signed)    | V      | แรงดันสั่งมอเตอร์ |

> jerk มี 2 ตัว: `ref_j` = jerk ของ trajectory (สั่ง), `mon_j_out` = jerk จริงที่วัด/diff ได้
> ตัวแปรเป็น `float` ทั้งหมด → เลือก display type = **float**

---

## 2) สร้าง Flow ใน CubeMonitor
CubeMonitor เปิดเป็นหน้าเว็บ (Node-RED) มี node สำเร็จรูปให้ลาก

**โครง flow:**
```
[ myVariables ] → [ myProbe (acquisition out) ] → [ Chart ]  ...
                                                 → [ myLogging (write file) ]   ← เก็บ CSV
```

### a) Acquisition / Variables node
1. ดับเบิลคลิก node **acquisition** (variables) → **Edit**
2. **Executable**: เลือกไฟล์ `Debug/OneDofRobot.elf` → มันจะ parse symbol
3. **Add variable** → พิมพ์/เลือกชื่อทั้ง 9 จากตารางข้อ 1 → ตั้ง type **float**
   (filter พิมพ์ `mon_` ติ๊ก 8 ตัว แล้วพิมพ์ `ref_j` ติ๊กอีก 1)
4. **Acquisition mode = Direct** (อ่าน live ขณะ CPU วิ่ง ไม่ halt)
5. **Frequency**: เริ่ม **1000 Hz** (ถ้า ST-Link V2 อืด/หลุด ลดเป็น 500/200 Hz — 8 ตัวแปร float)

### b) Probe / Connection node
1. เลือก **ST-Link** ที่เสียบอยู่ → interface **SWD**
2. ปล่อย default อื่น (ไม่ต้อง reset target)

### c) Chart (dashboard)
หลักการ: **1 Chart node = 1 การ์ด/กราฟ** บน dashboard. acquisition out 1 ตัว
ลากไปเข้าได้หลาย chart พร้อมกัน → ในแต่ละ chart ค่อยเลือกว่าจะโชว์ตัวแปรไหน

**แยกกราฟทีละช่อง (9 กราฟ):**
1. ลาก **Chart** node มา 1 ตัว → ดับเบิลคลิก Edit:
   - **Variables / Series**: ติ๊กเลือก **ตัวเดียว** (เช่น `mon_q_ref`)
   - **Label**: ตั้งชื่อกราฟ (เช่น `q_ref [rad]`)
   - **Group**: สร้าง/เลือก ui group (กำหนด layout) — ดูข้อ (d)
   - **Y-axis** min/max: ปล่อย auto หรือกำหนดเอง
2. ลากเส้นจาก **acquisition out** → เข้า chart นี้
3. **ทำซ้ำให้ครบ 9** — เร็วสุด: **copy-paste** chart ที่ตั้งเสร็จแล้ว (Ctrl+C/V)
   แล้วแก้แค่ *Variables* + *Label* ในตัวที่ copy มา
4. ทุก chart ต่อจาก **acquisition out ตัวเดียวกัน** (1 output → 8 charts ได้)

> **ทำไมควรแยก:** q (~±6 rad), qdd (~±50), Vin (~±24) สเกลต่างกันมาก
> ถ้ารวมกราฟเดียว ตัวใหญ่จะกดตัวเล็กแบน → แยกกราฟ = แต่ละช่องมีแกน Y ของตัวเอง อ่านง่าย

**ทางเลือก — จับคู่ ref/out แกนเดียว** (เทียบ tracking): chart ละ 2 series
เช่น chart "Position" เลือก `mon_q_ref` + `mon_q_out`, chart "Velocity" เลือก `mon_qd_ref` + `mon_qd_out` ฯลฯ → ได้ 4 กราฟ เห็น error ชัด

- ตั้ง X-axis เป็น time window เท่ากันทุก chart (เช่น 5–10 s) จะ scroll พร้อมกัน

### d) จัด layout (ui group / tab)
- แต่ละ chart ต้องมี **Group** (การ์ด) — กดที่ช่อง Group → **add new ui_group** ตั้งชื่อ + ความกว้าง (เช่น 6 = ครึ่งจอ)
- หลาย group อยู่ใน **Tab** เดียว → จัดเป็นกริด 2 คอลัมน์ได้ (9 กราฟ ≈ 5 แถว)
- จัดผ่าน sidebar **Dashboard → Layout** (ลากเรียง group/chart ได้)

### e) Logging (เก็บ CSV — ตัวที่ตอบโจทย์ "เก็บกราฟ")
- ลาก **out → write file (myLogging)** node ต่อจาก acquisition
- ตั้ง path + format **CSV** → ทุก sample ถูกบันทึกลงไฟล์ พร้อม timestamp → เอาไป plot/วิเคราะห์ต่อใน MATLAB/Excel ได้

---

## 3) รัน
1. กด **Deploy** (มุมขวาบน) — push flow
2. เปิดหน้า **Dashboard** (ไอคอนกราฟ มุมขวา → เปิด `/ui`)
3. กด **START** (acquisition) → กราฟ live ขึ้นทั้ง 9 ช่อง
4. สั่งหุ่นทำงาน (AUTO P&P / step ใน MANUAL) → เห็น ref กับ out วิ่งตามกัน real-time
5. กด **STOP** → ไฟล์ CSV ถูกเก็บไว้ตาม path ที่ตั้ง

---

## 4) เคล็ดลับ / ข้อควรรู้
- **เห็น ref ตอน AUTO ชัดสุด** (Septic generate q/qd/qdd/jerk ref ครบ) — ใน MANUAL ref เป็น step คงที่, `ref_j`=0
- **Sampling rate** ขึ้นกับ ST-Link: V3 ได้ ~kHz สบาย, V2 อาจ ~200–500 Hz กับ 8 ตัวแปร — ถ้ากราฟกระตุก/หลุด ลด Frequency
- รัน **คู่ Python dashboard ได้** (สั่งงานทาง dashboard, ดูกราฟทาง CubeMonitor) — แต่ **อย่าเปิด debug ใน CubeIDE พร้อมกัน**
- ถ้า CubeMonitor **มองไม่เห็นตัวแปร**: เช็คว่า build เป็น **Debug** (`-O0`) ไม่ใช่ Release (`-O2` อาจ optimize ตัวแปรหาย) และ flash ตรงกับ `.elf` ที่ import
- ค่าเป็น SI (rad, rad/s…) ถ้าอยากเป็น deg ใส่ scale ใน chart หรือ map node ได้

---

## 5) ทำไมเลือก CubeMonitor (เทียบ Simulink/dashboard)
| | CubeMonitor | Simulink Serial Rx | Python dashboard |
|---|---|---|---|
| งาน firmware | ไม่มี (มี global แล้ว) | ต้องเขียน stream + flash | — |
| กิน UART | ไม่ (ใช้ SWD) | ใช่ (Modbus หยุด) | ใช่ |
| เก็บ CSV | **ในตัว** | ต้องตั้ง To Workspace | มี (REC/burst) |
| rate | ~kHz | ~500Hz | poll ช้า |
| คู่ dashboard | ได้ | ไม่ได้ | — |
