# Lab 3 — Kalman Filter: แผนการทำ (ฉบับ SDI logging)

> KF 4 states `[q, q̇, τ_d, i]` บน STM32G474RE
> เก็บข้อมูล: USART2 → Simulink → **Simulation Data Inspector (SDI)** → export เป็น .mat
> อัปเดต: 2026-06-07

---

## รายการที่ต้องส่ง (5 ข้อ)

| # | หัวข้อ | ต้องเข้าหุ่น? | ผลที่เก็บ |
|---|---|:---:|---|
| 1 | State-Space Model (discrete, 4 states) | ✗ | สมการ + ตารางเมทริกซ์ |
| 2 | เหตุผลเลือก state + บทบาท | ✓ | กราฟ τ_d ตอนดันมือ + กราฟ i vs i_model |
| 3 | KF library (matrix op) บน STM32 | ✗ | โค้ด + screenshot |
| 4 | Q,R + trade-off (สถิติ) | ✓ | กราฟ overlay + ตาราง std/lag |
| 5 | ตรวจ state ที่ไม่ได้วัด | ✓ | กราฟ KF vs finite-diff + ตาราง |

---

## ทำเรียงตามนี้

| ลำดับ | ทำอะไร | ได้ข้อไหน |
|---|---|---|
| **1** | เขียนทฤษฎี item 1 + item 3 (ไม่ต้องต่อหุ่น) | 1, 3 |
| **2** | เข้าหุ่น — ทดลอง A: move 0→90° | 5 + 2B |
| **3** | เข้าหุ่น — ทดลอง B: Q sweep 3 ค่า | 4 |
| **4** | เข้าหุ่น — ทดลอง C: ดันมือ | 2A |
| **5** | วิเคราะห์ + เขียนเล่ม | 2, 4, 5 |

> ทำข้อ 1,3 ก่อน (งานเขียน) → เข้าหุ่นรวดเดียว 3 การทดลอง → ทดลอง A ได้ 2 ข้อในรอบเดียว

---

## เตรียมก่อน (ครั้งเดียว)

1. **ตั้งชื่อ signal ใน Simulink** ให้ครบ 17 เส้น — สำคัญ! ตอนนี้ 3 เส้นยังเป็น `MATLAB Function:15/16/17`
   → คลิกที่เส้น → ตั้งชื่อ: `qd_fd`, `tau_d`, `i_est` (คลิกขวาเส้น → Properties → Signal name, + ติ๊ก Log)
2. Build + Flash firmware ล่าสุด (มี 17 ช่องแล้ว)
3. Debug → Live Expressions: `analysis_mode = 1`, selector = MANUAL
4. Serial Receive block: **packet = 36 byte**, header `[170 85]`

---

## ▶ วิธีเก็บ 1 run (ใช้ซ้ำทุกการทดลอง)

```
1. cmd_deg = 0       (แขนนิ่งที่ 0° รอ ~1 วิ)
2. กด Run ใน Simulink   (เริ่มไหลเข้า SDI)
3. cmd_deg = 90      (แขนวิ่ง 0→90°)
4. รอจนนิ่ง + ค้างต่อ ~2 วิ
5. กด Stop
6. ใน MATLAB:  lab3_save_run('lab3_baseline.mat')   ← เซฟ run ล่าสุดเป็น .mat
```

> `lab3_save_run.m` ดึง run ล่าสุดจาก SDI เซฟทุกสัญญาณลงไฟล์เดียว
> `load('lab3_baseline.mat')` → ได้ `t, q_ref, qd_out, qd_fd, tau_d, i_est, v_in, ...` ครบ

ชื่อไฟล์ที่จะเก็บ:
```
lab3_baseline.mat    (เฟส2)
lab3_Q_1e-6.mat / lab3_Q_1e-4.mat / lab3_Q_1e-2.mat   (เฟส3)
lab3_disturb.mat     (เฟส4)
```

---

## เฟส 1 — ทฤษฎี (item 1 + 3) — ไม่ต้องต่อหุ่น

**Item 1 (State-Space):** คัดจาก comment ใน [kalman_filter.c](../Core/Src/lib/kalman_filter.c)
- state `[q, q̇, τ_d, i]` + ความหมายแต่ละตัว
- เมทริกซ์ Ac, Bc, C (มีตัวเลข) → discretize `Ad=I+Ac·Δt`, `Bd=Bc·Δt` (Δt=1ms)
- ตาราง Ad (4×4) + Bd พร้อมตัวเลข
- **เก็บ:** สมการ + ตารางเมทริกซ์ + diagram

**Item 3 (Library):** จาก [kalman_filter.c](../Core/Src/lib/kalman_filter.c)
- โค้ด `mat4_mul`, `mat4_mul_BT`, `KF_Update` (predict → update + Joseph form)
- **เก็บ:** code listing + screenshot Live Expressions โชว์ `hkf.x[0..3]` วิ่ง + ระบุรัน 1kHz ใน TIM6 ISR

---

## เฟส 2 — ทดลอง A: Baseline move → item 5 + 2B

เก็บ 1 run (0→90°) Q default → `lab3_baseline.mat`

**Item 5 — KF velocity เทียบ finite-diff:**
```matlab
load lab3_baseline.mat
idx = t >= t(end)-1.5;                       % ช่วงแขนนิ่ง
nr  = std(qd_fd(idx)) / std(qd_out(idx));    % noise ลดกี่เท่า (ควร >>1)
rms = sqrt(mean((qd_out-qd_fd).^2));         % KF ตามแนวโน้มไหม
plot(t,qd_fd,'Color',[.7 .7 .7]); hold on; plot(t,qd_out,'c','LineWidth',1.5);
legend('finite-diff (ดิบ)','Kalman'); xlabel('t (s)'); ylabel('q̇ (rad/s)');
title(sprintf('noise ลด %.1f เท่า | RMSE=%.3f',nr,rms));
```
**เก็บ:** กราฟ overlay (KF เนียน vs finite-diff noisy) + zoom ช่วงนิ่ง + ตาราง `std(qd_fd)`, `std(qd_out)`, `noise ratio`, `RMSE`

**Item 2B — current เทียบ model:**
```matlab
Ke=0.1045; N=2; R=2.1142;
i_model = (v_in - Ke*N*qd_out)/R;
plot(t,i_model,'w--'); hold on; plot(t,i_est,'y');
legend('i_{model}','i_{KF}'); ylabel('A');
title(sprintf('RMSE=%.3f A', sqrt(mean((i_est-i_model).^2))));
```
**เก็บ:** กราฟ i_KF vs i_model + RMSE

---

## เฟส 3 — ทดลอง B: Q sweep → item 4

เก็บ run เดิม (0→90°) **3 รอบ** เปลี่ยน `hkf.Q[1][1]` (Q_VEL) ใน Live Expressions:
`1e-6` → `1e-4` → `1e-2` (เซฟแยกไฟล์) — แก้สดไม่ต้อง reflash

**วิเคราะห์แต่ละไฟล์ (2 ตัวเลข):**
```matlab
load lab3_Q_1e-6.mat
idx = t >= t(end)-1.5;
smooth = std(qd_out(idx));            % ต่ำ = เนียน (smoothness)
Fs  = 1/median(diff(t));
lag = finddelay(qd_ref,qd_out)*1000/Fs;   % ms — ต่ำ = ไว (responsiveness)
fprintf('std=%.4f  lag=%.1f ms\n',smooth,lag);
```
**เก็บ:**
- กราฟ overlay `qd_out` 3 ค่า Q บนแกนเดียว
- ตาราง:

| Q_VEL | std (smooth) | lag ms (responsive) |
|---|---|---|
| 1e-6 | ต่ำ | สูง |
| 1e-4 | กลาง | กลาง |
| 1e-2 | สูง | ต่ำ |
- สรุป physical: Q↑ = เชื่อ measurement = ไวแต่ noisy / Q↓ = เนียนแต่หน่วง

---

## เฟส 4 — ทดลอง C: ดันมือ → item 2A

`cmd_deg` ค้างที่ 45° → กด Run → **ดันมือเบาๆ** ค้าง 2-3 วิ ปล่อย ทำ 2-3 ครั้ง → `lab3_disturb.mat`
```matlab
load lab3_disturb.mat
subplot(3,1,1); plot(t,q_out,'c');  ylabel('q (rad)');
subplot(3,1,2); plot(t,qd_out,'g'); ylabel('q̇');
subplot(3,1,3); plot(t,tau_d,'r');  ylabel('τ_d (N·m)'); xlabel('t (s)');
```
**ควรเห็น:** ดัน → `τ_d` เด้งต้านแรง แต่ `q_out` ยังนิ่ง = state τ_d ดูดซับ disturbance

**(ทางเลือก) ablation:** ทำซ้ำแต่ตั้ง `hkf.Q[2][2]=1e-15` (แช่แข็ง τ_d) → เทียบ std(position error) สองกรณี
**เก็บ:** กราฟ 3 subplot มาร์คจุดดัน (+ ตาราง ablation ถ้าทำ)

---

## เฟส 5 — รวมเล่ม

เขียน item 2 (เฟส 2B+4), item 4 (เฟส 3), item 5 (เฟส 2)
+ เหตุผลเลือก state: q=วัดได้ / q̇=เป้าควบคุม / i=เชื่อม V↔torque / τ_d=ดูดซับ load+friction

---

## เช็คก่อนส่ง

- [ ] item 1: สมการ + ตารางเมทริกซ์ + diagram
- [ ] item 2A: กราฟ q/q̇/τ_d (ดันมือ)
- [ ] item 2B: กราฟ i vs i_model + RMSE
- [ ] item 3: โค้ด + screenshot รันจริง
- [ ] item 4: กราฟ overlay 3 Q + ตาราง std/lag
- [ ] item 5: กราฟ KF vs finite-diff + ตาราง noise/RMSE
- [ ] ทุกกราฟ: label แกน + หน่วย + legend + title → export PNG
