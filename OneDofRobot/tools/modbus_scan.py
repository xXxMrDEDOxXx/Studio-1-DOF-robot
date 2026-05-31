"""
modbus_scan.py — Quick Modbus RTU scanner
ทดสอบว่า board ตอบ Modbus ไหม และอ่าน register พื้นฐาน
"""

import serial.tools.list_ports
import minimalmodbus
import time

# ══════════════════════════════════════════════
#  CONFIG — แก้ตรงนี้
# ══════════════════════════════════════════════
SLAVE_ID   = 21       # ลอง 1–247 ถ้าไม่รู้
BAUD       = 19200
PARITY     = 'E'      # E=Even, N=None, O=Odd
STOPBITS   = 1
TIMEOUT    = 0.5
# ══════════════════════════════════════════════

def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("❌ ไม่พบ COM port ใดเลย")
        return []
    print("📌 COM ports ที่มี:")
    for p in ports:
        print(f"   {p.device:8s} — {p.description}")
    return [p.device for p in ports]

def try_connect(port, slave_id):
    try:
        inst = minimalmodbus.Instrument(port, slave_id)
        inst.serial.baudrate = BAUD
        inst.serial.parity   = PARITY
        inst.serial.stopbits = STOPBITS
        inst.serial.timeout  = TIMEOUT
        inst.mode            = minimalmodbus.MODE_RTU
        inst.debug           = False
        return inst
    except Exception as e:
        print(f"   ❌ เปิด port ไม่ได้: {e}")
        return None

def read_regs(inst, start, count, label=""):
    try:
        regs = inst.read_registers(start, count, functioncode=3)
        print(f"   ✅ [{label}] addr 0x{start:02X}–0x{start+count-1:02X}: {regs}")
        return regs
    except Exception as e:
        print(f"   ❌ [{label}] อ่านไม่ได้: {e}")
        return None

def to_signed(val):
    return val if val < 32768 else val - 65536

# ══════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════
print("=" * 55)
print("  Modbus RTU Quick Scanner")
print(f"  Baud={BAUD}  Parity={PARITY}  Slave={SLAVE_ID}")
print("=" * 55)

ports = list_ports()
if not ports:
    exit(1)

# ให้ผู้ใช้เลือก port
if len(ports) == 1:
    port = ports[0]
    print(f"\n→ ใช้ port เดียวที่มี: {port}")
else:
    port = input(f"\nเลือก port (เช่น COM9): ").strip()

print(f"\n🔌 กำลัง connect → {port}  Slave={SLAVE_ID} ...")
inst = try_connect(port, SLAVE_ID)
if inst is None:
    exit(1)

# ── Test 1: Heartbeat (0x00) ──────────────────
print("\n[1] Heartbeat register (0x00)")
regs = read_regs(inst, 0x00, 1, "Heartbeat")
if regs:
    val = regs[0]
    if val == 22881:
        print(f"      → Board ส่ง YA (22881) = รอ HI จาก PC ✓")
    elif val == 18537:
        print(f"      → ค่า HI (18537) = PC ตอบแล้ว ✓")
    else:
        print(f"      → ค่า {val} (ไม่ใช่ heartbeat pattern)")

# ── Test 2: อ่าน block status 0x26–0x31 ─────
print("\n[2] Status block (0x26–0x31)")
regs = read_regs(inst, 0x26, 6, "Status")
if regs:
    labels = ["Reed sensors", "Current task", "Position×10", "Velocity×10", "Accel×10", "E-stop"]
    for i, (r, lbl) in enumerate(zip(regs, labels)):
        signed = to_signed(r)
        print(f"      0x{0x26+i:02X}  {lbl:15s}: raw={r:6d}  signed={signed:6d}", end="")
        if i >= 2:  # scaled ×10
            print(f"  → {signed/10:.1f}", end="")
        print()

# ── Test 3: ISR counter debug 0x30 ───────────
print("\n[3] ISR counter (0x30) — นับ 2 ครั้ง ห่างกัน 1s")
r1 = read_regs(inst, 0x30, 1, "ISR cnt t0")
time.sleep(1.0)
r2 = read_regs(inst, 0x30, 1, "ISR cnt t1")
if r1 and r2:
    delta = to_signed(r2[0]) - to_signed(r1[0])
    if delta > 0:
        print(f"      → TIM6 ISR ทำงาน! เพิ่มขึ้น {delta} counts/s ✓")
    else:
        print(f"      → ISR counter ไม่เพิ่ม ({delta}) = TIM6 ไม่รัน ❌")

# ── Test 4: Telemetry 0x20–0x22 ──────────────
print("\n[4] Tune telemetry (0x20–0x22)")
regs = read_regs(inst, 0x20, 3, "Telemetry")
if regs:
    ref_qd = to_signed(regs[0]) / 100.0
    qd_act = to_signed(regs[1]) / 100.0
    v_in   = regs[2] / 100.0
    print(f"      ref_qd = {ref_qd:+.2f} rad/s")
    print(f"      qd_act = {qd_act:+.2f} rad/s")
    print(f"      Vin    = {v_in:.2f} V")

print("\n" + "=" * 55)
print("  Scan เสร็จแล้ว")
print("=" * 55)
