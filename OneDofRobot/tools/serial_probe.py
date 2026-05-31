"""
serial_probe.py — ASCII serial probe
สำหรับ board ที่ตอบ plain text (ไม่ใช่ Modbus)
"""

import serial
import time

PORT     = 'COM3'
BAUD     = 19200
PARITY   = 'E'   # ลอง Even ก่อน
TIMEOUT  = 1.0

def try_open(baud, parity):
    try:
        s = serial.Serial(PORT, baudrate=baud, bytesize=8,
                          parity=parity, stopbits=1, timeout=TIMEOUT)
        return s
    except Exception as e:
        print(f"  ❌ เปิดไม่ได้: {e}")
        return None

def send_recv(ser, cmd, wait=0.5):
    ser.reset_input_buffer()
    ser.write(cmd)
    time.sleep(wait)
    resp = ser.read_all()
    return resp

# ── ลอง baud/parity หลายแบบ ─────────────────────
configs = [
    (19200, 'E'),
    (19200, 'N'),
    (115200, 'N'),
    (9600,  'N'),
]

print("=" * 50)
print("  Serial ASCII Probe")
print("=" * 50)

working_ser = None
working_cfg = None

for baud, par in configs:
    print(f"\n🔌 ลอง {baud} 8{par}1 ...")
    ser = try_open(baud, par)
    if ser is None:
        continue

    # flush แล้วอ่าน spontaneous message
    time.sleep(0.3)
    spontaneous = ser.read_all()
    if spontaneous:
        print(f"  📥 Spontaneous: {spontaneous}")
        working_ser = ser
        working_cfg = (baud, par)
        break

    # ส่ง newline ดูว่าตอบอะไร
    resp = send_recv(ser, b'\r\n')
    if resp:
        print(f"  📥 ตอบ '\\r\\n': {resp}")
        working_ser = ser
        working_cfg = (baud, par)
        break

    ser.close()

if working_ser is None:
    print("\n❌ ไม่มี config ไหนตอบ — ลองค่า default (19200 8E1)")
    working_ser = try_open(19200, 'E')
    working_cfg = (19200, 'E')

if working_ser is None:
    exit(1)

print(f"\n✅ ใช้ {working_cfg[0]} 8{working_cfg[1]}1")
print("\n" + "=" * 50)
print("  ทดสอบคำสั่งพื้นฐาน")
print("=" * 50)

# คำสั่งที่อาจ board รู้จัก
test_cmds = [
    (b'\r\n',               "empty newline"),
    (b'?\r\n',              "help ?"),
    (b'HELP\r\n',           "HELP"),
    (b'STATUS\r\n',         "STATUS"),
    (b'READ\r\n',           "READ"),
    (b'ID\r\n',             "ID"),
    (b'\x03',               "Ctrl+C"),
    (b'AT\r\n',             "AT command"),
]

for cmd, label in test_cmds:
    resp = send_recv(working_ser, cmd, wait=0.3)
    if resp:
        print(f"  [{label:15s}] → {resp}")
    else:
        print(f"  [{label:15s}] → (ไม่ตอบ)")

print("\n" + "=" * 50)
print("  Mini terminal — พิมพ์คำสั่งเองได้ (q = ออก)")
print("=" * 50)
working_ser.timeout = 0.2

while True:
    try:
        cmd = input("TX> ").strip()
        if cmd.lower() == 'q':
            break
        working_ser.reset_input_buffer()
        working_ser.write((cmd + '\r\n').encode())
        time.sleep(0.3)
        resp = working_ser.read_all()
        if resp:
            print(f"RX> {resp}")
        else:
            print("RX> (ไม่มีคำตอบ)")
    except KeyboardInterrupt:
        break

working_ser.close()
print("ปิด port แล้ว")
