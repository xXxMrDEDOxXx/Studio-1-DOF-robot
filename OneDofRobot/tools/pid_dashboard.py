"""
================================================================================
  1-DOF Robot — PID Tuner Dashboard
  Modes: Main | Velocity Tune | Position Tune | Cascade Tune | Modbus Test
================================================================================
  Register Map  (Dashboard zone: 0x32–0x45, ไม่ชนกับ Base System 0x00–0x31)

  SHARED:
    0x31  estop        — 0=OK  1=ESTOP  (ใช้ร่วมกับ Base System)
    0x32  sys_mode     — 0=Homing  1=Auto  2=Manual  (physical switch)

  DASHBOARD WRITE (PC → STM32, MODE_MANUAL เท่านั้น):
    0x33  vel_Kp × 100      0x34  vel_Ki × 100      0x35  vel_Kd × 100
    0x36  speed × 10        0x37  half_period (ms)   0x38  waveform (0/1/2)
    0x39  run flag (0/1)
    0x3A  pos_Kp × 100  (0 = disable pos loop)
    0x3B  pos_Ki × 100   0x3C  pos_Kd × 100
    0x3D  drive mode  0=Cascade(pos→vel→V)  1=Direct(pos→V)
    0x3E  target_pos  deg × 10  (int16 signed)

  DASHBOARD READ (STM32 → PC, MODE_MANUAL เท่านั้น):
    0x3F ref_qd×100   0x40 qd_out×100  0x41 V_in×100
    0x42 q_out×100    0x43 est_I×1000  0x44 ref_q×100

  STATUS:
    0x45  ISR_counter
================================================================================
"""

import time, threading, datetime
from collections import deque

import minimalmodbus, serial, serial.tools.list_ports
import dash
from dash import dcc, html, Input, Output, State
import dash_bootstrap_components as dbc
import plotly.graph_objs as go

# ─────────────────────────────────────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────────────────────────────────────
SLAVE_ID  = 21
BAUD      = 230400   # ตรงกับ base system + firmware (usart.c) baud ใหม่
TIMEOUT_S = 0.3
POLL_HZ   = 15
MAX_PTS   = 600

# ── Shared / Status ────────────────────────────────────────────────────────
REG_ESTOP        = 0x31
REG_SYS_MODE     = 0x32

# ── Dashboard WRITE (PC → STM32, MODE_MANUAL only) ──────────────────────────
REG_VEL_KP      = 0x33
REG_VEL_KI      = 0x34
REG_VEL_KD      = 0x35
REG_SPEED        = 0x36
REG_PERIOD       = 0x37
REG_WAVE         = 0x38
REG_RUN          = 0x39
REG_POS_KP       = 0x3A
REG_POS_KI       = 0x3B
REG_POS_KD       = 0x3C
REG_DRIVE_MODE   = 0x3D
REG_POS_TARGET   = 0x3E

# ── Dashboard READ (STM32 → PC, MODE_MANUAL only) ───────────────────────────
REG_TELEMETRY    = 0x3F     # first of 6 telemetry registers
REG_N_TEL        = 6        # 0x3F … 0x44

# ── Status ──────────────────────────────────────────────────────────────────
REG_ISR_CNT      = 0x45

WAVE_CODES = {"square": 0, "sine": 1, "step": 2}
RAD2DEG    = 180.0 / 3.14159265358979

# Register name + decoder สำหรับ Modbus test panel
# (ครอบคลุมทุก register ในระบบ — Base System + Dashboard)
REG_META = {
    # ── Base System READ (อ้างอิง — main.exe เขียน) ──────────────────────────
    0x00: ("heartbeat",    lambda v: "YA✓" if v == 22881 else f"0x{v:04X}"),
    0x26: ("reed_sensors", lambda v: f"0x{v:04X}"),
    0x27: ("current_task", lambda v: f"0x{v:04X}"),
    0x28: ("BS_pos÷10",   lambda v: f"{to_s16(v)/10:.1f} °"),
    0x29: ("BS_vel÷10",   lambda v: f"{to_s16(v)/10:.1f}"),
    0x30: ("BS_acc÷10",   lambda v: f"{to_s16(v)/10:.1f}"),
    # ── Shared ───────────────────────────────────────────────────────────────
    0x31: ("estop_flag",  lambda v: "⚠ ESTOP" if v else "OK"),
    0x32: ("sys_mode",    lambda v: ["Homing","Auto","Manual"][v] if v < 3 else f"?{v}"),
    # ── Dashboard WRITE ───────────────────────────────────────────────────────
    0x33: ("vel_Kp",      lambda v: f"{to_s16(v)/100:.2f}"),
    0x34: ("vel_Ki",      lambda v: f"{to_s16(v)/100:.2f}"),
    0x35: ("vel_Kd",      lambda v: f"{to_s16(v)/100:.2f}"),
    0x36: ("speed×10",    lambda v: f"{v/10:.1f} rad/s"),
    0x37: ("half_T_ms",   lambda v: f"{v} ms"),
    0x38: ("waveform",    lambda v: ["square","sine","step"][v] if v < 3 else str(v)),
    0x39: ("run",         lambda v: "RUN ▶" if v else "STOP ⏹"),
    0x3A: ("pos_Kp",      lambda v: f"{to_s16(v)/100:.2f}"),
    0x3B: ("pos_Ki",      lambda v: f"{to_s16(v)/100:.2f}"),
    0x3C: ("pos_Kd",      lambda v: f"{to_s16(v)/100:.2f}"),
    0x3D: ("drive_mode",  lambda v: "DIRECT pos→V" if v else "CASCADE pos→vel→V"),
    0x3E: ("target_pos",  lambda v: f"{to_s16(v)/10:.1f} °"),
    # ── Dashboard READ ────────────────────────────────────────────────────────
    0x3F: ("ref_qd",      lambda v: f"{to_s16(v)/100:.2f} rad/s"),
    0x40: ("qd_out",      lambda v: f"{to_s16(v)/100:.2f} rad/s"),
    0x41: ("V_in",        lambda v: f"{to_s16(v)/100:.2f} V"),
    0x42: ("q_out",       lambda v: f"{to_s16(v)/100:.2f} rad  ({to_s16(v)/100*RAD2DEG:.1f}°)"),
    0x43: ("est_I",       lambda v: f"{to_s16(v)/1000:.3f} A"),
    0x44: ("ref_q",       lambda v: f"{to_s16(v)/100:.2f} rad  ({to_s16(v)/100*RAD2DEG:.1f}°)"),
    # ── Status ───────────────────────────────────────────────────────────────
    0x45: ("ISR_cnt",     lambda v: str(v)),
}

# ─────────────────────────────────────────────────────────────────────────────
#  SHARED STATE
# ─────────────────────────────────────────────────────────────────────────────
data = {
    "t":     deque(maxlen=MAX_PTS),
    "ref":   deque(maxlen=MAX_PTS),
    "act":   deque(maxlen=MAX_PTS),
    "vin":   deque(maxlen=MAX_PTS),
    "pos":   deque(maxlen=MAX_PTS),
    "cur":   deque(maxlen=MAX_PTS),
    "ref_q": deque(maxlen=MAX_PTS),
}
data_lock       = threading.Lock()
instrument_lock = threading.Lock()

instrument   = None
connected    = False
conn_port    = ""
last_err     = ""
is_running   = False
_reconnecting        = False
_last_reconnect_time = 0.0

last_vals = {
    "ref": 0.0, "act": 0.0, "vin": 0.0, "pos": 0.0, "cur": 0.0, "ref_q": 0.0,
    "kp":  2.5, "ki":  0.5, "kd":  0.0,
    "pos_kp": 10.0, "pos_ki": 0.0, "pos_kd": 0.0,
    "drive_direct": False,
}
t0 = time.time()

# ─────────────────────────────────────────────────────────────────────────────
#  STEP RESPONSE ANALYSIS
# ─────────────────────────────────────────────────────────────────────────────
SETTLE_BAND_DEG   = 2.0
SETTLE_DURATION_S = 0.5

step_info = {
    "active": False, "t_start": None,
    "target_deg": 0.0, "initial_deg": 0.0,
}
step_results = {
    "overshoot_pct": None, "peak_deg": None,
    "ss_error_deg":  None, "settle_time_s": None, "rise_time_s": None,
}
step_lock = threading.Lock()

# ─────────────────────────────────────────────────────────────────────────────
#  KU SEARCH  (Ziegler-Nichols Ultimate Gain Finder)
# ─────────────────────────────────────────────────────────────────────────────
ku_search = {
    "active":      False,
    "kp_cur":      10.0,
    "kp_step":     0.5,
    "kp_start":    10.0,
    "interval_s":  4.0,
    "t_step":      0.0,   # เวลาที่ step Kp ครั้งล่าสุด
    "ku":          None,
    "tu":          None,
    "oscillating": False,
    "osc_amp":     0.0,
}
ku_lock = threading.Lock()


def _detect_oscillation(pos_deg_list, t_list, window_s=8.0, amp_thresh=5.0):
    """ตรวจ sustained oscillation — คืน (osc: bool, period_s or None, amplitude)"""
    if len(t_list) < 30:
        return False, None, 0.0
    t_now = t_list[-1]
    pairs = [(t, p) for t, p in zip(t_list, pos_deg_list)
             if t >= t_now - window_s]
    if len(pairs) < 20:
        return False, None, 0.0
    ts = [x[0] for x in pairs]
    ps = [x[1] for x in pairs]

    # หา peaks / valleys ด้วย window 3 samples ทั้งสองข้าง
    w = 3
    peaks, valleys = [], []
    for i in range(w, len(ps) - w):
        if all(ps[i] >= ps[j] for j in range(i - w, i + w + 1) if j != i):
            peaks.append((ts[i], ps[i]))
        if all(ps[i] <= ps[j] for j in range(i - w, i + w + 1) if j != i):
            valleys.append((ts[i], ps[i]))

    if len(peaks) < 2 or len(valleys) < 2:
        return False, None, 0.0

    amp = max(p[1] for p in peaks) - min(v[1] for v in valleys)
    if amp < amp_thresh:
        return False, None, round(amp, 1)

    # Period จาก peak-to-peak
    periods = [peaks[i+1][0] - peaks[i][0] for i in range(len(peaks) - 1)]
    if not periods:
        return False, None, round(amp, 1)
    avg_p = sum(periods) / len(periods)
    if avg_p < 0.05 or avg_p > 10.0:
        return False, None, round(amp, 1)
    max_var = max(abs(p - avg_p) / avg_p for p in periods)
    if max_var > 0.35:   # ยอมให้ period ต่างกัน ≤35%
        return False, None, round(amp, 1)
    return True, round(avg_p, 3), round(amp, 1)


def _ku_tick(pos_deg_list, t_list):
    """เรียกใน update() ทุก tick ตอน ku_search active"""
    with ku_lock:
        if not ku_search["active"] or not t_list:
            return
        t_now = t_list[-1]

        osc, period, amp = _detect_oscillation(pos_deg_list, t_list, amp_thresh=5.0)
        ku_search["osc_amp"] = amp

        if osc and period is not None:
            # พบ oscillation → บันทึก Ku, Tu แล้วหยุด
            ku_search["oscillating"] = True
            ku_search["ku"]    = round(ku_search["kp_cur"], 2)
            ku_search["tu"]    = period
            ku_search["active"] = False
            try:
                write_run_flag(False)
            except Exception:
                pass
            add_log("INFO", "KU",
                    f"✅ Found! Ku={ku_search['ku']:.2f}  Tu={ku_search['tu']:.3f}s  "
                    f"amp={amp:.1f}°")
            return

        # Step Kp ตาม interval
        if (t_now - ku_search["t_step"]) >= ku_search["interval_s"]:
            ku_search["kp_cur"] = round(ku_search["kp_cur"] + ku_search["kp_step"], 2)
            ku_search["t_step"] = t_now
            try:
                write_pos_pid(ku_search["kp_cur"], 0, 0)
            except Exception as e:
                add_log("ERROR", "KU", str(e))
            add_log("INFO", "KU",
                    f"Kp → {ku_search['kp_cur']:.2f}  amp={amp:.1f}°")


def analyze_step(t_list, pos_deg_list, t_start, target_deg, initial_deg):
    step_data = [(t, p) for t, p in zip(t_list, pos_deg_list) if t >= t_start]
    if len(step_data) < 10:
        return
    ts = [d[0] - t_start for d in step_data]
    ps = [d[1]           for d in step_data]
    delta = target_deg - initial_deg
    if abs(delta) < 0.5:
        return

    if delta > 0:
        peak  = max(ps); over = max(0.0, peak - target_deg)
    else:
        peak  = min(ps); over = max(0.0, target_deg - peak)
    overshoot_pct = over / abs(delta) * 100.0

    lo = initial_deg + 0.10 * delta
    hi = initial_deg + 0.90 * delta
    t_lo = next((t for t, p in zip(ts, ps)
                 if (delta > 0 and p >= lo) or (delta < 0 and p <= lo)), None)
    t_hi = next((t for t, p in zip(ts, ps)
                 if (delta > 0 and p >= hi) or (delta < 0 and p <= hi)), None)
    rise_time = (t_hi - t_lo) if (t_lo is not None and t_hi is not None) else None

    tail     = [p for t, p in zip(ts, ps) if t >= ts[-1] - 1.0]
    ss_error = abs(target_deg - (sum(tail) / len(tail) if tail else ps[-1]))

    settle_time = None
    for i in range(len(ts)):
        if abs(ps[i] - target_deg) <= SETTLE_BAND_DEG:
            t_end = ts[i] + SETTLE_DURATION_S
            if t_end > ts[-1]:
                break
            if all(abs(ps[j] - target_deg) <= SETTLE_BAND_DEG
                   for j in range(i, len(ts)) if ts[j] <= t_end):
                settle_time = ts[i]; break

    with step_lock:
        step_results.update(
            overshoot_pct=overshoot_pct, peak_deg=peak,
            ss_error_deg=ss_error, settle_time_s=settle_time,
            rise_time_s=rise_time,
        )

# ─────────────────────────────────────────────────────────────────────────────
#  LOG
# ─────────────────────────────────────────────────────────────────────────────
LOG_MAX = 200
log_entries  = deque(maxlen=LOG_MAX)
log_lock     = threading.Lock()
_poll_err_count = 0
_poll_ok_count  = 0

LOG_STYLE = {
    "INFO":    ("INFO ",  "#51cf66"),
    "WARN":    ("WARN ",  "#ffd43b"),
    "ERROR":   ("ERROR",  "#ff6b6b"),
    "TIMEOUT": ("T/O  ",  "#ff9f43"),
    "CRC":     ("CRC  ",  "#f783ac"),
    "SERIAL":  ("SERIAL", "#cc5de8"),
    "WRITE":   ("WRITE",  "#74c0fc"),
}

def add_log(level, source, msg):
    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    with log_lock:
        log_entries.append({"ts": ts, "level": level, "src": source, "msg": msg})
    print(f"[{ts}][{level:6s}][{source}] {msg}")

def classify_error(e):
    s = str(e).lower(); t = type(e).__name__
    if "timeout" in s or "noresponse" in t.lower(): return "TIMEOUT"
    if "checksum" in s or "crc" in s:               return "CRC"
    if "serial"   in t.lower() or "port" in s:      return "SERIAL"
    return "ERROR"

# ─────────────────────────────────────────────────────────────────────────────
#  MODBUS
# ─────────────────────────────────────────────────────────────────────────────
def to_s16(v): return v - 65536 if v > 32767 else v
def list_ports(): return [p.device for p in serial.tools.list_ports.comports()]

def connect_modbus(port):
    global instrument, connected, conn_port, last_err
    try:
        inst = minimalmodbus.Instrument(port, SLAVE_ID)
        inst.serial.baudrate = BAUD
        inst.serial.parity   = serial.PARITY_EVEN
        inst.serial.bytesize = 8; inst.serial.stopbits = 1
        inst.serial.timeout  = TIMEOUT_S; inst.serial.write_timeout = 1.0
        inst.mode = minimalmodbus.MODE_RTU
        inst.clear_buffers_before_each_transaction = True
        inst.close_port_after_each_call = False
        instrument = inst; connected = True; conn_port = port; last_err = ""
        add_log("INFO", "CONNECT", f"OK  {port}  {BAUD} 8E1  slave={SLAVE_ID}")
    except Exception as e:
        connected = False; last_err = str(e)
        add_log(classify_error(e), "CONNECT", str(e))

def reconnect_modbus():
    global instrument, connected, _reconnecting, _last_reconnect_time
    if _reconnecting: return
    _reconnecting = True; connected = False
    _last_reconnect_time = time.time()
    add_log("WARN", "CONNECT", f"Reconnecting {conn_port} …")
    with instrument_lock:
        try:
            if instrument and instrument.serial.is_open:
                instrument.serial.close()
        except Exception: pass
    time.sleep(2.0); connect_modbus(conn_port); _reconnecting = False

def _wr(reg, vals):
    """Modbus RTU standard: list → FC16 (write_registers), int → FC06 (write_register)"""
    with instrument_lock:
        if isinstance(vals, list):
            instrument.write_registers(reg, vals)
        else:
            instrument.write_register(reg, int(vals), functioncode=6)

def write_vel_pid(kp, ki, kd):
    if not connected: add_log("WARN","VEL_PID","not connected"); return "not connected"
    try:
        # ใช้ round() แทน int() ป้องกัน float truncation (เช่น 0.3*100=29.999→30)
        # ไม่ clamp min=1 อีกต่อไป → Ki/Kd=0 ส่งได้จริง (firmware ไม่มี != 0 guard แล้ว)
        r = [int(round(kp*100)), int(round(ki*100)), int(round(kd*100))]
        _wr(REG_VEL_KP, r)
        last_vals.update(kp=kp, ki=ki, kd=kd)
        add_log("WRITE","VEL_PID",f"Kp={kp:.2f} Ki={ki:.2f} Kd={kd:.2f}"); return "ok"
    except Exception as e:
        add_log(classify_error(e),"VEL_PID",str(e)); return str(e)

def write_pos_pid(kp, ki, kd):
    if not connected: add_log("WARN","POS_PID","not connected"); return "not connected"
    try:
        # ใช้ round() ป้องกัน float truncation
        r = [int(round(kp*100)), int(round(ki*100)), int(round(kd*100))]
        _wr(REG_POS_KP, r)
        last_vals.update(pos_kp=kp, pos_ki=ki, pos_kd=kd)
        s = "ACTIVE" if int(kp*100) != 0 else "DISABLED"
        add_log("WRITE","POS_PID",f"Kp={kp:.2f} Ki={ki:.2f} Kd={kd:.2f} [{s}]"); return "ok"
    except Exception as e:
        add_log(classify_error(e),"POS_PID",str(e)); return str(e)

def write_waveform(speed, half_ms, wave):
    if not connected: add_log("WARN","WAVE","not connected"); return "not connected"
    try:
        _wr(REG_SPEED, [max(1,int(speed*10)), max(100,int(half_ms)),
                         WAVE_CODES.get(wave, 0)])
        add_log("WRITE","WAVE",f"{wave} {speed} half={half_ms}ms"); return "ok"
    except Exception as e:
        add_log(classify_error(e),"WAVE",str(e)); return str(e)

def write_run_flag(run):
    if not connected: add_log("WARN","RUN","not connected"); return "not connected"
    try:
        _wr(REG_RUN, 1 if run else 0)
        add_log("WRITE","RUN","START" if run else "STOP"); return "ok"
    except Exception as e:
        add_log(classify_error(e),"RUN",str(e)); return str(e)

def write_drive_mode(direct):
    if not connected: add_log("WARN","MODE","not connected"); return "not connected"
    try:
        _wr(REG_DRIVE_MODE, 1 if direct else 0)
        last_vals["drive_direct"] = bool(direct)
        add_log("WRITE","MODE","DIRECT(pos→V)" if direct else "CASCADE(pos→vel→V)"); return "ok"
    except Exception as e:
        add_log(classify_error(e),"MODE",str(e)); return str(e)

_last_tgt_raw = None   # cache ค่าล่าสุดที่เขียนสำเร็จ — ป้องกัน repeated FC06 → echo filter discard

def write_pos_target(deg, force=False):
    """เขียน REG_POS_TARGET — skip ถ้าค่าเหมือนครั้งก่อน (ลด timeout จาก echo filter)
    force=True บังคับเขียนแม้ค่าซ้ำ (ใช้ตอน init หรือ reset)
    """
    global _last_tgt_raw
    if not connected: add_log("WARN","TGT","not connected"); return "not connected"
    raw = max(-3600, min(3600, int(round(deg * 10)))) & 0xFFFF
    if not force and raw == _last_tgt_raw:
        return "ok"   # ค่าเดิม — ไม่ส่งซ้ำ
    try:
        _wr(REG_POS_TARGET, raw)
        _last_tgt_raw = raw
        add_log("WRITE","TGT",f"{deg:+.1f}°"); return "ok"
    except Exception as e:
        _last_tgt_raw = None   # force retry รอบถัดไป
        add_log(classify_error(e),"TGT",str(e)); return str(e)

# ─────────────────────────────────────────────────────────────────────────────
#  READ THREAD
# ─────────────────────────────────────────────────────────────────────────────
RECONNECT_CD = 15.0

def read_loop():
    global last_err, _poll_err_count, _poll_ok_count, connected, _reconnecting
    cerr = 0
    while True:
        if connected and instrument:
            try:
                # ── อ่าน telemetry block (0x3F–0x44) + sys_mode (0x32) ──
                with instrument_lock:
                    regs = instrument.read_registers(REG_TELEMETRY, REG_N_TEL, functioncode=3)
                ref  = to_s16(regs[0]) / 100.0
                act  = to_s16(regs[1]) / 100.0
                vin  = to_s16(regs[2]) / 100.0
                pos  = to_s16(regs[3]) / 100.0
                cur  = to_s16(regs[4]) / 1000.0
                refq = to_s16(regs[5]) / 100.0

                # อ่าน sys_mode แยก (ไม่อยู่ใน block เดิม)
                try:
                    with instrument_lock:
                        sm = instrument.read_registers(REG_SYS_MODE, 1, functioncode=3)
                    sys_mode = sm[0]
                except Exception:
                    sys_mode = last_vals.get("sys_mode", 2)

                # MODE_AUTO (1): override pos ด้วย REG_BS_POS (อัปเดตโดย firmware)
                if sys_mode == 1:
                    try:
                        with instrument_lock:
                            bp = instrument.read_registers(REG_BS_POS, 1, functioncode=3)
                        pos = to_s16(bp[0]) / 10.0 / RAD2DEG   # deg×10 → rad
                    except Exception:
                        pass

                tn = time.time() - t0
                with data_lock:
                    data["t"].append(tn);   data["ref"].append(ref)
                    data["act"].append(act); data["vin"].append(vin)
                    data["pos"].append(pos); data["cur"].append(cur)
                    data["ref_q"].append(refq)
                last_vals.update(ref=ref, act=act, vin=vin, pos=pos, cur=cur,
                                 ref_q=refq, sys_mode=sys_mode)
                last_err = ""
                if cerr >= 3: add_log("INFO","READ",f"Recovered after {cerr} errors")
                cerr = 0; _poll_ok_count += 1
            except Exception as e:
                cerr += 1; _poll_err_count += 1
                level = classify_error(e); last_err = f"[#{cerr}] {e}"
                if cerr == 1 or cerr % 5 == 0:
                    add_log(level,"READ",f"#{cerr} {type(e).__name__}: {e}")
                cd_ok = (time.time() - _last_reconnect_time) > RECONNECT_CD
                if ((level == "SERIAL" or cerr >= 20) and cd_ok and not _reconnecting):
                    connected = False; cerr = 0
                    threading.Thread(target=reconnect_modbus, daemon=True).start()
                try:
                    with instrument_lock: instrument.serial.reset_input_buffer()
                except Exception: pass
        time.sleep(1.0 / POLL_HZ)

# ─────────────────────────────────────────────────────────────────────────────
#  PICK & PLACE  STATE MACHINE  (MANUAL mode — Python drives firmware via TGT)
# ─────────────────────────────────────────────────────────────────────────────
# Base System registers (ใช้สื่อสาร config sequence ไปยัง firmware)
REG_BS_MODE       = 0x01
BS_MODE_AUTO      = 4
BS_MODE_HOME      = 1
REG_BS_PP_SEQ     = 0x12
REG_BS_PAIR_COUNT = 0x22
REG_BS_SOFT_STOP  = 0x25
REG_BS_TASK       = 0x27
REG_BS_POS        = 0x28
HOLE_STEP_DEG     = 5.0        # ต้องตรงกับ firmware

# State constants
PP_IDLE        = 0
PP_MOVE_PICK   = 1
PP_DWELL_PICK  = 2
PP_MOVE_PLACE  = 3
PP_DWELL_PLACE = 4
PP_DONE        = 5

_PP_LABELS = {
    PP_IDLE:        ("IDLE",          "#888888"),
    PP_MOVE_PICK:   ("→ PICK",        "#ffd43b"),
    PP_DWELL_PICK:  ("⏱ DWELL PICK",  "#ff9f43"),
    PP_MOVE_PLACE:  ("→ PLACE",       "#74c0fc"),
    PP_DWELL_PLACE: ("⏱ DWELL PLACE", "#a9e34b"),
    PP_DONE:        ("✅ DONE",        "#51cf66"),
}

# Globals
pp_state    = PP_IDLE
pp_rod_idx  = 0
pp_running  = False
pp_t_enter  = 0.0
pp_lock     = threading.Lock()
_pp_arrive_ticks = 0          # ต้องอยู่ใน zone ติดต่อกัน PP_ARRIVE_NEED ticks
PP_ARRIVE_NEED   = 5          # 5 × 100ms = 500ms settle ก่อน advance
pp_cfg = {
    "n_rods": 1,
    "picks":  [0.0],
    "places": [45.0],
    "dwell":  2.0,
    "thresh": 3.0,
    "repeat": False,
    "_run_confirmed": False,
}

# Per-place step response analysis
pp_place_results  = {}
pp_place_lock     = threading.Lock()
_pp_move_t0       = 0.0
_pp_move_init_deg = 0.0


def _deg_to_index(deg):
    return int(round(float(deg) / HOLE_STEP_DEG)) & 0xFFFF


def write_pp_config(picks, places, n_rods):
    if not connected: return "not connected"
    try:
        n   = max(1, min(5, int(n_rods)))
        seq = [0] * 10
        for i in range(n):
            seq[i * 2]     = _deg_to_index(picks[i]  if i < len(picks)  else 0.0)
            seq[i * 2 + 1] = _deg_to_index(places[i] if i < len(places) else 0.0)
        with instrument_lock:
            instrument.write_registers(REG_BS_PP_SEQ, seq)
        _wr(REG_BS_PAIR_COUNT, n)
        add_log("WRITE","BS_SEQ",
                f"n={n} picks={[round(float(p),1) for p in picks[:n]]} "
                f"places={[round(float(p),1) for p in places[:n]]}")
        return "ok"
    except Exception as e:
        add_log(classify_error(e),"BS_SEQ",str(e)); return str(e)


def _pp_set_state(new_state, msg=""):
    global pp_state, pp_t_enter
    pp_state  = new_state
    pp_t_enter = time.time() - t0
    if msg: add_log("INFO","P&P",msg)


def _pp_analyze_place(rod_idx, t_list, pos_deg_list, t_start, target_deg, initial_deg):
    """วิเคราะห์ step response ของการวาง rod — เรียกใน background thread
    อ้างอิง: Tao & Kokotovic, Automatica 1993 / analog of standard step metrics
    """
    step_data = [(t, p) for t, p in zip(t_list, pos_deg_list) if t >= t_start]
    if len(step_data) < 3:
        return
    ts    = [d[0] - t_start for d in step_data]
    ps    = [d[1]           for d in step_data]
    delta = target_deg - initial_deg
    if abs(delta) < 0.5:
        return

    # Overshoot
    if delta > 0:
        peak = max(ps); over = max(0.0, peak - target_deg)
    else:
        peak = min(ps); over = max(0.0, target_deg - peak)
    overshoot_pct = over / abs(delta) * 100.0

    # Rise time 10–90 %
    lo = initial_deg + 0.10 * delta
    hi = initial_deg + 0.90 * delta
    t_lo = next((t for t, p in zip(ts, ps)
                 if (delta > 0 and p >= lo) or (delta < 0 and p <= lo)), None)
    t_hi = next((t for t, p in zip(ts, ps)
                 if (delta > 0 and p >= hi) or (delta < 0 and p <= hi)), None)
    rise_time = (t_hi - t_lo) if (t_lo is not None and t_hi is not None) else None

    # SS error — เฉลี่ยจุดสุดท้าย (หรือทั้งหมดถ้า < 1s)
    tail     = [p for t, p in zip(ts, ps) if t >= ts[-1] - 0.5] or ps
    ss_error = abs(target_deg - sum(tail) / len(tail))

    # Settling time (band = SETTLE_BAND_DEG, duration = SETTLE_DURATION_S)
    settle_time = None
    for i in range(len(ts)):
        if abs(ps[i] - target_deg) <= SETTLE_BAND_DEG:
            t_end = ts[i] + SETTLE_DURATION_S
            if t_end > ts[-1]:
                break
            if all(abs(ps[j] - target_deg) <= SETTLE_BAND_DEG
                   for j in range(i, len(ts)) if ts[j] <= t_end):
                settle_time = ts[i]; break

    with pp_place_lock:
        pp_place_results[rod_idx] = {
            "overshoot_pct": round(overshoot_pct, 1),
            "ss_error_deg":  round(ss_error,       2),
            "settle_time_s": round(settle_time, 3) if settle_time is not None else None,
            "rise_time_s":   round(rise_time,   3) if rise_time   is not None else None,
            "target_deg":    target_deg,
        }


def _pp_in_zone(pdeg, target_deg, thresh):
    """True ถ้า position อยู่ใน ±thresh ของ target"""
    return abs(pdeg - target_deg) <= thresh


def pp_tick():
    """เรียกทุก tick (Dash interval) ตอน P&P running — MANUAL mode เท่านั้น"""
    global pp_state, pp_rod_idx, pp_running, _pp_move_t0, _pp_move_init_deg
    global _pp_arrive_ticks
    with pp_lock:
        if not pp_running:
            return
        cfg  = pp_cfg
        now  = time.time() - t0
        pdeg = last_vals.get("pos", 0.0) * RAD2DEG

        pick  = cfg["picks"][pp_rod_idx]  if pp_rod_idx < len(cfg["picks"])  else 0.0
        place = cfg["places"][pp_rod_idx] if pp_rod_idx < len(cfg["places"]) else 0.0
        n     = cfg["n_rods"]
        thr   = cfg["thresh"]

        if pp_state in (PP_MOVE_PICK, PP_MOVE_PLACE):
            if not cfg.get("_run_confirmed", False):
                if write_run_flag(True) == "ok":
                    pp_cfg["_run_confirmed"] = True

        if pp_state == PP_MOVE_PICK:
            write_pos_target(pick)
            if _pp_in_zone(pdeg, pick, thr):
                _pp_arrive_ticks += 1
                if _pp_arrive_ticks >= PP_ARRIVE_NEED:
                    _pp_arrive_ticks = 0
                    _pp_set_state(PP_DWELL_PICK,
                        f"Rod {pp_rod_idx+1}/{n}: arrived pick {pick:+.1f}° "
                        f"(settled {PP_ARRIVE_NEED} ticks)")
            else:
                _pp_arrive_ticks = 0   # ออกนอก zone → reset

        elif pp_state == PP_DWELL_PICK:
            _pp_arrive_ticks = 0
            if (now - pp_t_enter) >= cfg["dwell"]:
                if write_pos_target(place) == "ok":
                    _pp_move_t0       = now
                    _pp_move_init_deg = pdeg
                    _pp_set_state(PP_MOVE_PLACE,
                        f"Rod {pp_rod_idx+1}/{n}: → place {place:+.1f}°")

        elif pp_state == PP_MOVE_PLACE:
            write_pos_target(place)
            if _pp_in_zone(pdeg, place, thr):
                _pp_arrive_ticks += 1
                if _pp_arrive_ticks >= PP_ARRIVE_NEED:
                    _pp_arrive_ticks = 0
                    rod_i = pp_rod_idx
                    t_s, init, tgt = _pp_move_t0, _pp_move_init_deg, place
                    with data_lock:
                        t_snap = list(data["t"])
                        p_snap = [v * RAD2DEG for v in data["pos"]]
                    threading.Thread(
                        target=_pp_analyze_place,
                        args=(rod_i, t_snap, p_snap, t_s, tgt, init),
                        daemon=True,
                    ).start()
                    _pp_set_state(PP_DWELL_PLACE,
                        f"Rod {pp_rod_idx+1}/{n}: arrived place {place:+.1f}°")
            else:
                _pp_arrive_ticks = 0

        elif pp_state == PP_DWELL_PLACE:
            _pp_arrive_ticks = 0
            if (now - pp_t_enter) >= cfg["dwell"]:
                pp_rod_idx += 1
                if pp_rod_idx >= n:
                    if cfg["repeat"]:
                        pp_rod_idx = 0
                        next_pick  = cfg["picks"][0]
                        if write_pos_target(next_pick) == "ok":
                            _pp_arrive_ticks = 0
                            _pp_set_state(PP_MOVE_PICK, "🔁 Repeat cycle started")
                    else:
                        pp_running = False
                        write_run_flag(False)
                        _pp_set_state(PP_DONE, f"✅ Mission complete! {n} rods placed")
                else:
                    next_pick = cfg["picks"][pp_rod_idx]
                    if write_pos_target(next_pick) == "ok":
                        _pp_arrive_ticks = 0
                        _pp_set_state(PP_MOVE_PICK,
                            f"Rod {pp_rod_idx+1}/{n}: → pick {next_pick:+.1f}°")


# ─────────────────────────────────────────────────────────────────────────────
#  DASH APP
# ─────────────────────────────────────────────────────────────────────────────
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.CYBORG],
                title="PID Tuner — 1-DOF Robot")

C_BG   = "#0f0f1a"; C_CARD = "#1a1a2e"; C_BDR = "#2a2a4e"
C_REF  = "#ff6b6b"; C_ACT  = "#51cf66"; C_VIN = "#ffd43b"
C_POS  = "#74c0fc"; C_CUR  = "#a9e34b"; C_TXT = "#e0e0e0"
C_MUT  = "#888888"; C_STOP = "#e03131"; C_RUN  = "#2f9e44"

CARD = {"backgroundColor": C_CARD, "border": f"1px solid {C_BDR}",
        "borderRadius": "8px", "padding": "14px", "marginBottom": "10px"}
LBL  = {"color": C_MUT, "fontSize": "11px", "fontWeight": "600",
        "letterSpacing": "0.05em", "marginBottom": "4px"}
INP  = {"backgroundColor": "#0f0f1a", "color": C_TXT,
        "border": f"1px solid {C_BDR}", "borderRadius": "4px"}
HR   = {"borderColor": C_BDR, "margin": "8px 0"}

GLAY = dict(
    paper_bgcolor=C_CARD, plot_bgcolor="#0f0f1a",
    font=dict(color=C_TXT, size=11),
    margin=dict(l=52, r=10, t=26, b=28),
    xaxis=dict(gridcolor="#222", color=C_MUT, title="Time (s)"),
    yaxis=dict(gridcolor="#222", color=C_MUT),
    legend=dict(orientation="h", x=0, y=1.2,
                bgcolor="rgba(0,0,0,0)", font=dict(size=11)),
    hovermode="x unified",
)

def metric(label, mid, unit="", color=C_TXT):
    return html.Div([
        html.Div(label, style=LBL),
        html.Div([
            html.Span("—", id=mid,
                      style={"fontSize": "18px", "fontWeight": "700", "color": color}),
            html.Span(f" {unit}", style={"color": C_MUT, "fontSize": "10px"}),
        ])
    ], style={"textAlign": "center", "padding": "4px 2px"})

def pid_sliders(prefix, params):
    out = []
    for n, k, v, mx, s in params:
        out.append(html.Div([
            dbc.Row([
                dbc.Col(html.Div(n, style=LBL), width=3),
                dbc.Col(dbc.Input(id=f"in-{prefix}-{k}", type="number",
                                  value=v, step=s, min=0, max=mx,
                                  style={**INP,"height":"26px","padding":"2px 8px"}), width=9),
            ], align="center", className="mb-1"),
            dcc.Slider(id=f"sl-{prefix}-{k}", min=0, max=mx, step=s, value=v,
                       marks={0:{"label":"0","style":{"color":C_MUT}},
                              mx:{"label":str(mx),"style":{"color":C_MUT}}},
                       tooltip={"placement":"bottom"}, className="mb-2"),
        ]))
    return out

def preset_btn(label, bid, val):
    return dbc.Col(dbc.Button(label, id=bid, color="secondary", size="sm",
                              className="w-100"), width=3)

def target_controls():
    """Target input + preset buttons — ใช้ใน left panel (Main / Pos tab)"""
    return html.Div(style={**CARD,"marginBottom":"6px"}, children=[
        dbc.Row([
            dbc.Col(html.Div("TARGET POSITION", style={**LBL,"marginBottom":"0"}), width="auto"),
            dbc.Col(html.Div(id="tgt-status",
                             style={"color":C_POS,"fontSize":"11px","textAlign":"right"}),
                    style={"textAlign":"right"}),
        ], align="center", className="mb-1"),
        dbc.Row([
            dbc.Col(dbc.Input(id="in-tgt", type="number", value=0, step=5,
                              min=-360, max=360, placeholder="deg…",
                              style={**INP,"height":"34px","padding":"4px 8px",
                                     "fontSize":"15px","fontWeight":"700"}), width=7),
            dbc.Col(dbc.Button("Go ⟶", id="btn-go", color="info", size="sm",
                               className="w-100",
                               style={"height":"34px","fontWeight":"700"}), width=5),
        ], className="g-1 mb-1"),
        dbc.Row([
            dbc.Col(dbc.Button("▶ START", id="btn-run", color="success", size="sm",
                               className="w-100",
                               style={"fontWeight":"700"}), width=6),
            dbc.Col(dbc.Button("⏹ STOP", id="btn-stop", color="danger", size="sm",
                               className="w-100",
                               style={"fontWeight":"700"}), width=6),
        ], className="g-1 mb-1"),
        dbc.Row([
            preset_btn("-90°",  "p-n90",  -90),
            preset_btn("-45°",  "p-n45",  -45),
            preset_btn("0°",    "p-0",      0),
            preset_btn("+45°",  "p-p45",   45),
        ], className="g-1 mb-1"),
        dbc.Row([
            preset_btn("+90°",  "p-p90",   90),
            preset_btn("+135°", "p-p135", 135),
            preset_btn("+180°", "p-p180", 180),
            preset_btn("-135°", "p-n135",-135),
        ], className="g-1"),
    ])

# ── Port dropdown options ──────────────────────────────────────────────────
_ports     = list_ports()
_port_opts = [{"label": p, "value": p} for p in _ports] \
             if _ports else [{"label": "ไม่พบ COM port", "value": ""}]

# ─────────────────────────────────────────────────────────────────────────────
#  LAYOUT
# ─────────────────────────────────────────────────────────────────────────────
app.layout = dbc.Container(fluid=True,
    style={"backgroundColor": C_BG, "minHeight": "100vh", "padding": "14px"},
    children=[

    # ── HEADER ────────────────────────────────────────────────────────────────
    dbc.Row([
        dbc.Col(html.H5("G3 — 1-DOF Robot ",
                        style={"color": C_TXT, "margin": "0", "fontSize": "16px"}),
                width="auto"),
        dbc.Col(html.Div(id="mode-badge",
                         style={"fontSize": "12px", "fontWeight": "700",
                                "color": C_POS, "paddingTop": "4px"}),
                width="auto"),
        dbc.Col(html.Div(id="hdr-status",
                         style={"color": C_MUT, "fontSize": "11px",
                                "textAlign": "right", "paddingTop": "6px"}),
                style={"textAlign": "right"}),
    ], align="center",
       style={"backgroundColor": C_CARD, "border": f"1px solid {C_BDR}",
              "borderRadius": "8px", "padding": "8px 16px", "marginBottom": "12px"}),

    dbc.Row([

        # ── LEFT PANEL ────────────────────────────────────────────────────────
        dbc.Col(width=3, children=[

            # Connection
            html.Div(style=CARD, children=[
                html.Div("MODBUS RTU", style={**LBL,"marginBottom":"8px"}),
                dcc.Dropdown(id="dd-port", options=_port_opts,
                             value=_ports[0] if _ports else "",
                             clearable=False,
                             style={"backgroundColor":"#0f0f1a","color":"#000",
                                    "marginBottom":"8px","fontSize":"12px"}),
                dbc.Row([
                    dbc.Col(dbc.Button("เชื่อมต่อ", id="btn-connect",
                                       color="success", size="sm",
                                       className="w-100"), width=7),
                    dbc.Col(dbc.Button("Refresh", id="btn-refresh",
                                       color="secondary", size="sm",
                                       className="w-100"), width=5),
                ], className="g-1"),
                html.Div(id="conn-status",
                         style={"color":C_MUT,"fontSize":"11px",
                                "marginTop":"6px","textAlign":"center","minHeight":"14px"}),
            ]),

            # Mode Tabs
            html.Div(style={**CARD, "padding":"0"}, children=[
                dbc.Tabs(id="mode-tabs", active_tab="main",
                         style={"fontSize":"12px"},
                         children=[
                    dbc.Tab(label="🏠 Main",     tab_id="main"),
                    dbc.Tab(label="🔄 Vel",      tab_id="vel"),
                    dbc.Tab(label="📍 Pos",      tab_id="pos"),
                    dbc.Tab(label="⛓ Cascade",  tab_id="cascade"),
                    dbc.Tab(label="🤖 P&P",      tab_id="pp"),
                    dbc.Tab(label="🔧 Modbus",   tab_id="modbus"),
                ]),
                html.Div(id="mode-cfg-status",
                         style={"color":C_MUT,"fontSize":"10px","textAlign":"center",
                                "padding":"4px","minHeight":"14px"}),
            ]),

            # ── TARGET PANEL (แสดงเฉพาะ Main + Pos tab) ───────────────────
            html.Div(id="target-panel", style={"display":"block"}, children=[
                target_controls(),
            ]),

            # ── TAB CONTENT (show/hide via CSS) ────────────────────────────
            # Main tab — big position display
            html.Div(id="tab-main", style={"display":"block"}, children=[
                html.Div(style={**CARD,"textAlign":"center"}, children=[
                    html.Div("CURRENT POSITION", style={**LBL,"marginBottom":"6px"}),
                    html.Div([
                        html.Span("—", id="main-pos-big",
                                  style={"fontSize":"44px","fontWeight":"900","color":C_POS}),
                        html.Span(" °", style={"color":C_MUT,"fontSize":"16px"}),
                    ], style={"marginBottom":"4px"}),
                    html.Div([
                        html.Span("Target  ", style={"color":C_MUT,"fontSize":"11px"}),
                        html.Span("—", id="main-tgt-big",
                                  style={"color":C_REF,"fontWeight":"700","fontSize":"14px"}),
                        html.Span(" °", style={"color":C_MUT,"fontSize":"11px"}),
                    ]),
                ]),
            ]),

            # Vel tune tab
            html.Div(id="tab-vel", style={"display":"none"}, children=[
                html.Div(style=CARD, children=[
                    html.Div("VELOCITY PID",
                             style={**LBL,"color":C_ACT,"marginBottom":"8px"}),
                    *pid_sliders("vel", [
                        ("Kp","kp",2.5, 30.0,0.1),
                        ("Ki","ki",0.5,  5.0,0.05),
                        ("Kd","kd",0.0,  2.0,0.01),
                    ]),
                    dbc.Button("Apply Vel PID", id="btn-vel-pid",
                               color="primary", size="sm", className="w-100 mt-1"),
                    html.Div(id="vel-pid-status",
                             style={"color":C_ACT,"fontSize":"11px",
                                    "marginTop":"4px","textAlign":"center","minHeight":"14px"}),
                ]),
                html.Div(style=CARD, children=[
                    html.Div("WAVEFORM", style={**LBL,"marginBottom":"8px"}),
                    dbc.RadioItems(id="radio-wave",
                        options=[{"label":"⬜ Square","value":"square"},
                                 {"label":"〜 Sine","value":"sine"},
                                 {"label":"⟶ Step","value":"step"}],
                        value="square", inline=True,
                        style={"fontSize":"12px","color":C_TXT,"marginBottom":"8px"},
                        labelStyle={"marginRight":"10px"}),
                    dbc.Row([
                        dbc.Col(html.Div("Speed (rad/s)", style=LBL), width=6),
                        dbc.Col(html.Div("Half-T (ms)",   style=LBL), width=6),
                    ]),
                    dbc.Row([
                        dbc.Col(dbc.Input(id="in-speed", type="number",
                                          value=3.0, step=0.5, min=0.5, max=20,
                                          style={**INP,"height":"28px","padding":"2px 8px"}),
                                width=6),
                        dbc.Col(dbc.Input(id="in-period", type="number",
                                          value=3000, step=500, min=100, max=10000,
                                          style={**INP,"height":"28px","padding":"2px 8px"}),
                                width=6),
                    ], className="g-1 mb-2"),
                    dbc.Button("Apply Waveform", id="btn-wave",
                               color="warning", size="sm", className="w-100"),
                    html.Div(id="wave-status",
                             style={"color":C_VIN,"fontSize":"11px",
                                    "marginTop":"4px","textAlign":"center","minHeight":"14px"}),
                ]),
            ]),

            # Pos tune tab
            html.Div(id="tab-pos", style={"display":"none"}, children=[
                html.Div(style=CARD, children=[
                    html.Div("POSITION PID  (REG_POS_KP=0 → disable)",
                             style={**LBL,"color":C_POS,"marginBottom":"8px"}),
                    *pid_sliders("pos", [
                        ("Kp","kp",10.0,80.0,0.5),
                        ("Ki","ki", 0.0, 5.0,0.05),
                        ("Kd","kd", 0.0,15.0,0.1),
                    ]),
                    dbc.Button("Apply Pos PID", id="btn-pos-pid",
                               color="info", size="sm", className="w-100 mt-1"),
                    html.Div(id="pos-pid-status",
                             style={"color":C_POS,"fontSize":"11px",
                                    "marginTop":"4px","textAlign":"center","minHeight":"14px"}),
                ]),
                html.Div(style=CARD, children=[
                    html.Div("DRIVE MODE (0x3D)", style={**LBL,"marginBottom":"6px"}),
                    dbc.RadioItems(id="radio-drive-mode",
                        options=[
                            {"label":"⛓ Cascade  pos→vel→V","value":"cascade"},
                            {"label":"⚡ Direct   pos→V only","value":"direct"},
                        ],
                        value="direct", inline=False,
                        style={"fontSize":"12px","color":C_TXT},
                        labelStyle={"marginBottom":"4px","display":"block"}),
                    html.Div(id="drive-mode-status",
                             style={"color":C_MUT,"fontSize":"11px",
                                    "marginTop":"4px","textAlign":"center","minHeight":"14px"}),
                    dcc.Store(id="store-drive-mode", data="direct"),
                ]),

                # ── Ku Search card ───────────────────────────────────────
                html.Div(style={**CARD,"borderColor":"#6b3fa0","borderWidth":"2px"}, children=[
                    html.Div("ZIEGLER-NICHOLS  Ku SEARCH",
                             style={**LBL,"color":"#cc5de8","marginBottom":"6px"}),
                    html.Div("⚠ Ki=0, Kd=0 จะถูก force อัตโนมัติ — ระวัง Kp สูงเกิน",
                             style={"color":"#ffd43b","fontSize":"10px","marginBottom":"8px"}),
                    dbc.Row([
                        dbc.Col([html.Div("Start Kp",    style=LBL),
                                 dbc.Input(id="ku-kp-start", type="number",
                                           value=10.0, step=0.5, min=1.0, max=80.0,
                                           style={**INP,"height":"28px","padding":"2px 8px"})],
                                width=4),
                        dbc.Col([html.Div("Kp step",     style=LBL),
                                 dbc.Input(id="ku-kp-step",  type="number",
                                           value=0.5,  step=0.1, min=0.1, max=10.0,
                                           style={**INP,"height":"28px","padding":"2px 8px"})],
                                width=4),
                        dbc.Col([html.Div("Interval (s)", style=LBL),
                                 dbc.Input(id="ku-interval",  type="number",
                                           value=4.0,  step=0.5, min=1.0, max=15.0,
                                           style={**INP,"height":"28px","padding":"2px 8px"})],
                                width=4),
                    ], className="g-1 mb-2"),
                    dbc.Row([
                        dbc.Col(dbc.Button("▶ Start", id="btn-ku-start",
                                           color="warning", size="sm",
                                           className="w-100"), width=6),
                        dbc.Col(dbc.Button("⏹ Stop",  id="btn-ku-stop",
                                           color="secondary", size="sm",
                                           className="w-100"), width=6),
                    ], className="g-1 mb-2"),
                    html.Div(id="ku-progress",
                             style={"fontFamily":"monospace","fontSize":"11px",
                                    "color":C_MUT,"minHeight":"14px","marginBottom":"4px"}),
                    html.Div(id="ku-result",
                             style={"color":"#cc5de8","fontWeight":"700","fontSize":"12px",
                                    "minHeight":"14px","marginBottom":"4px"}),
                    html.Div(id="ku-suggest",
                             style={"color":C_ACT,"fontSize":"11px","fontFamily":"monospace",
                                    "backgroundColor":"#080810","borderRadius":"4px",
                                    "padding":"4px 6px","minHeight":"28px","marginBottom":"6px"}),
                    dbc.Button("✅ Apply suggested gains", id="btn-ku-apply",
                               color="info", size="sm", className="w-100"),
                    html.Div(id="ku-apply-status",
                             style={"color":C_POS,"fontSize":"11px",
                                    "marginTop":"4px","textAlign":"center","minHeight":"14px"}),
                ]),
            ]),

            # Cascade tune tab
            html.Div(id="tab-cascade", style={"display":"none"}, children=[
                html.Div(style=CARD, children=[
                    html.Div("POSITION PID",
                             style={**LBL,"color":C_POS,"marginBottom":"8px"}),
                    *pid_sliders("pos-c", [
                        ("Kp","kp",10.0,80.0,0.5),
                        ("Ki","ki", 0.0, 5.0,0.05),
                        ("Kd","kd", 0.0,15.0,0.1),
                    ]),
                    dbc.Button("Apply Pos PID", id="btn-pos-pid-c",
                               color="info", size="sm", className="w-100 mt-1"),
                    html.Div(id="pos-pid-c-status",
                             style={"color":C_POS,"fontSize":"11px",
                                    "marginTop":"4px","textAlign":"center","minHeight":"14px"}),
                ]),
                html.Div(style=CARD, children=[
                    html.Div("VELOCITY PID",
                             style={**LBL,"color":C_ACT,"marginBottom":"8px"}),
                    *pid_sliders("vel-c", [
                        ("Kp","kp",2.5, 30.0,0.1),
                        ("Ki","ki",0.5,  5.0,0.05),
                        ("Kd","kd",0.0,  2.0,0.01),
                    ]),
                    dbc.Button("Apply Vel PID", id="btn-vel-pid-c",
                               color="primary", size="sm", className="w-100 mt-1"),
                    html.Div(id="vel-pid-c-status",
                             style={"color":C_ACT,"fontSize":"11px",
                                    "marginTop":"4px","textAlign":"center","minHeight":"14px"}),
                ]),

                # ── Active HW Gains display ───────────────────────────────
                html.Div(style={**CARD,"backgroundColor":"#080818",
                                "borderColor":"#3a3a6e"}, children=[
                    html.Div("ACTIVE ON HARDWARE  (อ่านจาก last write)",
                             style={**LBL,"color":C_VIN,"marginBottom":"6px"}),
                    html.Div(id="cascade-hw-gains",
                             style={"fontFamily":"monospace","fontSize":"12px",
                                    "color":C_TXT,"minHeight":"32px"}),
                    html.Div("💡 ถ้า P&P override ค่าไป ดูที่นี่แทน slider",
                             style={"color":C_MUT,"fontSize":"10px","marginTop":"4px"}),
                ]),
            ]),

            # ── P&P tab ────────────────────────────────────────────────────
            html.Div(id="tab-pp", style={"display":"none"}, children=[

                # Config card
                html.Div(style=CARD, children=[
                    html.Div("PICK & PLACE CONFIG", style={**LBL,"color":"#ffd43b","marginBottom":"8px"}),
                    html.Div("Picks (deg, comma-sep)", style=LBL),
                    dbc.Input(id="pp-picks", type="text", value="0,90,180,270",
                              style={**INP,"height":"28px","padding":"2px 8px","marginBottom":"6px"}),
                    html.Div("Places (deg, comma-sep)", style=LBL),
                    dbc.Input(id="pp-places", type="text", value="45,135,225,315",
                              style={**INP,"height":"28px","padding":"2px 8px","marginBottom":"6px"}),
                    dbc.Row([
                        dbc.Col([html.Div("Dwell (s)", style=LBL),
                                 dbc.Input(id="pp-dwell", type="number", value=2.0,
                                           min=0.0, max=30.0, step=0.5,
                                           style={**INP,"height":"28px","padding":"2px 8px"})],
                                width=4),
                        dbc.Col([html.Div("Thresh (°)", style=LBL),
                                 dbc.Input(id="pp-thresh", type="number", value=3.0,
                                           min=0.5, max=20.0, step=0.5,
                                           style={**INP,"height":"28px","padding":"2px 8px"})],
                                width=4),
                        dbc.Col([html.Div("Repeat", style=LBL),
                                 dbc.Checklist(id="pp-repeat",
                                               options=[{"label":"♾","value":"r"}],
                                               value=[],
                                               style={"marginTop":"4px"})],
                                width=4),
                    ], className="g-1 mb-2"),
                    dbc.Row([
                        dbc.Col(dbc.Button("▶ Go", id="btn-pp-go",
                                           color="success", size="sm", className="w-100"), width=6),
                        dbc.Col(dbc.Button("⏹ Stop", id="btn-pp-stop",
                                           color="danger",  size="sm", className="w-100"), width=6),
                    ], className="g-1"),
                    html.Div(id="pp-cfg-status",
                             style={"color":C_MUT,"fontSize":"11px","marginTop":"4px",
                                    "textAlign":"center","minHeight":"14px"}),
                ]),

                # Status card
                html.Div(style=CARD, children=[
                    html.Div("P&P STATUS", style={**LBL,"color":C_ACT,"marginBottom":"6px"}),
                    html.Div(id="pp-state-badge",
                             style={"textAlign":"center","fontWeight":"700","fontSize":"12px",
                                    "padding":"6px 4px","borderRadius":"4px","marginBottom":"6px",
                                    "backgroundColor":"#0f0f1a","color":"#888888",
                                    "border":"1px solid #888888"},
                             children="IDLE"),
                    html.Div(id="pp-progress",
                             style={"color":C_MUT,"fontSize":"11px",
                                    "textAlign":"center","marginBottom":"4px"},
                             children="Rod —"),
                    html.Div(id="pp-timer",
                             style={"color":C_VIN,"fontSize":"11px","textAlign":"center"},
                             children=""),
                ]),

                # PLACE ANALYSIS card
                html.Div(style=CARD, children=[
                    html.Div("PLACE ANALYSIS", style={**LBL,"color":C_POS,"marginBottom":"8px"}),
                    dbc.Row([
                        dbc.Col(html.Div("Rod",      style={**LBL,"marginBottom":"0"}), width=2),
                        dbc.Col(html.Div("Tgt °",    style={**LBL,"marginBottom":"0"}), width=2),
                        dbc.Col(html.Div("Over%",    style={**LBL,"marginBottom":"0","color":"#ff9f43"}), width=2),
                        dbc.Col(html.Div("Settle s", style={**LBL,"marginBottom":"0","color":"#ffd43b"}), width=3),
                        dbc.Col(html.Div("SS err°",  style={**LBL,"marginBottom":"0","color":"#a9e34b"}), width=3),
                    ], className="mb-1"),
                    html.Div(id="pp-place-analysis",
                             children=[html.Div("—", style={"color":C_MUT,"fontSize":"10px",
                                                             "textAlign":"center"})]),
                ]),
            ]),

            # Modbus test tab
            html.Div(id="tab-modbus", style={"display":"none"}, children=[

                # ── Ping ──────────────────────────────────────────────────────
                html.Div(style=CARD, children=[
                    html.Div("CONNECTION TEST", style={**LBL,"color":"#a9e34b","marginBottom":"8px"}),
                    dbc.Button("⚡ Ping (read 0x3F)", id="btn-mb-ping",
                               color="success", size="sm", className="w-100"),
                    html.Div(id="mb-ping-result",
                             style={"color":C_MUT,"fontSize":"11px",
                                    "marginTop":"6px","textAlign":"center","minHeight":"14px"}),
                ]),

                # ── Single R/W ────────────────────────────────────────────────
                html.Div(style=CARD, children=[
                    html.Div("SINGLE REGISTER R/W",
                             style={**LBL,"color":C_VIN,"marginBottom":"8px"}),

                    html.Div("Address (hex, e.g. 0x3F)", style=LBL),
                    dbc.Input(id="mb-addr", type="text", value="0x3F",
                              placeholder="0x00",
                              style={**INP,"height":"28px","padding":"2px 8px",
                                     "marginBottom":"6px","fontFamily":"monospace"}),

                    dbc.Button("📖 Read (FC3)", id="btn-mb-read",
                               color="info", size="sm", className="w-100 mb-2"),
                    html.Div(id="mb-read-result",
                             style={"fontFamily":"monospace","fontSize":"11px",
                                    "minHeight":"20px","marginBottom":"8px"}),

                    html.Hr(style=HR),
                    html.Div("Write Value (uint16 dec)", style=LBL),
                    dbc.Input(id="mb-write-val", type="number", value=0,
                              min=0, max=65535, step=1,
                              style={**INP,"height":"28px","padding":"2px 8px",
                                     "marginBottom":"6px"}),
                    dbc.Button("✏ Write (FC6)", id="btn-mb-write",
                               color="warning", size="sm", className="w-100 mb-1"),
                    html.Div(id="mb-write-result",
                             style={"fontFamily":"monospace","fontSize":"11px","minHeight":"14px"}),
                ]),

                # ── Scan ──────────────────────────────────────────────────────
                html.Div(style=CARD, children=[
                    html.Div("REGISTER SCAN (FC3)",
                             style={**LBL,"color":C_ACT,"marginBottom":"8px"}),
                    dbc.Row([
                        dbc.Col([
                            html.Div("Start (hex)", style=LBL),
                            dbc.Input(id="mb-scan-start", type="text", value="0x31",
                                      style={**INP,"height":"28px","padding":"2px 8px",
                                             "fontFamily":"monospace"}),
                        ], width=6),
                        dbc.Col([
                            html.Div("Count", style=LBL),
                            dbc.Input(id="mb-scan-count", type="number", value=22,
                                      min=1, max=64, step=1,
                                      style={**INP,"height":"28px","padding":"2px 8px"}),
                        ], width=6),
                    ], className="g-1 mb-2"),
                    dbc.Button("🔍 Scan", id="btn-mb-scan",
                               color="primary", size="sm", className="w-100 mb-2"),
                    html.Div(id="mb-scan-result",
                             style={"fontFamily":"monospace","fontSize":"10px",
                                    "maxHeight":"260px","overflowY":"auto",
                                    "backgroundColor":"#080810",
                                    "border":f"1px solid {C_BDR}",
                                    "borderRadius":"4px","padding":"6px 8px",
                                    "minHeight":"30px"}),
                ]),
            ]),

            # Register map (always)
            html.Div(style={**CARD,"fontSize":"10px","color":C_MUT}, children=[
                html.Div("REGISTER MAP", style={**LBL,"marginBottom":"6px"}),
                *[html.Div(t) for t in [
                    "── WRITE (PC→STM32, MANUAL only) ──",
                    "0x33-35  vel Kp/Ki/Kd ×100",
                    "0x36     speed ×10",
                    "0x37     half_period (ms)",
                    "0x38     waveform 0/1/2",
                    "0x39     run (0/1)",
                    "0x3A-3C  pos Kp/Ki/Kd ×100",
                    "0x3D     drive 0=cascade 1=direct",
                    "0x3E     target deg×10 (s16)",
                    "── READ (STM32→PC, MANUAL only) ──",
                    "0x3F     ref_qd ×100",
                    "0x40     qd_out ×100",
                    "0x41     V_in   ×100",
                    "0x42     q_out  ×100",
                    "0x43     est_I  ×1000",
                    "0x44     ref_q  ×100",
                    "── STATUS ──",
                    "0x31     estop (shared)",
                    "0x32     sys_mode (0/1/2)",
                    "0x45     ISR_counter",
                ]],
            ]),
        ]),

        # ── RIGHT PANEL ───────────────────────────────────────────────────────
        dbc.Col(width=9, children=[

            # ── Active PID Gains + Drive Mode ────────────────────────────────
            html.Div(style={**CARD,"padding":"8px 14px","marginBottom":"6px"}, children=[
                dbc.Row([
                    # Drive Mode badge
                    dbc.Col(html.Div([
                        html.Div("DRIVE MODE", style={**LBL,"marginBottom":"2px"}),
                        html.Span("—", id="gc-drive",
                                  style={"fontWeight":"700","fontSize":"12px","color":C_VIN}),
                    ]), width="auto"),
                    dbc.Col(html.Div(style={"borderLeft":f"1px solid {C_BDR}","height":"100%",
                                            "paddingLeft":"12px"}, children=[
                        html.Div("POSITION PID",
                                 style={**LBL,"color":C_POS,"marginBottom":"2px"}),
                        html.Div([
                            html.Span("Kp ", style={"color":C_MUT,"fontSize":"10px"}),
                            html.Span("—", id="gc-pkp",
                                      style={"color":C_POS,"fontWeight":"700","marginRight":"8px"}),
                            html.Span("Ki ", style={"color":C_MUT,"fontSize":"10px"}),
                            html.Span("—", id="gc-pki",
                                      style={"color":C_POS,"fontWeight":"700","marginRight":"8px"}),
                            html.Span("Kd ", style={"color":C_MUT,"fontSize":"10px"}),
                            html.Span("—", id="gc-pkd",
                                      style={"color":C_POS,"fontWeight":"700"}),
                        ]),
                    ])),
                    dbc.Col(html.Div(style={"borderLeft":f"1px solid {C_BDR}","height":"100%",
                                            "paddingLeft":"12px"}, children=[
                        html.Div("VELOCITY PID",
                                 style={**LBL,"color":C_ACT,"marginBottom":"2px"}),
                        html.Div([
                            html.Span("Kp ", style={"color":C_MUT,"fontSize":"10px"}),
                            html.Span("—", id="gc-vkp",
                                      style={"color":C_ACT,"fontWeight":"700","marginRight":"8px"}),
                            html.Span("Ki ", style={"color":C_MUT,"fontSize":"10px"}),
                            html.Span("—", id="gc-vki",
                                      style={"color":C_ACT,"fontWeight":"700","marginRight":"8px"}),
                            html.Span("Kd ", style={"color":C_MUT,"fontSize":"10px"}),
                            html.Span("—", id="gc-vkd",
                                      style={"color":C_ACT,"fontWeight":"700"}),
                        ]),
                    ])),
                ], align="center"),
            ]),

            # ── Live Metrics ─────────────────────────────────────────────────
            html.Div(style={**CARD,"padding":"8px 14px","marginBottom":"6px"}, children=[
                dbc.Row([
                    dbc.Col(metric("REF POS",  "m-refq", "deg",   C_REF)),
                    dbc.Col(metric("ACT POS",  "m-posq", "deg",   C_POS)),
                    dbc.Col(metric("POS ERR",  "m-errq", "deg",   "#ff9f43")),
                    dbc.Col(html.Div(style={"borderLeft":f"1px solid {C_BDR}","height":"100%"})),
                    dbc.Col(metric("REF VEL",  "m-ref",  "rad/s", C_REF)),
                    dbc.Col(metric("ACT VEL",  "m-act",  "rad/s", C_ACT)),
                    dbc.Col(metric("VEL ERR",  "m-err",  "rad/s", "#ff9f43")),
                    dbc.Col(metric("V_IN",     "m-vin",  "V",     C_VIN)),
                    dbc.Col(metric("CURRENT",  "m-cur",  "A",     C_CUR)),
                ]),
            ]),

            # Position graph
            html.Div(style={**CARD,"padding":"10px"}, children=[
                dcc.Graph(id="g-pos", style={"height":"210px"},
                          config={"displayModeBar":False}),
            ]),

            # Velocity graph
            html.Div(style={**CARD,"padding":"10px"}, children=[
                dcc.Graph(id="g-vel", style={"height":"180px"},
                          config={"displayModeBar":False}),
            ]),

            # Voltage graph
            html.Div(style={**CARD,"padding":"10px"}, children=[
                dcc.Graph(id="g-vin", style={"height":"120px"},
                          config={"displayModeBar":False}),
            ]),

            # Step Response Analysis
            html.Div(style={**CARD,"padding":"12px"}, children=[
                dbc.Row([
                    dbc.Col(html.Div("STEP RESPONSE ANALYSIS",
                                     style={**LBL,"marginBottom":"0"}), width="auto"),
                    dbc.Col(html.Div(id="sr-status",
                                     style={"color":C_MUT,"fontSize":"10px","textAlign":"right"}),
                            style={"textAlign":"right"}),
                ], align="center", className="mb-2"),
                dbc.Row([
                    dbc.Col(html.Div([
                        html.Div("OVERSHOOT", style=LBL),
                        html.Div([
                            html.Span("—", id="sr-overshoot",
                                      style={"fontSize":"26px","fontWeight":"900","color":C_REF}),
                            html.Span(" %", style={"color":C_MUT,"fontSize":"11px"}),
                        ]),
                    ], style={"textAlign":"center","padding":"4px 0"})),
                    dbc.Col(html.Div([
                        html.Div("RISE TIME", style=LBL),
                        html.Div([
                            html.Span("—", id="sr-rise",
                                      style={"fontSize":"26px","fontWeight":"900","color":C_VIN}),
                            html.Span(" s", style={"color":C_MUT,"fontSize":"11px"}),
                        ]),
                    ], style={"textAlign":"center","padding":"4px 0"})),
                    dbc.Col(html.Div([
                        html.Div("SETTLING TIME", style=LBL),
                        html.Div([
                            html.Span("—", id="sr-settle",
                                      style={"fontSize":"26px","fontWeight":"900","color":C_ACT}),
                            html.Span(" s", style={"color":C_MUT,"fontSize":"11px"}),
                        ]),
                    ], style={"textAlign":"center","padding":"4px 0"})),
                    dbc.Col(html.Div([
                        html.Div("SS ERROR", style=LBL),
                        html.Div([
                            html.Span("—", id="sr-ss-err",
                                      style={"fontSize":"26px","fontWeight":"900","color":C_POS}),
                            html.Span(" °", style={"color":C_MUT,"fontSize":"11px"}),
                        ]),
                    ], style={"textAlign":"center","padding":"4px 0"})),
                ], className="g-0"),
            ]),

            # Event Log
            html.Div(style={**CARD,"padding":"10px"}, children=[
                dbc.Row([
                    dbc.Col(html.Div("EVENT LOG", style={**LBL,"marginBottom":"0"}), width="auto"),
                    dbc.Col(html.Div(id="log-stats",
                                     style={"color":C_MUT,"fontSize":"10px","textAlign":"right"}),
                            style={"textAlign":"right"}),
                    dbc.Col(dbc.Button("Clear", id="btn-clear-log",
                                       color="secondary", size="sm",
                                       style={"fontSize":"10px","padding":"1px 8px"}),
                            width="auto"),
                ], align="center", className="mb-1"),
                html.Div(id="log-panel",
                         style={"height":"140px","overflowY":"auto",
                                "backgroundColor":"#080810",
                                "border":f"1px solid {C_BDR}","borderRadius":"4px",
                                "padding":"6px 8px","fontFamily":"monospace",
                                "fontSize":"11px"}),
            ]),
        ]),
    ]),

    dcc.Interval(id="tick", interval=100, n_intervals=0),
    dcc.Store(id="store-running",   data=False),
    dcc.Store(id="store-ports",     data=_ports),
    dcc.Store(id="store-log-clear", data=0),
    dcc.Store(id="store-ku-result", data=None),   # {"ku":…,"tu":…} เมื่อหาเจอ
])

# ─────────────────────────────────────────────────────────────────────────────
#  CALLBACKS
# ─────────────────────────────────────────────────────────────────────────────

# ── Refresh ports ─────────────────────────────────────────────────────────────
@app.callback(
    Output("dd-port","options"), Output("dd-port","value"), Output("store-ports","data"),
    Input("btn-refresh","n_clicks"), prevent_initial_call=True,
)
def refresh_ports(_):
    pts = list_ports()
    opts = [{"label":p,"value":p} for p in pts] if pts else [{"label":"ไม่พบ","value":""}]
    return opts, (pts[0] if pts else ""), pts

# ── Connect ───────────────────────────────────────────────────────────────────
@app.callback(
    Output("conn-status","children"), Output("hdr-status","children"),
    Input("btn-connect","n_clicks"), State("dd-port","value"),
    prevent_initial_call=True,
)
def on_connect(_, port):
    if not port:
        s = html.Span("ไม่พบ port", style={"color":C_REF}); return s, s
    connect_modbus(port)
    txt = f"Connected {port} {BAUD} 8E1  slave={SLAVE_ID}" if connected else f"Error: {last_err}"
    col = C_ACT if connected else C_REF
    return (html.Span(txt, style={"color":col,"fontSize":"11px"}),
            html.Span(("● " if connected else "✕ ")+txt, style={"color":col,"fontSize":"11px"}))

# ── Mode tabs: show/hide + auto-configure ─────────────────────────────────────
@app.callback(
    Output("tab-main","style"),    Output("tab-vel","style"),
    Output("tab-pos","style"),     Output("tab-cascade","style"),
    Output("tab-pp","style"),      Output("tab-modbus","style"),
    Output("target-panel","style"),
    Output("mode-badge","children"), Output("mode-cfg-status","children"),
    Input("mode-tabs","active_tab"),
)
def switch_mode(tab):
    show = {"display":"block"}; hide = {"display":"none"}
    # (tab-main, tab-vel, tab-pos, tab-cascade, tab-pp, tab-modbus, target-panel)
    styles = {
        "main":    (show, hide, hide, hide, hide, hide, show),
        "vel":     (hide, show, hide, hide, hide, hide, hide),
        "pos":     (hide, hide, show, hide, hide, hide, show),
        "cascade": (hide, hide, hide, show, hide, hide, hide),
        "pp":      (hide, hide, hide, hide, show, hide, hide),
        "modbus":  (hide, hide, hide, hide, hide, show, hide),
    }
    badges = {
        "main":    ("🏠 MAIN",      C_TXT),
        "vel":     ("🔄 VEL TUNE",  C_ACT),
        "pos":     ("📍 POS TUNE",  C_POS),
        "cascade": ("⛓ CASCADE",   "#cc5de8"),
        "pp":      ("🤖 P&P",       "#ffd43b"),
        "modbus":  ("🔧 MODBUS",    "#ff9f43"),
    }
    s = styles.get(tab, styles["main"])
    btext, bcol = badges.get(tab, ("—", C_MUT))
    badge = html.Span(btext, style={"color": bcol})

    cfg_msg = ""
    if connected:
        try:
            if tab != "pp":   # P&P หยุดเองใน callback stop
                write_run_flag(False)
            if tab == "vel":
                write_pos_pid(0, 0, 0)
                write_drive_mode(False)
                cfg_msg = "pos off | cascade | stopped"
            elif tab == "pos":
                cfg_msg = "stopped  (choose drive mode below)"
            elif tab == "cascade":
                write_drive_mode(False)
                cfg_msg = "cascade | stopped"
            elif tab == "pp":
                write_drive_mode(False)
                cfg_msg = (f"cascade | Pos Kp={last_vals.get('pos_kp',0):.2f} "
                           f"Ki={last_vals.get('pos_ki',0):.2f} "
                           f"Kd={last_vals.get('pos_kd',0):.2f} | ready")
            elif tab in ("main", "modbus"):
                write_drive_mode(False)
                cfg_msg = "cascade | stopped"
        except Exception as e:
            cfg_msg = f"cfg err: {e}"

    return (*s, badge, html.Span(cfg_msg, style={"color": C_MUT}))

# ── Vel slider → input sync ───────────────────────────────────────────────────
for _p, _k in [("vel","kp"),("vel","ki"),("vel","kd"),
               ("vel-c","kp"),("vel-c","ki"),("vel-c","kd")]:
    @app.callback(Output(f"in-{_p}-{_k}","value"),
                  Input(f"sl-{_p}-{_k}","value"), prevent_initial_call=True)
    def _sv(v): return v

# ── Pos slider → input sync ───────────────────────────────────────────────────
for _p, _k in [("pos","kp"),("pos","ki"),("pos","kd"),
               ("pos-c","kp"),("pos-c","ki"),("pos-c","kd")]:
    @app.callback(Output(f"in-{_p}-{_k}","value"),
                  Input(f"sl-{_p}-{_k}","value"), prevent_initial_call=True)
    def _sp(v): return v

# ── Apply Vel PID (vel tab) ───────────────────────────────────────────────────
@app.callback(
    Output("vel-pid-status","children"),
    Input("btn-vel-pid","n_clicks"),
    State("in-vel-kp","value"), State("in-vel-ki","value"), State("in-vel-kd","value"),
    prevent_initial_call=True,
)
def on_vel_pid(_, kp, ki, kd):
    r = write_vel_pid(float(kp or 0), float(ki or 0), float(kd or 0))
    if r == "ok": return html.Span(f"OK Kp={kp:.2f} Ki={ki:.2f} Kd={kd:.2f}", style={"color":C_ACT})
    return html.Span(f"Error: {r}", style={"color":C_REF})

# ── Apply Vel PID (cascade tab) ───────────────────────────────────────────────
@app.callback(
    Output("vel-pid-c-status","children"),
    Input("btn-vel-pid-c","n_clicks"),
    State("in-vel-c-kp","value"), State("in-vel-c-ki","value"), State("in-vel-c-kd","value"),
    prevent_initial_call=True,
)
def on_vel_pid_c(_, kp, ki, kd):
    r = write_vel_pid(float(kp or 0), float(ki or 0), float(kd or 0))
    if r == "ok": return html.Span(f"OK Kp={kp:.2f} Ki={ki:.2f} Kd={kd:.2f}", style={"color":C_ACT})
    return html.Span(f"Error: {r}", style={"color":C_REF})

# ── Apply Pos PID (pos tab) ───────────────────────────────────────────────────
@app.callback(
    Output("pos-pid-status","children"),
    Input("btn-pos-pid","n_clicks"),
    State("in-pos-kp","value"), State("in-pos-ki","value"), State("in-pos-kd","value"),
    prevent_initial_call=True,
)
def on_pos_pid(_, kp, ki, kd):
    r = write_pos_pid(float(kp or 0), float(ki or 0), float(kd or 0))
    mode = "ACTIVE" if float(kp or 0) > 0 else "DISABLED"
    if r == "ok": return html.Span(f"OK Kp={kp:.2f} Ki={ki:.2f} Kd={kd:.2f} [{mode}]", style={"color":C_POS})
    return html.Span(f"Error: {r}", style={"color":C_REF})

# ── Apply Pos PID (cascade tab) ───────────────────────────────────────────────
@app.callback(
    Output("pos-pid-c-status","children"),
    Input("btn-pos-pid-c","n_clicks"),
    State("in-pos-c-kp","value"), State("in-pos-c-ki","value"), State("in-pos-c-kd","value"),
    prevent_initial_call=True,
)
def on_pos_pid_c(_, kp, ki, kd):
    r = write_pos_pid(float(kp or 0), float(ki or 0), float(kd or 0))
    if r == "ok": return html.Span(f"OK Kp={kp:.2f} Ki={ki:.2f} Kd={kd:.2f}", style={"color":C_POS})
    return html.Span(f"Error: {r}", style={"color":C_REF})

# ── Apply Waveform ─────────────────────────────────────────────────────────────
@app.callback(
    Output("wave-status","children"),
    Input("btn-wave","n_clicks"),
    State("in-speed","value"), State("in-period","value"), State("radio-wave","value"),
    prevent_initial_call=True,
)
def on_wave(_, speed, period, wave):
    r = write_waveform(float(speed or 3), int(period or 3000), wave or "square")
    if r == "ok": return html.Span(f"OK {wave} {speed} half={period}ms", style={"color":C_VIN})
    return html.Span(f"Error: {r}", style={"color":C_REF})

# ── Drive Mode ────────────────────────────────────────────────────────────────
@app.callback(
    Output("drive-mode-status","children"), Output("store-drive-mode","data"),
    Input("radio-drive-mode","value"), prevent_initial_call=True,
)
def on_drive_mode(mode):
    r = write_drive_mode(mode == "direct")
    txt = "⚡ DIRECT pos→V" if mode == "direct" else "⛓ CASCADE pos→vel→V"
    col = C_VIN if mode == "direct" else C_ACT
    if r != "ok": return html.Span(f"Error: {r}", style={"color":C_REF}), mode
    return html.Span(txt, style={"color":col}), mode

# ── Preset buttons → fill target input ────────────────────────────────────────
@app.callback(
    Output("in-tgt","value"),
    Input("p-n90","n_clicks"), Input("p-n45","n_clicks"),
    Input("p-0","n_clicks"),   Input("p-p45","n_clicks"),
    Input("p-p90","n_clicks"), Input("p-p135","n_clicks"),
    Input("p-p180","n_clicks"),Input("p-n135","n_clicks"),
    prevent_initial_call=True,
)
def preset_fill(*_):
    ctx = dash.callback_context
    if not ctx.triggered: return 0
    m = {"p-n90":-90,"p-n45":-45,"p-0":0,"p-p45":45,
         "p-p90":90,"p-p135":135,"p-p180":180,"p-n135":-135}
    return m.get(ctx.triggered[0]["prop_id"].split(".")[0], 0)

# ── Go button ─────────────────────────────────────────────────────────────────
@app.callback(
    Output("tgt-status","children"),
    Input("btn-go","n_clicks"), State("in-tgt","value"),
    prevent_initial_call=True,
)
def on_go(_, deg):
    deg = float(deg or 0)
    r = write_pos_target(deg)
    if r == "ok":
        with step_lock:
            step_info.update(active=True, t_start=time.time()-t0,
                             target_deg=deg, initial_deg=last_vals["pos"]*RAD2DEG)
            step_results.update({k: None for k in step_results})
        return html.Span(f"→ {deg:+.1f}°", style={"color":C_POS,"fontWeight":"700"})
    return html.Span(f"Error: {r}", style={"color":C_REF})

# ── START / STOP ──────────────────────────────────────────────────────────────
@app.callback(
    Output("store-running","data"),
    Input("btn-run","n_clicks"), Input("btn-stop","n_clicks"),
    State("store-running","data"), prevent_initial_call=True,
)
def on_run_stop(run_clicks, stop_clicks, currently):
    ctx = dash.callback_context
    if not ctx.triggered: return currently
    btn = ctx.triggered[0]["prop_id"].split(".")[0]
    new = (btn == "btn-run")
    write_run_flag(new)
    return new

# ── Clear log ─────────────────────────────────────────────────────────────────
@app.callback(
    Output("store-log-clear","data"),
    Input("btn-clear-log","n_clicks"), prevent_initial_call=True,
)
def clear_log(_):
    with log_lock: log_entries.clear()
    add_log("INFO","LOG","Log cleared"); return time.time()

# ── P&P Go ────────────────────────────────────────────────────────────────────
@app.callback(
    Output("pp-cfg-status","children"),
    Input("btn-pp-go","n_clicks"),
    State("pp-picks","value"), State("pp-places","value"),
    State("pp-dwell","value"), State("pp-thresh","value"),
    State("pp-repeat","value"),
    prevent_initial_call=True,
)
def on_pp_go(_, picks_s, places_s, dwell, thresh, repeat_v):
    global pp_state, pp_rod_idx, pp_running, _last_tgt_raw, _pp_arrive_ticks
    try:
        picks  = [float(x.strip()) for x in (picks_s  or "0").split(",") if x.strip()]
        places = [float(x.strip()) for x in (places_s or "45").split(",") if x.strip()]
        n      = max(1, min(5, min(len(picks), len(places))))
        with pp_lock:
            pp_cfg.update(n_rods=n, picks=picks[:n], places=places[:n],
                          dwell=float(dwell or 2.0), thresh=float(thresh or 3.0),
                          repeat=bool(repeat_v), _run_confirmed=False)
            pp_rod_idx = 0
            pp_running = True
            pp_state   = PP_MOVE_PICK
        with pp_place_lock:
            pp_place_results.clear()
        _pp_arrive_ticks = 0
        _last_tgt_raw    = None   # force first write
        write_run_flag(True)
        write_drive_mode(False)
        add_log("INFO","P&P",
                f"P&P started — {n} rods | picks={[round(p,1) for p in picks[:n]]} | dwell={dwell}s")
        return f"▶ {n} rods | dwell={dwell}s | thresh=±{thresh}°"
    except Exception as e:
        add_log("ERROR","P&P",str(e)); return f"Error: {e}"

# ── P&P Stop ──────────────────────────────────────────────────────────────────
@app.callback(
    Output("pp-cfg-status","children", allow_duplicate=True),
    Input("btn-pp-stop","n_clicks"),
    prevent_initial_call=True,
)
def on_pp_stop(_):
    global pp_running, pp_state
    with pp_lock:
        pp_running = False
        pp_state   = PP_IDLE
    try:
        _wr(REG_BS_SOFT_STOP, 1)
        add_log("INFO","P&P","P&P stopped by user")
        write_run_flag(False)
        _wr(REG_BS_SOFT_STOP, 0)
    except Exception: pass
    return "⏹ Stopped"

# ── P&P Status update ─────────────────────────────────────────────────────────
@app.callback(
    Output("pp-state-badge","children"),
    Output("pp-state-badge","style"),
    Output("pp-progress","children"),
    Output("pp-timer","children"),
    Output("pp-place-analysis","children"),
    Input("tick","n_intervals"),
)
def update_pp_status(_):
    # รัน state machine
    if pp_running:
        try: pp_tick()
        except Exception as e: add_log("ERROR","P&P",f"tick: {e}")

    with pp_lock:
        state = pp_state
        rod   = pp_rod_idx
        n     = pp_cfg["n_rods"]
        dwell = pp_cfg["dwell"]
        t_ent = pp_t_enter
        picks  = list(pp_cfg["picks"])
        places = list(pp_cfg["places"])

    label, color = _PP_LABELS.get(state, ("?", "#888888"))
    badge_style = {
        "textAlign":"center","fontWeight":"700","fontSize":"12px",
        "padding":"6px 4px","borderRadius":"4px","marginBottom":"6px",
        "backgroundColor":"#0f0f1a","color":color,"border":f"1px solid {color}",
    }

    # Progress
    rod_disp = min(rod + 1, n)
    if state in (PP_IDLE, PP_DONE):
        prog = f"Rod —/{n}" if state == PP_IDLE else f"✅ {n}/{n} complete"
    else:
        tgt = (picks[rod]  if state in (PP_MOVE_PICK, PP_DWELL_PICK) else
               places[rod]) if rod < len(picks) else 0.0
        prog = html.Span([
            html.Span(f"Rod ", style={"color":C_MUT}),
            html.Span(f"{rod_disp}/{n}", style={"color":color,"fontWeight":"700","fontSize":"13px"}),
            html.Span(f"  → {tgt:+.1f}°", style={"color":C_MUT}),
        ])

    # Dwell timer
    timer_txt = ""
    if state in (PP_DWELL_PICK, PP_DWELL_PLACE):
        elapsed = (time.time() - t0) - t_ent
        remain  = max(0.0, dwell - elapsed)
        pct     = min(100, int(elapsed / dwell * 100)) if dwell > 0 else 100
        timer_txt = f"⏱ {remain:.1f}s  [{pct}%]"

    # Place analysis rows
    with pp_place_lock:
        results_snap = dict(pp_place_results)

    if not results_snap:
        analysis = [html.Div("—", style={"color":C_MUT,"fontSize":"10px","textAlign":"center"})]
    else:
        rows = []
        for rod_i in sorted(results_snap):
            r      = results_snap[rod_i]
            over   = r["overshoot_pct"]
            settle = r["settle_time_s"]
            ss     = r["ss_error_deg"]
            tgt    = r["target_deg"]
            oc = "#ff6b6b" if over   > 10  else "#ffd43b" if over   > 5   else "#51cf66"
            sc = "#ff6b6b" if (settle is None or settle > 2.0) else \
                 "#ffd43b" if settle > 1.0 else "#51cf66"
            ec = "#ff6b6b" if ss > 3.0 else "#ffd43b" if ss > 1.0 else "#51cf66"
            rows.append(dbc.Row([
                dbc.Col(html.Span(f"{rod_i+1}", style={"color":C_MUT,"fontWeight":"700"}), width=2),
                dbc.Col(html.Span(f"{tgt:+.0f}", style={"color":C_POS}), width=2),
                dbc.Col(html.Span(f"{over:.1f}", style={"color":oc,"fontWeight":"700"}), width=2),
                dbc.Col(html.Span("—" if settle is None else f"{settle:.2f}",
                                  style={"color":sc,"fontWeight":"700"}), width=3),
                dbc.Col(html.Span(f"{ss:.2f}", style={"color":ec,"fontWeight":"700"}), width=3),
            ], className="mb-1"))
        analysis = rows

    return label, badge_style, prog, timer_txt, analysis

# ── Main update callback ───────────────────────────────────────────────────────
@app.callback(
    Output("g-pos",        "figure"),
    Output("g-vel",        "figure"),
    Output("g-vin",        "figure"),
    # Live metrics
    Output("m-refq",  "children"), Output("m-posq",  "children"),
    Output("m-errq",  "children"),
    Output("m-ref",   "children"), Output("m-act",   "children"),
    Output("m-err",   "children"), Output("m-vin",   "children"),
    Output("m-cur",   "children"),
    # Active Gains card
    Output("gc-drive","children"),
    Output("gc-pkp",  "children"), Output("gc-pki",  "children"),
    Output("gc-pkd",  "children"),
    Output("gc-vkp",  "children"), Output("gc-vki",  "children"),
    Output("gc-vkd",  "children"),
    # Step response
    Output("sr-overshoot","children"), Output("sr-rise",   "children"),
    Output("sr-settle",   "children"), Output("sr-ss-err", "children"),
    Output("sr-status",   "children"),
    # Main tab big display
    Output("main-pos-big","children"), Output("main-tgt-big","children"),
    # Log
    Output("log-panel","children"), Output("log-stats","children"),
    # Ku search
    Output("ku-progress","children"), Output("ku-result","children"),
    Output("ku-suggest","children"),  Output("store-ku-result","data"),
    # Cascade HW display
    Output("cascade-hw-gains","children"),
    Input("tick","n_intervals"),
)
def update(_):
    with data_lock:
        t     = list(data["t"]);     ref   = list(data["ref"])
        act   = list(data["act"]);   vin   = list(data["vin"])
        pos   = list(data["pos"]);   cur   = list(data["cur"])
        ref_q = list(data["ref_q"])

    # ── Ku search tick ──
    pos_deg_full = [v * RAD2DEG for v in pos]
    _ku_tick(pos_deg_full, t)

    t_win = max(t[-1]-t[0], 10.0) if t else 10.0
    pos_deg   = [v*RAD2DEG for v in pos]
    ref_q_deg = [v*RAD2DEG for v in ref_q]

    # ── Position graph ────────────────────────────────────────────────────
    fp = go.Figure()
    if t:
        fp.add_trace(go.Scatter(x=t, y=ref_q_deg, name="ref_q",
                                line=dict(color=C_REF,width=1.5,dash="dash"), mode="lines"))
        fp.add_trace(go.Scatter(x=t, y=pos_deg, name="q_out",
                                line=dict(color=C_POS,width=2), mode="lines"))
        fp.add_hline(y=0, line_color="#333", line_width=1)
        fp.update_xaxes(range=[t[-1]-t_win, t[-1]])
    fp.update_layout(**GLAY, yaxis_title="deg",
                     title=dict(text="Position Tracking",
                                font=dict(size=11,color=C_MUT), x=0.01))

    # ── Velocity graph ────────────────────────────────────────────────────
    fv = go.Figure()
    if t:
        fv.add_trace(go.Scatter(x=t, y=ref, name="ref_qd",
                                line=dict(color=C_REF,width=1.5,dash="dash"), mode="lines"))
        fv.add_trace(go.Scatter(x=t, y=act, name="qd_out",
                                line=dict(color=C_ACT,width=2), mode="lines"))
        fv.add_hline(y=0, line_color="#333", line_width=1)
        fv.update_xaxes(range=[t[-1]-t_win, t[-1]])
    fv.update_layout(**GLAY, yaxis_title="rad/s",
                     title=dict(text="Velocity",
                                font=dict(size=11,color=C_MUT), x=0.01))

    # ── Voltage graph ─────────────────────────────────────────────────────
    fvin = go.Figure()
    if t:
        fvin.add_trace(go.Scatter(x=t, y=vin, name="V_in",
                                  line=dict(color=C_VIN,width=1.5),
                                  fill="tozeroy",
                                  fillcolor="rgba(255,212,59,0.07)", mode="lines"))
        fvin.add_hline(y=24,  line=dict(color=C_REF, dash="dot", width=1))
        fvin.add_hline(y=-24, line=dict(color=C_REF, dash="dot", width=1))
        fvin.update_xaxes(range=[t[-1]-t_win, t[-1]])
    fvin.update_layout(**{**GLAY,
        "yaxis":dict(range=[-26,26],gridcolor="#222",color=C_MUT,title="V"),
        "title":dict(text="Voltage",font=dict(size=11,color=C_MUT),x=0.01)})

    # ── Metrics ───────────────────────────────────────────────────────────
    rq   = last_vals["ref_q"] * RAD2DEG
    pq   = last_vals["pos"]   * RAD2DEG
    rv   = last_vals["ref"];  av = last_vals["act"]
    vv   = last_vals["vin"];  cv = last_vals["cur"]
    perr = rq - pq;   verr = rv - av

    def f(v, d=2): return f"{v:+.{d}f}"

    perr_s = html.Span(f"{perr:+.1f}", style={"color": C_REF if abs(perr)>3 else C_POS})
    verr_s = html.Span(f(verr),        style={"color": C_REF if abs(verr)>0.5 else C_ACT})
    cur_s  = html.Span(f"{cv:+.2f}",   style={"color": C_REF if abs(cv)>8 else C_CUR})

    # ── Active Gains card ─────────────────────────────────────────────────
    direct = last_vals["drive_direct"]
    drive_txt  = "⚡ DIRECT pos→V" if direct else "⛓ CASCADE pos→vel→V"
    drive_col  = C_VIN if direct else C_ACT
    gc_drive   = html.Span(drive_txt, style={"color": drive_col, "fontWeight":"700"})
    gc_pkp = f"{last_vals['pos_kp']:.2f}"
    gc_pki = f"{last_vals['pos_ki']:.2f}"
    gc_pkd = f"{last_vals['pos_kd']:.2f}"
    gc_vkp = f"{last_vals['kp']:.2f}"
    gc_vki = f"{last_vals['ki']:.2f}"
    gc_vkd = f"{last_vals['kd']:.2f}"

    # ── Step response ─────────────────────────────────────────────────────
    with step_lock: si = dict(step_info); sr = dict(step_results)
    if si["active"] and si["t_start"] is not None:
        analyze_step(t, pos_deg, si["t_start"], si["target_deg"], si["initial_deg"])
        with step_lock: sr = dict(step_results)

    def fsr(v, fmt=".2f", fb="—"):
        return f"{v:{fmt}}" if v is not None else fb

    oc  = C_REF if (sr["overshoot_pct"] or 0)>15 else \
          C_VIN if (sr["overshoot_pct"] or 0)>5  else C_ACT
    sc  = C_REF if sr["settle_time_s"] is None else \
          C_VIN if (sr["settle_time_s"] or 0)>2 else C_ACT
    sec = C_REF if (sr["ss_error_deg"] or 0)>SETTLE_BAND_DEG else C_POS

    def sr_span(v, fmt, fb, color, size="26px"):
        return html.Span(fsr(v,fmt,fb), style={"fontSize":size,"fontWeight":"900","color":color})

    sr_over   = sr_span(sr["overshoot_pct"],  ".1f", "—", oc)
    sr_rise   = sr_span(sr["rise_time_s"],    ".3f", "—", C_VIN)
    sr_settle = sr_span(sr["settle_time_s"],  ".3f",
                        "pending…" if si["active"] else "—", sc)
    sr_ss     = sr_span(sr["ss_error_deg"],   ".2f", "—", sec)

    # sr-status: แสดง target, delta, เวลาที่ผ่านมา
    if si["active"] and si["t_start"] is not None:
        elapsed = (list(t)[-1] if t else 0) - si["t_start"]
        delta   = si["target_deg"] - si["initial_deg"]
        sr_status = (f"tgt={si['target_deg']:+.1f}°  "
                     f"Δ={delta:+.1f}°  "
                     f"band ±{SETTLE_BAND_DEG:.0f}°  "
                     f"t={elapsed:.1f}s")
    else:
        sr_status = f"band ±{SETTLE_BAND_DEG:.0f}°  settle >{SETTLE_DURATION_S:.1f}s  — press Go to start"

    # ── Log ───────────────────────────────────────────────────────────────
    clr_map = {k:v[1] for k,v in LOG_STYLE.items()}
    lbl_map = {k:v[0] for k,v in LOG_STYLE.items()}
    with log_lock: entries = list(log_entries)
    rows = []
    for e in entries:
        lvl=e["level"]; c=clr_map.get(lvl,C_MUT); l=lbl_map.get(lvl,lvl[:5].ljust(5))
        rows.append(html.Div([
            html.Span(e["ts"],            style={"color":"#555","marginRight":"6px"}),
            html.Span(f"[{l}]",           style={"color":c,"fontWeight":"700",
                                                  "marginRight":"6px",
                                                  "minWidth":"52px","display":"inline-block"}),
            html.Span(f"[{e['src']:7s}]", style={"color":"#777","marginRight":"6px"}),
            html.Span(e["msg"],           style={"color": c if lvl not in ("INFO","WRITE") else C_TXT}),
        ], style={"whiteSpace":"nowrap","borderBottom":"1px solid #111",
                  "paddingBottom":"1px","marginBottom":"1px"}))

    errc  = sum(1 for e in entries if e["level"] in ("ERROR","TIMEOUT","CRC","SERIAL"))
    warnc = sum(1 for e in entries if e["level"] == "WARN")
    stats = f"total {len(entries)}  err {errc}  warn {warnc}  poll {_poll_ok_count}"
    if last_err: stats = html.Span(f"⚠ {last_err[:60]}", style={"color":C_REF})

    # ── Ku search UI ──────────────────────────────────────────────────────
    with ku_lock:
        ks = dict(ku_search)

    if ks["active"]:
        ku_prog = (f"🔄 Kp={ks['kp_cur']:.2f}  amp={ks['osc_amp']:.1f}°  "
                   f"next step in {max(0, ks['interval_s']-(t[-1]-ks['t_step']) if t else 0):.1f}s")
    elif ks["ku"] is not None:
        ku_prog = f"✅ Done — Ku={ks['ku']:.2f}  Tu={ks['tu']:.3f}s"
    else:
        ku_prog = "— กด Start เพื่อเริ่ม search"

    if ks["ku"] is not None and ks["tu"] is not None:
        ku_val, tu_val = ks["ku"], ks["tu"]
        ku_res  = f"Ku = {ku_val:.2f}   Tu = {tu_val:.3f} s"
        kp_sug  = round(0.20 * ku_val,  2)
        ki_sug  = round(ku_val / (2.2 * tu_val), 3)
        kd_sug  = round(ku_val * tu_val / 6.3,   3)
        ku_sug  = (f"Kp = {kp_sug:.2f}\n"
                   f"Ki = {ki_sug:.3f}\n"
                   f"Kd = {kd_sug:.3f}\n"
                   f"(No-Overshoot variant: ×0.2Ku)")
        ku_store = {"ku": ku_val, "tu": tu_val,
                    "kp": kp_sug, "ki": ki_sug, "kd": kd_sug}
    else:
        ku_res   = "ยังไม่พบ Ku"
        ku_sug   = "—"
        ku_store = None

    # ── Cascade HW gains display ──────────────────────────────────────────
    hw_pkp = last_vals.get("pos_kp", 0.0)
    hw_pki = last_vals.get("pos_ki", 0.0)
    hw_pkd = last_vals.get("pos_kd", 0.0)
    hw_vkp = last_vals.get("kp",     0.0)
    hw_vki = last_vals.get("ki",     0.0)
    hw_vkd = last_vals.get("kd",     0.0)
    cascade_hw = html.Div([
        html.Div([
            html.Span("POS  ", style={"color":C_POS}),
            html.Span(f"Kp={hw_pkp:.2f}  Ki={hw_pki:.2f}  Kd={hw_pkd:.2f}",
                      style={"color":C_TXT}),
        ]),
        html.Div([
            html.Span("VEL  ", style={"color":C_ACT}),
            html.Span(f"Kp={hw_vkp:.2f}  Ki={hw_vki:.2f}  Kd={hw_vkd:.2f}",
                      style={"color":C_TXT}),
        ]),
    ])

    return (
        fp, fv, fvin,
        f"{rq:+.1f}", f"{pq:+.1f}", perr_s,
        f(rv), f(av), verr_s, f"{vv:+.1f}", cur_s,
        gc_drive, gc_pkp, gc_pki, gc_pkd, gc_vkp, gc_vki, gc_vkd,
        sr_over, sr_rise, sr_settle, sr_ss, sr_status,
        f"{pq:+.1f}", f"{si['target_deg']:+.1f}",
        rows, stats,
        ku_prog, ku_res, ku_sug, ku_store,
        cascade_hw,
    )

# ─────────────────────────────────────────────────────────────────────────────
#  MODBUS TEST CALLBACKS
# ─────────────────────────────────────────────────────────────────────────────

def _parse_addr(s):
    """แปลง '0x3F' หรือ '63' หรือ '3F' (ตีความเป็น hex ถ้ามี 0x) → int"""
    s = (s or "0x3F").strip()
    try:
        return int(s, 16) if s.lower().startswith("0x") else int(s, 10)
    except ValueError:
        return None

def _decode_reg(addr, raw):
    """คืนค่า decoded string ของ register address นั้น"""
    meta = REG_META.get(addr)
    if meta:
        try:
            return meta[1](raw)
        except Exception:
            return "decode err"
    return "—"

def _reg_name(addr):
    return REG_META[addr][0] if addr in REG_META else "unknown"


# ── Ping ──────────────────────────────────────────────────────────────────────
@app.callback(
    Output("mb-ping-result","children"),
    Input("btn-mb-ping","n_clicks"),
    prevent_initial_call=True,
)
def on_mb_ping(_):
    if not connected:
        return html.Span("❌ not connected", style={"color":C_REF})
    try:
        t0p = time.perf_counter()
        with instrument_lock:
            regs = instrument.read_registers(REG_TELEMETRY, 1, functioncode=3)
        rtt = (time.perf_counter() - t0p) * 1000
        add_log("INFO","MB_PING",f"OK  raw[0x3F]={regs[0]}  RTT={rtt:.1f} ms")
        return html.Span(
            f"✅ OK  RTT={rtt:.1f} ms  raw[0x3F]={regs[0]}",
            style={"color":"#a9e34b","fontWeight":"700"})
    except Exception as e:
        add_log(classify_error(e),"MB_PING",str(e))
        return html.Span(f"❌ {e}", style={"color":C_REF})


# ── Read single register ───────────────────────────────────────────────────────
@app.callback(
    Output("mb-read-result","children"),
    Input("btn-mb-read","n_clicks"),
    State("mb-addr","value"),
    prevent_initial_call=True,
)
def on_mb_read(_, addr_str):
    if not connected:
        return html.Span("❌ not connected", style={"color":C_REF})
    addr = _parse_addr(addr_str)
    if addr is None:
        return html.Span("❌ address ไม่ถูกต้อง (ใช้ 0x3F หรือ 63)", style={"color":C_REF})
    try:
        t0p = time.perf_counter()
        with instrument_lock:
            regs = instrument.read_registers(addr, 1, functioncode=3)
        rtt = (time.perf_counter() - t0p) * 1000
        raw  = regs[0]
        s16  = to_s16(raw)
        dec  = _decode_reg(addr, raw)
        name = _reg_name(addr)
        add_log("INFO","MB_READ",
                f"0x{addr:02X} [{name}]  raw={raw}  s16={s16}  → {dec}  RTT={rtt:.1f}ms")
        return html.Div([
            html.Div([
                html.Span(f"0x{addr:02X} [{name}]",
                          style={"color":C_VIN,"fontWeight":"700","marginRight":"10px"}),
                html.Span(f"RTT={rtt:.1f} ms",
                          style={"color":C_MUT,"fontSize":"10px"}),
            ]),
            dbc.Row([
                dbc.Col([html.Div("uint16",style=LBL), html.Div(str(raw),style={"color":C_TXT})]),
                dbc.Col([html.Div("int16", style=LBL), html.Div(str(s16),style={"color":C_ACT})]),
                dbc.Col([html.Div("hex",   style=LBL), html.Div(f"0x{raw:04X}",style={"color":C_MUT})]),
                dbc.Col([html.Div("decoded",style=LBL),html.Div(dec,style={"color":C_POS})]),
            ], className="mt-1"),
        ])
    except Exception as e:
        add_log(classify_error(e),"MB_READ",str(e))
        return html.Span(f"❌ {e}", style={"color":C_REF})


# ── Write single register ──────────────────────────────────────────────────────
@app.callback(
    Output("mb-write-result","children"),
    Input("btn-mb-write","n_clicks"),
    State("mb-addr","value"), State("mb-write-val","value"),
    prevent_initial_call=True,
)
def on_mb_write(_, addr_str, val):
    if not connected:
        return html.Span("❌ not connected", style={"color":C_REF})
    addr = _parse_addr(addr_str)
    if addr is None:
        return html.Span("❌ address ไม่ถูกต้อง", style={"color":C_REF})
    raw = max(0, min(65535, int(val or 0)))
    try:
        with instrument_lock:
            instrument.write_register(addr, raw, functioncode=6)
        name = _reg_name(addr)
        dec  = _decode_reg(addr, raw)
        add_log("WRITE","MB_WRITE",f"0x{addr:02X} [{name}] ← {raw} (0x{raw:04X})  → {dec}")
        return html.Span(
            f"✅ 0x{addr:02X} [{name}] ← {raw}  decoded: {dec}",
            style={"color":C_ACT})
    except Exception as e:
        add_log(classify_error(e),"MB_WRITE",str(e))
        return html.Span(f"❌ {e}", style={"color":C_REF})


# ── Scan registers ─────────────────────────────────────────────────────────────
@app.callback(
    Output("mb-scan-result","children"),
    Input("btn-mb-scan","n_clicks"),
    State("mb-scan-start","value"), State("mb-scan-count","value"),
    prevent_initial_call=True,
)
def on_mb_scan(_, start_str, count):
    if not connected:
        return html.Span("❌ not connected", style={"color":C_REF})
    start = _parse_addr(start_str)
    if start is None:
        return html.Span("❌ start address ไม่ถูกต้อง", style={"color":C_REF})
    n = max(1, min(64, int(count or 22)))
    try:
        t0p = time.perf_counter()
        with instrument_lock:
            regs = instrument.read_registers(start, n, functioncode=3)
        rtt = (time.perf_counter() - t0p) * 1000
        add_log("INFO","MB_SCAN",
                f"0x{start:02X}..0x{start+n-1:02X} ({n} regs)  RTT={rtt:.1f}ms")

        rows = [html.Div(
            f"RTT={rtt:.1f} ms  |  {n} registers  0x{start:02X}–0x{start+n-1:02X}",
            style={"color":C_MUT,"marginBottom":"4px","fontSize":"10px"}
        )]
        for i, raw in enumerate(regs):
            addr  = start + i
            s16   = to_s16(raw)
            name  = _reg_name(addr)
            dec   = _decode_reg(addr, raw)
            known = addr in REG_META
            rows.append(html.Div([
                html.Span(f"0x{addr:02X} ",
                          style={"color":C_VIN if known else C_MUT,
                                 "minWidth":"38px","display":"inline-block"}),
                html.Span(f"{name:<14s}",
                          style={"color":C_POS if known else "#444",
                                 "minWidth":"110px","display":"inline-block"}),
                html.Span(f"{raw:>5d}  0x{raw:04X}  s16={s16:>6d}",
                          style={"color":C_TXT,"minWidth":"180px","display":"inline-block"}),
                html.Span(f"→ {dec}" if known else "",
                          style={"color":C_ACT}),
            ], style={"whiteSpace":"nowrap","borderBottom":"1px solid #111",
                      "paddingBottom":"1px","marginBottom":"1px"}))
        return rows
    except Exception as e:
        add_log(classify_error(e),"MB_SCAN",str(e))
        return html.Span(f"❌ {e}", style={"color":C_REF})


# ─────────────────────────────────────────────────────────────────────────────
#  KU SEARCH CALLBACKS
# ─────────────────────────────────────────────────────────────────────────────

@app.callback(
    Output("btn-ku-start","children"),
    Input("btn-ku-start","n_clicks"),
    State("ku-kp-start","value"), State("ku-kp-step","value"),
    State("ku-interval","value"),
    prevent_initial_call=True,
)
def on_ku_start(_, kp_s, kp_step, interval):
    if not connected:
        add_log("WARN","KU","not connected"); return "▶ Start"
    try:
        kp0 = float(kp_s or 10.0)
        step = float(kp_step or 0.5)
        iv   = float(interval or 4.0)
        with ku_lock:
            ku_search.update(
                active=True, kp_cur=kp0, kp_start=kp0,
                kp_step=step, interval_s=iv,
                t_step=(list(data["t"])[-1] if data["t"] else 0.0),
                ku=None, tu=None, oscillating=False, osc_amp=0.0,
            )
        # Force Ki=Kd=0, set starting Kp, enable run
        write_pos_pid(kp0, 0, 0)
        write_run_flag(True)
        add_log("INFO","KU",
                f"Search started  Kp0={kp0}  step={step}  interval={iv}s")
        return "⏳ Searching…"
    except Exception as e:
        add_log("ERROR","KU",str(e)); return "▶ Start"


@app.callback(
    Output("btn-ku-stop","children"),
    Input("btn-ku-stop","n_clicks"),
    prevent_initial_call=True,
)
def on_ku_stop(_):
    with ku_lock:
        ku_search["active"] = False
    try:
        write_run_flag(False)
        add_log("INFO","KU","Search stopped by user")
    except Exception: pass
    return "⏹ Stop"


@app.callback(
    Output("ku-apply-status","children"),
    Input("btn-ku-apply","n_clicks"),
    State("store-ku-result","data"),
    prevent_initial_call=True,
)
def on_ku_apply(_, store):
    if store is None:
        return html.Span("ยังไม่มีผล — กด Start ก่อน", style={"color":C_REF})
    kp, ki, kd = store["kp"], store["ki"], store["kd"]
    r = write_pos_pid(kp, ki, kd)
    if r == "ok":
        add_log("INFO","KU",
                f"Applied  Kp={kp:.2f}  Ki={ki:.3f}  Kd={kd:.3f}")
        return html.Span(
            f"✅ Applied  Kp={kp:.2f}  Ki={ki:.3f}  Kd={kd:.3f}",
            style={"color":C_ACT})
    return html.Span(f"Error: {r}", style={"color":C_REF})


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 60)
    print("  1-DOF Robot PID Tuner Dashboard")
    print("  http://127.0.0.1:8050")
    print("=" * 60)
    print(f"  COM ports: {list_ports()}")
    threading.Thread(target=read_loop, daemon=True).start()
    app.run(debug=False, host="127.0.0.1", port=8050)
