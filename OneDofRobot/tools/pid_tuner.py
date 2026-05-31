"""
================================================================================
  Velocity PID Tuner — 1-DOF Robot
  Real-time Monitor + Live PID Tuning ผ่าน Modbus RTU
================================================================================
  Install:  pip install minimalmodbus matplotlib pyserial
  Run:      python pid_tuner.py
================================================================================
"""

import time
import threading
import serial
import serial.tools.list_ports
import minimalmodbus
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button
from matplotlib.gridspec import GridSpec

# ─────────────────────────────────────────────────────────────────────────────
#  CONFIG — แก้ค่าตรงนี้
# ─────────────────────────────────────────────────────────────────────────────
SLAVE_ID   = 21
BAUD       = 19200
PARITY     = 'E'     # Even
TIMEOUT_S  = 0.15    # วินาที
POLL_HZ    = 20      # อ่านข้อมูล 20 ครั้ง/วิ
MAX_PTS    = 400     # จำนวนจุดบน graph

# Modbus Register Map
REG_KP           = 0x10   # Write: Kp × 100  (int16)
REG_KI           = 0x11   # Write: Ki × 100  (int16)
REG_KD           = 0x12   # Write: Kd × 100  (int16)
REG_TUNE_SPEED   = 0x13   # Write: speed × 10 (rad/s)
REG_TUNE_PERIOD  = 0x14   # Write: period (ms)
REG_REF_QD       = 0x20   # Read:  ref_qd × 100
REG_ACT_QD       = 0x21   # Read:  qd_out × 100
REG_V_IN         = 0x22   # Read:  V_in × 100

# ─────────────────────────────────────────────────────────────────────────────
#  STATE
# ─────────────────────────────────────────────────────────────────────────────
t_buf   = []
ref_buf = []
act_buf = []
vin_buf = []
buf_lock = threading.Lock()

instrument   = None
connected    = False
status_msg   = "ยังไม่ได้เชื่อมต่อ"
current_kp   = 7.0
current_ki   = 1.0
current_kd   = 0.0

# ─────────────────────────────────────────────────────────────────────────────
#  UTILS
# ─────────────────────────────────────────────────────────────────────────────
def to_signed16(val):
    """แปลง uint16 → signed int16"""
    return val - 65536 if val > 32767 else val


def list_ports():
    ports = serial.tools.list_ports.comports()
    return [p.device for p in ports]


# ─────────────────────────────────────────────────────────────────────────────
#  MODBUS CONNECT
# ─────────────────────────────────────────────────────────────────────────────
def connect(port):
    global instrument, connected, status_msg
    try:
        inst = minimalmodbus.Instrument(port, SLAVE_ID)
        inst.serial.baudrate = BAUD
        inst.serial.parity   = serial.PARITY_EVEN
        inst.serial.bytesize = 8
        inst.serial.stopbits = 1
        inst.serial.timeout  = TIMEOUT_S
        inst.mode = minimalmodbus.MODE_RTU
        instrument = inst
        connected  = True
        status_msg = f"✅ เชื่อมต่อสำเร็จ: {port}  {BAUD} 8E1"
        print(status_msg)
    except Exception as e:
        connected  = False
        status_msg = f"❌ เชื่อมต่อไม่ได้: {e}"
        print(status_msg)


# ─────────────────────────────────────────────────────────────────────────────
#  READ THREAD
# ─────────────────────────────────────────────────────────────────────────────
def read_loop():
    global status_msg
    t0 = time.time()
    while True:
        if connected and instrument:
            try:
                regs = instrument.read_registers(REG_REF_QD, 3, functioncode=3)
                ref = to_signed16(regs[0]) / 100.0
                act = to_signed16(regs[1]) / 100.0
                vin = to_signed16(regs[2]) / 100.0
                t_now = time.time() - t0

                with buf_lock:
                    t_buf.append(t_now)
                    ref_buf.append(ref)
                    act_buf.append(act)
                    vin_buf.append(vin)
                    if len(t_buf) > MAX_PTS:
                        t_buf.pop(0);  ref_buf.pop(0)
                        act_buf.pop(0); vin_buf.pop(0)

                status_msg = (f"ref={ref:+.2f}  act={act:+.2f}  "
                              f"Vin={vin:.1f}V  "
                              f"Kp={current_kp:.1f}  Ki={current_ki:.2f}")
            except Exception as e:
                status_msg = f"⚠️ อ่านไม่ได้: {e}"
        time.sleep(1.0 / POLL_HZ)


# ─────────────────────────────────────────────────────────────────────────────
#  WRITE PID
# ─────────────────────────────────────────────────────────────────────────────
def write_pid(kp, ki, kd):
    global current_kp, current_ki, current_kd
    if not connected or instrument is None:
        print("ยังไม่ได้เชื่อมต่อ")
        return
    current_kp, current_ki, current_kd = kp, ki, kd
    kp_raw = max(1, int(kp * 100))
    ki_raw = max(1, int(ki * 100))
    kd_raw = max(1, int(kd * 100))
    try:
        instrument.write_registers(REG_KP, [kp_raw, ki_raw, kd_raw], functioncode=16)
        print(f"→ เขียน PID: Kp={kp:.2f}  Ki={ki:.2f}  Kd={kd:.2f}")
    except Exception as e:
        print(f"❌ เขียนไม่ได้: {e}")


# ─────────────────────────────────────────────────────────────────────────────
#  GUI
# ─────────────────────────────────────────────────────────────────────────────
def build_gui():
    fig = plt.figure(figsize=(13, 8), facecolor='#1e1e2e')
    fig.canvas.manager.set_window_title("Velocity PID Tuner — 1-DOF Robot")

    gs = GridSpec(3, 1, figure=fig,
                  top=0.93, bottom=0.32,
                  hspace=0.35)

    # ── กราฟบน: Velocity Tracking ──────────────────────────────────────────
    ax_vel = fig.add_subplot(gs[0:2])
    ax_vel.set_facecolor('#2a2a3e')
    ax_vel.set_ylabel('Velocity (rad/s)', color='white')
    ax_vel.set_xlabel('Time (s)', color='white')
    ax_vel.set_title('Velocity Tracking', color='white', pad=4)
    ax_vel.tick_params(colors='white')
    for spine in ax_vel.spines.values(): spine.set_edgecolor('#555')
    ax_vel.grid(True, color='#333', linestyle='--', alpha=0.6)

    line_ref, = ax_vel.plot([], [], color='#ff6b6b', lw=1.5,
                            label='ref_qd (target)')
    line_act, = ax_vel.plot([], [], color='#51cf66', lw=1.8,
                            label='qd_out (actual)')
    ax_vel.legend(loc='upper right', facecolor='#2a2a3e',
                  labelcolor='white', fontsize=9)

    # ── กราฟล่าง: Voltage ─────────────────────────────────────────────────
    ax_vin = fig.add_subplot(gs[2])
    ax_vin.set_facecolor('#2a2a3e')
    ax_vin.set_ylabel('V_in (V)', color='white')
    ax_vin.set_xlabel('Time (s)', color='white')
    ax_vin.tick_params(colors='white')
    for spine in ax_vin.spines.values(): spine.set_edgecolor('#555')
    ax_vin.grid(True, color='#333', linestyle='--', alpha=0.6)
    ax_vin.set_ylim(-26, 26)

    line_vin, = ax_vin.plot([], [], color='#ffd43b', lw=1.2,
                             label='monitor_V_in')
    ax_vin.axhline(y=24,  color='#ff6b6b', lw=0.8, linestyle=':')
    ax_vin.axhline(y=-24, color='#ff6b6b', lw=0.8, linestyle=':')
    ax_vin.legend(loc='upper right', facecolor='#2a2a3e',
                  labelcolor='white', fontsize=9)

    # ── Status Text ───────────────────────────────────────────────────────
    txt_status = fig.text(0.01, 0.965, status_msg,
                          color='#aaa', fontsize=8.5,
                          transform=fig.transFigure)

    # ─────────────────────────────────────────────────────────────────────
    #  SLIDERS
    # ─────────────────────────────────────────────────────────────────────
    slider_color = '#3a3a5c'
    active_color = '#74c0fc'

    ax_kp = fig.add_axes([0.10, 0.22, 0.75, 0.025], facecolor=slider_color)
    ax_ki = fig.add_axes([0.10, 0.17, 0.75, 0.025], facecolor=slider_color)
    ax_kd = fig.add_axes([0.10, 0.12, 0.75, 0.025], facecolor=slider_color)

    sl_kp = Slider(ax_kp, 'Kp', 0.0, 30.0,  valinit=current_kp,
                   color=active_color, initcolor='none')
    sl_ki = Slider(ax_ki, 'Ki', 0.0,  5.0,  valinit=current_ki,
                   color=active_color, initcolor='none')
    sl_kd = Slider(ax_kd, 'Kd', 0.0,  1.0,  valinit=current_kd,
                   color=active_color, initcolor='none')

    for sl in (sl_kp, sl_ki, sl_kd):
        sl.label.set_color('white')
        sl.valtext.set_color('#ffd43b')

    def on_slider_changed(_):
        write_pid(sl_kp.val, sl_ki.val, sl_kd.val)

    sl_kp.on_changed(on_slider_changed)
    sl_ki.on_changed(on_slider_changed)
    sl_kd.on_changed(on_slider_changed)

    # ─────────────────────────────────────────────────────────────────────
    #  BUTTONS
    # ─────────────────────────────────────────────────────────────────────
    # ปุ่ม Reset Integral
    ax_reset = fig.add_axes([0.10, 0.05, 0.18, 0.045])
    btn_reset = Button(ax_reset, 'Reset PID\nIntegral',
                       color='#5c3a3a', hovercolor='#8b4444')
    btn_reset.label.set_color('white')
    btn_reset.label.set_fontsize(8)

    def on_reset(_):
        if connected and instrument:
            try:
                # เขียน 0 ชั่วคราวเพื่อ trigger reset integral
                # (ค่า 0 จะถูก ignore ใน STM32 → ไม่อัปเดต gain)
                # แต่เราใช้วิธีเขียนซ้ำค่าปัจจุบันแทน
                write_pid(sl_kp.val, sl_ki.val, sl_kd.val)
                print("→ Reset integral (เขียนค่า PID ใหม่)")
            except Exception as e:
                print(f"Reset error: {e}")

    btn_reset.on_clicked(on_reset)

    # ปุ่ม Auto-connect COM ports
    ax_conn = fig.add_axes([0.32, 0.05, 0.22, 0.045])
    available = list_ports()
    port_label = '\n'.join(available) if available else 'ไม่พบ COM port'
    btn_conn = Button(ax_conn, f'Connect\n{available[0] if available else "—"}',
                      color='#1a4a2e', hovercolor='#2d7a4a')
    btn_conn.label.set_color('white')
    btn_conn.label.set_fontsize(8)

    def on_connect(_):
        ports = list_ports()
        if ports:
            connect(ports[0])
            btn_conn.label.set_text(f"Connected\n{ports[0]}")
            fig.canvas.draw_idle()

    btn_conn.on_clicked(on_connect)

    # ปุ่ม Clear Graph
    ax_clear = fig.add_axes([0.58, 0.05, 0.15, 0.045])
    btn_clear = Button(ax_clear, 'Clear\nGraph',
                       color='#2a2a4a', hovercolor='#4a4a6a')
    btn_clear.label.set_color('white')
    btn_clear.label.set_fontsize(8)

    def on_clear(_):
        with buf_lock:
            t_buf.clear(); ref_buf.clear()
            act_buf.clear(); vin_buf.clear()

    btn_clear.on_clicked(on_clear)

    # ─────────────────────────────────────────────────────────────────────
    #  ANIMATION
    # ─────────────────────────────────────────────────────────────────────
    def animate(_):
        with buf_lock:
            if len(t_buf) < 2:
                return line_ref, line_act, line_vin

            t   = list(t_buf)
            ref = list(ref_buf)
            act = list(act_buf)
            vin = list(vin_buf)

        line_ref.set_data(t, ref)
        line_act.set_data(t, act)
        line_vin.set_data(t, vin)

        # Auto-scale X
        t_min, t_max = t[0], t[-1]
        span = max(t_max - t_min, 10.0)
        ax_vel.set_xlim(t_max - span, t_max)
        ax_vin.set_xlim(t_max - span, t_max)

        # Auto-scale Y velocity
        all_v = ref + act
        if all_v:
            v_pad = max(abs(max(all_v)), abs(min(all_v)), 1.0) * 1.3
            ax_vel.set_ylim(-v_pad, v_pad)

        txt_status.set_text(status_msg)
        return line_ref, line_act, line_vin, txt_status

    ani = animation.FuncAnimation(fig, animate, interval=80,
                                  blit=False, cache_frame_data=False)

    plt.show()
    return ani


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("=" * 60)
    print("  Velocity PID Tuner — 1-DOF Robot")
    print("=" * 60)

    # แสดง COM ports ที่มี
    ports = list_ports()
    print(f"COM Ports ที่พบ: {ports}")

    # Auto-connect พอร์ตแรก
    if ports:
        connect(ports[0])
    else:
        print("⚠️  ไม่พบ COM Port — กด Connect ในหน้าต่าง GUI")

    # เริ่ม read thread
    t = threading.Thread(target=read_loop, daemon=True)
    t.start()

    # เปิด GUI
    ani = build_gui()
