#!/usr/bin/env python3
"""
fs_monitor.py — Drone Telemetry Monitor
Підключення: UDP:14561 (додай --out=udp:127.0.0.1:14561 до sim_vehicle.py)

Запуск SITL:
  sim_vehicle.py -v ArduCopter --console --out=udp:127.0.0.1:14560 --out=udp:127.0.0.1:14561

Запуск монітора:
  python3 fs_monitor.py
"""

import time
import sys
import math
import threading
from datetime import datetime
from pymavlink import mavutil

TELEM_HOST = "127.0.0.1"
TELEM_PORT = 14561

MODE_NAMES = {
    0:  "STABILIZE",
    1:  "ACRO",
    2:  "ALT_HOLD",
    3:  "AUTO",
    4:  "GUIDED",
    5:  "LOITER",
    6:  "RTL",
    7:  "CIRCLE",
    9:  "LAND",
    11: "DRIFT",
    13: "SPORT",
    14: "FLIP",
    15: "AUTOTUNE",
    16: "POSHOLD",
    17: "BRAKE",
    18: "THROW",
    20: "GUIDED_NOGPS",
    21: "SMART_RTL",
    22: "FLOWHOLD",
}

# ANSI colors
R  = "\033[91m"
G  = "\033[92m"
Y  = "\033[93m"
C  = "\033[96m"
GR = "\033[90m"
B  = "\033[1m"
N  = "\033[0m"

def ts():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

def mode_str(m):
    name = MODE_NAMES.get(m, f"MODE_{m}")
    if m == 9:              return f"{R}{B}{name}{N}"   # LAND — червоний
    if m == 17:             return f"{C}{name}{N}"      # BRAKE — cyan
    if m == 20:             return f"{C}{name}{N}"      # GUIDED_NOGPS — cyan
    if m == 5:              return f"{G}{name}{N}"      # LOITER — зелений
    if m in (0, 1, 2):      return f"{Y}{name}{N}"      # manual — жовтий
    return name

# ── Shared state ──────────────────────────────────────────────────────────────
state = {
    "armed":        False,
    "mode":         -1,
    "alt":          0.0,
    "climb":        0.0,
    "roll":         0.0,
    "pitch":        0.0,
    "yaw":          0.0,
    "vx":           0.0,   # m/s north
    "vy":           0.0,   # m/s east
    "speed_xy":     0.0,   # horizontal speed m/s
    "airspeed":     0.0,
    "throttle":     0,
    "bat_v":        0.0,
    "bat_pct":      -1,
    "fs_radio":     False,  # radio failsafe flag from SYS_STATUS
}
state_lock = threading.Lock()

# ── Telemetry thread ──────────────────────────────────────────────────────────
def telemetry_thread():
    while True:
        try:
            print(f"{GR}[{ts()}] Підключення UDP:{TELEM_HOST}:{TELEM_PORT}...{N}")
            conn = mavutil.mavlink_connection(
                f"udpin:{TELEM_HOST}:{TELEM_PORT}", source_system=254)
            print(f"{G}[{ts()}] ✓ Телеметрія підключена{N}")
        except Exception as e:
            print(f"{R}[{ts()}] Помилка: {e} — retry 2s{N}")
            time.sleep(2)
            continue

        prev_mode  = -1
        prev_armed = False

        while True:
            while True:
                try:
                    msg = conn.recv_match(blocking=False)
                except Exception as e:
                    print(f"{R}[{ts()}] Розрив: {e}{N}")
                    break
                if msg is None:
                    break

                t = msg.get_type()
                with state_lock:
                    if t == "HEARTBEAT":
                        state["armed"] = bool(
                            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        state["mode"] = msg.custom_mode

                    elif t == "GLOBAL_POSITION_INT":
                        state["alt"]      = msg.relative_alt / 1000.0
                        state["vx"]       = msg.vx / 100.0
                        state["vy"]       = msg.vy / 100.0
                        state["speed_xy"] = math.sqrt(
                            (msg.vx/100.0)**2 + (msg.vy/100.0)**2)

                    elif t == "VFR_HUD":
                        state["climb"]    = msg.climb
                        state["airspeed"] = msg.airspeed
                        state["throttle"] = msg.throttle

                    elif t == "ATTITUDE":
                        state["roll"]  = math.degrees(msg.roll)
                        state["pitch"] = math.degrees(msg.pitch)
                        state["yaw"]   = math.degrees(msg.yaw)

                    elif t == "BATTERY_STATUS":
                        if msg.voltages[0] != 65535:
                            state["bat_v"]   = msg.voltages[0] / 1000.0
                        state["bat_pct"] = msg.battery_remaining

                    elif t == "SYS_STATUS":
                        # bit 11 = MAV_SYS_STATUS_SENSOR_RC_RECEIVER
                        # якщо присутній але не healthy — radio failsafe
                        present = msg.onboard_control_sensors_present
                        health  = msg.onboard_control_sensors_health
                        rc_bit  = (1 << 11)
                        if (present & rc_bit):
                            state["fs_radio"] = not bool(health & rc_bit)

                    elif t == "STATUSTEXT":
                        text = msg.text.rstrip('\x00').strip()
                        sev  = msg.severity
                        col  = R if sev <= 3 else Y if sev <= 4 else C if sev <= 5 else GR
                        print(f"{col}[{ts()}] ▶ {text}{N}")

                with state_lock:
                    cur_mode  = state["mode"]
                    cur_armed = state["armed"]

                if cur_mode != prev_mode and prev_mode != -1:
                    old = MODE_NAMES.get(prev_mode, str(prev_mode))
                    new_m = MODE_NAMES.get(cur_mode, str(cur_mode))
                    col = R if cur_mode == 9 else G if cur_mode == 5 else Y
                    print(f"{col}{B}[{ts()}] ══ MODE: {old} → {new_m} ══{N}")
                prev_mode = cur_mode

                if cur_armed != prev_armed:
                    col = G if cur_armed else Y
                    print(f"{col}{B}[{ts()}] {'▲ ARMED' if cur_armed else '▼ DISARMED'}{N}")
                prev_armed = cur_armed

            time.sleep(0.01)

# ── Main loop ─────────────────────────────────────────────────────────────────
def main():
    print(f"\n{B}{C}{'━'*70}")
    print(f"  fs_monitor  |  UDP:{TELEM_PORT}")
    print(f"{'━'*70}{N}\n")

    t1 = threading.Thread(target=telemetry_thread, daemon=True)
    t1.start()

    prev_fs = False

    while True:
        time.sleep(1)

        with state_lock:
            armed    = state["armed"]
            mode     = state["mode"]
            alt      = state["alt"]
            climb    = state["climb"]
            roll     = state["roll"]
            pitch    = state["pitch"]
            yaw      = state["yaw"]
            speed_xy = state["speed_xy"]
            airspeed = state["airspeed"]
            throttle = state["throttle"]
            bat_v    = state["bat_v"]
            bat_pct  = state["bat_pct"]
            fs_radio = state["fs_radio"]

        # Radio failsafe banner
        if fs_radio and not prev_fs:
            print(f"\n{R}{B}[{ts()}] ⚠  RADIO FAILSAFE ACTIVE{N}\n")
        elif not fs_radio and prev_fs:
            print(f"\n{G}{B}[{ts()}] ✓  RADIO FAILSAFE CLEARED{N}\n")
        prev_fs = fs_radio

        arm_s  = f"{G}ARMED {N}" if armed else f"{GR}DISARM{N}"
        fs_s   = f" {R}[RC-FS]{N}" if fs_radio else ""
        bat_s  = (f"{bat_v:.1f}V" if bat_v > 0 else "?V")
        if bat_pct >= 0:
            bat_col = G if bat_pct > 50 else Y if bat_pct > 20 else R
            bat_s += f" {bat_col}{bat_pct}%{N}"

        print(
            f"[{ts()}] {arm_s}{fs_s} "
            f"mode={mode_str(mode):<28}"
            f"alt={alt:+7.2f}m  "
            f"vz={climb:+5.2f}  "
            f"vxy={speed_xy:5.2f}m/s  "
            f"thr={throttle:3d}%  "
            f"r={roll:+6.1f}  p={pitch:+6.1f}  y={yaw:+7.1f}  "
            f"bat={bat_s}"
        )

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n{GR}Monitor зупинено.{N}")
        sys.exit(0)