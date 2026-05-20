#!/usr/bin/env python3
"""
fs_monitor.py — GCS Failsafe Monitor
Телеметрія: SITL serial1 TCP:5761 (завжди живий)
GCS detection: heartbeat від sysid=255 через той самий потік (не працює з serial1)
               → використовуємо окремий UDP:14560 від MAVProxy

Запуск MAVProxy:
  mavproxy.py --master=tcp:127.0.0.1:5760 --out=udp:192.168.112.1:14550 --out=udp:127.0.0.1:14560

Запуск монітора:
  python3 fs_monitor.py
"""

import time
import sys
import math
import threading
from datetime import datetime
from pymavlink import mavutil

TELEMETRY_HOST = "127.0.0.1"
TELEMETRY_PORT = 5761       # serial1 — завжди живий

GCS_DETECT_HOST = "127.0.0.1"
GCS_DETECT_PORT = 14560     # UDP від MAVProxy — для визначення втрати GCS

GCS_FS_TIMEOUT_S = 5.0

MODE_NAMES = {
    0:  "STABILIZE",    1:  "ACRO",         2:  "ALT_HOLD",
    3:  "AUTO",         4:  "GUIDED",        5:  "LOITER",
    6:  "RTL",          7:  "CIRCLE",        9:  "LAND",
    11: "DRIFT",        13: "SPORT",         16: "POSHOLD",
    17: "BRAKE",        20: "GUIDED_NOGPS",  21: "SMART_RTL",
}

R="\033[91m"; G="\033[92m"; Y="\033[93m"; C="\033[96m"
GR="\033[90m"; B="\033[1m"; N="\033[0m"

def ts():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

def mode_str(m):
    name = MODE_NAMES.get(m, f"MODE_{m}")
    if m == 9:          return f"{R}{name}{N}"
    if m == 20:         return f"{C}{name}{N}"
    if m in (0, 1, 2):  return f"{Y}{name}{N}"
    return name

# ── Shared state ───────────────────────────────────────────────────────────────
state = {
    "armed":      False,
    "mode":       -1,
    "alt":        0.0,
    "climb":      0.0,
    "roll":       0.0,
    "pitch":      0.0,
    "yaw":        0.0,
    "gcs_last_s": 0.0,
}
state_lock = threading.Lock()

# ── Thread 1: телеметрія з serial1 ────────────────────────────────────────────
def telemetry_thread():
    while True:
        try:
            print(f"{GR}[{ts()}] Підключення до TCP:{TELEMETRY_HOST}:{TELEMETRY_PORT}...{N}")
            conn = mavutil.mavlink_connection(
                f"tcp:{TELEMETRY_HOST}:{TELEMETRY_PORT}", source_system=254)
            print(f"{G}[{ts()}] ✓ Телеметрія підключена{N}")
        except Exception as e:
            print(f"{R}[{ts()}] Телеметрія: {e} — retry 2s{N}")
            time.sleep(2)
            continue

        prev_mode  = -1
        prev_armed = False

        while True:
            # Дренуємо весь буфер за один прохід
            while True:
                try:
                    msg = conn.recv_match(blocking=False)
                except Exception as e:
                    print(f"{R}[{ts()}] Телеметрія розірвана: {e}{N}")
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
                        state["alt"] = msg.relative_alt / 1000.0

                    elif t == "VFR_HUD":
                        state["climb"] = msg.climb

                    elif t == "ATTITUDE":
                        state["roll"]  = math.degrees(msg.roll)
                        state["pitch"] = math.degrees(msg.pitch)
                        state["yaw"]   = math.degrees(msg.yaw)

                    elif t == "STATUSTEXT":
                        text = msg.text.rstrip('\x00').strip()
                        sev  = msg.severity
                        col  = R if sev<=3 else Y if sev<=4 else C if sev<=5 else GR
                        print(f"{col}[{ts()}] MSG: {text}{N}")

                # Зміни mode/armed виводимо одразу
                with state_lock:
                    cur_mode  = state["mode"]
                    cur_armed = state["armed"]

                if cur_mode != prev_mode and prev_mode != -1:
                    old = MODE_NAMES.get(prev_mode, str(prev_mode))
                    new = MODE_NAMES.get(cur_mode,  str(cur_mode))
                    print(f"{Y}[{ts()}] MODE: {old} → {new}{N}")
                prev_mode = cur_mode

                if cur_armed != prev_armed:
                    print(f"{G}[{ts()}] {'✓ ARMED' if cur_armed else 'DISARMED'}{N}")
                prev_armed = cur_armed

            time.sleep(0.01)

# ── Thread 2: GCS heartbeat detection через UDP:14560 ─────────────────────────
def gcs_detect_thread():
    while True:
        try:
            print(f"{GR}[{ts()}] GCS detect: UDP:{GCS_DETECT_HOST}:{GCS_DETECT_PORT}...{N}")
            conn = mavutil.mavlink_connection(
                f"udpin:{GCS_DETECT_HOST}:{GCS_DETECT_PORT}", source_system=254)
            print(f"{G}[{ts()}] ✓ GCS detect підключено{N}")
        except Exception as e:
            print(f"{R}[{ts()}] GCS detect: {e} — retry 2s{N}")
            time.sleep(2)
            continue

        while True:
            try:
                msg = conn.recv_match(blocking=False)
            except Exception as e:
                print(f"{R}[{ts()}] GCS detect розірвано: {e}{N}")
                break
            if msg and msg.get_type() == "HEARTBEAT":
                if msg.get_srcSystem() == 255:
                    with state_lock:
                        state["gcs_last_s"] = time.monotonic()
            time.sleep(0.05)

# ── Main: вивід статусу ────────────────────────────────────────────────────────
def main():
    print(f"\n{B}{C}{'━'*56}")
    print(f"  fs_monitor  |  telemetry=TCP:{TELEMETRY_PORT}  gcs=UDP:{GCS_DETECT_PORT}")
    print(f"{'━'*56}{N}\n")

    t1 = threading.Thread(target=telemetry_thread, daemon=True)
    t2 = threading.Thread(target=gcs_detect_thread, daemon=True)
    t1.start()
    t2.start()

    fs_active = False
    fs_since  = 0.0

    while True:
        time.sleep(0.5)
        now = time.monotonic()

        with state_lock:
            armed     = state["armed"]
            mode      = state["mode"]
            alt       = state["alt"]
            climb     = state["climb"]
            roll      = state["roll"]
            pitch     = state["pitch"]
            yaw       = state["yaw"]
            gcs_last  = state["gcs_last_s"]

        # GCS failsafe detection
        if gcs_last > 0:
            gap    = now - gcs_last
            fs_now = gap > GCS_FS_TIMEOUT_S

            if fs_now and not fs_active:
                fs_active = True
                fs_since  = now
                print(f"\n{R}{B}[{ts()}] ⚠  GCS LINK LOST  (gap={gap:.1f}s){N}\n")
            elif not fs_now and fs_active:
                fs_active = False
                dur = now - fs_since
                print(f"\n{G}{B}[{ts()}] ✓  GCS LINK RESTORED  (втрата {dur:.1f}s){N}\n")

        # Рядок статусу
        arm_s = f"{G}ARMED {N}" if armed else f"{GR}DISARM{N}"

        if gcs_last == 0:
            gcs_s = f"{GR}GCS?{N}"
        elif fs_active:
            gcs_s = f"{R}FS={now-gcs_last:.1f}s{N}"
        else:
            gcs_s = f"{G}GCS+{now-gcs_last:.1f}s{N}"

        print(
            f"[{ts()}] {arm_s} "
            f"mode={mode_str(mode):<22}"
            f"alt={alt:+7.1f}m  "
            f"vz={climb:+5.1f}m/s  "
            f"r={roll:+6.1f}  p={pitch:+6.1f}  y={yaw:+7.1f}  "
            f"{gcs_s}"
        )

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n{GR}Monitor зупинено.{N}")
        sys.exit(0)
