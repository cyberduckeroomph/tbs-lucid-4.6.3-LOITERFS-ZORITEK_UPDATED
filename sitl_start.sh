#!/bin/bash
# sitl_start.sh
# Запускає SITL, MAVProxy і fs_monitor в окремих вікнах терміналу.
#
# Схема:
#   SITL TCP:5760 ← MAVProxy → UDP:192.168.112.1:14550 → Mission Planner
#   SITL TCP:5761 ← fs_monitor (завжди живий, незалежно від MAVProxy)
#
# Імітація втрати GCS: закрити вікно MAVProxy або Ctrl+C в ньому.

# ── Налаштування ──────────────────────────────────────────────────────────────
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
VENV_DIR="${VENV_DIR:-$HOME/venv-ardupilot}"
WINDOWS_IP="${WINDOWS_IP:-192.168.112.1}"   # ip addr show eth0 → SourceAddress
MP_UDP_PORT=14550

COPTER_DIR="$ARDUPILOT_DIR/ArduCopter"
SIM_VEHICLE="$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py"
MAVPROXY="$VENV_DIR/bin/mavproxy.py"
MONITOR="$(dirname "$(realpath "$0")")/fs_monitor.py"

# ── Кольори ───────────────────────────────────────────────────────────────────
R="\033[91m"; G="\033[92m"; Y="\033[93m"; GR="\033[90m"; N="\033[0m"
ok()   { echo -e "${G}[$(date +%H:%M:%S)] ✓  $*${N}"; }
err()  { echo -e "${R}[$(date +%H:%M:%S)] ✗  $*${N}"; exit 1; }
info() { echo -e "${GR}[$(date +%H:%M:%S)] $*${N}"; }

# ── Перевірки ─────────────────────────────────────────────────────────────────
[ -f "$SIM_VEHICLE" ] || err "sim_vehicle.py не знайдено: $SIM_VEHICLE"
[ -f "$MAVPROXY"    ] || err "mavproxy.py не знайдено: $MAVPROXY"
[ -f "$MONITOR"     ] || err "fs_monitor.py не знайдено: $MONITOR"

command -v gnome-terminal &>/dev/null || command -v xterm &>/dev/null \
    || err "Потрібен gnome-terminal або xterm"

# ── Функція відкриття вікна ───────────────────────────────────────────────────
open_term() {
    local title="$1"; shift
    if command -v gnome-terminal &>/dev/null; then
        gnome-terminal --title="$title" -- bash -c "$*; exec bash" &
    else
        xterm -title "$title" -e bash -c "$*; exec bash" &
    fi
}

# ── Вікно 1: SITL ─────────────────────────────────────────────────────────────
info "Запуск SITL..."
open_term "SITL" \
    "cd '$COPTER_DIR' && python3 '$SIM_VEHICLE' -v ArduCopter --no-mavproxy --console --map -A '--serial1=tcp:5761'"

info "Чекаємо TCP:5760..."
elapsed=0
until nc -z 127.0.0.1 5760 2>/dev/null; do
    sleep 1
    elapsed=$((elapsed+1))
    [ $((elapsed % 5)) -eq 0 ] && info "  ...${elapsed}s"
done
ok "SITL готовий (TCP:5760, TCP:5761)"

# ── Вікно 2: MAVProxy ─────────────────────────────────────────────────────────
info "Запуск MAVProxy → UDP:${WINDOWS_IP}:${MP_UDP_PORT}..."
open_term "MAVProxy" \
    "python3 '$MAVPROXY' --master=tcp:127.0.0.1:5760 --out=udp:${WINDOWS_IP}:${MP_UDP_PORT} --console --map"
sleep 1
ok "MAVProxy запущено"

# ── Вікно 3: Monitor ─────────────────────────────────────────────────────────
info "Запуск fs_monitor (TCP:5761)..."
open_term "fs_monitor" \
    "python3 '$MONITOR'"
sleep 1
ok "Monitor запущено"

echo ""
echo -e "${G}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${N}"
echo -e "  Mission Planner: UDP  ${WINDOWS_IP}:${MP_UDP_PORT}"
echo -e "  Імітація втрати GCS:  закрий вікно MAVProxy"
echo -e "${G}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${N}"

# ── Інтерактивні команди ──────────────────────────────────────────────────────
MAVPROXY_PID=""

# Знайти PID MAVProxy
find_mavproxy_pid() {
    MAVPROXY_PID=$(pgrep -f "mavproxy.py.*5760" | head -1)
}

kill_mavproxy() {
    find_mavproxy_pid
    if [ -z "$MAVPROXY_PID" ]; then
        echo -e "${Y}MAVProxy не знайдено${N}"
        return
    fi
    kill "$MAVPROXY_PID" 2>/dev/null
    echo -e "${R}[$(date +%H:%M:%S)] ⚠  GCS LINK LOST (PID=$MAVPROXY_PID killed)${N}"
    MAVPROXY_PID=""
}

restore_mavproxy() {
    find_mavproxy_pid
    if [ -n "$MAVPROXY_PID" ]; then
        echo -e "${Y}MAVProxy вже запущений (PID=$MAVPROXY_PID)${N}"
        return
    fi
    open_term "MAVProxy" \
        "python3 '$MAVPROXY' --master=tcp:127.0.0.1:5760 --out=udp:${WINDOWS_IP}:${MP_UDP_PORT} --console --map"
    echo -e "${G}[$(date +%H:%M:%S)] ✓  MAVProxy відновлено${N}"
}

echo ""
echo -e "  Команди: ${G}kill <N>${N} — вбити на N секунд  |  ${G}kill${N} — назавжди  |  ${G}restore${N}  |  ${G}quit${N}"
echo ""

while true; do
    printf "> "
    read -r line
    cmd=$(echo "$line" | awk '{print $1}')
    arg=$(echo "$line" | awk '{print $2}')

    case "$cmd" in
        kill)
            kill_mavproxy
            if [ -n "$arg" ] && [ "$arg" -gt 0 ] 2>/dev/null; then
                echo -e "${GR}Відновлення через ${arg}s...${N}"
                (
                    sleep "$arg"
                    python3 "$MAVPROXY" \
                        --master=tcp:127.0.0.1:5760 \
                        --out=udp:${WINDOWS_IP}:${MP_UDP_PORT} \
                        --console --map &
                    echo -e "\n${G}[$(date +%H:%M:%S)] ✓  MAVProxy авто-відновлено після ${arg}s${N}"
                ) &
            fi
            ;;
        restore)
            restore_mavproxy
            ;;
        quit|exit|q)
            echo "Завершення..."
            pkill -f "arducopter" 2>/dev/null
            pkill -f "mavproxy.py" 2>/dev/null
            pkill -f "fs_monitor.py" 2>/dev/null
            break
            ;;
        "")
            ;;
        *)
            echo -e "${Y}Невідома команда. kill <N> | kill | restore | quit${N}"
            ;;
    esac
done
