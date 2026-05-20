#!/bin/bash
export PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter --no-mavproxy & SIM_PID=$!

echo "Waiting for SITL on tcp:127.0.0.1:5760..."
until nc -z 127.0.0.1 5760 2>/dev/null; do
	sleep 0.5
done
echo "SITL ready. Launching MAVProxy..."

mavproxy.py --master=tcp:127.0.0.1:5760 --out=udp:127.0.0.1:14550

kill $SIM_PID 2>/dev/null
