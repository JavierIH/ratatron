#!/bin/bash
# straight_battery.sh speed...: per speed a CAL TURN 2 and a CAL STRAIGHT 3, then the centring summary (layout I).
cd "$(dirname "$0")/.."
for v in "$@"; do
    software/robot.sh -w "data saved" -t 25 "CAL TURN 2" | grep -E "!!"
    software/robot.sh -w "data saved" -t 25 "CAL STRAIGHT 3 $v" | grep -E "forward|!!"
    python3 software/calib_analyze.py "$(ls -t software/calib_data/*straight*.csv | head -1)" | grep -E "centring:|per 45"
done
