#!/bin/bash

mqtt_broker="192.168.11.100"


mosquitto_pub -h $mqtt_broker -t "arena/sparknode07/cmd" -m "config set calibration 1 1.05"
sleep 2


echo "Starting square cycle..."
echo "Press Ctrl+C to stop"


# # #######   turn right and square
# cycle=1
# while true; do
# 	echo "=== Break-in Cycle $cycle ==="
# 	echo "Turning right..."
# 	mosquitto_pub -h $mqtt_broker -t "arena/sparknode07/cmd" -m "config set calibration 1 1.05"
# 	mosquitto_pub -h $mqtt_broker -t "arena/sparknode07/cmd" -m "turn right 30 15"
# 	sleep 2
# 	echo "Adjusting castor wheels..."
# 	mosquitto_pub -h $mqtt_broker -t "arena/sparknode07/cmd" -m "config set calibration 1 1.45"
# 	mosquitto_pub -h $mqtt_broker -t "arena/sparknode07/cmd" -m "drive forward 10 1500"
# 	sleep 2.5
# 	echo "Cycle $cycle is complete"
# 	((cycle++))
# done

# ########   turn left and square
# cycle=1
# while true; do
# 	echo "=== Break-in Cycle $cycle ==="
# 	echo "Turning left..."
# 	mosquitto_pub -h $mqtt_broker -t "arena/sparknode07/cmd" -m "config set calibration 1 1.05"
# 	mosquitto_pub -h $mqtt_broker -t "arena/sparknode07/cmd" -m "turn left 30 15"
# 	sleep 2
# 	echo "Adjusting castor wheels..."
# 	mosquitto_pub -h $mqtt_broker -t "arena/sparknode07/cmd" -m "config set calibration 1 1.2"
# 	mosquitto_pub -h $mqtt_broker -t "arena/sparknode07/cmd" -m "drive forward 10 1600"
# 	sleep 2.5
# 	echo "Cycle $cycle is complete"
# 	((cycle++))
# done
