#!/bin/bash
mqtt_broker="192.168.11.100"
sparknode="sparknode11"

#mosquitto_pub -h $mqtt_broker -t "arena/sparknode03/cmd" -m "calibrate_mag"
#sleep 20
#mosquitto_pub -h $mqtt_broker -t "arena/sparknode03/cmd" -m "calibrate_gyro"
#sleep 20

#mosquitto_pub -h $mqtt_broker -t "arena/sparknode03/cmd" -m "config set turn_kick 40 20"
#sleep 1
#mosquitto_pub -h $mqtt_broker -t "arena/sparknode03/cmd" -m "config set drive_kick 50 100"
#sleep 1

# dagu motors may not need any kicks
mosquitto_pub -h $mqtt_broker -t "arena/sparknode03/cmd" -m "config set turn_kick 0 0"
sleep 1
mosquitto_pub -h $mqtt_broker -t "arena/sparknode03/cmd" -m "config set drive_kick 0 0"
sleep 1

mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "config show"
sleep 2

echo "Starting forward/reverse cycles..."
echo "Press Ctrl+C to stop"
cycle=1
while true; do
	echo "=== Cycle $cycle ==="
	echo "Driving forward..."
	mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "drive forward 30 500"
	sleep 2

	echo "Driving reverse..."
	mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "drive reverse 30 500"
	sleep 2

	echo "Cycle $cycle is complete"
	((cycle++))
	sleep 1
done

