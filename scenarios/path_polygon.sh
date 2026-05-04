#!/bin/bash
mqtt_broker="192.168.11.100"
sparknode="sparknode11"

#mosquitto_pub -h $mqtt_broker -t "arena/sparknode03/cmd" -m "calibrate_mag"
#sleep 20
#mosquitto_pub -h $mqtt_broker -t "arena/sparknode03/cmd" -m "calibrate_gyro"
#sleep 20

# dagu motors may not need any kicks
mosquitto_pub -h $mqtt_broker -t "arena/sparknode03/cmd" -m "config set turn_kick 0 0"
sleep 1
mosquitto_pub -h $mqtt_broker -t "arena/sparknode03/cmd" -m "config set drive_kick 0 0"
sleep 1

mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "config show"
sleep 2

echo "Starting the polygon pattern trajectory cycle..."
echo "Press Ctrl+C to stop"

cycle=1
while true; do
	echo "=== Cycle $cycle ==="
	echo "Driving forward..."
	mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "drive forward 20 500"
	sleep 2
	echo "Right turn..."
	mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "turn right 20 300"
	sleep 1
	echo "Cycle $cycle is complete"
	((cycle++))
done

