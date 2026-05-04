#!/bin/bash
mqtt_broker="192.168.11.100"

mosquitto_pub -h $mqtt_broker -t "arena/all/cmd" -m "config show"

echo "Starting turn test: 90 degree turns (repeating until stopped)"
echo "Broadcasting to ALL sparknodes - Press Ctrl+C to stop the test"

cycle=1
while true; do
	echo "=== Cycle $cycle: Starting from 0 degrees ==="
	mosquitto_pub -h $mqtt_broker -t "arena/all/cmd" -m "turn to 0 20"
	sleep 10
#	mosquitto_pub -h $mqtt_broker -t "arena/all/cmd" -m "turn to 90 40"
#	sleep 10
	mosquitto_pub -h $mqtt_broker -t "arena/all/cmd" -m "turn to 180 20"
	sleep 10
#	mosquitto_pub -h $mqtt_broker -t "arena/all/cmd" -m "turn to 270 40"
#	sleep 10
	echo "Cycle $cycle is complete"
	((cycle++))
done
