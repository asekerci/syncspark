#!/bin/bash
mqtt_broker="192.168.11.100"

mosquitto_pub -h $mqtt_broker -t "arena/all/cmd" -m "config show"

echo "Starting turn test: 8 right turns, then 8 left turns (repeating until stopped)"
echo "Broadcasting to ALL sparknodes - Press Ctrl+C to stop the test"

cycle=1
while true; do
	echo "=== Cycle $cycle ==="
	echo "Performing 8 right turns..."
	for i in {1..8}; do
		echo "Right turn $i/8"
		mosquitto_pub -h $mqtt_broker -t "arena/all/cmd" -m "turn right 20 90"
		sleep 1
	done
	sleep 2
	echo "Performing 8 left turns..."
	for i in {1..8}; do
		echo "Left turn $i/8"
		mosquitto_pub -h $mqtt_broker -t "arena/all/cmd" -m "turn left 20 90"
		sleep 1
	done
	echo "Cycle $cycle completed!"
	sleep 2
	((cycle++))
done
