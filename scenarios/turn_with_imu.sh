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

echo "Starting the cycle..."
echo "Press Ctrl+C to stop"

cycle=1
angle=0
while true; do
	echo "=== Cycle $cycle ==="
	
	for i in {1..4}; do
		echo "Turning to $angle degrees..."
		mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "turn to $angle 30"
		sleep 10
		mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "show heading"
		sleep 1
#		echo "Driving forward..."
#		mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "drive forward 20 400"
#		sleep 2
		
		# Increment angle by 90 degrees (modulo 360)
		angle=$(( (angle + 90) % 360 ))
	done
	
	echo "Cycle $cycle is complete"
	((cycle++))
done

