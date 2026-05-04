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
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "config set turn_kick 0 0"
sleep 1
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "config set drive_kick 0 0"
sleep 1

mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "config show"
sleep 2

# Initialize counters
forward_count=0
reverse_count=0
left_count=0
right_count=0

# Function to print statistics on exit
print_stats() {
	echo ""
	echo "========================================"
	echo "Random Walk Statistics:"
	echo "========================================"
	echo "Forward movements: $forward_count"
	echo "Reverse movements: $reverse_count"
	echo "Left turns: $left_count"
	echo "Right turns: $right_count"
	echo "Total movements: $((forward_count + reverse_count))"
	echo "Total turns: $((left_count + right_count))"
	echo "========================================"
	exit 0
}

# Trap CTRL-C (SIGINT) and call print_stats
trap print_stats SIGINT

echo "Starting random walk.."
echo "Press Ctrl+C to stop"

while true; do
	# Randomly choose forward or reverse
	if (( RANDOM % 2 == 0 )); then
		direction="forward"
		((forward_count++))
	else
		direction="reverse"
		((reverse_count++))
	fi
	
	# Randomly choose left or right
	if (( RANDOM % 2 == 0 )); then
		turn="left"
		((left_count++))
	else
		turn="right"
		((right_count++))
	fi
	
	echo "Driving $direction..."
	mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "drive $direction 20 500"
	sleep 2

	echo "Turning $turn..."
	mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "turn $turn 20 300"
	sleep 1
done

