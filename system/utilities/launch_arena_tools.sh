#!/bin/bash

# Determine MQTT broker address based on current user
current_user=$(whoami)
if [ "$current_user" = "ahmet" ]; then
    broker_address="192.168.11.100"
else
    broker_address="192.168.8.3"
fi

echo "Current user: $current_user"
echo "Using MQTT broker: $broker_address"

xterm -title "Log Listener" -geometry 80x24+100+0 -fa 'Monospace' -fs 10 -hold -e "./log_listener.py" &
xterm -title "HTTP Server"  -geometry 80x24+760+0 -fa 'Monospace' -fs 10 -hold -e "python3 -m http.server --bind 0.0.0.0 8000 --directory /home/ahmet/projects/syncspark/system/ota" &
xterm -title "MQTT Status"  -geometry 80x24+100+450 -fa 'Monospace' -fs 10 -hold -e "mosquitto_sub -h $broker_address -t 'arena/+/status'" &
xterm -title "MQTT Arena All Messages" -geometry 80x24+760+450 -fa 'Monospace' -fs 10 -hold -e "mosquitto_sub -h $broker_address -t 'arena/#'" &

# Notes:
# To test the server, use:
# while true; do curl -v http://192.168.11.100:8000?id=http_ping; sleep 120; done
#
# If the UDP port 10001 is not free because of a stale log_listener:
# sudo ss -tulnp | grep :10001
# sudo kill -9 <PID>
