#!/bin/bash
set -euo pipefail

# Edit these for your environment
BROKER="192.168.1.19"
NODE="sparknode03"
TOPIC="arena/$NODE/cmd"

pub() {
  local msg="$1"
  echo "-> $TOPIC : $msg"
  mosquitto_pub -h "$BROKER" -t "$TOPIC" -m "$msg"
}

echo "Broker: $BROKER"
echo "Node:   $NODE"
echo "Topic:  $TOPIC"

# Example sequence
pub "show heading 5 100"
sleep 1

# Tighter final heading control (default)
pub "rotate to 180 15 10 300 3.0"
sleep 15

# Alternative: faster turn, may overshoot more
# pub "turn to 180 25"
# sleep 12

pub "show heading 5 100"

echo "Sequence complete."
