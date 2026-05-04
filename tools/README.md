# Python Tools Summary

## mqtt_orientation_viewer.py
Live 3D viewer for yaw/pitch/roll over MQTT. Renders a simple 3‑wheel robot model.

Usage:
```bash
python mqtt_orientation_viewer.py --broker 192.168.1.19 --node sparknode03
```

Options:
- `--topic <topic>`: override topic (default: `arena/<node>/orientation`)
- `--poll-ms <int>`: UI update interval (ms)
- `--yaw-offset <deg>`: alignment offset (default -90)
- `--invert-yaw`, `--invert-pitch`, `--invert-roll`

Notes:
- Requires `numpy`, `matplotlib`, `paho-mqtt`.
- Needs an interactive backend (PyQt5 or Tk). If none found, it fails with a message.

## mag_calibrate_3d_all_in_one.py
All‑in‑one 3D magnetometer calibration pipeline: capture samples → fit ellipsoid → apply calibration.

Usage:
```bash
python mag_calibrate_3d_all_in_one.py --broker 192.168.1.19 --node sparknode03 --samples 500 --delay-ms 50
```

Options:
- `--mode all|capture` (default: `all`)
- `--timeout-s <sec>`
- `--output <csv>`
- `--target-radius <uT>` (optional fixed scaling)

Notes:
- Uses `mosquitto_sub` and `mosquitto_pub`.
- Default output: `mag_capture_<node>.csv`.

## mag_capture_to_csv.py
Captures raw magnetometer samples into a CSV file.

Usage:
```bash
python mag_capture_to_csv.py --broker 192.168.1.19 --node sparknode03 --samples 500 --delay-ms 50 --trigger
```

Options:
- `--trigger`: sends `calibrate mag capture <samples> <delay_ms>` before listening.
- `--output <csv>`
- `--timeout-s <sec>`

## plot_heading_vs_euler.py
Plots heading vs yaw (top) and pitch/roll (bottom) from a log `.txt` or a CSV.
Also writes optional CSV output and annotates filename, start/stop times, and mode.

Usage (TXT input):
```bash
python plot_heading_vs_euler.py log_YYYYMMDD_HHMMSS.txt --csv-out heading_vs_yaw_YYYYMMDD.csv
```

Usage (CSV input):
```bash
python plot_heading_vs_euler.py heading_vs_yaw_YYYYMMDD.csv
```

Options:
- `--no-unwrap`: keep 0–360 wrap instead of unwrapped angles

