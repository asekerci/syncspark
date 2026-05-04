# SPDX-License-Identifier: GPL-3.0-or-later
#
# plot_heading_vs_euler.py - Plot heading vs Euler angle data
# 
# Copyright (C) 2025-2026 Ahmet Sekercioglu and Ismet Atalar
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

import argparse
import csv
import os
import re
import matplotlib.pyplot as plt


RE_TS = re.compile(r"\((\d+)\)")
RE_EULER_FULL = re.compile(
    r"Euler: Yaw=([-0-9]+\.?[0-9]*)deg Pitch=([-0-9]+\.?[0-9]*)deg Roll=([-0-9]+\.?[0-9]*)deg"
)
RE_EULER_YAW_ONLY = re.compile(r"Euler: Yaw=([-0-9]+\.?[0-9]*)deg")
RE_HEADING = re.compile(r"Magnetometer heading = ([0-9]+\.?[0-9]*)")
RE_MODE1 = re.compile(r"sensor_stream mode ([a-z_]+)")
RE_MODE2 = re.compile(r"mode=([a-z_]+)")


def parse_log_txt(path):
    euler = {}
    heading = {}
    first_ts = None
    last_ts = None
    mode_name = None
    with open(path, "r", errors="ignore") as f:
        for line in f:
            ts_m = RE_TS.search(line)
            if ts_m:
                ts = int(ts_m.group(1))
                if first_ts is None:
                    first_ts = ts
                last_ts = ts

            m = RE_MODE1.search(line) or RE_MODE2.search(line)
            if m:
                mode_name = m.group(1)

            if not ts_m:
                continue

            me_full = RE_EULER_FULL.search(line)
            if me_full:
                euler[ts] = (float(me_full.group(1)), float(me_full.group(2)), float(me_full.group(3)))
            else:
                me_yaw = RE_EULER_YAW_ONLY.search(line)
                if me_yaw:
                    euler[ts] = (float(me_yaw.group(1)), 0.0, 0.0)

            mh = RE_HEADING.search(line)
            if mh:
                heading[ts] = float(mh.group(1))

    common = sorted(set(euler) & set(heading))
    rows = [(t, euler[t][0], heading[t], euler[t][1], euler[t][2]) for t in common]
    return rows, first_ts, last_ts, mode_name


def parse_csv(path):
    rows = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            yaw = float(row["yaw_deg"])
            heading = float(row["heading_deg"])
            pitch = float(row.get("pitch_deg", 0.0))
            roll = float(row.get("roll_deg", 0.0))
            rows.append((int(row["t_ms"]), yaw, heading, pitch, roll))
    return rows


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "yaw_deg", "heading_deg", "pitch_deg", "roll_deg"])
        for r in rows:
            w.writerow(r)


def unwrap_degrees(values):
    if not values:
        return values
    out = [values[0]]
    for v in values[1:]:
        prev = out[-1]
        delta = v - (prev % 360.0)
        while delta > 180.0:
            delta -= 360.0
        while delta < -180.0:
            delta += 360.0
        out.append(prev + delta)
    return out


def main():
    parser = argparse.ArgumentParser(description="Plot heading vs yaw from log TXT or CSV.")
    parser.add_argument("input_path", help="Log .txt or .csv file")
    parser.add_argument("--csv-out", help="Optional CSV output path when input is .txt")
    parser.add_argument("--no-unwrap", action="store_true",
                        help="Disable angle unwrapping (keep 0-360 wrap)")
    args = parser.parse_args()

    ext = os.path.splitext(args.input_path)[1].lower()
    if ext == ".txt":
        rows, first_ts, last_ts, mode_name = parse_log_txt(args.input_path)
        if not rows:
            raise SystemExit("No aligned heading/yaw samples found in log")
        if args.csv_out:
            write_csv(rows, args.csv_out)
    elif ext == ".csv":
        rows = parse_csv(args.input_path)
        if not rows:
            raise SystemExit("No data in CSV")
        # CSV doesn't carry absolute log timestamps; use the CSV time range.
        first_ts = rows[0][0]
        last_ts = rows[-1][0]
        mode_name = None
    else:
        raise SystemExit("Unsupported input file. Use .txt or .csv")

    t = [r[0] for r in rows]
    yaw = [r[1] for r in rows]
    heading = [r[2] for r in rows]
    pitch = [r[3] for r in rows]
    roll = [r[4] for r in rows]

    t0 = t[0]
    t = [(x - t0) / 1000.0 for x in t]

    if not args.no_unwrap:
        yaw = unwrap_degrees(yaw)
        heading = unwrap_degrees(heading)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    # Top: heading + yaw
    ax1.plot(t, heading, label="Heading (mag)")
    ax1.plot(t, yaw, label="Yaw (Euler)")
    ax1.set_ylabel("Degrees (Yaw/Heading)")
    ax1.legend(loc="upper left")

    # Bottom: pitch + roll
    ax2.plot(t, pitch, label="Pitch (Euler)", alpha=0.7, color="red")
    ax2.plot(t, roll, label="Roll (Euler)", alpha=0.7, color="green")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Degrees (Pitch/Roll)")
    ax2.legend(loc="upper left")

    fig.suptitle("Sparknode ICM20948 Fused Sensor Data")

    # Footer with filename and absolute log times when available
    start_s = first_ts / 1000.0 if first_ts is not None else 0.0
    stop_s = last_ts / 1000.0 if last_ts is not None else 0.0
    footer = f"{os.path.basename(args.input_path)} | Start={start_s:.3f}s Stop={stop_s:.3f}s"
    fig.text(0.5, 0.01, footer, ha="center", va="bottom")

    # Mode label at bottom-left if available
    if mode_name:
        fig.text(0.01, 0.01, f"Mode: {mode_name}", ha="left", va="bottom")

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()


if __name__ == "__main__":
    main()
