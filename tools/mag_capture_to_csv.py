#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

# mag_capture_to_csv.py - Capture magnetometer data to CSV format
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
import signal
import subprocess
import sys
import time


SAMPLE_RE = re.compile(
    r"mx=([-+]?\d*\.?\d+),\s*my=([-+]?\d*\.?\d+),\s*mz=([-+]?\d*\.?\d+)"
)


def default_output(node: str) -> str:
    return f"mag_capture_{node}.csv"


def run_capture(args: argparse.Namespace) -> int:
    topic = f"arena/{args.node}/status"
    out_path = args.output or default_output(args.node)

    sub_cmd = ["mosquitto_sub", "-h", args.broker, "-t", topic, "-v"]
    try:
        sub_proc = subprocess.Popen(
            sub_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
    except FileNotFoundError:
        print("ERROR: mosquitto_sub not found in PATH.", file=sys.stderr)
        return 2

    def stop_subscriber() -> None:
        if sub_proc.poll() is None:
            sub_proc.send_signal(signal.SIGINT)
            try:
                sub_proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                sub_proc.kill()

    time.sleep(0.4)

    if args.trigger:
        pub_msg = f"calibrate mag capture {args.samples} {args.delay_ms}"
        pub_cmd = [
            "mosquitto_pub",
            "-h",
            args.broker,
            "-t",
            f"arena/{args.node}/cmd",
            "-m",
            pub_msg,
        ]
        try:
            subprocess.run(pub_cmd, check=True)
            print(f"Triggered: {pub_msg}")
        except FileNotFoundError:
            print("ERROR: mosquitto_pub not found in PATH.", file=sys.stderr)
            stop_subscriber()
            return 2
        except subprocess.CalledProcessError as exc:
            print(f"ERROR: failed to send trigger command ({exc}).", file=sys.stderr)
            stop_subscriber()
            return 2

    deadline = time.time() + args.timeout_s
    sample_count = 0

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)

        print(f"Listening on {topic}")
        print(f"Writing samples to {out_path}")
        print(f"Waiting for {args.samples} samples...")

        while sample_count < args.samples:
            if time.time() > deadline:
                print(
                    f"Timeout: collected {sample_count}/{args.samples} samples.",
                    file=sys.stderr,
                )
                stop_subscriber()
                return 1

            line = sub_proc.stdout.readline() if sub_proc.stdout else ""
            if not line:
                if sub_proc.poll() is not None:
                    print("Subscriber process exited unexpectedly.", file=sys.stderr)
                    return 1
                time.sleep(0.05)
                continue

            line = line.strip()
            match = SAMPLE_RE.search(line)
            if not match:
                continue

            mx, my, mz = match.groups()
            writer.writerow([mx, my, mz])
            sample_count += 1

            if sample_count % 50 == 0 or sample_count == args.samples:
                print(f"Captured {sample_count}/{args.samples}")

    stop_subscriber()
    print(f"Done: {sample_count} samples saved to {out_path}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture magnetometer samples from MQTT status stream into CSV."
    )
    parser.add_argument("--broker", default="192.168.1.19", help="MQTT broker host")
    parser.add_argument("--node", default="sparknode03", help="Node name")
    parser.add_argument(
        "--samples", type=int, default=500, help="Number of samples to collect"
    )
    parser.add_argument(
        "--delay-ms", type=int, default=50, help="Capture delay in ms for trigger command"
    )
    parser.add_argument("--output", default="", help="Output CSV path")
    parser.add_argument(
        "--timeout-s", type=int, default=240, help="Capture timeout in seconds"
    )
    parser.add_argument(
        "--trigger",
        action="store_true",
        help="Send 'calibrate mag capture <samples> <delay_ms>' before collecting",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if args.samples <= 0:
        print("ERROR: --samples must be > 0", file=sys.stderr)
        sys.exit(2)
    if args.delay_ms <= 0:
        print("ERROR: --delay-ms must be > 0", file=sys.stderr)
        sys.exit(2)
    sys.exit(run_capture(args))
