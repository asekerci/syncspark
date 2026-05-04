#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

# mag_calibrate_3d_all_in_one.py - 3D magnetometer calibration tool
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

import numpy as np

SAMPLE_RE = re.compile(
    r"mx=([-+]?\d*\.?\d+),\s*my=([-+]?\d*\.?\d+),\s*mz=([-+]?\d*\.?\d+)"
)


def run_cmd(cmd: list[str]) -> None:
    print("+", " ".join(cmd))
    subprocess.run(cmd, check=True)


def default_csv(node: str) -> str:
    return f"mag_capture_{node}.csv"


def capture_samples(args: argparse.Namespace, csv_path: str) -> None:
    print(f"[1/3] Capturing samples to {csv_path}")
    topic = f"arena/{args.node}/status"

    sub_cmd = ["mosquitto_sub", "-h", args.broker, "-t", topic, "-v"]
    try:
        sub_proc = subprocess.Popen(
            sub_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
    except FileNotFoundError as e:
        raise RuntimeError("mosquitto_sub not found in PATH") from e

    def stop_subscriber() -> None:
        if sub_proc.poll() is None:
            sub_proc.send_signal(signal.SIGINT)
            try:
                sub_proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                sub_proc.kill()

    time.sleep(0.4)

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
    except FileNotFoundError as e:
        stop_subscriber()
        raise RuntimeError("mosquitto_pub not found in PATH") from e
    except subprocess.CalledProcessError as e:
        stop_subscriber()
        raise RuntimeError(f"failed to send trigger command: {e}") from e

    deadline = time.time() + args.timeout_s
    sample_count = 0
    os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)

        print(f"Listening on {topic}")
        print(f"Writing samples to {csv_path}")
        print(f"Waiting for {args.samples} samples...")

        while sample_count < args.samples:
            if time.time() > deadline:
                stop_subscriber()
                raise RuntimeError(
                    f"capture timeout: collected {sample_count}/{args.samples} samples"
                )

            line = sub_proc.stdout.readline() if sub_proc.stdout else ""
            if not line:
                if sub_proc.poll() is not None:
                    raise RuntimeError("subscriber process exited unexpectedly")
                time.sleep(0.05)
                continue

            match = SAMPLE_RE.search(line.strip())
            if not match:
                continue
            writer.writerow(match.groups())
            sample_count += 1
            if sample_count % 50 == 0 or sample_count == args.samples:
                print(f"Captured {sample_count}/{args.samples}")

    stop_subscriber()
    print(f"Done: {sample_count} samples saved to {csv_path}")


def compute_3d_calibration(csv_path: str, target_radius: float | None) -> tuple[np.ndarray, np.ndarray]:
    print("[2/3] Computing 3D calibration")
    samples = np.loadtxt(csv_path, delimiter=",")
    if samples.ndim != 2 or samples.shape[1] != 3 or samples.shape[0] < 50:
        raise ValueError("CSV must contain at least 50 rows of mx,my,mz")

    # Step A: midpoint bias to stabilize fit.
    bias0 = (samples.max(axis=0) + samples.min(axis=0)) / 2.0
    shifted = samples - bias0
    x, y, z = shifted[:, 0], shifted[:, 1], shifted[:, 2]

    # Step B: algebraic ellipsoid fit.
    D = np.column_stack(
        [
            x * x,
            y * y,
            z * z,
            2 * x * y,
            2 * x * z,
            2 * y * z,
            2 * x,
            2 * y,
            2 * z,
            np.ones_like(x),
        ]
    )
    _, _, v = np.linalg.svd(D, full_matrices=False)
    coef = v[-1, :]

    A = np.array(
        [
            [coef[0], coef[3], coef[4], coef[6]],
            [coef[3], coef[1], coef[5], coef[7]],
            [coef[4], coef[5], coef[2], coef[8]],
            [coef[6], coef[7], coef[8], coef[9]],
        ],
        dtype=float,
    )

    Q = A[0:3, 0:3]
    u = coef[6:9] / 2.0
    center_shifted = -np.linalg.inv(Q).dot(u)
    bias = bias0 + center_shifted

    shifted2 = samples - bias

    # Step C: get soft-iron transform to sphere.
    T = np.eye(4)
    T[3, 0:3] = center_shifted
    R = T @ A @ T.T
    Qn = R[0:3, 0:3] / -R[3, 3]
    evals, evecs = np.linalg.eigh(Qn)
    if np.any(evals <= 0):
        raise ValueError("Invalid ellipsoid fit (non-positive eigenvalues)")

    radii = np.sqrt(1.0 / evals)
    soft_unit = evecs @ np.diag(1.0 / radii) @ evecs.T

    if target_radius is None:
        target_radius = float(np.median(np.linalg.norm(shifted2, axis=1)))
    soft = soft_unit * target_radius

    normed = shifted2 @ soft.T
    norm_radius = np.linalg.norm(normed, axis=1)
    print(f"Bias: [{bias[0]:.3f}, {bias[1]:.3f}, {bias[2]:.3f}]")
    print(
        "Diag: "
        f"[{soft[0,0]:.3f}, {soft[1,1]:.3f}, {soft[2,2]:.3f}] "
        f"radius mean/std={np.mean(norm_radius):.3f}/{np.std(norm_radius):.3f}"
    )
    return bias, soft


def apply_calibration(args: argparse.Namespace, bias: np.ndarray, soft: np.ndarray) -> None:
    print("[3/3] Applying calibration to node")
    payload = (
        "calibrate mag apply "
        f"{bias[0]:.6f} {bias[1]:.6f} {bias[2]:.6f} "
        f"{soft[0,0]:.6f} {soft[0,1]:.6f} {soft[0,2]:.6f} "
        f"{soft[1,0]:.6f} {soft[1,1]:.6f} {soft[1,2]:.6f} "
        f"{soft[2,0]:.6f} {soft[2,1]:.6f} {soft[2,2]:.6f} 3"
    )
    run_cmd(
        [
            "mosquitto_pub",
            "-h",
            args.broker,
            "-t",
            f"arena/{args.node}/cmd",
            "-m",
            payload,
        ]
    )
    print("Apply command published.")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Capture, solve, and apply 3D mag calibration.")
    p.add_argument("--broker", default="192.168.1.19")
    p.add_argument("--node", default="sparknode03")
    p.add_argument("--samples", type=int, default=500)
    p.add_argument("--delay-ms", type=int, default=50)
    p.add_argument("--timeout-s", type=int, default=240)
    p.add_argument("--output", default="")
    p.add_argument(
        "--target-radius",
        type=float,
        default=None,
        help="Optional fixed radius scaling in uT (default: median norm from captured data)",
    )
    p.add_argument(
        "--mode",
        choices=["all", "capture"],
        default="all",
        help="all: capture+compute+apply, capture: collect CSV only",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    csv_path = args.output or default_csv(args.node)
    try:
        capture_samples(args, csv_path)
        if args.mode == "capture":
            print("Capture-only mode complete.")
            return 0
        bias, soft = compute_3d_calibration(csv_path, args.target_radius)
        apply_calibration(args, bias, soft)
        print("Done.")
        return 0
    except subprocess.CalledProcessError as e:
        print(f"Command failed: {e}", file=sys.stderr)
        return 2
    except Exception as e:
        print(f"Calibration pipeline failed: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
