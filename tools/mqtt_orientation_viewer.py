#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

# mqtt_orientation_viewer.py - MQTT orientation data visualization tool
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
import json
import threading
import time

import numpy as np
import matplotlib
import paho.mqtt.client as mqtt


def select_backend() -> str:
    for backend in ("Qt5Agg", "TkAgg"):
        try:
            matplotlib.use(backend, force=True)
            import matplotlib.pyplot as _plt  # noqa: F401
            fig = _plt.figure()
            _plt.close(fig)
            return backend
        except Exception:
            continue
    matplotlib.use("Agg", force=True)
    return "Agg"


BACKEND = select_backend()
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection


def rotation_matrix(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    # The new mounting is X-forward. Yaw is compass-style (clockwise positive
    # from north), pitch is nose-up positive, and roll is right-side-down positive.
    yaw = np.deg2rad(-yaw_deg)
    pitch = np.deg2rad(-pitch_deg)
    roll = np.deg2rad(roll_deg)

    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)

    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return Rz @ Ry @ Rx


def make_robot_model():
    # Chassis box with the model nose along +X.
    length = 1.0
    width = 0.7
    height = 0.2
    lx = length / 2.0
    ly = width / 2.0
    lz = height / 2.0
    chassis = np.array(
        [
            [-lx, -ly, -lz],
            [ lx, -ly, -lz],
            [ lx,  ly, -lz],
            [-lx,  ly, -lz],
            [-lx, -ly,  lz],
            [ lx, -ly,  lz],
            [ lx,  ly,  lz],
            [-lx,  ly,  lz],
        ],
        dtype=float,
    )
    chassis_faces = [
        [0, 1, 2, 3],  # bottom
        [4, 5, 6, 7],  # top
        [0, 1, 5, 4],  # -Y side
        [2, 3, 7, 6],  # +Y side
        [1, 2, 6, 5],  # front (+X)
        [0, 3, 7, 4],  # back (-X)
    ]

    # Simple front "nose" triangle on top to show direction (+X)
    nose = np.array(
        [
            [lx + 0.10, 0.0,  lz],
            [lx - 0.05, -0.15, lz],
            [lx - 0.05, 0.15, lz],
        ],
        dtype=float,
    )

    # Outline edges for chassis
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]

    return chassis, chassis_faces, edges, nose


def parse_args():
    p = argparse.ArgumentParser(description="Live 3D orientation viewer via MQTT yaw/pitch/roll.")
    p.add_argument("--broker", default="192.168.1.19")
    p.add_argument("--node", default="sparknode03")
    p.add_argument("--topic", default="")
    p.add_argument("--poll-ms", type=int, default=50, help="UI update interval in ms")
    p.add_argument("--yaw-offset", type=float, default=0.0, help="Yaw offset in degrees for model alignment")
    p.add_argument("--invert-yaw", action="store_true", help="Invert yaw sign")
    p.add_argument("--invert-pitch", action="store_true", help="Invert pitch sign")
    p.add_argument("--invert-roll", action="store_true", help="Invert roll sign")
    return p.parse_args()


def main():
    args = parse_args()
    topic = args.topic or f"arena/{args.node}/orientation"

    state = {"yaw": 0.0, "pitch": 0.0, "roll": 0.0}
    lock = threading.Lock()

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            with lock:
                state["yaw"] = float(payload.get("yaw", state["yaw"]))
                state["pitch"] = float(payload.get("pitch", state["pitch"]))
                state["roll"] = float(payload.get("roll", state["roll"]))
        except Exception:
            pass

    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.on_message = on_message
    client.connect(args.broker, 1883, 60)
    client.subscribe(topic)
    client.loop_start()

    if BACKEND == "Agg":
        raise RuntimeError(
            "No interactive Matplotlib backend found. Install Tk or Qt (PyQt5) "
            "and rerun to enable live viewer."
        )

    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_zlim(-1.2, 1.2)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(f"MQTT Orientation: {topic}")

    chassis, chassis_faces, edges, nose = make_robot_model()

    # Initialize with a dummy chassis so mpl doesn't try to autoscale empty segments.
    pts0 = chassis.copy()
    segs0 = [(pts0[i], pts0[j]) for i, j in edges]
    lines = Line3DCollection(segs0, colors="black", linewidths=1.5)
    ax.add_collection3d(lines)

    face_colors = [
        (0.3, 0.3, 0.3, 1.0),  # bottom - dark gray
        (0.6, 0.6, 0.6, 1.0),  # top - light gray
        (0.2, 0.4, 0.9, 1.0),  # back - blue
        (0.9, 0.4, 0.2, 1.0),  # front - orange
        (0.3, 0.7, 0.3, 1.0),  # right - green
        (0.7, 0.2, 0.7, 1.0),  # left - magenta
    ]
    face_poly = Poly3DCollection([], facecolors=face_colors, edgecolors="none")
    ax.add_collection3d(face_poly)

    nose_poly = Poly3DCollection([], facecolors=[(0.9, 0.2, 0.2, 1.0)], edgecolors="none")
    ax.add_collection3d(nose_poly)

    # Static axis triad for reference
    ax.quiver(0, 0, 0, 0.9, 0, 0, color="r", linewidth=2)
    ax.quiver(0, 0, 0, 0, 0.9, 0, color="g", linewidth=2)
    ax.quiver(0, 0, 0, 0, 0, 0.9, color="b", linewidth=2)
    ax.text(1.0, 0, 0, "X", color="r")
    ax.text(0, 1.0, 0, "Y", color="g")
    ax.text(0, 0, 1.0, "Z", color="b")

    def update_plot():
        with lock:
            yaw = state["yaw"] + args.yaw_offset
            pitch = state["pitch"]
            roll = state["roll"]
        if args.invert_yaw:
            yaw = -yaw
        if args.invert_pitch:
            pitch = -pitch
        if args.invert_roll:
            roll = -roll
        R = rotation_matrix(yaw, pitch, roll)
        pts = (chassis @ R.T)
        segs = [(pts[i], pts[j]) for i, j in edges]
        lines.set_segments(segs)
        face_verts = [[pts[i] for i in face] for face in chassis_faces]
        face_poly.set_verts(face_verts)

        nose_rot = (nose @ R.T)
        nose_poly.set_verts([nose_rot.tolist()])

        fig.canvas.draw_idle()

    while plt.fignum_exists(fig.number):
        update_plot()
        plt.pause(args.poll_ms / 1000.0)
        time.sleep(0.0)

    client.loop_stop()
    client.disconnect()


if __name__ == "__main__":
    main()
