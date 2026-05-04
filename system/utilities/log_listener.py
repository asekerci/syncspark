#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

# log_listener.py - Listen for log messages from SynchroSpark devices
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

import socket
import datetime

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 10001))

# Generate unique log file name with timestamp
import os
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
log_dir = os.path.expanduser("~/tmp")
os.makedirs(log_dir, exist_ok=True)
log_file_path = os.path.join(log_dir, f"log_{timestamp}.txt")
log_file = open(log_file_path, "a")
print(f"Listening for UDP datagrams on port 10001...")
print(f"Logging to {log_file_path}")

try:
    while True:
        data, addr = sock.recvfrom(65535)  # Max UDP datagram size
        msg = f'[{addr[0]}:{addr[1]}] {data.decode("utf-8", errors="ignore")}'
        print(msg, end='')
        log_file.write(msg)
        log_file.flush()
        # print(f"{len(data)} {addr}: {data}")        
except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    log_file.close()
    sock.close()
