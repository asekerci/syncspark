#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

# rcv_frames_udp.py - Receive camera frames via UDP
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

# $Id: rcv_frames_udp.py,v 1.3 2025/01/28 09:41:51 ahmet Exp ahmet $
import socket
import os
import time

UDP_IP = "0.0.0.0"  # Listen on all network interfaces
UDP_PORT = 10000    # Port to listen on

# Define the maximum UDP packet size
MAX_UDP_PACKET_SIZE = 1024

def main():
    frame_no = 1
    print(f"Listening for UDP packets on {UDP_IP}:{UDP_PORT}")
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    
    # Initialize variables for frame assembly
    frame_data = bytearray()  # To store the reassembled frame data
    total_bytes_received = 0  # Track the total number of bytes received

    try:
        while True:
            # Receive a UDP packet
            data, addr = sock.recvfrom(MAX_UDP_PACKET_SIZE)
            total_bytes_received += len(data)
            print(f"Received {len(data)} bytes from {addr}")
            
            # Add the received data to the frame buffer
            frame_data.extend(data)
            
            # Stop condition: a way to identify the frame is fully received
            # (could be based on a marker, metadata, or specific size in production)
            # For this example, assume sender stops sending after a full frame
            # is transmitted.

            # Save the frame after every full transmission for simplicity
            if len(data) < MAX_UDP_PACKET_SIZE:  # Last packet is likely smaller
                print(f"Frame fully received, total bytes: {total_bytes_received}")

                frame_file = './frames/frame_'+str(frame_no)+'.jpg'
                frame_no += 1
                with open(frame_file, "wb") as f:
                    f.write(frame_data)
                print("Frame saved to: " + frame_file)
                
                # Reset the variables for the next frame
                frame_data.clear()
                total_bytes_received = 0

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        sock.close()
        print("Socket closed.")

if __name__ == "__main__":
    main()
