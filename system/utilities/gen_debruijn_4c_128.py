#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

# gen_debruijn_4c_128.py - Generate De Bruijn sequences for 128 robots
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

import random

NUM_ROBOTS = 128
NUM_LEDS   =  16
WINDOW     =   6
COLORS     =   4

def seq_to_windows(seq):
    """Return all cyclic windows of length WINDOW as base-4 ints."""
    windows = []
    for i in range(NUM_LEDS):
        code = 0
        for j in range(WINDOW):
            code = code * COLORS + seq[(i + j) % NUM_LEDS]
        windows.append(code)
    return windows

def is_valid(seq, seen):
    """Check that all windows of seq are new."""
    for w in seq_to_windows(seq):
        if w in seen:
            return False
    return True

def greedy_generate(nrobots):
    seen = set()
    robots = []
    trials = 0
    while len(robots) < nrobots and trials < nrobots * 10000:
        seq = [random.randrange(COLORS) for _ in range(NUM_LEDS)]
        if is_valid(seq, seen):
            robots.append(seq)
            seen.update(seq_to_windows(seq))
        trials += 1
    return robots

robots = greedy_generate(NUM_ROBOTS)

print(f"const uint8_t debruijn_sequences[{len(robots)}][{NUM_LEDS}] = {{")
for r, seq in enumerate(robots):
    row = ", ".join(str(x) for x in seq)
    print(f"    {{ {row} }},")
print("};")
