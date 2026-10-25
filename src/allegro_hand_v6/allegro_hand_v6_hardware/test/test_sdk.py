#!/usr/bin/env python3
#
# Software License Agreement (BSD License)
#
#  Copyright (c) 2026, Wonik Robotics.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of Wonik Robotics nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

"""
Test script for Allegro Hand V6 Python SDK
('c')void
"""

import time
import numpy as np
import allegro_hand_v6_sdk as sdk


def main():
    print("==================================================")
    print("  Allegro Hand V6 Python SDK Verification Test    ")
    print("==================================================")

    # 1. Initialize Options
    opts = sdk.Options()
    opts.comm_type = "tcp"
    opts.ip_address = "192.168.1.100"
    opts.tcp_port = 502
    print(f"Connecting with options: {opts}")

    # 2. Connect to Hand
    hand = sdk.HandV6ModBus(opts)
    if not hand.is_ready():
        print("[ERROR] Hand is not ready!")
        return

    dev_info = hand.get_device_info()
    print(f"[Device Info] FW: 0x{dev_info.firmware_version:04X}, Type: {dev_info.hand_type}, SN: {dev_info.serial_number}")

    # 3. Read States
    positions, velocities, torques, imu_quat, pressures = hand.read_states()
    print("\n--- Telemetry States ---")
    print(f"Joint Positions (rad, shape={positions.shape}):\n  {np.round(positions, 4)}")
    print(f"IMU Quaternion [w, x, y, z]: {np.round(imu_quat, 4)}")
    print(f"Tactile Pressures (hPa, 18 sensors):\n  {pressures}")

    # 4. Read Diagnostics
    diag = hand.read_diagnostics()
    print("\n--- System Diagnostics ---")
    print(f"Torque Enabled: {diag.torque_enabled}")
    print(f"Calibration Running: {diag.calibration_running}")
    print(f"Operation Mode: {diag.operation_mode}")
    print(f"Motor Temperatures (C): {diag.motor_temperatures}")
    print(f"Sensor Temperatures (raw): {diag.sensor_temperatures}")

    # 5. 100-cycle High-Rate Read Loop
    print("\n--- High-Rate 100-cycle Test ---")
    t_start = time.perf_counter()
    for _ in range(100):
        pos, vel, tau, imu, pres = hand.read_states()
    t_end = time.perf_counter()
    elapsed = t_end - t_start
    hz = 100.0 / elapsed
    print(f"100 cycles completed in {elapsed * 1000.0:.2f} ms (~{hz:.2f} Hz)")

    # 6. Read PID Gains
    print("\n--- PID Gains ---")
    p_gains = hand.read_gains("p_gain")
    i_gains = hand.read_gains("i_gain")
    d_gains = hand.read_gains("d_gain")
    print(f"P Gains ({len(p_gains)}): {p_gains}")
    print(f"I Gains ({len(i_gains)}): {i_gains}")
    print(f"D Gains ({len(d_gains)}): {d_gains}")

    # 7. Signal Rate Monitor
    print("\n--- Signal Rate Monitor ---")
    print(hand.get_rate_monitor().to_string())
    print("\n[SUCCESS] Python SDK verification passed!")


if __name__ == "__main__":
    main()
