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
Standalone continuous finger wave motion test for Allegro Hand V6 using allegro_hand_v6_sdk.
Motion profile adopted from allegro_hand_v6_bringup/script/finger_wave.py.
('c')void
"""

import argparse
import signal
import sys
import time
import numpy as np

import allegro_hand_v6_sdk as sdk

g_running = True


def sigint_handler(signum, frame):
    global g_running
    print("\n[INFO] Interrupt signal received. Stopping motion safely...")
    g_running = False


class MotionGenerator:
    """
    Generates joint commands for a continuous 5-finger wave motion on Allegro Hand V6.
    Fingers: Thumb (00~03), Index (10~13), Middle (20~23), Ring (30~33), Pinky (40~43).
    """

    def __init__(self, frequency=8.0):
        self.frequency = frequency  # rad/s

        amplitude_rad_default = np.pi / 7.2  # ~25 deg
        amplitudes = np.full(20, amplitude_rad_default)
        # Scale joint amplitudes
        amplitudes[0] = np.pi / 20.0  # ~9 deg thumb root
        amplitudes[1] = np.pi / 20.0  # ~9 deg thumb base pitch
        amplitudes[2] = np.pi / 9.0   # ~20 deg thumb mid flexion
        amplitudes[3] = np.pi / 9.0   # ~20 deg thumb tip flexion
        amplitudes[4] = np.pi / 18.0
        amplitudes[8] = np.pi / 18.0
        amplitudes[12] = np.pi / 18.0
        amplitudes[16] = np.pi / 18.0
        self.amplitudes = np.array(amplitudes)

        self.center_positions = np.array(
            [
                # Thumb (joint00-03): deeper natural flexion wave
                0.40, 0.18, 0.58, 0.58,
                # Index (joint10-13)
                0.0, 1.169 / 2.0, 1.169 / 2.0, 1.169 / 2.0,
                # Middle (joint20-23)
                0.0, 1.169 / 2.0, 1.169 / 2.0, 1.169 / 2.0,
                # Ring (joint30-33)
                0.0, 1.169 / 2.0, 1.169 / 2.0, 1.169 / 2.0,
                # Pinky (joint40-43)
                0.0, 1.169 / 2.0, 1.169 / 2.0, 1.169 / 2.0,
            ]
        )

        self.phase_offsets = {
            "THUMB": 0.0,
            "INDEX": np.pi / 4.0,
            "MIDDLE": np.pi / 2.0,
            "RING": 3.0 * np.pi / 4.0,
            "PINKY": np.pi,
        }

    def get_command(self, t):
        current_command = np.zeros(20, dtype=np.float64)
        for i, finger in enumerate(["THUMB", "INDEX", "MIDDLE", "RING", "PINKY"]):
            phase = self.phase_offsets[finger]
            oscillation = np.sin(self.frequency * t + phase)
            start_idx, end_idx = 4 * i, 4 * i + 4
            current_command[start_idx:end_idx] = (
                self.center_positions[start_idx:end_idx]
                + self.amplitudes[start_idx:end_idx] * oscillation
            )
        return current_command


def main():
    global g_running

    signal.signal(signal.SIGINT, sigint_handler)
    signal.signal(signal.SIGTERM, sigint_handler)

    parser = argparse.ArgumentParser(description="Allegro Hand V6 Standalone Finger Wave Motion Test")
    parser.add_argument("--ip", type=str, default="192.168.1.100", help="Modbus TCP IP address")
    parser.add_argument("--port", type=int, default=502, help="Modbus TCP Port")
    parser.add_argument("--rate", type=float, default=100.0, help="Control loop rate in Hz")
    parser.add_argument("--freq", type=float, default=8.0, help="Motion wave frequency in rad/s")
    parser.add_argument("--duration", type=float, default=0.0, help="Duration in seconds (0 = infinite)")
    args = parser.parse_args()

    print("=========================================================")
    print("   Allegro Hand V6 Standalone Finger Wave Motion Test    ")
    print(f"   Target: {args.ip}:{args.port} | Loop Rate: {args.rate} Hz")
    print(f"   Wave Frequency: {args.freq} rad/s")
    print("=========================================================")

    # 1. Initialize Options
    opts = sdk.Options()
    opts.comm_type = "tcp"
    opts.ip_address = args.ip
    opts.tcp_port = args.port
    opts.slave_id = 1
    opts.k_p_table = [1.0] * sdk.JOINT_COUNT
    opts.k_d_table = [0.05] * sdk.JOINT_COUNT
    opts.torque_limit_table = [0.23] * sdk.JOINT_COUNT

    # 2. Connect to Hand
    try:
        hand = sdk.HandV6ModBus(opts)
    except Exception as e:
        print(f"[ERROR] Connection failed: {e}")
        return 1

    if not hand.is_ready():
        print("[ERROR] Hand is not ready.")
        return 1

    dev_info = hand.get_device_info()
    print(f"[INFO] Connected to Hand V6 (FW: 0x{dev_info.firmware_version:04X}, Type: {dev_info.hand_type}, SN: {dev_info.serial_number})")

    # 3. Enable Position Control & Servo Torque
    print("[INFO] Activating Position Control Mode & Servo Torque...")
    hand.set_control_mode(True)
    time.sleep(0.2)

    motion_gen = MotionGenerator(frequency=args.freq)
    loop_period = 1.0 / args.rate

    # 4. Smooth Transition from Current Pose to Wave Start Pose
    print("[INFO] Smoothly moving to initial wave start pose (1.5s)...")
    curr_pos, _, _, _, _ = hand.read_states()
    start_wave_pose = motion_gen.get_command(0.0)

    transition_steps = int(1.5 * args.rate)
    for step in range(transition_steps):
        if not g_running:
            break
        alpha = (step + 1) / transition_steps
        interp_pos = (1.0 - alpha) * curr_pos + alpha * start_wave_pose
        hand.write_commands(position=interp_pos)
        hand.read_states()
        time.sleep(loop_period)

    # 5. Continuous Finger Wave Loop
    print("[INFO] Starting continuous finger wave motion. Press Ctrl+C to stop.\n")
    start_time = time.perf_counter()
    cycle_count = 0

    while g_running:
        t_now = time.perf_counter()
        elapsed = t_now - start_time

        if args.duration > 0.0 and elapsed >= args.duration:
            print(f"\n[INFO] Target duration ({args.duration}s) reached.")
            break

        # Compute next target command
        cmd_pos = motion_gen.get_command(elapsed)

        # Write command & read feedback states
        hand.write_commands(position=cmd_pos)
        pos, vel, tau, imu, pres = hand.read_states()

        cycle_count += 1

        # Periodic status logging (every 1 second)
        if cycle_count % int(args.rate) == 0:
            thumb_deg = np.rad2deg(pos[0:4])
            index_deg = np.rad2deg(pos[4:8])
            middle_deg = np.rad2deg(pos[8:12])
            ring_deg = np.rad2deg(pos[12:16])
            pinky_deg = np.rad2deg(pos[16:20])

            sys.stdout.write(
                f"\r[T={elapsed:5.1f}s | Cyc={cycle_count:5d}] "
                f"T0={thumb_deg[0]:5.1f}° I1={index_deg[1]:5.1f}° M1={middle_deg[1]:5.1f}° "
                f"R1={ring_deg[1]:5.1f}° P1={pinky_deg[1]:5.1f}° | IMU_norm={np.linalg.norm(imu):.3f}"
            )
            sys.stdout.flush()

        # Rate regulation
        t_elapsed_cycle = time.perf_counter() - t_now
        t_sleep = loop_period - t_elapsed_cycle
        if t_sleep > 0:
            time.sleep(t_sleep)

    # 6. Safe Shutdown & Final Telemetry
    print("\n\n--- Final Motion Summary ---")
    total_time = time.perf_counter() - start_time
    avg_hz = cycle_count / total_time if total_time > 0 else 0.0
    print(f"Total Cycles: {cycle_count} in {total_time:.2f}s (~{avg_hz:.2f} Hz)")

    print("\n--- Signal Rate Monitor ---")
    print(hand.get_rate_monitor().to_string())

    print("[INFO] Finger wave motion test completed safely.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
