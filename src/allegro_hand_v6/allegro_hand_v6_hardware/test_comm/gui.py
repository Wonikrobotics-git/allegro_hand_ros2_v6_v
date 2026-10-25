import sys
import time
import signal
import threading
import logging
import ctypes
import collections
import numpy as np
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QPushButton, QComboBox,
    QSpinBox, QDoubleSpinBox, QTextEdit, QTableWidget, QTableWidgetItem, QHeaderView, QSplitter,
    QTabWidget, QProgressBar, QCheckBox, QLineEdit, QDialog
)
from PyQt6.QtCore import QThread, pyqtSignal, Qt
from PyQt6.QtGui import QColor, QFont, QBrush, QPainter, QPen

from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException
import serial

# pymodbus logs per-transaction debug info (frame bytes, PDU details, etc.)
# by default. If anything in the process attaches a logging handler (even
# indirectly, e.g. via logging.basicConfig() somewhere), those log calls
# actually execute and do string formatting + I/O on every single
# transaction - at a 500 Hz+ poll rate this alone can dominate the round
# trip. Force it quiet regardless of what else configures logging.
logging.getLogger("pymodbus").setLevel(logging.WARNING)

_WIN_HIRES_TIMER_ACTIVE = False


def set_windows_high_res_timer(enable: bool):
    """Windows only, no-op elsewhere.

    Windows' default system timer resolution is ~15.6ms. time.sleep() and
    many blocking I/O waits - including pyserial's internal read polling -
    get quantized to that tick unless an application explicitly requests a
    finer resolution via winmm's timeBeginPeriod(). This is almost certainly
    why every RTT measured here clusters right around 15-16ms even though
    the underlying UART link itself is fast enough (a dedicated tool like
    Modbus Poll requests 1ms resolution on startup for exactly this reason).
    Called once at app startup / shutdown - it's a process-wide OS setting,
    not something that needs to be requested per-thread.
    """
    global _WIN_HIRES_TIMER_ACTIVE
    if sys.platform != "win32":
        return
    try:
        if enable and not _WIN_HIRES_TIMER_ACTIVE:
            ctypes.windll.winmm.timeBeginPeriod(1)
            _WIN_HIRES_TIMER_ACTIVE = True
        elif not enable and _WIN_HIRES_TIMER_ACTIVE:
            ctypes.windll.winmm.timeEndPeriod(1)
            _WIN_HIRES_TIMER_ACTIVE = False
    except Exception:
        pass

#################################
# CONFIGS & CONSTANTS
#################################
COM_PORT = "COM25"
BAUDRATE = 2000000
SLAVE_ID = 0x01

# Default TCP connection settings (Modbus TCP)
TCP_HOST = "192.168.1.100"
TCP_PORT = 502

# Connection type identifiers
CONN_RTU = "Modbus RTU"
CONN_TCP = "Modbus TCP"

FINGER_CNT = 5
JOINT_CNT = 4
TOTAL_JOINT_CNT = FINGER_CNT * JOINT_CNT

FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
PALM_SENSORS = ["Palm_1", "Palm_2", "Palm_3"]

# g_hand_status (0x0066, 16-bit) 비트마스크 구조
#
#  Bit [15:14] : Reserved (향후 확장용, 0으로 유지)
#  Bit [13: 9] : 손가락별 Fault 상태 (Finger 0~4 내부 관절 에러/Fault 발생 여부)
#                - Bit 9 : Finger 0 Fault   - Bit 12: Finger 3 Fault
#                - Bit 10: Finger 1 Fault   - Bit 13: Finger 4 Fault
#                - Bit 11: Finger 2 Fault
#  Bit [ 8: 4] : 손가락별 통신 에러 (Finger 0~4 모터 통신 및 ID 일치 실패 여부)
#                - Bit 4 : Finger 0 Comm Err   - Bit 7: Finger 3 Comm Err
#                - Bit 5 : Finger 1 Comm Err   - Bit 8: Finger 4 Comm Err
#                - Bit 6 : Finger 2 Comm Err
#  Bit [ 3: 0] : 하드웨어 통합 에러 (모든 관절 중 하나라도 발생 시 켜짐)
#                - Bit 0 : Overload (과부하)
#                - Bit 1 : Electrical Shock (전기적 충격)
#                - Bit 2 : Overheat (과열)
#                - Bit 3 : Input Voltage (입력 전압)
#
# 인덱스 == 비트 위치. FINGER_NAMES 순서(Thumb/Index/Middle/Ring/Pinky)를
# Finger 0~4에 그대로 매핑.
HAND_STATUS_BITS = (
    ["Overload", "Electrical Shock", "Overheat", "Input Voltage"]           # bit 0-3
    + [f"{name} Comm Err" for name in FINGER_NAMES]                         # bit 4-8
    + [f"{name} Fault" for name in FINGER_NAMES]                            # bit 9-13
    + ["Reserved", "Reserved"]                                              # bit 14-15
)

# BNO055 imu_config(profile) 값 — firmware imu_service.c의
# imu_profile_is_valid()가 허용하는 8개 모드만. 값 자체가 BNO055 OPR_MODE
# 레지스터 값과 1:1 (CONFIGMODE/ACCMAG류는 사용자 선택 목록에서 제외됨).
# raw=True 모드는 imu_data[4]가 [x, y, z, 0(예약)], False면 쿼터니언 [w,x,y,z].
IMU_PROFILES = {
    0x01: ("RAW_ACCEL", True),
    0x02: ("RAW_MAG",   True),
    0x03: ("RAW_GYRO",  True),
    0x08: ("IMU",       False),   # 가속도+자이로 융합 (자력계 미사용)
    0x09: ("COMPASS",   False),
    0x0A: ("M4G",       False),
    0x0B: ("NDOF_FMC_OFF", False),
    0x0C: ("NDOF",      False),
}

# Encoder 1 revolution(360 deg) == 4096 LSB
# C Macro : #define DEG2PULSE(deg) ((int16_t)(deg * 4096.0f / 360.0f))
ENCODER_LSB_PER_REV = 4096.0


def deg2pulse(deg):
    """Convert a single degree value to encoder pulse (int16, clamped)."""
    pulse = int(deg * ENCODER_LSB_PER_REV / 360.0)
    # int16 clamp (-32768 ~ 32767)
    if pulse > 32767:
        pulse = 32767
    elif pulse < -32768:
        pulse = -32768
    return pulse


def deg_list_to_pulse(deg_list):
    """Convert a list of degree values to encoder pulses (list of int16)."""
    return [deg2pulse(d) for d in deg_list]


def pulse2deg(pulse):
    """Inverse conversion : encoder pulse -> degree (for monitoring/verify)."""
    return pulse * 360.0 / ENCODER_LSB_PER_REV


MODBUS_ADDR = {
    # ---- Holding Registers (FC03 Read / FC16,FC23 Write) ----
    # Real-time control block (0x0100 ~ 0x012A)
    'cmd_pos': 0x0100,          # [20] int16  Goal Position
    'cmd_cur': 0x0114,          # [20] int16  Goal Current / Limit
    'cmd_op_mode': 0x0128,      # [1]  uint16 Operating Mode (0:Current, 5:Pos)
    'cmd_grasp_save': 0x0129,   # [1]  uint16 Grasp Save Slot (1~5)
    'cmd_grasp_play': 0x012A,   # [1]  uint16 Grasp Play Slot (1~5)
    # PID gain / motion profile holding registers (R/W), 0x0200 ~ 0x028B
    'cmd_gain_kp': 0x0200,      # uint16 x20 -> 20 regs
    'cmd_gain_ki': 0x0214,      # uint16 x20 -> 20 regs
    'cmd_gain_kd': 0x0228,      # uint16 x20 -> 20 regs
    'cmd_gain_vel': 0x023C,     # uint32 x20 -> 40 regs
    'cmd_gain_acc': 0x0264,     # uint32 x20 -> 40 regs
    # System / Network / Factory config holding registers, 0x0300 ~ 0x0320.
    # NOTE: there is no device-side RS-485-baud-select register wired up
    # yet (cfg_sys_rev0 is reserved for it, marked TBD) - the serial port
    # baudrate is still a PC-side (pyserial) setting only (see BAUDRATE /
    # self.baud_input) and is not written to the device.
    'cfg_comm_sel': 0x0300,
    'cfg_slave_id': 0x0301,
    'cfg_imu_config': 0x0302,
    'cfg_sys_rev0': 0x0303,     # uint16 RS-485 BaudRate CFG (TBD, reserved)
    'cfg_sys_rev1': 0x0304,     # uint16 Dev Option (TBD, reserved)
    # 0x0305 ~ 0x030F reserved (11 regs)
    'cfg_net_ip': 0x0310,       # uint32 -> 2 regs
    'cfg_net_mask': 0x0312,     # uint32 -> 2 regs
    'cfg_net_gw': 0x0314,       # uint32 -> 2 regs
    'cfg_net_dns': 0x0316,      # uint32 -> 2 regs
    'cfg_hand_sn': 0x0318,      # [4] uint64 Hand Serial No (factory-auth required to write), ends 0x031B
    # 0x031C ~ 0x031F reserved (4 regs)
    'cfg_factory_pwd': 0x0320,  # [1] uint16 Factory Auth Password

    # ---- Feedback registers (now part of the SAME holding-register map,
    # read via FC03 or FC23 - the firmware no longer implements FC04 /
    # MODBUS_FC_READ_REGS at all, sending it now returns Illegal Function).
    # Fast feedback block (~50 Hz), contiguous 0x0000 ~ 0x003D (62 regs)
    'get_pos': 0x0000,          # [20] int16
    'get_cur': 0x0014,          # [20] int16
    'get_imu': 0x0028,          # [4]  int16  BNO055 IMU data
    'get_sensor_hpa': 0x002C,   # [18] uint16 pressure (hPa)
    # Slow feedback block (~2 Hz), 0x0040 ~ 0x0071 - NOTE: 0x0067 ~ 0x006F
    # is a 9-register RESERVED gap between hand_status and fw_ver, and
    # fw_ver/hand_type swapped order again vs the previous firmware rev.
    'get_finger_temp': 0x0040,  # [20] uint16 motor temp
    'get_sensor_temp': 0x0054,  # [18] uint16 pressure-sensor temp
    'get_hand_status': 0x0066,  # [1]  uint16 joint status bitmask
    # 0x0067 : IMU calibration status (SYS/GYR/ACC/MAG packed), rest of
    # 0x0067~0x006F reserved gap still free for future diagnostics
    'get_imu_calib': 0x0067,    # [1]  uint16 bit[7:6]=SYS bit[5:4]=GYR bit[3:2]=ACC bit[1:0]=MAG
    'get_fw_ver': 0x0070,       # [1]  uint16 firmware version
    'get_hand_type': 0x0071,    # [1]  uint16 (0:Left, 1:Right)
}

# Coil map (FC01 Read / FC05,FC15 Write). Torque enable and joint
# calibration moved from holding registers to coils in the current firmware.
MODBUS_COIL_ADDR = {
    'torque_enable': 0x0000,        # [1] bool R/W (0:OFF, 1:ON)
    'joint_calib': 0x0001,          # [1] bool R/W (0:Idle, 1:Start Calib)
    'factory_auth_status': 0x0010,  # [1] bool RO  (0:locked, 1:PW match)
}

DEMO_POSITIONS = {
    "Zero (Default)": [0] * 20,
    "Demo 1 (Grasp)": [700, 512, 512, 512, 180, 300, 512, 512, 0, 300, 512, 512, -90, 300, 512, 512, -120, 300, 650, 550],
    "Demo 2 (Pinch)": [170, 0, 0, 0, 170, 0, 0, 0, 170, 0, 0, 0, 170, 0, 0, 0, 170, 0, 0, 0],
    "Demo 3 (Flex)":  [0, 0, 0, 0, -170, 0, 0, 0, -170, 0, 0, 0, -170, 0, 0, 0, -170, 0, 0, 0]
}

# C Language Trajectory Converter Data
DEMO_TRAJECTORIES = {
    "Pinching Motion": [
        {"duration": 400, "angles": [10, 45, 0, 0, 15, 10, 5, 5, 0, 10, 5, 5, -10, 10, 5, 5, -15, 20, 5, 5]},
        {"duration": 800, "angles": [100, 75, 20, 50, 15, 10, 5, 5, 0, 10, 5, 5, -10, 10, 5, 5, -45, 70, 15, 60]},
        {"duration": 400, "angles": [10, 45, 0, 0, 15, 10, 5, 5, 0, 10, 5, 5, -10, 10, 5, 5, -15, 20, 5, 5]},
        {"duration": 800, "angles": [70, 70, 45, 40, 15, 10, 5, 5, 0, 10, 5, 5, -10, 45, 45, 45, -15, 20, 5, 5]},
        {"duration": 400, "angles": [10, 45, 0, 0, 15, 10, 5, 5, 0, 10, 5, 5, -10, 10, 5, 5, -15, 20, 5, 5]},
        {"duration": 800, "angles": [60, 70, 40, 40, 15, 10, 5, 5, 0, 40, 45, 45, -10, 10, 5, 5, -15, 20, 5, 5]},
        {"duration": 400, "angles": [10, 45, 0, 0, 15, 10, 5, 5, 0, 10, 5, 5, -10, 10, 5, 5, -15, 20, 5, 5]},
        {"duration": 800, "angles": [50, 70, 30, 40, 15, 30, 50, 50, 0, 10, 5, 5, -10, 10, 5, 5, -15, 20, 5, 5]},
        {"duration": 400, "angles": [10, 45, 0, 0, 15, 10, 5, 5, 0, 10, 5, 5, -10, 10, 5, 5, -15, 20, 5, 5]},
        {"duration": 400, "angles": [50, 70, 30, 40, 15, 30, 50, 50, 0, 10, 5, 5, -10, 10, 5, 5, -15, 20, 5, 5]},
        {"duration": 800, "angles": [10, 45, 0, 0, 15, 10, 5, 5, 0, 10, 5, 5, -10, 10, 5, 5, -15, 15, 5, 5]},
        {"duration": 400, "angles": [60, 70, 40, 40, 15, 10, 5, 5, 0, 40, 45, 45, -10, 10, 5, 5, -15, 20, 5, 5]},
        {"duration": 800, "angles": [10, 45, 0, 0, 15, 10, 5, 5, 0, 10, 5, 5, -10, 10, 5, 5, -15, 15, 5, 5]},
        {"duration": 800, "angles": [70, 70, 45, 40, 15, 10, 5, 5, 0, 10, 5, 5, -10, 45, 45, 45, -15, 20, 5, 5]},
        {"duration": 800, "angles": [10, 45, 0, 0, 15, 10, 5, 5, 0, 10, 5, 5, -10, 10, 5, 5, -15, 15, 5, 5]},
        {"duration": 400, "angles": [100, 75, 20, 50, 15, 10, 5, 5, 0, 10, 5, 5, -10, 10, 5, 5, -45, 70, 15, 60]},
        {"duration": 800, "angles": [10, 45, 0, 0, 15, 10, 5, 5, 0, 10, 5, 5, -10, 10, 5, 5, -15, 15, 5, 5]}
    ],
    "Rock-Scissor-Paper": [
        {"duration": 1200, "angles": [10, 45, 10, 10, 10, 15, 5, 5, 0, 15, 5, 5, -5, 15, 5, 5, -15, 15, 10, 5]},
        {"duration": 800, "angles": [60, 30, 70, 60, 10, 50, 60, 60, 0, 50, 60, 60, -5, 50, 60, 60, -15, 15, 70, 70]},
        {"duration": 1200, "angles": [10, 45, 10, 10, 10, 15, 5, 5, 0, 15, 5, 5, -5, 15, 5, 5, -15, 15, 10, 5]},
        {"duration": 800, "angles": [60, 30, 70, 60, 10, 10, 5, 5, 0, 10, 5, 5, -5, 50, 60, 60, -15, 15, 70, 70]},
        {"duration": 400, "angles": [10, 40, 10, 10, 10, 10, 5, 5, 0, 10, 5, 5, -5, 10, 5, 5, -15, 15, 5, 5]},
        {"duration": 1200, "angles": [60, 30, 70, 60, 10, 10, 5, 5, 0, 10, 5, 5, -5, 50, 60, 60, -15, 15, 70, 70]},
        {"duration": 800, "angles": [60, 30, 70, 60, 10, 50, 60, 60, 0, 50, 60, 60, -5, 50, 60, 60, -15, 15, 70, 70]}
    ],
    "Gesture No": [
        {"duration": 500, "angles": [60, 30, 70, 60, 15, 10, 5, 5, 0, 50, 60, 60, 0, 50, 60, 60, -5, 15, 70, 70]},
        {"duration": 500, "angles": [60, 30, 70, 60, -5, 10, 5, 5, 0, 50, 60, 60, 0, 50, 60, 60, -5, 15, 70, 70]},
        {"duration": 500, "angles": [60, 30, 70, 60, 15, 10, 5, 5, 0, 50, 60, 60, 0, 50, 60, 60, -5, 15, 70, 70]},
        {"duration": 500, "angles": [60, 30, 70, 60, -5, 10, 5, 5, 0, 50, 60, 60, 0, 50, 60, 60, -5, 15, 70, 70]},
        {"duration": 500, "angles": [60, 30, 70, 60, 15, 10, 5, 5, 0, 50, 60, 60, 0, 50, 60, 60, -5, 15, 70, 70]}
    ],
    "Gesture Etc (Good/Salute)": [
        {"duration": 400, "angles": [0, 0, 10, 10, 15, 50, 60, 60, 0, 50, 60, 60, -5, 50, 60, 60, -15, 15, 70, 70]},
        {"duration": 800, "angles": [10, 30, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, -5, 0, 0, 0, -5, 0, 0, 0]},
        {"duration": 1200, "angles": [0, 70, 60, 30, 7, 10, 5, 5, 0, 15, 5, 5, -7, 10, 5, 5, -60, 5, 80, 80]}
    ],
    "Workspace Test": [
        {"duration": 800, "angles": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]},
        {"duration": 500, "angles": [0, 70, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]},
        {"duration": 500, "angles": [0, 90, 50, 50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]},
        {"duration": 800, "angles": [0, 70, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]},
        {"duration": 500, "angles": [70, 20, 60, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]},
        {"duration": 300, "angles": [0, 0, 0, 0, 0, 0, 0, 80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]},
        {"duration": 500, "angles": [0, 0, 0, 0, 0, 0, 80, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]},
        {"duration": 500, "angles": [0, 0, 0, 0, 0, 80, 60, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]},
        {"duration": 500, "angles": [0, 0, 0, 0, 10, 80, 60, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]},
        {"duration": 500, "angles": [0, 0, 0, 0, -10, 80, 60, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]},
        {"duration": 300, "angles": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 80, 0, 0, 0, 0, 0, 0, 0, 0]},
        {"duration": 500, "angles": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 80, 60, 0, 0, 0, 0, 0, 0, 0, 0]},
        {"duration": 500, "angles": [0, 0, 0, 0, 0, 0, 0, 0, 0, 80, 60, 60, 0, 0, 0, 0, 0, 0, 0, 0]},
        {"duration": 500, "angles": [0, 0, 0, 0, 0, 0, 0, 0, 8, 80, 60, 60, 0, 0, 0, 0, 0, 0, 0, 0]},
        {"duration": 500, "angles": [0, 0, 0, 0, 0, 0, 0, 0, -8, 80, 60, 60, 0, 0, 0, 0, 0, 0, 0, 0]},
        {"duration": 300, "angles": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 80, 0, 0, 0, 0]},
        {"duration": 500, "angles": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 80, 60, 0, 0, 0, 0]},
        {"duration": 500, "angles": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 80, 60, 60, 0, 0, 0, 0]},
        {"duration": 500, "angles": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 80, 60, 60, 0, 0, 0, 0]},
        {"duration": 500, "angles": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -5, 80, 60, 60, 0, 0, 0, 0]},
        {"duration": 300, "angles": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 80]},
        {"duration": 500, "angles": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 60, 60]},
        {"duration": 500, "angles": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 45, 60, 60]},
        {"duration": 500, "angles": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -20, 45, 60, 60]}
    ]
}

# Special combo entry: play every DEMO_TRAJECTORIES motion back-to-back once.
TRAJ_ALL_SEQUENTIAL = "▶ ALL Motions (Sequential)"

# Real-time control budget: the MCU is expected to service each modbus
# transaction within this interval. Round-trip times (RTT) measured at the
# master that exceed this are flagged as real-time violations. Note this is
# the FULL round trip (request tx + MCU processing + response rx + transport),
# so it is an upper bound on the MCU's own processing time.
RT_BUDGET_MS = 2.0

# Default target period for the fast R/W poll loop. Matches RT_BUDGET_MS by
# default - driving the loop at this cadence and watching the "LOOP cycle"
# row in the RX/TX Timing Monitor tab is how you verify the MCU can actually
# keep up with a real-time control loop at this rate. Adjustable at runtime
# via the "Fast Loop Period" spinbox.
FAST_LOOP_PERIOD_MS = 2.0

# How many recent RTT samples to keep per operation for the live RTT chart
# on the RX/TX Timing Monitor tab. A rolling window, not a running average -
# oldest samples drop off as new ones come in.
RTT_HISTORY_LEN = 300

# Any RTT above this is logged as a "spike" event (with the time/request
# count since the previous spike for that op), to help correlate periodic
# large-RTT anomalies with something on the MCU or OS/USB side. Well above
# normal RTT here (~1-5ms) so this doesn't fire on routine noise, but well
# below the ~20-100ms anomalies actually being chased.
SPIKE_LOG_THRESHOLD_MS = 15.0

#################################
# TRAJECTORY RUNNER THREAD
#################################
class TrajectoryRunner(QThread):
    # step_signal(step_idx, angles, motion_name) : angles in deg for UI display
    step_signal = pyqtSignal(int, list, str)
    finished_signal = pyqtSignal()
    log_signal = pyqtSignal(str)

    def __init__(self, worker, playlist, loop=False):
        """Play one or more named trajectories back-to-back.

        playlist : list of (motion_name, steps) tuples, where steps is the
        list of {"duration", "angles"} dicts. A single motion is just a
        playlist of length 1. Playing the whole playlist in sequence (with
        loop off) realises the 'ALL Motions (Sequential)' demo.
        """
        super().__init__()
        self.worker = worker
        self.playlist = playlist
        self.loop = loop
        self.is_running = True

    def run(self):
        loop_count = 0
        # Outer loop : repeat the whole playlist while loop mode is on
        while self.is_running:
            loop_count += 1
            if self.loop:
                self.log_signal.emit(f"Trajectory Loop #{loop_count} started.")

            for m_idx, (motion_name, steps) in enumerate(self.playlist):
                if not self.is_running:
                    break

                total_steps = len(steps)
                if len(self.playlist) > 1:
                    self.log_signal.emit(
                        f">>> Motion [{m_idx+1}/{len(self.playlist)}] "
                        f"'{motion_name}' started ({total_steps} steps)."
                    )

                for idx, step in enumerate(steps):
                    if not self.is_running:
                        break

                    duration_ms = step["duration"]
                    angles = step["angles"]

                    # Convert degree -> encoder pulse (DEG2PULSE) before sending
                    pulses = deg_list_to_pulse(angles)
                    pos_data = np.array(pulses, dtype=np.int16).astype(np.uint16).tolist()

                    if self.worker and self.worker.isRunning():
                        self.worker.write_regs(MODBUS_ADDR['cmd_pos'], pos_data)
                        # Emit step index, original angles (deg) and motion name
                        self.step_signal.emit(idx + 1, angles, motion_name)
                        self.log_signal.emit(
                            f"[{motion_name}] Step [{idx+1}/{total_steps}] "
                            f"deg->pulse sent. Wait {duration_ms}ms"
                        )

                    # Duration Delay
                    time.sleep(duration_ms / 1000.0)

            # If loop mode is disabled, run the playlist only once
            if not self.loop:
                break

        self.finished_signal.emit()

    def stop(self):
        self.is_running = False

#################################
# RAW MODBUS RTU TRANSPORT
#################################
class RawModbusError(Exception):
    """Raised by RawRTUTransport on any transaction failure (short/garbled
    response, CRC mismatch, or a Modbus exception response)."""
    pass


class RawReadResult:
    """Minimal pymodbus-response-compatible wrapper so the rest of the app
    (which does `res.registers` / `res.isError()`) doesn't need to change
    just because reads now come from RawRTUTransport instead of pymodbus."""
    __slots__ = ("registers",)

    def __init__(self, registers):
        self.registers = registers

    def isError(self):
        return False


class RawRTUTransport:
    """Hand-rolled Modbus RTU client over raw pyserial (FC01/03/05/16 only -
    everything this app needs). Exists because benchmarking showed pymodbus
    itself adds ~1.5-2ms of pure Python-level overhead per transaction on
    top of the raw pyserial round trip (frame building, CRC, response
    parsing, transaction management) - see raw_serial_bench.py. At a 2ms
    real-time budget that overhead matters, so RTU mode uses this instead
    of pymodbus's ModbusSerialClient. (TCP mode is unaffected and still
    uses pymodbus's ModbusTcpClient - it wasn't the bottleneck here and two
    different transports can't share one COM port anyway, so RTU switches
    over entirely rather than splitting reads/writes across two clients.)
    """

    def __init__(self, port, baudrate, timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

    def connect(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=self.timeout,
            )
            return True
        except Exception:
            self.ser = None
            return False

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    @staticmethod
    def _crc16(data):
        """Standard CRC-16/Modbus (poly 0xA001, init 0xFFFF, reflected)."""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def _transact(self, request, expected_len, retries=2):
        """Write request, read exactly expected_len bytes, verify CRC and
        check for a Modbus exception response. Returns the raw response
        bytes (including CRC) on success; raises RawModbusError otherwise.

        Retries up to `retries` extra times (immediate re-send, no pymodbus-
        style backoff needed at this baud rate) on any failure - short/no
        response, CRC mismatch, or a garbled frame. This matters because a
        20-register FC16 write occasionally takes the MCU slightly longer
        than the configured timeout to process and answer (observed mainly
        during Trajectory Motion playback, where writes land every 400-
        800ms while the fast loop is also polling continuously) - without a
        retry that shows up as a hard failure even though the device is
        completely healthy and would have answered a beat later. This is
        the same safety net pymodbus provided by default before RTU
        switched to this raw transport - losing it was an oversight, not a
        deliberate tradeoff. With the port's per-attempt timeout at 20ms
        (see RawRTUTransport.__init__), worst case here is 3 attempts x
        20ms = 60ms - still far better than the old single-retry-at-100ms
        worst case of 200ms, while being more resilient to two misses in a
        row.
        """
        if self.ser is None:
            raise RawModbusError("not connected")
        last_err = None
        for attempt in range(retries + 1):
            self.ser.reset_input_buffer()
            self.ser.write(request)
            resp = self.ser.read(expected_len)
            if len(resp) != expected_len:
                last_err = RawModbusError(f"short response: got {len(resp)}/{expected_len} bytes")
                continue
            crc_recv = resp[-2] | (resp[-1] << 8)
            crc_calc = self._crc16(resp[:-2])
            if crc_recv != crc_calc:
                last_err = RawModbusError("CRC mismatch")
                continue
            if resp[1] & 0x80:
                ex_code = resp[2] if len(resp) > 2 else 0
                last_err = RawModbusError(f"Modbus exception 0x{ex_code:02X}")
                continue
            return resp
        raise last_err

    def read_holding_registers(self, slave_id, address, count):
        """FC03. Returns a list of `count` uint16 register values."""
        req = bytes([slave_id & 0xFF, 0x03,
                     (address >> 8) & 0xFF, address & 0xFF,
                     (count >> 8) & 0xFF, count & 0xFF])
        req += self._crc16(req).to_bytes(2, "little")
        expected = 3 + count * 2 + 2
        resp = self._transact(req, expected)
        data = resp[3:3 + count * 2]
        return [int.from_bytes(data[i:i + 2], "big") for i in range(0, len(data), 2)]

    def write_registers(self, slave_id, address, values):
        """FC16. `values` is a list of uint16s."""
        count = len(values)
        payload = bytearray()
        for v in values:
            payload += (int(v) & 0xFFFF).to_bytes(2, "big")
        req = bytes([slave_id & 0xFF, 0x10,
                     (address >> 8) & 0xFF, address & 0xFF,
                     (count >> 8) & 0xFF, count & 0xFF,
                     count * 2]) + bytes(payload)
        req += self._crc16(req).to_bytes(2, "little")
        self._transact(req, 8)  # echo response: slave+func+addr(2)+count(2)+crc(2)
        return True

    def write_coil(self, slave_id, address, value):
        """FC05."""
        val = 0xFF00 if value else 0x0000
        req = bytes([slave_id & 0xFF, 0x05,
                     (address >> 8) & 0xFF, address & 0xFF,
                     (val >> 8) & 0xFF, val & 0xFF])
        req += self._crc16(req).to_bytes(2, "little")
        self._transact(req, 8)
        return True

    def read_coils(self, slave_id, address, count):
        """FC01. Returns a list of `count` booleans."""
        req = bytes([slave_id & 0xFF, 0x01,
                     (address >> 8) & 0xFF, address & 0xFF,
                     (count >> 8) & 0xFF, count & 0xFF])
        req += self._crc16(req).to_bytes(2, "little")
        byte_cnt = (count + 7) // 8
        expected = 3 + byte_cnt + 2
        resp = self._transact(req, expected)
        data = resp[3:3 + byte_cnt]
        return [bool(data[i // 8] & (1 << (i % 8))) for i in range(count)]


#################################
# WORKER THREAD FOR MODBUS
#################################
class ModbusWorker(QThread):
    fast_data_updated = pyqtSignal(list, list, list, list)
    slow_data_updated = pyqtSignal(list, list, int, int)
    log_signal = pyqtSignal(str)
    connection_status = pyqtSignal(bool, int)
    # Error monitoring: (label, detail, is_error)
    #   is_error True  -> a transaction failed (timeout / exception / modbus error)
    #   is_error False -> a transaction recovered (used to update stats/health)
    comm_event = pyqtSignal(str, str, bool)
    # RX/TX timing monitoring: (op_key, rtt_ms, is_error)
    #   op_key  -> short operation tag (e.g. "FC23 rw", "FC16 write")
    #   rtt_ms  -> measured round-trip time in milliseconds
    #   is_error-> True if this transaction failed (rtt still reported for context)
    timing_event = pyqtSignal(str, float, bool)
    # Emitted periodically (not per-transaction) so the GUI can refresh its
    # aggregate counters/tables from tx_count/err_count/timing_stats without
    # taking a Qt signal hit on every single fast-loop transaction.
    stats_tick = pyqtSignal()

    def __init__(self, conn_type=CONN_TCP, port=COM_PORT, baudrate=BAUDRATE,
                 host=TCP_HOST, tcp_port=TCP_PORT, slave_id=SLAVE_ID,
                 fast_loop_period_ms=FAST_LOOP_PERIOD_MS):
        super().__init__()
        self.conn_type = conn_type
        self.port = port
        self.baudrate = baudrate
        self.host = host
        self.tcp_port = tcp_port
        self.slave_id = slave_id
        # Target cycle time for the fast R/W loop. Plain attribute (not
        # behind a lock) - the GUI thread may update it live from a spinbox
        # while the poll loop reads it every cycle; a torn read of a float
        # is not a concern here, worst case one cycle uses a stale value.
        self.fast_loop_period_ms = fast_loop_period_ms
        self.running = False
        self.client = None
        # Communication statistics (for the error-monitoring tab)
        self.tx_count = 0
        self.err_count = 0
        # RX/TX timing statistics (for the timing-monitoring tab). Per-op stats
        # are keyed by a short op tag; each value tracks count / cumulative /
        # min / max / last rtt (ms) and the number of transactions that
        # exceeded the real-time budget (RT_BUDGET_MS) or timed out.
        self.timing_stats = {}
        self.timeout_count = 0
        self.rt_violation_count = 0
        self.last_rtt_ms = 0.0
        # Error category -> count (for the error-type breakdown table on
        # the Comm/Error Monitor tab). See _classify_error().
        self.error_type_counts = {}
        # Per-op rolling RTT sample history (for the live chart on the
        # RX/TX Timing Monitor tab). Populated in _record_timing() - every
        # transaction, regardless of notify - so the chart reflects true
        # history even between throttled GUI refreshes.
        self.rtt_history = {}
        # Per-op "last spike" tracking (wall-clock time + tx_count at the
        # last RTT that exceeded SPIKE_LOG_THRESHOLD_MS), so each new spike
        # can log the interval since the previous one - see _record_timing().
        self._last_spike_time = {}
        self._last_spike_tx = {}
        # Serial/TCP link + pymodbus client are NOT safe for concurrent use.
        # The GUI thread calls write_regs()/write_coil() directly (button
        # clicks) while this worker's own run() loop is simultaneously
        # polling on a background QThread. Without a lock, a large write
        # (e.g. a 20-register demo-position write) can interleave on the
        # wire with a fast-loop read and get corrupted/dropped.
        self._bus_lock = threading.Lock()
        # Cache which slave-id kwarg name this pymodbus version accepts
        # ('device_id' on 3.14+) after the first successful call, so every
        # subsequent transaction skips straight to it instead of looping
        # through candidates.
        self._device_id_kwarg = None
        # Slow feedback (finger_temp/sensor_temp/hand_status/fw_ver/
        # hand_type) is now polled on-demand rather than continuously: the
        # GUI flips this on only while the Slow Loop Monitor tab or the
        # Sensor Monitor popup is actually visible, so it doesn't compete
        # for bus time with the fast loop the rest of the time. Plain bool/
        # flag attributes (not locked) - worst case one extra/missed cycle,
        # not worth a lock for this.
        self.slow_loop_enabled = False
        self._slow_poll_due_now = False

    def _create_client(self):
        """Create the modbus client according to the selected connection type."""
        if self.conn_type == CONN_TCP:
            return ModbusTcpClient(
                host=self.host,
                port=self.tcp_port,
                timeout=1.0,
            )
        # Default : Modbus RTU (serial). Uses our own RawRTUTransport, not
        # pymodbus's ModbusSerialClient - see the RawRTUTransport docstring
        # for why (pymodbus measured ~1.5-2ms slower per transaction than
        # raw pyserial at this baud rate, which matters at a 2ms budget).
        #
        # timeout=0.02 (20ms): typical successful RTT here is 1-5ms, so this
        # still leaves 4-20x margin. Was 0.1s (100ms) - but that meant any
        # time a request's first attempt got no response (periodic cause
        # still unconfirmed - possibly USB/OS-level, not this app), the
        # retry in _transact() had to wait out the FULL 100ms timeout before
        # even trying again, which is exactly why "no more errors" (retry
        # succeeds) still showed up as periodic ~100ms RTT spikes on the
        # chart - the error was gone but the wasted wait wasn't. A much
        # shorter per-attempt timeout bounds that wasted wait far tighter.
        return RawRTUTransport(
            port=self.port,
            baudrate=self.baudrate,
            timeout=0.02,
        )

    def connect_client(self):
        self.client = self._create_client()
        if self.client.connect():
            for sid in range(1, 4):
                res = self.read_holding_regs(0x0000, 1, sid)
                if res is not None and not res.isError():
                    self.slave_id = sid
                    self.connection_status.emit(True, sid)
                    self.log_signal.emit(
                        f"Connected via {self.conn_type} (Slave ID: {sid})"
                    )
                    return True
            self.log_signal.emit(
                "Port/socket opened but no slave responded (ID 1-3). "
                "Check slave_id / wiring / unit-id."
            )
            self.connection_status.emit(False, 0)
            return False
        else:
            if self.conn_type == CONN_TCP:
                self.log_signal.emit(
                    f"Failed to connect TCP {self.host}:{self.tcp_port}"
                )
            else:
                self.log_signal.emit("Failed to open Serial Port")
            self.connection_status.emit(False, 0)
            return False

    def _op_key(self, label):
        """Derive a short, stable operation tag from a transaction label so
        timing stats aggregate by operation type rather than by address.
        e.g. 'FC23 rw read@0x0000x62 write@0x0100x20' -> 'FC23 rw'."""
        parts = label.split()
        return " ".join(parts[:2]) if len(parts) >= 2 else label

    def _record_timing(self, op_key, rtt_ms, is_error, is_timeout):
        """Accumulate per-operation RTT statistics (pure bookkeeping - does
        NOT touch Qt/the GUI). Called on every single transaction regardless
        of whether that transaction's result gets pushed to the GUI, so
        stats stay fully accurate even while the fast loop runs silently
        between periodic stats_tick refreshes."""
        st = self.timing_stats.get(op_key)
        if st is None:
            st = {"count": 0, "sum": 0.0, "min": float("inf"),
                  "max": 0.0, "last": 0.0, "over": 0, "err": 0}
            self.timing_stats[op_key] = st
        st["count"] += 1
        st["sum"] += rtt_ms
        st["last"] = rtt_ms
        if rtt_ms < st["min"]:
            st["min"] = rtt_ms
        if rtt_ms > st["max"]:
            st["max"] = rtt_ms
        if rtt_ms > RT_BUDGET_MS:
            st["over"] += 1
            self.rt_violation_count += 1
        if is_error:
            st["err"] += 1
        if is_timeout:
            self.timeout_count += 1
        self.last_rtt_ms = rtt_ms

        hist = self.rtt_history.get(op_key)
        if hist is None:
            hist = collections.deque(maxlen=RTT_HISTORY_LEN)
            self.rtt_history[op_key] = hist
        hist.append(rtt_ms)

        if rtt_ms > SPIKE_LOG_THRESHOLD_MS:
            self._log_spike(op_key, rtt_ms)

    def _log_spike(self, op_key, rtt_ms):
        """Log a large-RTT 'spike' event with the time and request-count
        interval since the previous spike for this op, to help correlate
        the anomaly with a periodic MCU task, USB polling, or anything
        else with a characteristic period."""
        now = time.perf_counter()
        prev_time = self._last_spike_time.get(op_key)
        prev_tx = self._last_spike_tx.get(op_key)
        if prev_time is not None:
            interval_ms = (now - prev_time) * 1000.0
            interval_tx = self.tx_count - prev_tx
            self.log_signal.emit(
                f"[SPIKE] {op_key}: RTT={rtt_ms:.1f}ms "
                f"(+{interval_ms:.0f}ms / +{interval_tx} tx since last spike)"
            )
        else:
            self.log_signal.emit(f"[SPIKE] {op_key}: RTT={rtt_ms:.1f}ms (first spike observed)")
        self._last_spike_time[op_key] = now
        self._last_spike_tx[op_key] = self.tx_count

    @staticmethod
    def _classify_error(detail):
        """Bucket an error detail string into a short, stable category so
        the Comm/Error Monitor tab can show a breakdown instead of just a
        raw scrolling log."""
        d = detail.lower()
        if "short response" in d:
            return "No Response" if "got 0/" in d else "Partial Response"
        if "crc mismatch" in d:
            return "CRC Mismatch"
        if "modbus exception" in d:
            return "Modbus Exception"
        if "not connected" in d:
            return "Not Connected"
        if "timeout" in d or "no response" in d:
            return "Timeout"
        if "modbus error resp" in d:
            return "Error Response"
        if "kwarg" in d:
            return "API Mismatch"
        return "Other"

    def _record_error(self, detail):
        """Tally an error into error_type_counts (pure bookkeeping, no Qt)."""
        category = self._classify_error(detail)
        self.error_type_counts[category] = self.error_type_counts.get(category, 0) + 1

    def _execute(self, label, fn, target_id, notify=True):
        """Run a single modbus transaction with proper exception handling.

        pymodbus 3.14 only accepts the `device_id` keyword; older param names
        (slave/unit/dev_id) raise TypeError. The first successful call caches
        which one worked in self._device_id_kwarg so every later call skips
        straight to it instead of re-probing candidates each time. Connection
        /IO problems raise ModbusException (NOT TypeError) and must be caught
        here so the caller gets a clean None instead of an exception leaking
        into the Qt loop. Every call updates comm statistics (tx_count/
        err_count/timing_stats) regardless of `notify`. When notify=False,
        successful transactions skip emitting comm_event/timing_event - used
        by the fast/slow poll loops so a 500 Hz+ background poll rate doesn't
        translate into 500+ Qt cross-thread signal emissions and widget
        updates per second. Errors always emit immediately regardless of
        notify, since they're rare and worth surfacing right away; the GUI
        otherwise gets periodic aggregate refreshes via ModbusWorker.stats_tick.
        """
        self.tx_count += 1
        op_key = self._op_key(label)
        last_type_err = None
        # A list (not tuple) so that if the cached kwarg unexpectedly stops
        # working mid-run, extending it in place is visible to the running
        # for-loop below (reassigning the name wouldn't be - Python's for
        # loop keeps iterating the original object).
        candidates = [self._device_id_kwarg] if self._device_id_kwarg else \
            ['device_id', 'slave', 'unit', 'dev_id']
        with self._bus_lock:
            for param in candidates:
                t0 = time.perf_counter()
                try:
                    res = fn(**{param: target_id})
                except TypeError as e:
                    # Wrong keyword for this pymodbus version -> try the next
                    # one (no real transaction happened, don't record timing).
                    # If this was our cached guess and it just started
                    # failing, drop the cache and fall back to probing the
                    # full candidate list within this same call.
                    last_type_err = e
                    if param == self._device_id_kwarg:
                        self._device_id_kwarg = None
                        for fallback in ('device_id', 'slave', 'unit', 'dev_id'):
                            if fallback not in candidates:
                                candidates.append(fallback)
                    continue
                except ModbusException as e:
                    rtt_ms = (time.perf_counter() - t0) * 1000.0
                    self.err_count += 1
                    # Timeouts surface as IO/connection ModbusExceptions.
                    is_timeout = "timeout" in str(e).lower() or \
                        "no response" in str(e).lower()
                    self._record_timing(op_key, rtt_ms, True, is_timeout)
                    detail = f"ModbusException: {e}"
                    self._record_error(detail)
                    self.comm_event.emit(label, detail, True)
                    self.timing_event.emit(op_key, rtt_ms, True)
                    return None
                except Exception as e:  # socket reset, serial unplugged, etc.
                    rtt_ms = (time.perf_counter() - t0) * 1000.0
                    self.err_count += 1
                    self._record_timing(op_key, rtt_ms, True, False)
                    detail = f"{type(e).__name__}: {e}"
                    self._record_error(detail)
                    self.comm_event.emit(label, detail, True)
                    self.timing_event.emit(op_key, rtt_ms, True)
                    return None

                rtt_ms = (time.perf_counter() - t0) * 1000.0
                self._device_id_kwarg = param  # cache the winner

                # Got a response object. A modbus exception response (illegal
                # address / function) is reported via isError().
                if res is None or res.isError():
                    self.err_count += 1
                    is_timeout = res is None
                    self._record_timing(op_key, rtt_ms, True, is_timeout)
                    resp_detail = str(res) if res is not None else "no response"
                    detail = f"Modbus error resp: {resp_detail}"
                    self._record_error(detail)
                    self.comm_event.emit(label, detail, True)
                    self.timing_event.emit(op_key, rtt_ms, True)
                    return res
                # Success -> record timing; only notify the GUI immediately
                # if asked to (see notify= docstring above).
                self._record_timing(op_key, rtt_ms, False, False)
                if notify:
                    self.comm_event.emit(label, "OK", False)
                    self.timing_event.emit(op_key, rtt_ms, False)
                return res

            # Every keyword raised TypeError (should not happen on 3.14)
            self.err_count += 1
            detail = f"No valid device_id kwarg: {last_type_err}"
            self._record_error(detail)
            self.comm_event.emit(label, detail, True)
            return None

    def _execute_raw(self, label, fn, notify=True):
        """Same statistics/timing/notify bookkeeping as _execute(), for
        RawRTUTransport calls. No device_id-kwarg guessing needed here -
        we wrote the API, it always takes slave_id directly - so this is
        simpler than _execute() and, more importantly, doesn't carry
        pymodbus's per-call overhead."""
        self.tx_count += 1
        op_key = self._op_key(label)
        with self._bus_lock:
            t0 = time.perf_counter()
            try:
                res = fn()
            except RawModbusError as e:
                rtt_ms = (time.perf_counter() - t0) * 1000.0
                self.err_count += 1
                is_timeout = "short response" in str(e).lower()
                self._record_timing(op_key, rtt_ms, True, is_timeout)
                detail = f"RawModbusError: {e}"
                self._record_error(detail)
                self.comm_event.emit(label, detail, True)
                self.timing_event.emit(op_key, rtt_ms, True)
                return None
            except Exception as e:  # serial unplugged, port closed, etc.
                rtt_ms = (time.perf_counter() - t0) * 1000.0
                self.err_count += 1
                self._record_timing(op_key, rtt_ms, True, False)
                detail = f"{type(e).__name__}: {e}"
                self._record_error(detail)
                self.comm_event.emit(label, detail, True)
                self.timing_event.emit(op_key, rtt_ms, True)
                return None

            rtt_ms = (time.perf_counter() - t0) * 1000.0
            self._record_timing(op_key, rtt_ms, False, False)
            if notify:
                self.comm_event.emit(label, "OK", False)
                self.timing_event.emit(op_key, rtt_ms, False)
            return res

    def read_holding_regs(self, address, count, sid=None, notify=True):
        target_id = sid if sid is not None else self.slave_id
        label = f"FC03 read_holding @0x{address:04X} x{count}"
        if self.conn_type == CONN_RTU:
            def _call():
                regs = self.client.read_holding_registers(target_id, address, count)
                return RawReadResult(regs)
            return self._execute_raw(label, _call, notify=notify)
        return self._execute(
            label,
            lambda **kw: self.client.read_holding_registers(address=address, count=count, **kw),
            target_id,
            notify=notify,
        )

    def readwrite_regs(self, read_address, read_count, write_address, values):
        """FC23 Read/Write Multiple Registers. Not currently used by the
        poll loop - main.c refreshes both the fast (mb_register_fast_shadow_
        update) and slow feedback shadows independently on a timer, so a
        plain FC03 read already returns live data and a plain FC16 write
        already lands correctly. Kept available (pymodbus-only, TCP or RTU)
        in case a future firmware revision goes back to tying shadow
        refresh to FC23; RawRTUTransport doesn't implement FC23 since
        nothing currently calls this."""
        return self._execute(
            f"FC23 rw read@0x{read_address:04X}x{read_count} write@0x{write_address:04X}x{len(values)}",
            lambda **kw: self.client.readwrite_registers(
                read_address=read_address, read_count=read_count,
                write_address=write_address, values=values, **kw
            ),
            self.slave_id,
        )

    def write_regs(self, address, values):
        """Write holding registers via FC16 (Write Multiple Registers).

        The V5SD MCU implements FC16 (0x10) and FC23 (0x17) for writes across
        the holding-register map (0x0100..0x0310); it does NOT implement FC06
        (Write Single), which returns Illegal Function. So we always use FC16.
        mb_write_holding_regs on the MCU refreshes each finger's 4 goal joints
        from its shadow buffer, so writing the full 20-register block at once
        is the correct/expected access pattern.

        NOTE: Torque enable and joint calibration are NOT holding registers
        in the current firmware - they are coils (FC01/05/15). Use
        write_coil()/read_coils() for those instead of write_regs().
        """
        label = f"FC16 write @0x{address:04X} x{len(values)}"
        if self.conn_type == CONN_RTU:
            def _call():
                return self.client.write_registers(self.slave_id, address, values)
            return self._execute_raw(label, _call)
        return self._execute(
            label,
            lambda **kw: self.client.write_registers(address=address, values=values, **kw),
            self.slave_id,
        )

    def read_coils(self, address, count=1):
        """Read coils via FC01. Used for torque_enable / joint_calib /
        factory_auth_status, which are coils (not holding registers) in the
        current firmware map. Always returns a plain List[bool] (or None on
        failure) regardless of transport - RawRTUTransport already returns
        that natively, but pymodbus's TCP client returns a response object
        with .bits/.isError(), so that path is unwrapped here to match."""
        label = f"FC01 read_coils @0x{address:04X} x{count}"
        if self.conn_type == CONN_RTU:
            def _call():
                return self.client.read_coils(self.slave_id, address, count)
            return self._execute_raw(label, _call)
        res = self._execute(
            label,
            lambda **kw: self.client.read_coils(address=address, count=count, **kw),
            self.slave_id,
        )
        if res is None or res.isError():
            return None
        return list(res.bits[:count])

    def write_coil(self, address, value):
        """Write a single coil via FC05. Used for torque_enable / joint_calib."""
        label = f"FC05 write_coil @0x{address:04X}"
        if self.conn_type == CONN_RTU:
            def _call():
                return self.client.write_coil(self.slave_id, address, value)
            return self._execute_raw(label, _call)
        return self._execute(
            label,
            lambda **kw: self.client.write_coil(address=address, value=value, **kw),
            self.slave_id,
        )

    def run(self):
        if not self.connect_client():
            return

        self.running = True
        # Slow feedback poll is paced by wall-clock time rather than an
        # iteration count, since the fast-loop period is now adjustable at
        # runtime - counting cycles would make the slow-loop cadence drift
        # whenever the fast period changes. It's also gated by
        # self.slow_loop_enabled - the GUI only turns this on while the
        # Slow Loop Monitor tab or the Sensor Monitor popup is actually
        # visible, so temp/status/fw/type polling doesn't compete for bus
        # time with the fast loop while nobody's looking at it.
        SLOW_LOOP_PERIOD_S = 0.5  # ~2 Hz is plenty; temp/status/fw don't
                                  # change fast, and it's fine if it drifts a
                                  # bit slower under load - it never has to
                                  # compete with the fast loop for priority.
        next_slow_poll = time.perf_counter()

        # GUI updates are throttled independently of the raw bus poll rate.
        # The bus polls as fast as fast_loop_period_ms allows (down to ~2ms/
        # 500Hz), but redrawing tables/labels at that rate is both wasted
        # (nobody can see 500 updates/sec) and, worse, becomes the actual
        # bottleneck: every Qt cross-thread signal emit + widget update
        # competes with the poll loop for the GIL. Decoupling lets the
        # background comms run as fast as the hardware allows while the GUI
        # redraws at a smooth, cheap ~20 Hz / ~7 Hz.
        FAST_UI_PERIOD_S = 0.05    # ~20 Hz joint/hPa/IMU table refresh
        STATS_UI_PERIOD_S = 0.15   # ~7 Hz counters/timing-table refresh
        next_fast_ui = time.perf_counter()
        next_stats_ui = time.perf_counter()

        while self.running:
            cycle_start = time.perf_counter()

            # Fast loop: plain FC03 read, 62 regs @0x0000 = pos(20)+cur(20)+
            # imu(4)+hPa(18). main.c calls mb_register_fast_shadow_update()
            # on its own periodic timer (independent of any modbus
            # transaction), so a straightforward FC03 read here already
            # returns live feedback - no need to combine it with a write via
            # FC23. notify=False: stats are still recorded every cycle, just
            # not pushed to the GUI per-transaction (see FAST_UI_PERIOD_S).
            res_fast = self.read_holding_regs(MODBUS_ADDR['get_pos'], 62, notify=False)
            if res_fast is not None and not res_fast.isError():
                now_fast = time.perf_counter()
                if now_fast >= next_fast_ui:
                    next_fast_ui = now_fast + FAST_UI_PERIOD_S
                    regs = res_fast.registers
                    pos = np.array(regs[0:20], dtype=np.uint16).astype(np.int16).tolist()
                    cur = np.array(regs[20:40], dtype=np.uint16).astype(np.int16).tolist()
                    imu = np.array(regs[40:44], dtype=np.uint16).astype(np.int16).tolist()
                    hpa = np.array(regs[44:62], dtype=np.uint16).tolist()
                    self.fast_data_updated.emit(pos, cur, hpa, imu)

            now = time.perf_counter()
            if self.slow_loop_enabled and (now >= next_slow_poll or self._slow_poll_due_now):
                self._slow_poll_due_now = False
                next_slow_poll = now + SLOW_LOOP_PERIOD_S
                # Slow loop: FC03 read, 0x0040..0x0067 inclusive (40 regs,
                # contiguous, no gap). finger_temp(20) + sensor_temp(18) +
                # hand_status(1) + imu_calib(1). fw_ver/hand_type live just
                # past the reserved gap at 0x0070/0x0071 and don't change
                # after connect, so they're read once in
                # _populate_initial_state() instead of on every slow-loop
                # cycle - no need to pay for reading across the gap here.
                res_slow = self.read_holding_regs(MODBUS_ADDR['get_finger_temp'], 40, notify=False)

                if res_slow is not None and not res_slow.isError():
                    regs = res_slow.registers
                    temp = np.array(regs[0:20], dtype=np.uint16).astype(np.int16).tolist()
                    sensor_temp = np.array(regs[20:38], dtype=np.uint16).astype(np.int16).tolist()
                    status = regs[38]
                    imu_calib = regs[39] & 0xFF
                else:
                    temp = [0] * 20
                    sensor_temp = [0] * 18
                    status = 0
                    imu_calib = 0

                self.slow_data_updated.emit(temp, sensor_temp, status, imu_calib)

            # Pace the loop to self.fast_loop_period_ms via time.sleep().
            #
            # HISTORY: during a lengthy debugging investigation into a
            # periodic "~every 32 requests, complete non-response" stall,
            # this was temporarily switched to a CPU-pegging busy-wait
            # after sleep-based pacing appeared correlated with the stall.
            # That correlation turned out to be coincidental - the real
            # root cause was firmware-side: the MCU's UART RX DMA used a
            # 256-byte circular buffer, and its Transfer-Complete (TC)
            # interrupt was left enabled alongside the IDLE-line interrupt.
            # Whenever the DMA write pointer wrapped mid-frame, TC fired a
            # second, spurious HAL_UARTEx_RxEventCallback for that same
            # frame, splitting one Modbus request into two incomplete
            # fragments (each failing CRC and getting silently dropped -
            # hence "no response" from the master's point of view). Since
            # 256 / 8 bytes-per-fast-loop-request = exactly 32, pure
            # monotonous fast-loop traffic always wrapped on a request
            # boundary and never hit it; mixing in a single write (any
            # address, any size other than 8 bytes) shifted every
            # subsequent request's alignment, making the wrap land
            # mid-frame exactly every 32 requests thereafter. Fixed by
            # disabling the DMA TC interrupt
            # (__HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_TC)) so only the
            # IDLE-line interrupt (the actual Modbus frame boundary) ever
            # triggers the callback. With that fixed at the source, a
            # plain time.sleep() here is safe and doesn't peg a CPU core
            # for no reason.
            period_s = max(self.fast_loop_period_ms, 0.1) / 1000.0
            elapsed = time.perf_counter() - cycle_start
            remaining = period_s - elapsed
            if remaining > 0:
                time.sleep(remaining)

            # Record the FULL achieved cycle time (transactions + sleep) as
            # its own "operation" in the existing timing-stats machinery, so
            # it shows up as a "LOOP cycle" row in the RX/TX Timing Monitor
            # tab (avg/min/max + how often it exceeds RT_BUDGET_MS). This is
            # the actual answer to "is the MCU keeping up at this rate?".
            # Pure bookkeeping (no Qt signal), so it costs nothing extra.
            cycle_ms = (time.perf_counter() - cycle_start) * 1000.0
            self._record_timing("LOOP cycle", cycle_ms, False, False)

            # Periodic aggregate refresh for the error/timing monitor tabs -
            # one cheap signal covering everything accumulated since the
            # last tick, instead of one signal per transaction.
            now_stats = time.perf_counter()
            if now_stats >= next_stats_ui:
                next_stats_ui = now_stats + STATS_UI_PERIOD_S
                self.stats_tick.emit()

    def stop(self):
        self.running = False
        self.wait()
        if self.client:
            self.write_coil(MODBUS_COIL_ADDR['torque_enable'], False)
            self.client.close()

#################################
# RTT HISTORY CHART (custom-drawn, no chart-library dependency)
#################################
class RttChartWidget(QWidget):
    """Rolling line chart of recent RTT samples for one operation. Drawn by
    hand with QPainter rather than pulling in matplotlib/pyqtgraph/QtCharts
    - this app's only dependencies are PyQt6/pymodbus/pyserial/numpy, and a
    simple rolling line plot doesn't need more than that."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(220)
        self._data = []
        self._budget_ms = RT_BUDGET_MS

    def set_data(self, data):
        self._data = list(data)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = self.rect()
        w, h = rect.width(), rect.height()
        painter.fillRect(rect, QColor("#1e1e1e"))

        margin_left, margin_right = 50, 12
        margin_top, margin_bottom = 12, 22
        plot_w = w - margin_left - margin_right
        plot_h = h - margin_top - margin_bottom
        if plot_w <= 10 or plot_h <= 10:
            return

        data = self._data
        if not data:
            painter.setPen(QColor("#888"))
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, "No data yet")
            return

        max_val = max(max(data), self._budget_ms) * 1.15
        if max_val <= 0:
            max_val = 1.0

        def x_for(i):
            if len(data) <= 1:
                return float(margin_left)
            return margin_left + plot_w * i / (len(data) - 1)

        def y_for(v):
            return margin_top + plot_h * (1.0 - min(v, max_val) / max_val)

        # Gridlines + y-axis labels (0 / mid / max)
        painter.setPen(QColor("#3a3a3a"))
        for frac in (0.0, 0.5, 1.0):
            y = margin_top + plot_h * (1.0 - frac)
            painter.drawLine(margin_left, int(y), margin_left + plot_w, int(y))
        painter.setPen(QColor("#999"))
        painter.drawText(2, margin_top + 5, f"{max_val:.1f}")
        painter.drawText(2, margin_top + int(plot_h * 0.5) + 5, f"{max_val / 2:.1f}")
        painter.drawText(2, margin_top + plot_h + 4, "0")

        # Real-time budget threshold (dashed)
        budget_y = y_for(self._budget_ms)
        pen = QPen(QColor("#e65100"))
        pen.setStyle(Qt.PenStyle.DashLine)
        painter.setPen(pen)
        painter.drawLine(margin_left, int(budget_y), margin_left + plot_w, int(budget_y))
        painter.drawText(margin_left + plot_w - 78, max(int(budget_y) - 4, margin_top + 10),
                          f"{self._budget_ms:.1f}ms budget")

        # RTT line
        points = [(x_for(i), y_for(v)) for i, v in enumerate(data)]
        pen = QPen(QColor("#42a5f5"))
        pen.setWidth(2)
        painter.setPen(pen)
        for i in range(1, len(points)):
            x1, y1 = points[i - 1]
            x2, y2 = points[i]
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        # Highlight samples over budget
        painter.setPen(QColor("#ff5252"))
        painter.setBrush(QColor("#ff5252"))
        for (x, y), v in zip(points, data):
            if v > self._budget_ms:
                painter.drawEllipse(int(x) - 2, int(y) - 2, 4, 4)

        painter.setPen(QColor("#999"))
        painter.drawText(margin_left, h - 6, "oldest")
        fm_latest = "latest"
        painter.drawText(margin_left + plot_w - 8 * len(fm_latest), h - 6, fm_latest)


#################################
# SENSOR MONITOR WINDOW (hPa + Joint Status)
#################################
class SensorMonitorWindow(QDialog):
    """Separate popup window that monitors pressure (hPa) sensors, IMU raw
    data, and hand_status bitmask side by side."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Sensor Monitor (hPa + IMU + Joint Status)")
        self.resize(900, 620)
        # Non-modal so the main window stays interactive
        self.setModal(False)

        main_layout = QHBoxLayout(self)

        # --- Left : Pressure (hPa) table ---
        hpa_group = QGroupBox("Pressure Sensor (hPa)")
        hpa_layout = QVBoxLayout()

        self.hpa_table = QTableWidget(18, 3)
        self.hpa_table.setHorizontalHeaderLabels(["Sensor Location", "Pressure (hPa)", "Visual Scale"])
        self.hpa_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        self.hpa_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        self.hpa_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)

        self.hpa_bars = []
        idx = 0
        for f in FINGER_NAMES:
            for s in range(3):
                self.hpa_table.setItem(idx, 0, QTableWidgetItem(f"{f}_Sensor_{s+1}"))
                self.hpa_table.setItem(idx, 1, QTableWidgetItem("0"))
                bar = QProgressBar()
                bar.setRange(0, 2000)
                bar.setValue(0)
                self.hpa_table.setCellWidget(idx, 2, bar)
                self.hpa_bars.append(bar)
                idx += 1

        for p in PALM_SENSORS:
            self.hpa_table.setItem(idx, 0, QTableWidgetItem(p))
            self.hpa_table.setItem(idx, 1, QTableWidgetItem("0"))
            bar = QProgressBar()
            bar.setRange(0, 2000)
            bar.setValue(0)
            self.hpa_table.setCellWidget(idx, 2, bar)
            self.hpa_bars.append(bar)
            idx += 1

        hpa_layout.addWidget(self.hpa_table)
        hpa_group.setLayout(hpa_layout)
        main_layout.addWidget(hpa_group, 2)

        # --- Middle : IMU (0x0028 data + 0x0067 calib), profile-aware ---
        imu_group = QGroupBox("IMU (BNO055)")
        imu_layout = QVBoxLayout()

        profile_row = QHBoxLayout()
        profile_row.addWidget(QLabel("Profile:"))
        self.imu_profile_combo = QComboBox()
        for val in sorted(IMU_PROFILES.keys()):
            name, _is_raw = IMU_PROFILES[val]
            self.imu_profile_combo.addItem(f"{name} (0x{val:02X})", val)
        self.imu_profile_combo.currentIndexChanged.connect(self._on_imu_profile_changed)
        profile_row.addWidget(self.imu_profile_combo)
        imu_layout.addLayout(profile_row)

        self._imu_profile_loading = False   # guards against write-on-refresh feedback loop
        self._current_imu_profile = 0x0C    # NDOF default, corrected on window open

        self.imu_data_labels = []
        for i in range(4):
            lbl = QLabel(f"[{i}]: 0")
            lbl.setStyleSheet("font-weight: bold; font-size: 13px;")
            imu_layout.addWidget(lbl)
            self.imu_data_labels.append(lbl)

        imu_layout.addWidget(QLabel(""))  # spacer

        calib_box = QGroupBox("Calibration (0x0067)")
        calib_layout = QVBoxLayout()
        self.imu_calib_labels = {}
        for key in ("SYS", "GYR", "ACC", "MAG"):
            lbl = QLabel(f"{key}: -/3")
            lbl.setStyleSheet("color: gray;")
            calib_layout.addWidget(lbl)
            self.imu_calib_labels[key] = lbl
        calib_box.setLayout(calib_layout)
        imu_layout.addWidget(calib_box)

        imu_layout.addStretch()
        imu_group.setLayout(imu_layout)
        main_layout.addWidget(imu_group, 1)

        # 창을 열 때 현재 profile 값을 device에서 한 번 읽어와 콤보박스에 반영
        self._refresh_imu_profile()

        # --- Right : Hand Status bitmask ---
        status_group = QGroupBox("Hand Status (0x0066)")
        status_layout = QVBoxLayout()

        self.lbl_status_raw = QLabel("Raw: 0x0000")
        self.lbl_status_raw.setStyleSheet("font-weight: bold; font-size: 14px;")
        status_layout.addWidget(self.lbl_status_raw)

        self.status_bit_labels = []
        for bit_idx, bit_name in enumerate(HAND_STATUS_BITS):
            lbl = QLabel(f"[{bit_idx}] {bit_name}: OFF")
            lbl.setStyleSheet("color: gray;")
            status_layout.addWidget(lbl)
            self.status_bit_labels.append(lbl)

        status_layout.addStretch()
        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group, 1)

    def update_hpa(self, hpa):
        """Update pressure sensor values (called from fast loop)."""
        for i in range(min(18, len(hpa))):
            self.hpa_table.item(i, 1).setText(str(hpa[i]))
            self.hpa_bars[i].setValue(hpa[i])

    def update_imu(self, imu):
        """Update IMU raw register values (called from fast loop). Label
        text depends on the currently-known profile (raw XYZ vs quaternion)."""
        name, is_raw = IMU_PROFILES.get(self._current_imu_profile, ("?", True))
        if is_raw:
            labels = [f"{name}_X", f"{name}_Y", f"{name}_Z", "(reserved)"]
        else:
            labels = ["W", "X", "Y", "Z"]
        for i in range(min(4, len(imu))):
            self.imu_data_labels[i].setText(f"{labels[i]}: {imu[i]}")

    def update_imu_calib(self, calib_packed):
        """Update calibration status (called from slow loop).
        bit[7:6]=SYS bit[5:4]=GYR bit[3:2]=ACC bit[1:0]=MAG, each 0~3."""
        fields = {
            "SYS": (calib_packed >> 6) & 0x03,
            "GYR": (calib_packed >> 4) & 0x03,
            "ACC": (calib_packed >> 2) & 0x03,
            "MAG": calib_packed & 0x03,
        }
        for key, val in fields.items():
            lbl = self.imu_calib_labels[key]
            lbl.setText(f"{key}: {val}/3")
            lbl.setStyleSheet("color: green; font-weight: bold;" if val == 3 else "color: gray;")

    def _refresh_imu_profile(self):
        """Read the device's current imu_config (0x0302) once and sync the
        combo box without triggering a write-back."""
        main_win = self.parent()
        if main_win is None or not hasattr(main_win, "worker"):
            return
        res = main_win.worker.read_holding_regs(MODBUS_ADDR['cfg_imu_config'], 1)
        if res is None or res.isError():
            return
        val = res.registers[0] & 0xFF
        if val not in IMU_PROFILES:
            return
        self._current_imu_profile = val
        self._imu_profile_loading = True
        idx = self.imu_profile_combo.findData(val)
        if idx >= 0:
            self.imu_profile_combo.setCurrentIndex(idx)
        self._imu_profile_loading = False

    def _on_imu_profile_changed(self, index):
        """User picked a new profile — write it (FC16) to the device.
        Guarded so refreshing the combo from the device doesn't loop back
        into a write."""
        if self._imu_profile_loading:
            return
        val = self.imu_profile_combo.itemData(index)
        if val is None:
            return
        main_win = self.parent()
        if main_win is None or not hasattr(main_win, "worker"):
            return
        main_win.worker.write_regs(MODBUS_ADDR['cfg_imu_config'], [val])
        self._current_imu_profile = val   # 낙관적 갱신 — 실패해도 다음 폴링에서 실제값과 다시 맞춰짐

    def update_status(self, status):
        """Update hand_status bitmask (called from slow loop)."""
        self.lbl_status_raw.setText(f"Raw: 0x{status:04X}")
        for bit_idx, lbl in enumerate(self.status_bit_labels):
            bit_on = bool(status & (1 << bit_idx))
            name = HAND_STATUS_BITS[bit_idx]
            if bit_on:
                lbl.setText(f"[{bit_idx}] {name}: ON")
                lbl.setStyleSheet("color: red; font-weight: bold;")
            else:
                lbl.setText(f"[{bit_idx}] {name}: OFF")
                lbl.setStyleSheet("color: gray;")

#################################
# MAIN GUI WINDOW
#################################
class HandControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HAND V6 CONTROLLER & TRAJECTORY PLAYER")
        self.resize(1300, 900)

        self.worker = None
        self.traj_runner = None
        self.torque_state = False
        self.calib_state = False
        self.sensor_window = None  # Separate hPa + joint status monitor window

        self.init_ui()

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        # Left Column: Control Panel
        left_layout = QVBoxLayout()
        left_layout.addWidget(self.create_conn_group())
        left_layout.addWidget(self.create_cmd_group())
        left_layout.addWidget(self.create_demo_pos_group())  # New Demo Position buttons
        left_layout.addWidget(self.create_grasp_group())  # Grasp Save/Play device slots
        left_layout.addWidget(self.create_trajectory_group())  # Trajectory Motion Group
        left_layout.addWidget(self.create_log_group())

        # Right Column: Data Monitor
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.create_status_group())

        self.tabs = QTabWidget()
        self.tabs.addTab(self.create_table_group(), "Joint Control & Monitor")
        self.SLOW_LOOP_TAB_INDEX = self.tabs.addTab(self.create_slow_loop_group(), "Slow Loop Monitor")
        self.tabs.addTab(self.create_pid_group(), "PID Gain Tuning")
        self.tabs.addTab(self.create_config_group(), "Network / System Config")
        self.tabs.addTab(self.create_error_group(), "Comm / Error Monitor")
        self.tabs.addTab(self.create_timing_group(), "RX/TX Timing Monitor")
        self.tabs.currentChanged.connect(self._update_slow_loop_gate)
        right_layout.addWidget(self.tabs)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        left_widget = QWidget()
        left_widget.setLayout(left_layout)
        right_widget = QWidget()
        right_widget.setLayout(right_layout)

        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setSizes([450, 850])

        main_layout.addWidget(splitter)

    # --- Group Creators ---
    def create_conn_group(self):
        group = QGroupBox("1. Communication Setup")
        layout = QGridLayout()

        # Connection type selection : RTU / TCP
        layout.addWidget(QLabel("Type:"), 0, 0)
        self.conn_type_input = QComboBox()
        self.conn_type_input.addItems([CONN_TCP, CONN_RTU])
        self.conn_type_input.setCurrentText(CONN_TCP)
        self.conn_type_input.currentIndexChanged.connect(self.on_conn_type_changed)
        layout.addWidget(self.conn_type_input, 0, 1, 1, 3)

        # --- RTU widgets (row 1) ---
        self.lbl_port = QLabel("Port:")
        layout.addWidget(self.lbl_port, 1, 0)
        self.port_input = QComboBox()
        self.port_input.addItems([f"COM{i}" for i in range(1, 65)])
        self.port_input.setCurrentText(COM_PORT)
        layout.addWidget(self.port_input, 1, 1)

        self.lbl_baud = QLabel("Baudrate:")
        layout.addWidget(self.lbl_baud, 1, 2)
        self.baud_input = QComboBox()
        self.baud_input.addItems(["921600", "2000000", "4000000"])
        self.baud_input.setCurrentText(str(BAUDRATE))
        layout.addWidget(self.baud_input, 1, 3)

        # --- TCP widgets (row 1, overlapped with RTU; only one set visible) ---
        self.lbl_ip = QLabel("IP:")
        layout.addWidget(self.lbl_ip, 1, 0)
        self.ip_input = QLineEdit(TCP_HOST)
        layout.addWidget(self.ip_input, 1, 1)

        self.lbl_tcp_port = QLabel("TCP Port:")
        layout.addWidget(self.lbl_tcp_port, 1, 2)
        self.tcp_port_input = QLineEdit(str(TCP_PORT))
        layout.addWidget(self.tcp_port_input, 1, 3)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connection)
        layout.addWidget(self.btn_connect, 2, 0, 1, 2)

        self.lbl_conn_status = QLabel("Status: Disconnected")
        self.lbl_conn_status.setStyleSheet("color: red; font-weight: bold;")
        layout.addWidget(self.lbl_conn_status, 2, 2, 1, 2)

        group.setLayout(layout)

        # Apply initial visibility (RTU shown, TCP hidden)
        self.on_conn_type_changed()
        return group

    def on_conn_type_changed(self):
        """Toggle RTU / TCP input widgets based on selected connection type."""
        is_tcp = (self.conn_type_input.currentText() == CONN_TCP)

        # RTU widgets
        for w in (self.lbl_port, self.port_input, self.lbl_baud, self.baud_input):
            w.setVisible(not is_tcp)
        # TCP widgets
        for w in (self.lbl_ip, self.ip_input, self.lbl_tcp_port, self.tcp_port_input):
            w.setVisible(is_tcp)

    def create_cmd_group(self):
        group = QGroupBox("2. Hand Basic Control")
        layout = QVBoxLayout()

        h1 = QHBoxLayout()
        self.btn_torque = QPushButton("TORQUE OFF")
        self.btn_torque.setStyleSheet("background-color: #ff4d4d; color: white; font-weight: bold; font-size: 14px;")
        self.btn_torque.setFixedHeight(40)
        self.btn_torque.clicked.connect(self.toggle_torque)
        h1.addWidget(self.btn_torque)

        self.combo_mode = QComboBox()
        self.combo_mode.addItems(["0: Current Control", "5: Pos Control (Curr-based)"])
        self.combo_mode.setCurrentIndex(1)
        self.combo_mode.currentIndexChanged.connect(self.change_mode)
        h1.addWidget(self.combo_mode)
        layout.addLayout(h1)

        h2 = QHBoxLayout()
        h2.addWidget(QLabel("Current Limit (mA):"))
        self.spin_cur_limit = QSpinBox()
        self.spin_cur_limit.setRange(0, 500)
        self.spin_cur_limit.setValue(200)
        h2.addWidget(self.spin_cur_limit)

        btn_apply_cur = QPushButton("Apply Limit")
        btn_apply_cur.clicked.connect(self.apply_current_limit)
        h2.addWidget(btn_apply_cur)
        layout.addLayout(h2)

        # Calibration toggle (joint_calib coil @0x0001 : 0=Stop, 1=Start)
        self.btn_calib = QPushButton("CALIBRATION: STOPPED")
        self.btn_calib.setStyleSheet("background-color: #9E9E9E; color: white; font-weight: bold;")
        self.btn_calib.setFixedHeight(34)
        self.btn_calib.clicked.connect(self.toggle_calibration)
        layout.addWidget(self.btn_calib)

        group.setLayout(layout)
        return group

    def create_demo_pos_group(self):
        """Demo position preset buttons. Click -> deg2pulse -> send goal_pos."""
        group = QGroupBox("3. Demo Position Presets (raw pulse)")
        layout = QGridLayout()

        self.demo_pos_buttons = []
        col_count = 2
        for i, name in enumerate(DEMO_POSITIONS.keys()):
            btn = QPushButton(name)
            btn.setStyleSheet("background-color: #607D8B; color: white; font-weight: bold;")
            btn.setFixedHeight(34)
            # Bind current name via default arg to avoid late-binding closure bug
            btn.clicked.connect(lambda _checked=False, n=name: self.send_demo_position(n))
            layout.addWidget(btn, i // col_count, i % col_count)
            self.demo_pos_buttons.append(btn)

        group.setLayout(layout)
        return group

    def create_grasp_group(self):
        """Grasp Save/Play slots (0x0129 / 0x012A) - the MCU's own on-device
        gesture memory, separate from the GUI-side Demo Position / Trajectory
        presets above. Save stores the hand's CURRENT position into a slot
        on the device; Play recalls a previously-saved slot. Firmware uses
        slots 1~5 (5 slots total); slot 0 is not a valid slot."""
        group = QGroupBox("4. Grasp Save / Play (device slots)")
        layout = QGridLayout()

        layout.addWidget(QLabel("Slot (1-5):"), 0, 0)
        self.spin_grasp_slot = QSpinBox()
        self.spin_grasp_slot.setRange(1, 5)
        layout.addWidget(self.spin_grasp_slot, 0, 1)

        btn_grasp_save = QPushButton("Save Grasp")
        btn_grasp_save.setStyleSheet("background-color: #6D4C41; color: white; font-weight: bold;")
        btn_grasp_save.setFixedHeight(34)
        btn_grasp_save.clicked.connect(self.save_grasp_slot)
        layout.addWidget(btn_grasp_save, 1, 0)

        btn_grasp_play = QPushButton("Play Grasp")
        btn_grasp_play.setStyleSheet("background-color: #00897B; color: white; font-weight: bold;")
        btn_grasp_play.setFixedHeight(34)
        btn_grasp_play.clicked.connect(self.play_grasp_slot)
        layout.addWidget(btn_grasp_play, 1, 1)

        group.setLayout(layout)
        return group

    def create_trajectory_group(self):
        group = QGroupBox("5. Trajectory Motion Playback")
        layout = QVBoxLayout()

        h_layout = QHBoxLayout()
        self.combo_traj = QComboBox()
        # Individual motions + a special entry that plays them all back-to-back
        self.combo_traj.addItems(list(DEMO_TRAJECTORIES.keys()))
        self.combo_traj.addItem(TRAJ_ALL_SEQUENTIAL)
        h_layout.addWidget(self.combo_traj)

        self.btn_play_traj = QPushButton("Start Motion")
        self.btn_play_traj.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;")
        self.btn_play_traj.clicked.connect(self.toggle_trajectory)
        h_layout.addWidget(self.btn_play_traj)

        layout.addLayout(h_layout)

        self.chk_traj_loop = QCheckBox("Loop Playback (repeat)")
        layout.addWidget(self.chk_traj_loop)

        self.lbl_traj_status = QLabel("Status: Idle")
        layout.addWidget(self.lbl_traj_status)

        group.setLayout(layout)
        return group

    def create_status_group(self):
        group = QGroupBox("System Information")
        layout = QHBoxLayout()

        self.lbl_fw = QLabel("FW Ver: -")
        self.lbl_type = QLabel("Hand Type: -")
        self.lbl_status = QLabel("Status Bit: -")

        layout.addWidget(self.lbl_fw)
        layout.addWidget(self.lbl_type)
        layout.addWidget(self.lbl_status)

        layout.addStretch()
        self.btn_sensor_win = QPushButton("Open Sensor Monitor")
        self.btn_sensor_win.setStyleSheet("background-color: #009688; color: white; font-weight: bold;")
        self.btn_sensor_win.clicked.connect(self.open_sensor_window)
        layout.addWidget(self.btn_sensor_win)

        group.setLayout(layout)
        return group

    def open_sensor_window(self):
        """Open (or bring to front) the separate hPa + joint status monitor."""
        if self.sensor_window is None:
            self.sensor_window = SensorMonitorWindow(self)
            # Clear reference when the window is destroyed
            self.sensor_window.finished.connect(self._on_sensor_window_closed)
        self.sensor_window.show()
        self.sensor_window.raise_()
        self.sensor_window.activateWindow()
        self._update_slow_loop_gate()

    def _on_sensor_window_closed(self, _result):
        self.sensor_window = None
        self._update_slow_loop_gate()

    def _update_slow_loop_gate(self, *_args):
        """Enable the worker's slow feedback poll (finger_temp/sensor_temp/
        hand_status/fw_ver/hand_type) only while something that actually
        shows it is visible: the Slow Loop Monitor tab, or the Sensor
        Monitor popup (which shows hand_status). Otherwise it's paused so
        it doesn't compete with the fast loop for bus time. Connected to
        QTabWidget.currentChanged and called directly from the sensor
        window's open/close handlers; *_args absorbs the tab-changed int.
        """
        if self.worker is None:
            return
        slow_tab_active = (self.tabs.currentIndex() == self.SLOW_LOOP_TAB_INDEX)
        sensor_window_open = (self.sensor_window is not None and self.sensor_window.isVisible())
        enabled = slow_tab_active or sensor_window_open
        if enabled and not self.worker.slow_loop_enabled:
            # Just became visible - get fresh data immediately rather than
            # waiting up to 500ms for the next scheduled poll.
            self.worker._slow_poll_due_now = True
        self.worker.slow_loop_enabled = enabled

    def create_table_group(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)

        self.table = QTableWidget(20, 4)
        self.table.setHorizontalHeaderLabels(["Finger / Joint", "Pos Feed (LSB)", "Goal (deg)", "Curr Feed (mA)"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)

        row_idx = 0
        for f in FINGER_NAMES:
            for j in range(JOINT_CNT):
                self.table.setItem(row_idx, 0, QTableWidgetItem(f"{f}_J{j}"))
                self.table.setItem(row_idx, 1, QTableWidgetItem("0"))
                self.table.setItem(row_idx, 2, QTableWidgetItem("0"))
                self.table.setItem(row_idx, 3, QTableWidgetItem("0"))
                row_idx += 1

        layout.addWidget(self.table)

        btn_send_custom_pos = QPushButton("Send Custom Joint Positions (Goal deg -> pulse)")
        btn_send_custom_pos.setStyleSheet("font-weight: bold; background-color: #e1e1e1;")
        btn_send_custom_pos.clicked.connect(self.send_custom_positions)
        layout.addWidget(btn_send_custom_pos)

        return widget

    def create_slow_loop_group(self):
        """Slow-loop input-register monitor tab: raw register dump of the
        0x0100+ block (finger temp, sensor temp, hand_status, fw/type/serial)."""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        info = QLabel(
            "Slow-loop registers (FC03, ~2 Hz). Read-only raw values "
            "from the 0x0040 block. Only polled while this tab (or the "
            "Sensor Monitor popup) is visible, so it doesn't compete with "
            "the fast loop for bus time otherwise. FW Ver / Hand Type "
            "don't change after connect, so they're shown in the System "
            "Information bar above (read once) rather than here."
        )
        info.setWordWrap(True)
        info.setStyleSheet("color: #555;")
        layout.addWidget(info)

        tables_layout = QHBoxLayout()

        # Finger/joint temperature table (0x0040, 20 regs)
        temp_group = QGroupBox("Motor Temp (0x0040)")
        temp_layout = QVBoxLayout()
        self.slow_temp_table = QTableWidget(20, 2)
        self.slow_temp_table.setHorizontalHeaderLabels(["Finger / Joint", "Temp (raw)"])
        self.slow_temp_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        r = 0
        for f in FINGER_NAMES:
            for j in range(JOINT_CNT):
                self.slow_temp_table.setItem(r, 0, QTableWidgetItem(f"{f}_J{j}"))
                self.slow_temp_table.setItem(r, 1, QTableWidgetItem("0"))
                r += 1
        temp_layout.addWidget(self.slow_temp_table)
        temp_group.setLayout(temp_layout)
        tables_layout.addWidget(temp_group)

        # Pressure-sensor temperature table (0x0054, 18 regs) - same
        # finger x3 + palm x3 slot layout as the hPa table.
        sensor_temp_group = QGroupBox("Sensor Temp (0x0054)")
        sensor_temp_layout = QVBoxLayout()
        self.slow_sensor_temp_table = QTableWidget(18, 2)
        self.slow_sensor_temp_table.setHorizontalHeaderLabels(["Sensor Location", "Temp (raw)"])
        self.slow_sensor_temp_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        r = 0
        for f in FINGER_NAMES:
            for s in range(3):
                self.slow_sensor_temp_table.setItem(r, 0, QTableWidgetItem(f"{f}_Sensor_{s+1}"))
                self.slow_sensor_temp_table.setItem(r, 1, QTableWidgetItem("0"))
                r += 1
        for p in PALM_SENSORS:
            self.slow_sensor_temp_table.setItem(r, 0, QTableWidgetItem(p))
            self.slow_sensor_temp_table.setItem(r, 1, QTableWidgetItem("0"))
            r += 1
        sensor_temp_layout.addWidget(self.slow_sensor_temp_table)
        sensor_temp_group.setLayout(sensor_temp_layout)
        tables_layout.addWidget(sensor_temp_group)

        layout.addLayout(tables_layout)

        # Summary
        self.lbl_slow_status = QLabel("hand_status (0x0066): 0x0000")
        layout.addWidget(self.lbl_slow_status)

        return widget

    def create_pid_group(self):
        """PID gain tuning tab. Reads/writes gain_kp/ki/kd (uint16 x20 each)
        as holding registers. vel/acc gains (uint32) are shown per-joint too."""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        info = QLabel(
            "PID / motion gains (holding registers). Edit Kp/Ki/Kd/Vel/Acc per "
            "joint, then Write. Read pulls current values from the device."
        )
        info.setStyleSheet("color: #555;")
        layout.addWidget(info)

        self.pid_table = QTableWidget(TOTAL_JOINT_CNT, 6)
        self.pid_table.setHorizontalHeaderLabels(
            ["Finger / Joint", "Kp", "Ki", "Kd", "Vel", "Acc"]
        )
        self.pid_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        r = 0
        for f in FINGER_NAMES:
            for j in range(JOINT_CNT):
                name_item = QTableWidgetItem(f"{f}_J{j}")
                name_item.setFlags(name_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                self.pid_table.setItem(r, 0, name_item)
                for c in range(1, 6):
                    self.pid_table.setItem(r, c, QTableWidgetItem("0"))
                r += 1
        layout.addWidget(self.pid_table)

        h = QHBoxLayout()
        btn_read_pid = QPushButton("Read Gains")
        btn_read_pid.clicked.connect(self.read_pid_gains)
        h.addWidget(btn_read_pid)
        btn_write_pid = QPushButton("Write Gains")
        btn_write_pid.setStyleSheet("font-weight: bold; background-color: #FFB300;")
        btn_write_pid.clicked.connect(self.write_pid_gains)
        h.addWidget(btn_write_pid)
        layout.addLayout(h)

        return widget

    def create_config_group(self):
        """Network / system config tab (holding registers @0x0100+)."""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # --- System config ---
        sys_group = QGroupBox("System Config")
        sys_form = QGridLayout()

        sys_form.addWidget(QLabel("Comm Select (comm_sel):"), 0, 0)
        self.cfg_comm_sel = QComboBox()
        self.cfg_comm_sel.addItems(["0: Modbus RTU", "1: Custom"])
        sys_form.addWidget(self.cfg_comm_sel, 0, 1)

        sys_form.addWidget(QLabel("Slave ID:"), 1, 0)
        self.cfg_slave_id = QSpinBox()
        self.cfg_slave_id.setRange(1, 247)
        self.cfg_slave_id.setValue(1)
        sys_form.addWidget(self.cfg_slave_id, 1, 1)

        sys_form.addWidget(QLabel("IMU Config:"), 2, 0)
        self.cfg_imu = QSpinBox()
        self.cfg_imu.setRange(0, 65535)
        sys_form.addWidget(self.cfg_imu, 2, 1)

        sys_form.addWidget(QLabel("Sys Rev0 (RS-485 Baud CFG, TBD):"), 3, 0)
        self.cfg_sys_rev0 = QSpinBox()
        self.cfg_sys_rev0.setRange(0, 65535)
        sys_form.addWidget(self.cfg_sys_rev0, 3, 1)

        sys_form.addWidget(QLabel("Sys Rev1 (Dev Option, TBD):"), 4, 0)
        self.cfg_sys_rev1 = QSpinBox()
        self.cfg_sys_rev1.setRange(0, 65535)
        sys_form.addWidget(self.cfg_sys_rev1, 4, 1)

        sys_note = QLabel(
            "Sys Rev0/Rev1 are reserved/TBD on the firmware side - shown "
            "here so the full 5-register system block round-trips cleanly, "
            "but they don't do anything yet."
        )
        sys_note.setWordWrap(True)
        sys_note.setStyleSheet("color: #888; font-style: italic;")
        sys_form.addWidget(sys_note, 5, 0, 1, 2)

        sys_group.setLayout(sys_form)
        layout.addWidget(sys_group)

        # --- Network config ---
        net_group = QGroupBox("Network Config")
        net_note = QLabel(
            "Note: firmware currently reports this block as always 0x0 on "
            "read-back; the network stack side of these registers is not "
            "wired up yet."
        )
        net_note.setWordWrap(True)
        net_note.setStyleSheet("color: #888; font-style: italic;")
        net_form = QGridLayout()

        net_form.addWidget(QLabel("IP Address:"), 0, 0)
        self.cfg_ip = QLineEdit("192.168.1.100")
        net_form.addWidget(self.cfg_ip, 0, 1)

        net_form.addWidget(QLabel("Subnet Mask:"), 1, 0)
        self.cfg_mask = QLineEdit("255.255.255.0")
        net_form.addWidget(self.cfg_mask, 1, 1)

        net_form.addWidget(QLabel("Gateway:"), 2, 0)
        self.cfg_gw = QLineEdit("192.168.1.1")
        net_form.addWidget(self.cfg_gw, 2, 1)

        net_form.addWidget(QLabel("DNS:"), 3, 0)
        self.cfg_dns = QLineEdit("8.8.8.8")
        net_form.addWidget(self.cfg_dns, 3, 1)

        net_wrap = QVBoxLayout()
        net_wrap.addWidget(net_note)
        net_wrap.addLayout(net_form)
        net_group.setLayout(net_wrap)
        layout.addWidget(net_group)

        # --- Factory settings ---
        factory_group = QGroupBox("Factory Settings (0x0318 Hand SN / 0x0320 Factory PW)")
        factory_layout = QGridLayout()

        factory_note = QLabel(
            "Writing the Hand Serial requires a successful factory-password "
            "authenticate first (checked device-side). A write transaction "
            "won't show an error even if the device silently rejected it "
            "for being unauthenticated - use Read Serial afterward to "
            "confirm it actually took."
        )
        factory_note.setWordWrap(True)
        factory_note.setStyleSheet("color: #888; font-style: italic;")
        factory_layout.addWidget(factory_note, 0, 0, 1, 3)

        factory_layout.addWidget(QLabel("Factory Password (hex):"), 1, 0)
        self.cfg_factory_pwd_input = QLineEdit("A5A5")
        self.cfg_factory_pwd_input.setMaxLength(4)
        factory_layout.addWidget(self.cfg_factory_pwd_input, 1, 1)
        btn_auth = QPushButton("Authenticate")
        btn_auth.clicked.connect(self.write_factory_password)
        factory_layout.addWidget(btn_auth, 1, 2)

        self.lbl_factory_auth_status = QLabel("Auth Status: Unknown")
        self.lbl_factory_auth_status.setStyleSheet("font-weight: bold;")
        factory_layout.addWidget(self.lbl_factory_auth_status, 2, 0, 1, 2)
        btn_check_auth = QPushButton("Check Status")
        btn_check_auth.clicked.connect(self.check_factory_auth_status)
        factory_layout.addWidget(btn_check_auth, 2, 2)

        factory_layout.addWidget(QLabel("Hand Serial No (up to 8 chars):"), 3, 0)
        self.cfg_hand_sn_input = QLineEdit()
        self.cfg_hand_sn_input.setMaxLength(8)
        self.cfg_hand_sn_input.setPlaceholderText("e.g. V5SD0001")
        factory_layout.addWidget(self.cfg_hand_sn_input, 3, 1)
        btn_write_sn = QPushButton("Write Serial")
        btn_write_sn.setStyleSheet("font-weight: bold; background-color: #FFB300;")
        btn_write_sn.clicked.connect(self.write_hand_serial)
        factory_layout.addWidget(btn_write_sn, 3, 2)

        self.lbl_hand_sn_current = QLabel("Current Serial: -")
        factory_layout.addWidget(self.lbl_hand_sn_current, 4, 0, 1, 2)
        btn_read_sn = QPushButton("Read Serial")
        btn_read_sn.clicked.connect(self.read_hand_serial)
        factory_layout.addWidget(btn_read_sn, 4, 2)

        factory_group.setLayout(factory_layout)
        layout.addWidget(factory_group)

        h = QHBoxLayout()
        btn_read_cfg = QPushButton("Read Config")
        btn_read_cfg.clicked.connect(self.read_config)
        h.addWidget(btn_read_cfg)
        btn_write_cfg = QPushButton("Write Config")
        btn_write_cfg.setStyleSheet("font-weight: bold; background-color: #FFB300;")
        btn_write_cfg.clicked.connect(self.write_config)
        h.addWidget(btn_write_cfg)
        layout.addLayout(h)

        layout.addStretch()
        return widget

    def create_error_group(self):
        """Communication / error monitoring tab. Shows live TX/error counters,
        per-request health, an error-type breakdown, and a scrolling error
        log fed by comm_event."""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        stat_group = QGroupBox("Communication Statistics")
        stat_layout = QGridLayout()
        self.lbl_tx = QLabel("TX: 0")
        self.lbl_err = QLabel("Errors: 0")
        self.lbl_err_rate = QLabel("Error Rate: 0.0 %")
        self.lbl_last_err = QLabel("Last Error: -")
        for lbl in (self.lbl_tx, self.lbl_err, self.lbl_err_rate):
            lbl.setStyleSheet("font-weight: bold; font-size: 13px;")
        self.lbl_err.setStyleSheet("font-weight: bold; font-size: 13px; color: #c62828;")
        stat_layout.addWidget(self.lbl_tx, 0, 0)
        stat_layout.addWidget(self.lbl_err, 0, 1)
        stat_layout.addWidget(self.lbl_err_rate, 0, 2)
        stat_layout.addWidget(self.lbl_last_err, 1, 0, 1, 3)
        stat_group.setLayout(stat_layout)
        layout.addWidget(stat_group)

        # --- Error type breakdown ---
        err_type_group = QGroupBox("Error Types")
        err_type_layout = QVBoxLayout()
        self.error_type_table = QTableWidget(0, 2)
        self.error_type_table.setHorizontalHeaderLabels(["Type", "Count"])
        self.error_type_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.error_type_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        self.error_type_table.verticalHeader().setVisible(False)
        self.error_type_table.setMaximumHeight(180)
        self._error_type_rows = {}  # category -> row index
        err_type_layout.addWidget(self.error_type_table)
        err_type_group.setLayout(err_type_layout)
        layout.addWidget(err_type_group)

        h = QHBoxLayout()
        btn_clear = QPushButton("Clear Error Log")
        btn_clear.clicked.connect(self.clear_error_log)
        h.addStretch()
        h.addWidget(btn_clear)
        layout.addLayout(h)

        self.txt_errlog = QTextEdit()
        self.txt_errlog.setReadOnly(True)
        self.txt_errlog.setStyleSheet("font-family: monospace; color: #b71c1c;")
        layout.addWidget(self.txt_errlog)

        return widget

    def create_timing_group(self):
        """RX/TX timing monitor tab. Shows live round-trip time (RTT) stats so
        the operator can verify the MCU services each transaction within the
        real-time budget (RT_BUDGET_MS = 2 ms) and spot packet errors/timeouts.

        RTT here is the FULL master-side round trip (request tx + MCU handling
        + response rx + serial/TCP transport), so it is an upper bound on the
        MCU's own per-packet processing time.
        """
        widget = QWidget()
        layout = QVBoxLayout(widget)

        info = QLabel(
            f"Round-trip time (RTT) measured at the master. Real-time budget: "
            f"{RT_BUDGET_MS:.1f} ms. RTT includes transport, so it is an upper "
            f"bound on MCU processing time. Rows exceeding the budget are "
            f"highlighted."
        )
        info.setWordWrap(True)
        info.setStyleSheet("color: #555;")
        layout.addWidget(info)

        # --- Fast loop period control ---
        period_group = QGroupBox("Fast Loop Period")
        period_layout = QHBoxLayout()
        period_layout.addWidget(QLabel("Target period (ms):"))
        self.spin_fast_period = QDoubleSpinBox()
        self.spin_fast_period.setRange(0.5, 100.0)
        self.spin_fast_period.setSingleStep(0.5)
        self.spin_fast_period.setDecimals(1)
        self.spin_fast_period.setValue(FAST_LOOP_PERIOD_MS)
        self.spin_fast_period.valueChanged.connect(self.on_fast_period_changed)
        period_layout.addWidget(self.spin_fast_period)
        period_note = QLabel(
            "Drives the fast R/W poll loop this fast to stress-test whether "
            "the MCU keeps up. Watch the 'LOOP cycle' row below - if its avg/"
            "max grows past the target, the MCU (or the transport) can't "
            "sustain this rate."
        )
        period_note.setWordWrap(True)
        period_note.setStyleSheet("color: #888; font-style: italic;")
        period_layout.addWidget(period_note, 1)
        period_group.setLayout(period_layout)
        layout.addWidget(period_group)

        # --- Global summary ---
        sum_group = QGroupBox("Timing Summary")
        sum_layout = QGridLayout()
        self.lbl_t_tx = QLabel("Total TX: 0")
        self.lbl_t_last = QLabel("Last RTT: - ms")
        self.lbl_t_max = QLabel("Max RTT: - ms")
        self.lbl_t_over = QLabel(f"> {RT_BUDGET_MS:.0f}ms: 0 (0.0 %)")
        self.lbl_t_timeout = QLabel("Timeouts: 0")
        for lbl in (self.lbl_t_tx, self.lbl_t_last, self.lbl_t_max):
            lbl.setStyleSheet("font-weight: bold; font-size: 13px;")
        self.lbl_t_over.setStyleSheet("font-weight: bold; font-size: 13px; color: #e65100;")
        self.lbl_t_timeout.setStyleSheet("font-weight: bold; font-size: 13px; color: #c62828;")
        sum_layout.addWidget(self.lbl_t_tx, 0, 0)
        sum_layout.addWidget(self.lbl_t_last, 0, 1)
        sum_layout.addWidget(self.lbl_t_max, 0, 2)
        sum_layout.addWidget(self.lbl_t_over, 1, 0, 1, 2)
        sum_layout.addWidget(self.lbl_t_timeout, 1, 2)
        sum_group.setLayout(sum_layout)
        layout.addWidget(sum_group)

        # --- Per-operation breakdown ---
        self.timing_table = QTableWidget(0, 8)
        self.timing_table.setHorizontalHeaderLabels(
            ["Operation", "Count", "Last (ms)", "Min (ms)",
             "Avg (ms)", "Max (ms)", f"> {RT_BUDGET_MS:.0f}ms", "Errors"]
        )
        # Readability: larger font, taller rows, clear grid + header, and
        # alternating row colours so each operation line stands out.
        self.timing_table.setStyleSheet(
            "QTableWidget {"
            "  font-size: 13px;"
            "  gridline-color: #bdbdbd;"
            "  background-color: #ffffff;"
            "  alternate-background-color: #f2f6fa;"
            "}"
            "QHeaderView::section {"
            "  background-color: #37474f;"
            "  color: white;"
            "  font-weight: bold;"
            "  font-size: 12px;"
            "  padding: 6px 4px;"
            "  border: 0px;"
            "}"
            "QTableWidget::item { padding: 4px 6px; }"
        )
        self.timing_table.setAlternatingRowColors(True)
        self.timing_table.verticalHeader().setVisible(False)
        self.timing_table.verticalHeader().setDefaultSectionSize(30)
        self.timing_table.setMinimumHeight(240)
        # 'Operation' column stretches; numeric columns get a comfortable
        # fixed width (Interactive) so headers like 'Count'/'Last (ms)' are
        # never truncated the way ResizeToContents made them.
        header = self.timing_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        header.setMinimumSectionSize(70)
        for c in range(1, 8):
            header.setSectionResizeMode(c, QHeaderView.ResizeMode.Interactive)
            self.timing_table.setColumnWidth(c, 82)
        # Map op_key -> table row for in-place updates
        self._timing_rows = {}
        layout.addWidget(self.timing_table)

        h = QHBoxLayout()
        btn_reset = QPushButton("Reset Timing Stats")
        btn_reset.clicked.connect(self.reset_timing_stats)
        h.addStretch()
        h.addWidget(btn_reset)
        layout.addLayout(h)

        # --- RTT history chart ---
        chart_group = QGroupBox("RTT History")
        chart_layout = QVBoxLayout()

        chart_header = QHBoxLayout()
        chart_header.addWidget(QLabel("Operation:"))
        self.combo_rtt_chart_op = QComboBox()
        self.combo_rtt_chart_op.setMinimumWidth(220)
        self.combo_rtt_chart_op.currentIndexChanged.connect(self._on_rtt_chart_op_changed)
        chart_header.addWidget(self.combo_rtt_chart_op)
        chart_header.addStretch()
        chart_header.addWidget(QLabel(
            f"Last {RTT_HISTORY_LEN} samples. Orange dashed line = "
            f"{RT_BUDGET_MS:.1f}ms budget. Red dots = over budget."
        ))
        chart_layout.addLayout(chart_header)

        self.rtt_chart = RttChartWidget()
        chart_layout.addWidget(self.rtt_chart)

        chart_group.setLayout(chart_layout)
        layout.addWidget(chart_group)

        return widget

    def create_log_group(self):
        group = QGroupBox("Event Log")
        layout = QVBoxLayout()
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        layout.addWidget(self.txt_log)
        group.setLayout(layout)
        return group

    # --- UI Handlers ---
    def log(self, text):
        timestamp = time.strftime("[%H:%M:%S] ")
        self.txt_log.append(timestamp + text)

    def toggle_connection(self):
        if self.worker and self.worker.isRunning():
            self.stop_trajectory()
            self.worker.stop()
            self.worker = None
            self.btn_connect.setText("Connect")
            self.lbl_conn_status.setText("Status: Disconnected")
            self.lbl_conn_status.setStyleSheet("color: red; font-weight: bold;")

            # worker.stop() force-writes the torque_enable coil to False on
            # the device; keep the GUI's torque/calib state in sync so a
            # reconnect doesn't show "TORQUE ON" while the device is
            # actually off.
            self.torque_state = False
            self.btn_torque.setText("TORQUE OFF")
            self.btn_torque.setStyleSheet("background-color: #ff4d4d; color: white; font-weight: bold; font-size: 14px;")
            self.calib_state = False
            self.btn_calib.setText("CALIBRATION: STOPPED")
            self.btn_calib.setStyleSheet("background-color: #9E9E9E; color: white; font-weight: bold;")

            self.log("Disconnected.")
        else:
            conn_type = self.conn_type_input.currentText()

            if conn_type == CONN_TCP:
                host = self.ip_input.text().strip()
                try:
                    tcp_port = int(self.tcp_port_input.text().strip())
                except ValueError:
                    self.log("Error: TCP Port must be an integer.")
                    return
                self.worker = ModbusWorker(
                    conn_type=CONN_TCP, host=host, tcp_port=tcp_port, slave_id=SLAVE_ID,
                    fast_loop_period_ms=self.spin_fast_period.value()
                )
                self.log(f"Connecting via TCP {host}:{tcp_port} ...")
            else:
                port = self.port_input.currentText()
                baud = int(self.baud_input.currentText())
                self.worker = ModbusWorker(
                    conn_type=CONN_RTU, port=port, baudrate=baud, slave_id=SLAVE_ID,
                    fast_loop_period_ms=self.spin_fast_period.value()
                )
                self.log(f"Connecting via RTU {port} @ {baud} ...")

            self.worker.fast_data_updated.connect(self.update_fast_ui)
            self.worker.slow_data_updated.connect(self.update_slow_ui)
            self.worker.log_signal.connect(self.log)
            self.worker.connection_status.connect(self.handle_conn_status)
            self.worker.comm_event.connect(self.handle_comm_event)
            self.worker.timing_event.connect(self.handle_timing_event)
            self.worker.stats_tick.connect(self.on_stats_tick)

            self.worker.start()

    def handle_conn_status(self, connected, sid):
        if connected:
            self.btn_connect.setText("Disconnect")
            self.lbl_conn_status.setText(f"Connected (ID: {sid})")
            self.lbl_conn_status.setStyleSheet("color: green; font-weight: bold;")
            self._sync_operating_mode_on_connect()
            self._populate_initial_state()
            self._update_slow_loop_gate()
        else:
            self.toggle_connection()

    def _populate_initial_state(self):
        """One-time reads right after connecting, so the PID Gain Tuning and
        Network/System Config tabs show real device values immediately
        instead of staying at their placeholder "0" until the user manually
        presses Read Gains / Read Config, and the System Information bar
        shows the real FW Ver / Hand Type. The fast/slow poll loops handle
        position/current/imu/hPa/temp/sensor_temp/status continuously on
        their own - these blocks change rarely or never after connect, so a
        one-shot read here (never repeated by the poll loop) is all they
        need."""
        self.read_pid_gains()
        self.read_config()
        self._read_device_info()

    def _read_device_info(self):
        """FW Ver (0x0070) + Hand Type (0x0071): fixed for the life of the
        connection (nothing on the MCU side changes them post-boot), so read
        them once here rather than on every slow-loop cycle."""
        if not self.worker or not self.worker.isRunning():
            return
        res = self.worker.read_holding_regs(MODBUS_ADDR['get_fw_ver'], 2)
        if res is not None and not res.isError():
            fw_ver, hand_type = res.registers[0], res.registers[1]
            self.lbl_fw.setText(f"FW Ver: 0x{fw_ver:04X}")
            self.lbl_type.setText(f"Hand Type: {'Right' if hand_type == 1 else 'Left'}")
            self.log(f"Device info read: FW 0x{fw_ver:04X}, "
                     f"Hand Type {'Right' if hand_type == 1 else 'Left'}")

    def _sync_operating_mode_on_connect(self):
        """Push the currently-selected operating mode to the device.

        The mode combo's initial selection is set in create_cmd_group()
        during __init__, before self.worker exists, so the resulting
        currentIndexChanged -> change_mode() call always no-ops back then
        (self.worker is None). That means the device is left at whatever
        mode it booted into (Current Control / mode 0) even though the GUI
        shows "Pos Control" selected.

        Symptom this causes: Torque ON works, but pressing a Demo Position
        button writes a Goal Position while the device is still in Current
        Control mode - the position target is misinterpreted as a current
        command, trips an over-current/fault condition, and the firmware
        drops torque instead of moving.

        Torque is guaranteed OFF immediately after a fresh connect
        (self.torque_state starts False and change_mode() requires torque
        OFF), so it's safe to push the mode here unconditionally.
        """
        if not self.worker or not self.worker.isRunning():
            return
        idx = self.combo_mode.currentIndex()
        mode_val = 5 if idx == 1 else 0
        self.worker.write_regs(MODBUS_ADDR['cmd_op_mode'], [mode_val])
        self.log(f"Operating Mode synced to device on connect: {mode_val}")

    def toggle_torque(self):
        if not self.worker or not self.worker.isRunning():
            return

        self.torque_state = not self.torque_state
        self.worker.write_coil(MODBUS_COIL_ADDR['torque_enable'], self.torque_state)

        if self.torque_state:
            self.btn_torque.setText("TORQUE ON")
            self.btn_torque.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; font-size: 14px;")
            self.log("Torque Enabled")
        else:
            self.btn_torque.setText("TORQUE OFF")
            self.btn_torque.setStyleSheet("background-color: #ff4d4d; color: white; font-weight: bold; font-size: 14px;")
            self.log("Torque Disabled")

    def change_mode(self, idx):
        if not self.worker or not self.worker.isRunning():
            return

        if self.torque_state:
            self.log("Warning: Turn OFF Torque before changing mode!")
            return

        mode_val = 5 if idx == 1 else 0
        self.worker.write_regs(MODBUS_ADDR['cmd_op_mode'], [mode_val])
        self.log(f"Operating Mode set to: {mode_val}")

    def apply_current_limit(self):
        if not self.worker or not self.worker.isRunning():
            return

        lim = self.spin_cur_limit.value()
        limits = [lim] * 20
        self.worker.write_regs(MODBUS_ADDR['cmd_cur'], limits)
        self.log(f"Current limits updated: {lim} mA")

    def toggle_calibration(self):
        """Toggle joint calibration (joint_calib coil @0x0001 : 0=Stop, 1=Start)."""
        if not self.worker or not self.worker.isRunning():
            self.log("Error: Connect to device first!")
            return

        self.calib_state = not self.calib_state
        self.worker.write_coil(MODBUS_COIL_ADDR['joint_calib'], self.calib_state)

        if self.calib_state:
            self.btn_calib.setText("CALIBRATION: RUNNING")
            self.btn_calib.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold;")
            self.log("Calibration Started")
        else:
            self.btn_calib.setText("CALIBRATION: STOPPED")
            self.btn_calib.setStyleSheet("background-color: #9E9E9E; color: white; font-weight: bold;")
            self.log("Calibration Stopped")

    # --- PID Gain Tuning ---
    def read_pid_gains(self):
        if not self.worker or not self.worker.isRunning():
            self.log("Error: Connect to device first!")
            return

        # Kp/Ki/Kd : uint16 x20 (1 reg each)
        res_kp = self.worker.read_holding_regs(MODBUS_ADDR['cmd_gain_kp'], 20)
        res_ki = self.worker.read_holding_regs(MODBUS_ADDR['cmd_gain_ki'], 20)
        res_kd = self.worker.read_holding_regs(MODBUS_ADDR['cmd_gain_kd'], 20)
        # Vel/Acc : uint32 x20 (2 regs each, hi<<16 | lo)
        res_vel = self.worker.read_holding_regs(MODBUS_ADDR['cmd_gain_vel'], 40)
        res_acc = self.worker.read_holding_regs(MODBUS_ADDR['cmd_gain_acc'], 40)

        def regs_or_none(r):
            return r.registers if (r is not None and not r.isError()) else None

        kp, ki, kd = regs_or_none(res_kp), regs_or_none(res_ki), regs_or_none(res_kd)
        vel_raw, acc_raw = regs_or_none(res_vel), regs_or_none(res_acc)

        if kp is None or ki is None or kd is None or vel_raw is None or acc_raw is None:
            self.log("Error: Failed to read one or more PID gain blocks.")
            return

        vel = [(vel_raw[2 * i] << 16) | vel_raw[2 * i + 1] for i in range(20)]
        acc = [(acc_raw[2 * i] << 16) | acc_raw[2 * i + 1] for i in range(20)]

        for i in range(TOTAL_JOINT_CNT):
            self.pid_table.item(i, 1).setText(str(kp[i]))
            self.pid_table.item(i, 2).setText(str(ki[i]))
            self.pid_table.item(i, 3).setText(str(kd[i]))
            self.pid_table.item(i, 4).setText(str(vel[i]))
            self.pid_table.item(i, 5).setText(str(acc[i]))
        self.log("PID gains read from device.")

    def write_pid_gains(self):
        if not self.worker or not self.worker.isRunning():
            self.log("Error: Connect to device first!")
            return

        try:
            kp, ki, kd, vel, acc = [], [], [], [], []
            for i in range(TOTAL_JOINT_CNT):
                kp.append(int(self.pid_table.item(i, 1).text()) & 0xFFFF)
                ki.append(int(self.pid_table.item(i, 2).text()) & 0xFFFF)
                kd.append(int(self.pid_table.item(i, 3).text()) & 0xFFFF)
                vel.append(int(self.pid_table.item(i, 4).text()) & 0xFFFFFFFF)
                acc.append(int(self.pid_table.item(i, 5).text()) & 0xFFFFFFFF)
        except (ValueError, AttributeError):
            self.log("Error: PID gain values must be integers.")
            return

        # uint32 -> [hi, lo] register pairs
        vel_regs = []
        acc_regs = []
        for v in vel:
            vel_regs += [(v >> 16) & 0xFFFF, v & 0xFFFF]
        for a in acc:
            acc_regs += [(a >> 16) & 0xFFFF, a & 0xFFFF]

        self.worker.write_regs(MODBUS_ADDR['cmd_gain_kp'], kp)
        self.worker.write_regs(MODBUS_ADDR['cmd_gain_ki'], ki)
        self.worker.write_regs(MODBUS_ADDR['cmd_gain_kd'], kd)
        self.worker.write_regs(MODBUS_ADDR['cmd_gain_vel'], vel_regs)
        self.worker.write_regs(MODBUS_ADDR['cmd_gain_acc'], acc_regs)
        self.log("PID gains written to device.")

    # --- Network / System Config ---
    @staticmethod
    def _ip_to_regs(ip_str):
        """Convert dotted IP string to two uint16 registers [hi, lo]."""
        parts = ip_str.strip().split(".")
        if len(parts) != 4:
            raise ValueError(f"Invalid IP: {ip_str}")
        octets = [int(p) for p in parts]
        for o in octets:
            if not 0 <= o <= 255:
                raise ValueError(f"Invalid IP octet: {o}")
        val = (octets[0] << 24) | (octets[1] << 16) | (octets[2] << 8) | octets[3]
        return [(val >> 16) & 0xFFFF, val & 0xFFFF]

    @staticmethod
    def _regs_to_ip(regs):
        """Convert two uint16 registers [hi, lo] to a dotted IP string."""
        val = ((regs[0] & 0xFFFF) << 16) | (regs[1] & 0xFFFF)
        return f"{(val >> 24) & 0xFF}.{(val >> 16) & 0xFF}.{(val >> 8) & 0xFF}.{val & 0xFF}"

    def read_config(self):
        if not self.worker or not self.worker.isRunning():
            self.log("Error: Connect to device first!")
            return

        # System config block: comm_sel, slave_id, imu_config, sys_rev0,
        # sys_rev1 (5 regs) @0x0300
        res_sys = self.worker.read_holding_regs(MODBUS_ADDR['cfg_comm_sel'], 5)
        if res_sys is not None and not res_sys.isError():
            comm_sel, slave_id, imu, sys_rev0, sys_rev1 = res_sys.registers[:5]
            self.cfg_comm_sel.setCurrentIndex(1 if comm_sel else 0)
            self.cfg_slave_id.setValue(max(1, min(247, slave_id)))
            self.cfg_imu.setValue(imu)
            self.cfg_sys_rev0.setValue(sys_rev0)
            self.cfg_sys_rev1.setValue(sys_rev1)

        # Network block: ip, mask, gw, dns (each uint32 = 2 regs) @0x0310,
        # 8 regs. Moved here from 0x0303 in this firmware rev, with plenty
        # of reserved space before HAND_SN (0x0318) now - no more overlap
        # to work around.
        res_net = self.worker.read_holding_regs(MODBUS_ADDR['cfg_net_ip'], 8)
        if res_net is not None and not res_net.isError():
            r = res_net.registers
            self.cfg_ip.setText(self._regs_to_ip(r[0:2]))
            self.cfg_mask.setText(self._regs_to_ip(r[2:4]))
            self.cfg_gw.setText(self._regs_to_ip(r[4:6]))
            self.cfg_dns.setText(self._regs_to_ip(r[6:8]))

        self.log("Network / System config read from device.")

    def write_config(self):
        if not self.worker or not self.worker.isRunning():
            self.log("Error: Connect to device first!")
            return

        try:
            sys_regs = [
                self.cfg_comm_sel.currentIndex(),
                self.cfg_slave_id.value(),
                self.cfg_imu.value(),
                self.cfg_sys_rev0.value(),
                self.cfg_sys_rev1.value(),
            ]
            net_regs = (
                self._ip_to_regs(self.cfg_ip.text())
                + self._ip_to_regs(self.cfg_mask.text())
                + self._ip_to_regs(self.cfg_gw.text())
                + self._ip_to_regs(self.cfg_dns.text())
            )
        except ValueError as e:
            self.log(f"Error: {e}")
            return

        self.worker.write_regs(MODBUS_ADDR['cfg_comm_sel'], sys_regs)
        self.worker.write_regs(MODBUS_ADDR['cfg_net_ip'], net_regs)
        self.log("Network / System config written to device.")

    # --- Factory Settings ---
    @staticmethod
    def _sn_to_regs(sn_str):
        """Pack up to 8 ASCII chars into 4 uint16 registers (big-endian
        byte pairs), null-padded if shorter than 8 chars."""
        raw = sn_str.encode("ascii", errors="replace")[:8]
        raw = raw + b"\x00" * (8 - len(raw))
        return [int.from_bytes(raw[i:i + 2], "big") for i in range(0, 8, 2)]

    @staticmethod
    def _regs_to_sn(regs):
        """Unpack 4 uint16 registers into an 8-byte value, returning a
        printable-ASCII rendering (non-printable bytes shown as '.') and a
        raw hex string, since the firmware just memcpy's these 8 bytes
        without assuming an encoding."""
        raw = b"".join(int(r & 0xFFFF).to_bytes(2, "big") for r in regs[:4])
        text = "".join(chr(b) if 32 <= b < 127 else "." for b in raw)
        return text, raw.hex(" ")

    def write_factory_password(self):
        """Write the Factory Password (0x0310). The MCU compares it against
        a fixed key on write and sets/clears the factory_auth_status coil
        (0x0010) accordingly - it isn't a stored credential you set once,
        it's checked every time you write it."""
        if not self.worker or not self.worker.isRunning():
            self.log("Error: Connect to device first!")
            return
        text = self.cfg_factory_pwd_input.text().strip()
        try:
            pwd = int(text, 16) & 0xFFFF
        except ValueError:
            self.log("Error: Factory password must be a hex value (e.g. A5A5).")
            return
        self.worker.write_regs(MODBUS_ADDR['cfg_factory_pwd'], [pwd])
        self.log(f"Factory password 0x{pwd:04X} sent.")
        self.check_factory_auth_status()

    def check_factory_auth_status(self):
        """Read the factory_auth_status coil (0x0010, read-only) to see
        whether the last password write matched."""
        if not self.worker or not self.worker.isRunning():
            self.log("Error: Connect to device first!")
            return
        bits = self.worker.read_coils(MODBUS_COIL_ADDR['factory_auth_status'], 1)
        if bits is None:
            self.lbl_factory_auth_status.setText("Auth Status: Read failed")
            self.lbl_factory_auth_status.setStyleSheet("font-weight: bold; color: #c62828;")
            return
        if bits[0]:
            self.lbl_factory_auth_status.setText("Auth Status: UNLOCKED (PW match)")
            self.lbl_factory_auth_status.setStyleSheet("font-weight: bold; color: #4CAF50;")
        else:
            self.lbl_factory_auth_status.setText("Auth Status: Locked")
            self.lbl_factory_auth_status.setStyleSheet("font-weight: bold; color: #9E9E9E;")

    def write_hand_serial(self):
        """Write the Hand Serial Number (0x0318, 4 regs / 8 bytes). Only
        takes effect device-side if factory_auth_status is currently
        unlocked - authenticate first. The write transaction itself won't
        report an error either way (the firmware silently no-ops an
        unauthenticated write rather than returning an exception), so this
        automatically re-reads afterward to show whether it actually took."""
        if not self.worker or not self.worker.isRunning():
            self.log("Error: Connect to device first!")
            return
        sn_text = self.cfg_hand_sn_input.text()
        if not sn_text:
            self.log("Error: Enter a serial number first.")
            return
        regs = self._sn_to_regs(sn_text)
        self.worker.write_regs(MODBUS_ADDR['cfg_hand_sn'], regs)
        self.log(f"Hand Serial write sent: \"{sn_text}\". Reading back to confirm...")
        self.read_hand_serial()

    def read_hand_serial(self):
        if not self.worker or not self.worker.isRunning():
            self.log("Error: Connect to device first!")
            return
        res = self.worker.read_holding_regs(MODBUS_ADDR['cfg_hand_sn'], 4)
        if res is None or res.isError():
            self.lbl_hand_sn_current.setText("Current Serial: read failed")
            return
        text, hexs = self._regs_to_sn(res.registers)
        self.lbl_hand_sn_current.setText(f"Current Serial: \"{text}\" ({hexs})")

    # --- Communication / Error Monitor ---
    def handle_comm_event(self, label, detail, is_error):
        """Update the error-monitor tab from ModbusWorker.comm_event. Only
        emitted immediately for errors (or for non-throttled/one-off calls
        like button-triggered writes) - the TX/Err/Rate counters themselves
        are refreshed periodically by on_stats_tick() instead, since they'd
        otherwise repaint on every single fast-loop transaction."""
        if is_error:
            timestamp = time.strftime("[%H:%M:%S] ")
            self.lbl_last_err.setText(f"Last Error: {label} -> {detail}")
            self.txt_errlog.append(f"{timestamp}{label} -> {detail}")

    def clear_error_log(self):
        self.txt_errlog.clear()
        self.lbl_last_err.setText("Last Error: -")
        if self.worker is not None:
            self.worker.tx_count = 0
            self.worker.err_count = 0
            self.worker.error_type_counts = {}
        self.lbl_tx.setText("TX: 0")
        self.lbl_err.setText("Errors: 0")
        self.lbl_err_rate.setText("Error Rate: 0.0 %")
        self.error_type_table.setRowCount(0)
        self._error_type_rows = {}

    # --- RX/TX Timing Monitor ---
    def handle_timing_event(self, op_key, rtt_ms, is_error):
        """Update the timing tab from ModbusWorker.timing_event. Only fires
        for non-throttled calls (occasional button-triggered writes/reads,
        or any error) - the fast/slow poll loops call with notify=False and
        rely on on_stats_tick() for periodic aggregate refresh instead."""
        if self.worker is None:
            return

        self._refresh_stats_labels()
        st = self.worker.timing_stats.get(op_key)
        if st is not None:
            self._refresh_timing_row(op_key, st)

    def on_stats_tick(self):
        """Periodic (throttled, ~7 Hz) refresh of everything the error- and
        timing-monitor tabs show, pulled straight from the worker's
        accumulated counters/timing_stats/error_type_counts/rtt_history.
        This is what lets the background poll loop run at 500 Hz+ without
        flooding the GUI thread with one Qt signal + widget update per
        transaction."""
        if self.worker is None:
            return

        # Error-monitor tab counters (previously refreshed on every
        # comm_event; now just once per tick).
        tx = self.worker.tx_count
        err = self.worker.err_count
        self.lbl_tx.setText(f"TX: {tx}")
        self.lbl_err.setText(f"Errors: {err}")
        rate = (err / tx * 100.0) if tx else 0.0
        self.lbl_err_rate.setText(f"Error Rate: {rate:.1f} %")

        # Error type breakdown, sorted by count descending.
        for category, count in sorted(
            self.worker.error_type_counts.items(), key=lambda kv: -kv[1]
        ):
            row = self._error_type_rows.get(category)
            if row is None:
                row = self.error_type_table.rowCount()
                self.error_type_table.insertRow(row)
                self._error_type_rows[category] = row
                self.error_type_table.setItem(row, 0, QTableWidgetItem(category))
                count_item = QTableWidgetItem("0")
                count_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.error_type_table.setItem(row, 1, count_item)
            self.error_type_table.item(row, 1).setText(str(count))

        # Timing-monitor tab: summary labels + every known op's row.
        self._refresh_stats_labels()
        for op_key, st in self.worker.timing_stats.items():
            self._refresh_timing_row(op_key, st)

        # RTT chart: keep the op combo in sync with whatever ops exist, and
        # redraw with the currently-selected op's rolling history.
        self._refresh_rtt_chart_combo()
        selected_op = self.combo_rtt_chart_op.currentText()
        if selected_op:
            hist = self.worker.rtt_history.get(selected_op)
            if hist is not None:
                self.rtt_chart.set_data(hist)

    def _on_rtt_chart_op_changed(self, _index):
        """User manually picked a different operation in the chart combo -
        redraw immediately instead of waiting for the next stats_tick."""
        if self.worker is None:
            return
        selected_op = self.combo_rtt_chart_op.currentText()
        hist = self.worker.rtt_history.get(selected_op)
        self.rtt_chart.set_data(hist if hist is not None else [])

    def _refresh_rtt_chart_combo(self):
        """Add any newly-seen op_key to the chart's operation combo,
        preserving the current selection. Ops only appear once first used,
        so this can't be populated up front."""
        known = {self.combo_rtt_chart_op.itemText(i)
                 for i in range(self.combo_rtt_chart_op.count())}
        missing = [op for op in self.worker.rtt_history.keys() if op not in known]
        if not missing:
            return
        current = self.combo_rtt_chart_op.currentText()
        self.combo_rtt_chart_op.blockSignals(True)
        for op in sorted(missing):
            self.combo_rtt_chart_op.addItem(op)
        if current:
            idx = self.combo_rtt_chart_op.findText(current)
            if idx >= 0:
                self.combo_rtt_chart_op.setCurrentIndex(idx)
        self.combo_rtt_chart_op.blockSignals(False)

    def _refresh_stats_labels(self):
        """Shared summary-label refresh used by both handle_timing_event
        (single non-throttled event) and on_stats_tick (periodic batch)."""
        self.lbl_t_tx.setText(f"Total TX: {self.worker.tx_count}")
        self.lbl_t_last.setText(f"Last RTT: {self.worker.last_rtt_ms:.2f} ms")
        over = self.worker.rt_violation_count
        tx = self.worker.tx_count
        rate = (over / tx * 100.0) if tx else 0.0
        self.lbl_t_over.setText(f"> {RT_BUDGET_MS:.0f}ms: {over} ({rate:.1f} %)")
        self.lbl_t_timeout.setText(f"Timeouts: {self.worker.timeout_count}")
        if self.worker.timing_stats:
            global_max = max(s["max"] for s in self.worker.timing_stats.values())
            self.lbl_t_max.setText(f"Max RTT: {global_max:.2f} ms")

    def _refresh_timing_row(self, op_key, st):
        """Create or update the per-operation row for op_key."""
        row = self._timing_rows.get(op_key)
        if row is None:
            row = self.timing_table.rowCount()
            self.timing_table.insertRow(row)
            self._timing_rows[op_key] = row
            op_item = QTableWidgetItem(op_key)
            op_item.setFont(QFont("monospace", 11, QFont.Weight.Bold))
            self.timing_table.setItem(row, 0, op_item)
            for c in range(1, 8):
                cell = QTableWidgetItem("-")
                cell.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.timing_table.setItem(row, c, cell)

        avg = (st["sum"] / st["count"]) if st["count"] else 0.0
        min_v = st["min"] if st["min"] != float("inf") else 0.0
        self.timing_table.item(row, 1).setText(str(st["count"]))
        self.timing_table.item(row, 2).setText(f"{st['last']:.2f}")
        self.timing_table.item(row, 3).setText(f"{min_v:.2f}")
        self.timing_table.item(row, 4).setText(f"{avg:.2f}")
        self.timing_table.item(row, 5).setText(f"{st['max']:.2f}")
        self.timing_table.item(row, 6).setText(str(st["over"]))
        self.timing_table.item(row, 7).setText(str(st["err"]))

        # Highlight rows whose average exceeds the real-time budget. Use a
        # solid red fill with white text for strong contrast (the previous
        # pale pink on default text was hard to read); normal rows keep the
        # table's own alternating background so text stays crisp black.
        over_budget = avg > RT_BUDGET_MS or st["over"] > 0
        if over_budget:
            bg = QColor("#c62828")   # strong red
            fg = QColor("#ffffff")   # white text
        else:
            bg = None                # let alternating-row colour show through
            fg = QColor("#000000")   # black text
        for c in range(8):
            item = self.timing_table.item(row, c)
            if item is None:
                continue
            if bg is not None:
                item.setBackground(bg)
            else:
                # Clear any previous highlight by resetting to a null brush
                item.setBackground(QBrush())
            item.setForeground(fg)

    def reset_timing_stats(self):
        if self.worker is not None:
            self.worker.timing_stats = {}
            self.worker.timeout_count = 0
            self.worker.rt_violation_count = 0
            self.worker.rtt_history = {}
            self.worker._last_spike_time = {}
            self.worker._last_spike_tx = {}
        self.timing_table.setRowCount(0)
        self._timing_rows = {}
        self.lbl_t_tx.setText("Total TX: 0")
        self.lbl_t_last.setText("Last RTT: - ms")
        self.lbl_t_max.setText("Max RTT: - ms")
        self.lbl_t_over.setText(f"> {RT_BUDGET_MS:.0f}ms: 0 (0.0 %)")
        self.lbl_t_timeout.setText("Timeouts: 0")
        self.combo_rtt_chart_op.clear()
        self.rtt_chart.set_data([])

    def on_fast_period_changed(self, value):
        """Apply the new target fast-loop period live if already connected;
        otherwise it's picked up the next time a worker is created (see
        toggle_connection())."""
        if self.worker is not None and self.worker.isRunning():
            self.worker.fast_loop_period_ms = value
            self.log(f"Fast loop period changed to {value:.1f} ms.")

    def send_demo_position(self, name):
        """Send a demo preset. DEMO_POSITIONS values are already pulse (LSB),
        so send them directly WITHOUT deg2pulse conversion."""
        if not self.worker or not self.worker.isRunning():
            self.log("Error: Connect to device first!")
            return

        pulses = DEMO_POSITIONS.get(name, [0] * 20)
        pos_data = np.array(pulses, dtype=np.int16).astype(np.uint16).tolist()
        self.worker.write_regs(MODBUS_ADDR['cmd_pos'], pos_data)

        # NOTE: Goal column shows degree (used by trajectory/custom input).
        # Demo presets are raw pulse, so we do NOT overwrite that column here
        # to avoid mixing units.
        self.log(f"Demo Position '{name}' sent (raw pulse).")

    def save_grasp_slot(self):
        """Write the selected slot number to Grasp Save (0x0129) - the MCU
        stores the hand's current position into that on-device slot."""
        if not self.worker or not self.worker.isRunning():
            self.log("Error: Connect to device first!")
            return
        slot = self.spin_grasp_slot.value()
        self.worker.write_regs(MODBUS_ADDR['cmd_grasp_save'], [slot])
        self.log(f"Grasp Save: slot {slot} sent.")

    def play_grasp_slot(self):
        """Write the selected slot number to Grasp Play (0x012A) - the MCU
        recalls a previously-saved on-device slot."""
        if not self.worker or not self.worker.isRunning():
            self.log("Error: Connect to device first!")
            return
        slot = self.spin_grasp_slot.value()
        self.worker.write_regs(MODBUS_ADDR['cmd_grasp_play'], [slot])
        self.log(f"Grasp Play: slot {slot} sent.")

    def send_custom_positions(self):
        if not self.worker or not self.worker.isRunning():
            return

        custom_deg = []
        try:
            for i in range(20):
                val = int(self.table.item(i, 2).text())
                custom_deg.append(val)

            pulses = deg_list_to_pulse(custom_deg)
            pos_data = np.array(pulses, dtype=np.int16).astype(np.uint16).tolist()
            self.worker.write_regs(MODBUS_ADDR['cmd_pos'], pos_data)
            self.log("Custom Position Array Sent (deg->pulse).")
        except (ValueError, AttributeError):
            self.log("Error: Target values must be integers.")

    # --- Trajectory Playback Logic ---
    def toggle_trajectory(self):
        if self.traj_runner and self.traj_runner.isRunning():
            self.stop_trajectory()
        else:
            self.start_trajectory()

    def start_trajectory(self):
        if not self.worker or not self.worker.isRunning():
            self.log("Error: Connect to device first!")
            return
        if not self.torque_state:
            self.log("Warning: Enable Torque (TORQUE ON) first!")
            return

        selected_name = self.combo_traj.currentText()

        # Build the playlist: either a single motion or every motion in order.
        if selected_name == TRAJ_ALL_SEQUENTIAL:
            playlist = [(name, steps) for name, steps in DEMO_TRAJECTORIES.items()]
            if not playlist:
                self.log("Error: No trajectories defined.")
                return
        else:
            steps = DEMO_TRAJECTORIES.get(selected_name, [])
            if not steps:
                self.log(f"Error: Trajectory '{selected_name}' has no steps.")
                return
            playlist = [(selected_name, steps)]

        loop_enabled = self.chk_traj_loop.isChecked()
        self.traj_runner = TrajectoryRunner(self.worker, playlist, loop=loop_enabled)
        self.traj_runner.step_signal.connect(self.on_traj_step)
        self.traj_runner.finished_signal.connect(self.on_traj_finished)
        self.traj_runner.log_signal.connect(self.log)

        self.traj_runner.start()
        self.btn_play_traj.setText("Stop Motion")
        self.btn_play_traj.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
        loop_txt = " [Loop]" if loop_enabled else ""
        self.lbl_traj_status.setText(f"Playing: {selected_name}{loop_txt}")

    def stop_trajectory(self):
        if self.traj_runner:
            self.traj_runner.stop()
            self.traj_runner.wait()
            self.traj_runner = None
        self.btn_play_traj.setText("Start Motion")
        self.btn_play_traj.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;")
        self.lbl_traj_status.setText("Status: Stopped")

    def on_traj_step(self, step_idx, angles, motion_name):
        for i in range(20):
            self.table.item(i, 2).setText(str(angles[i]))
        # During sequential-all playback, surface which motion is active now.
        if self.combo_traj.currentText() == TRAJ_ALL_SEQUENTIAL:
            loop_txt = " [Loop]" if self.chk_traj_loop.isChecked() else ""
            self.lbl_traj_status.setText(
                f"Playing ALL -> '{motion_name}' step {step_idx}{loop_txt}"
            )

    def on_traj_finished(self):
        self.stop_trajectory()
        self.lbl_traj_status.setText("Status: Completed")
        self.log("Trajectory Playback Finished.")

    # --- Real-Time UI Updates ---
    def update_fast_ui(self, pos, cur, hpa, imu):
        for i in range(20):
            self.table.item(i, 1).setText(str(pos[i]))
            self.table.item(i, 3).setText(str(cur[i]))

        # Mirror hPa + IMU data to the separate sensor monitor window if open
        if self.sensor_window is not None:
            self.sensor_window.update_hpa(hpa)
            self.sensor_window.update_imu(imu)

    def update_slow_ui(self, temp, sensor_temp, status, imu_calib):
        self.lbl_status.setText(f"Status Bit: 0x{status:04X}")

        # Feed the slow-loop monitor tab
        for i in range(20):
            self.slow_temp_table.item(i, 1).setText(str(temp[i]))
        for i in range(18):
            self.slow_sensor_temp_table.item(i, 1).setText(str(sensor_temp[i]))
        self.lbl_slow_status.setText(f"hand_status (0x0066): 0x{status:04X}")

        # Mirror hand_status / imu_calib to the separate sensor monitor window if open
        if self.sensor_window is not None:
            self.sensor_window.update_status(status)
            self.sensor_window.update_imu_calib(imu_calib)

    def closeEvent(self, event):
        self.stop_trajectory()
        if self.worker:
            self.worker.stop()
        if self.sensor_window is not None:
            self.sensor_window.close()
        event.accept()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    set_windows_high_res_timer(True)
    try:
        app = QApplication(sys.argv)
        window = HandControlGUI()
        window.show()
        sys.exit(app.exec())
    finally:
        set_windows_high_res_timer(False)