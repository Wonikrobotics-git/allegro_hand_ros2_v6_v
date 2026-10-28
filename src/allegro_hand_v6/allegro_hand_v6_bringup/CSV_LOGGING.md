# CSV Data Recording

Record hardware reads from the configured **500 Hz** control loop. F and V models use the same 62-column CSV format. IMU values and USB camera frames are not included. Recording is disabled by default.

## Build and Start Recording

Use the [root installation guide](../../../README.md#installation). These commands run from the cloned repository root; packages live under `src/`:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/allegro_hand_ros2_v6_v
colcon build --symlink-install --executor sequential --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash

ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py \
  model:=v6_v hand:=right ros2_control_hardware_type:=hardware \
  io_interface_descriptor:=modbus_tcp:192.168.1.100:502 gui:=false \
  csv_logging:=true csv_directory:=./data_logs
```

Use the connected hand's model, side, IP, and port. Start only one bringup instance. If already built, source `install/setup.bash` and run the launch command; rebuilding is not necessary just to enable logging. Relative output paths are resolved from the terminal's working directory. For a path with spaces, quote the whole argument, for example `"csv_directory:=./my data"`.

Mock and Isaac modes do not read the physical SDK and do not create these hardware recordings. Logging failure during startup prevents successful hardware activation; read the launch error and check the directory and available disk space.

## Files and Shutdown

Each activation creates a new file such as `allegro_hand_right_YYYYMMDD_HHMMSS_NNNNNNNNN.csv`. The side in the filename comes from device information. The startup log prints the absolute path; existing files are not overwritten.

End bringup normally with `Ctrl+C`. Hardware cleanup attempts to disable torque and drains queued samples. The companion `file.csv.summary.json` records final counters. GUI closure alone does not stop the base recording. Forced termination or power loss can lose buffered data; the logger does not run `fsync` for every row.

## CSV Columns

| Columns | Meaning |
|---|---|
| `timestamp` | Host wall-clock time just after the SDK read, KST, `YYYY-MM-DD HH:MM:SS.sss`. |
| `sequence` | Read attempt number, beginning at 1. |
| `read_ok` | `1` for a complete fast-block read, otherwise `0`. |
| `dropped_total` | Cumulative rows not queued because of overflow or a stopped/failed writer. |
| `joint00_position_rad` … `joint43_position_rad` | 20 joint positions, radians. |
| `joint00_current_ma` … `joint43_current_ma` | 20 signed measured currents, mA. |
| `pressure_0_hpa` … `pressure_17_hpa` | 18 pressure values in firmware order, hPa. |

There are **62 columns**: four time/status columns and 58 measurements. Joint order is thumb, index, middle, ring, pinky, with four joints per finger. A configured joint prefix is included in joint column names. Current values are reconstructed from SDK torque using `TORQUE_CONSTANT` and rounded back to mA.

The SDK still reads its existing 62-register fast block, including IMU registers; CSV omits the four IMU measurements. On a failed read, a `read_ok=0` row is retained with all 58 measurement cells empty. Timestamps have three decimal places and use host reception time, not the device's sampling clock or ROS `/clock`. Clock corrections can affect timestamp differences.

## Check the Actual Acquisition Rate

The controller's configured update rate is 500 Hz (2 ms). Actual read timing depends on the physical device, transport, and OS scheduling. No repeated samples are inserted to make the rate appear to be 500 Hz.

After normal shutdown:

```bash
python3 - <<'PY'
import csv
from datetime import datetime
from pathlib import Path
paths = list(Path('data_logs').glob('*.csv'))
if not paths:
    raise SystemExit('No CSV files in ./data_logs; use your actual recording directory.')
path = max(paths, key=lambda p: p.stat().st_mtime)
with path.open(newline='') as stream:
    reader = csv.DictReader(stream)
    columns = reader.fieldnames
    rows = list(reader)
print('File:', path)
print('Columns:', len(columns), 'Rows:', len(rows))
assert len(columns) == 62
if len(rows) > 1:
    start = datetime.fromisoformat(rows[0]['timestamp'])
    end = datetime.fromisoformat(rows[-1]['timestamp'])
    elapsed = (end - start).total_seconds()
    print('Average logged rows/s:', (len(rows) - 1) / elapsed if elapsed > 0 else 'invalid clock interval')
print('Failed reads:', sum(row['read_ok'] == '0' for row in rows))
print('Last dropped_total:', rows[-1]['dropped_total'] if rows else 'no rows')
PY
```

This is the rate of logged rows, including failed read attempts, not a guarantee of successful device samples. Examine `read_ok`, sequence gaps, `dropped_total`, and the summary's `dropped_samples`, `failed_reads`, and `write_failed`. Evaluate several seconds of data; millisecond precision and host-clock corrections make very short intervals unreliable. The 8192-row queue drops new rows on overflow rather than blocking the control loop.

## Device-Free Logger Tests

Tests are disabled in the ordinary build above. To build and run only the logger/decoder checks:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/allegro_hand_ros2_v6_v
colcon build --symlink-install --packages-up-to allegro_hand_v6_hardware --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select allegro_hand_v6_hardware \
  --ctest-args -R '^test_(csv_logger|sdk_read)$' --output-on-failure
colcon test-result --verbose
```

Unfiltered hardware-package tests can connect to or move a real hand. The filter above selects only the device-free logger and fast-block decoder tests.
