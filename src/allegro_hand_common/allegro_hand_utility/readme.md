# Allegro Hand Utility Library (`allegro_hand_utility`)

This package provides a collection of reusable C++ utility classes designed to support the Allegro Hand software ecosystem. It includes tools for signal processing and thread synchronization.

## Features

*   **Signal Processing Filters**:
    *   `LowpassFilter`: A simple first-order low-pass filter to smooth noisy signals.
    *   `DerivativeFilter`: A filter to calculate the derivative of a signal, often used in combination with a low-pass filter to manage noise.
*   **Thread Synchronization**:
    *   `SpinLock`: A lightweight, busy-waiting lock for protecting shared data in performance-critical, short-duration critical sections. It is implemented using `std::atomic_flag`.
*   **ROS 2 Utilities**:
    *   `dump_hardware_info`: A helper function to generate a formatted string dump of a `hardware_interface::HardwareInfo` struct, useful for debugging `ros2_control` setups.

## Dependencies

*   `spdlog`
*   `Boost`
*   `rclcpp`
*   `hardware_interface`

## Building

This package is a standard ament_cmake package and can be built within a ROS 2 workspace using `colcon`.

```bash
colcon build --packages-select allegro_hand_utility
```

## Usage

Below are examples of how to use the utility classes provided by this library.

### 1. LowpassFilter

```cpp
#include <allegro_hand_utility/utility.hpp>
#include <iostream>

int main() {
    // Create a low-pass filter for a 1kHz signal with a 100Hz cutoff frequency
    allegro_hand_utility::LowpassFilter lp_filter(1000.0, 100.0);

    double noisy_signal = 1.0; // Assume this is a noisy input
    double filtered_signal = lp_filter.filtering(noisy_signal);

    std::cout << "Original: " << noisy_signal << ", Filtered: " << filtered_signal << std::endl;

    return 0;
}
```

### 2. DerivativeFilter

```cpp
#include <allegro_hand_utility/utility.hpp>
#include <iostream>

int main() {
    // Create a derivative filter for a 100Hz signal
    allegro_hand_utility::DerivativeFilter deriv_filter(100.0);

    double position = 0.5; // Current position
    double velocity = deriv_filter.filtering(position);

    std::cout << "Position: " << position << ", Calculated Velocity: " << velocity << std::endl;

    return 0;
}
```

### 3. SpinLock

```cpp
#include <allegro_hand_utility/utility.hpp>
#include <iostream>
#include <thread>
#include <vector>

allegro_hand_utility::SpinLock spinlock;
int shared_counter = 0;

void increment_counter() {
    for (int i = 0; i < 10000; ++i) {
        std::lock_guard<allegro_hand_utility::SpinLock> lock(spinlock);
        shared_counter++;
    }
}

int main() {
    std::thread t1(increment_counter);
    std::thread t2(increment_counter);

    t1.join();
    t2.join();

    std::cout << "Final counter value: " << shared_counter << std::endl;
    return 0;
}
```

