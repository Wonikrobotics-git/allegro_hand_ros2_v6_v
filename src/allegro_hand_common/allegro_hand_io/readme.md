# Allegro Hand I/O Library (`allegro_hand_io`)

This package provides a C++ library that serves as a hardware abstraction layer for various communication protocols used with the Allegro Hand. It is designed to be modular, allowing different communication backends (e.g., CAN, UDP) to be used interchangeably through a common interface.

## Features

*   **Abstraction Layer**: A generic `CommIo` abstract base class defines the common interface for all communication protocols.
*   **Factory Pattern**: A factory method `CommIo::Create` allows for easy instantiation of a specific communication object using a string descriptor (e.g., `"can:can0"`).
*   **SocketCAN Implementation**: A concrete implementation `CommIoCan` for SocketCAN is provided.
*   **Extensible**: Designed for easy expansion. UDP and Modbus support are planned for future releases, and placeholder classes are already included.
*   **Logging**: Integrated with `spdlog` for flexible and efficient logging.

## Dependencies

*   `spdlog`

## Building

This package is a standard CMake package and can be built within a ROS 2 workspace using `colcon`.

```bash
colcon build --packages-select allegro_hand_io
```

## Usage

### 1. Factory Pattern and Interface Descriptors

This library uses a factory design pattern to create the appropriate communication object. The `CommIo::Create` static method takes a string descriptor as its first argument, which determines the type of communication object to instantiate. This approach decouples the client code from the concrete implementation classes.

The format of the descriptor string is `protocol:details`.

*   **CAN**: `"can:<interface_name>"` (e.g., `"can:can0"`)
*   **UDP**: `"udp:<ip_address>,<port>"` (e.g., `"udp:192.168.1.100,8080"`) - *Future support*
*   **Modbus**: `"modbus:<details>"` - *Future support*

By simply changing the descriptor string, you can switch the underlying communication protocol without changing the rest of the application code that uses the `CommIo` interface.

### 2. CAN Interface Setup (Linux)

Before running any application that uses this library with a CAN interface, the physical CAN device must be configured on the system. The following commands set up the `can0` interface with a bitrate of 1 Mbps.

```bash
# Bring the interface down
sudo ip link set can0 down

# Set the interface type to CAN and specify the bitrate
sudo ip link set can0 type can bitrate 1000000

# Bring the interface up
sudo ip link set can0 up
```

### 2. C++ Code Example

To use the library, include the main header and use the `CommIo::Create` factory method to get a communication object. The factory will return the correct object based on the descriptor string.

```cpp
#include <allegro_hand_io/comm.hpp>
#include <iostream>

// Example callback function to process received data
void my_decode_function(const uint32_t& id, uint8_t* data, int size) {
    // Process the incoming frame...
    std::cout << "Received frame with ID: " << id << std::endl;
}

// Example callback to generate a CAN ID
uint32_t my_encode_function(const int& message_id) {
    // Return a CAN-specific ID
    return message_id;
}

int main() {
    try {
        // Create a CAN communication object for the "can0" interface
        allegro_hand_io::CommIo::Ptr comm = allegro_hand_io::CommIo::Create(
            "can:can0", my_decode_function, my_encode_function);

        // Now you can use the 'comm' object to send messages
        // comm->send_message(...);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
```