# so100_hardware_interface

ROS 2 hardware interface for the SO100 arm using FEETECH BUS servos (STS series). It integrates with [ros2_control](https://control.ros.org/) and uses the FTServo Linux library for serial communication with the servos.

## Features

- **ros2_control** `SystemInterface` implementation
- Position state and command interfaces for each joint
- Serial communication via FTServo (SMS_STS) for FEETECH servos

## Prerequisites

### 1. ROS 2

- ROS 2 Humble or later (tested with Humble)
- `ros2_control` and `hardware_interface` packages

### 2. FTServo driver (git submodule)

This package depends on the **FTServo** library for low-level servo communication. It is included in this repo as the **ftservo_driver** git submodule at `src/ftservo_driver`. You only need to pull the submodule and install it.

**Setup and install**

```bash
# From the workspace root (e.g. robo_ws)
cd /path/to/robo_ws

# If you just cloned the repo: init and update the submodule
git submodule update --init --recursive

# Build and install ftservo_driver (required for so100_hardware_interface)
cd src/ftservo_driver
mkdir -p build && cd build
cmake ..
make
sudo make install
```

After installation, build the workspace as usual; `find_package(FTServo)` will locate the installed library.

**Documentation**

- Submodule path: `src/ftservo_driver`
- For build options and troubleshooting, see [ftservo_driver/INSTALL.md](../ftservo_driver/INSTALL.md).

**Verify installation**

```bash
# Check that CMake can find FTServo
pkg-config --modversion ftservo
ls /usr/local/include/ftservo/
ls /usr/local/lib/libSCServo.*
```

## Dependencies

- `hardware_interface` (ros2_control)
- `pluginlib`
- `rclcpp`, `rclcpp_lifecycle`
- **FTServo** (system-installed; see above)

## Building

From your ROS 2 workspace:

```bash
cd /path/to/robo_ws
source /opt/ros/<distro>/setup.bash
colcon build --packages-select so100_hardware_interface
source install/setup.bash
```

If CMake does not find FTServo, set the prefix path:

```bash
colcon build --packages-select so100_hardware_interface --cmake-args -DCMAKE_PREFIX_PATH=/usr/local
```

## Configuration

Hardware parameters are typically provided via the robot description (URDF/xacro) used by ros2_control. A sample config is provided in `config/motor_config.yaml`:

| Parameter     | Type   | Description                                  |
| ------------- | ------ | -------------------------------------------- |
| `use_serial`  | bool   | Enable serial communication with the servos. |
| `serial_port` | string | Serial device (e.g. `"/dev/ttyACM0"`).       |
| `baud_rate`   | int    | Baud rate (e.g. `1000000`).                  |

Ensure these parameters are passed into the `<ros2_control>` hardware block (e.g. from `so100_bringup` or `so100_moveit_config`) so the interface receives them.

## Usage

This package provides a **plugin** for ros2_control. You do not run it as a standalone node. It is loaded by the controller manager when your robot description declares the `so100_hardware_interface/SO100ArmInterface` type.

Example snippet in a ros2_control xacro:

```xml
<ros2_control name="So100Arm" type="system">
  <hardware>
    <plugin>so100_hardware_interface/SO100ArmInterface</plugin>
    <param name="use_serial">true</param>
    <param name="serial_port">/dev/ttyACM0</param>
    <param name="serial_baudrate">1000000</param>
  </hardware>
  <!-- joints ... -->
</ros2_control>
```

Launch your robot and controller manager as usual (e.g. via `so100_bringup` or `so100_moveit_config`); the hardware interface will be loaded automatically.

## Package layout

```bash
so100_hardware_interface/
├── config/
│   └── motor_config.yaml    # Sample hardware parameters
├── include/ros2_so100_interface/
│   └── so100_arm_interface.hpp
├── src/
│   └── so100_arm_interface.cpp
├── so100_hardware_interface.xml   # Plugin description
├── CMakeLists.txt
├── package.xml
└── README.md
```
