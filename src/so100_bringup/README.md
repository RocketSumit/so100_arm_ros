# so100_bringup

Launch and configuration package for bringing up the SO100 arm with ros2_control. It starts the robot description, controller manager, and position controllers (arm + gripper), and supports both real hardware and a mock (fake) hardware interface.

## Overview

This package provides:

- **Robot description** – Xacro-based URDF that includes the SO100 arm model from `so100_description` and the ros2_control block
- **Launch** – Starts `robot_state_publisher`, `controller_manager`, and spawns `joint_state_broadcaster`, `arm_controller`, and `gripper_controller`
- **Controller config** – YAML for controller manager and position controllers
- **Real vs mock hardware** – Toggle between `so100_hardware_interface` (real servos) and `mock_components/GenericSystem` (no hardware)

## Prerequisites

- ROS 2 (Humble or later)
- **so100_description** – Robot URDF and meshes (must be in the same workspace or installed)
- **so100_hardware_interface** – For real hardware (see [so100_hardware_interface](../so100_hardware_interface/README.md); requires FTServo Linux)
- ros2_control packages: `controller_manager`, `joint_state_broadcaster`, `position_controllers`
- **joint_state_publisher** (optional, for GUI)
- **MoveIt** (optional, for motion planning)

## Building

From your workspace:

```bash
cd /path/to/robo_ws
source /opt/ros/<distro>/setup.bash
colcon build --packages-select so100_bringup
source install/setup.bash
```

Build or install `so100_description` and, for real hardware, `so100_hardware_interface` first.

## Launching

### Bring up the robot (real or fake)

Default uses **real hardware** (see `use_real_hardware` in `config/so100_arm.urdf.xacro`):

```bash
ros2 launch so100_bringup controllers.yaml
```

To use **mock hardware** (no servos, for testing), set the default in `config/so100_arm.urdf.xacro` to `false`:

```xml
<xacro:arg name="use_real_hardware" default="false"/>
```

Then run the same launch command. You can override `use_sim_time` and `robot_description_file` via launch args if needed.

### Optional: RViz

The launch file has a commented-out RViz node. To visualize in RViz, run it separately:

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix so100_bringup)/share/so100_bringup/config/display.rviz
```

Or uncomment the RViz node in `launch/controllers.yaml` and relaunch.

## Configuration

### Robot description and hardware mode

The top-level URDF is `config/so100_arm.urdf.xacro`. It:

- Includes the arm model from **so100_description**
- Includes the ros2_control macro from `config/so100_arm.ros2_control.xacro`
- Uses xacro args:
  - **`use_real_hardware`** – `true` → `so100_hardware_interface/SO100ArmInterface`, `false` → `mock_components/GenericSystem`
  - **`initial_positions_file`** – YAML file with initial joint positions for the mock interface

When launching, pass these via the xacro file if your launch uses a custom description path (e.g. `robot_description_file` and xacro args).

### Controllers

Defined in `config/ros2_controllers.yaml`:

| Controller                | Type                                                | Role                                 |
| ------------------------- | --------------------------------------------------- | ------------------------------------ |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster`     | Publishes `/joint_states`            |
| `arm_controller`          | `position_controllers/JointGroupPositionController` | 5 joints (shoulder_pan → wrist_roll) |
| `gripper_controller`      | `position_controllers/JointGroupPositionController` | Gripper joint                        |

Controller manager update rate is 100 Hz.

### Joint names

- **Arm:** `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`
- **Gripper:** `gripper`

### Initial positions

`config/initial_positions.yaml` sets default joint positions (radians) for the mock hardware and initial state interface values. Adjust as needed for your robot or demo.

## Package layout

```
so100_bringup/
├── config/
│   ├── display.rviz              # RViz config
│   ├── initial_positions.yaml    # Default joint positions
│   ├── ros2_controllers.yaml     # Controller manager + controller configs
│   ├── so100_arm.ros2_control.xacro  # ros2_control hardware & joints
│   ├── so100_arm.srdf             # Semantic robot description (if used)
│   └── so100_arm.urdf.xacro       # Top-level robot description xacro
├── launch/
│   └── controllers.yaml         # Main launch file (YAML format)
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Real hardware notes

1. **Serial port** – The ros2_control xacro uses `/dev/ttyACM0` and 1000000 baud. Change `serial_port` (and optionally `serial_baudrate`) in `config/so100_arm.ros2_control.xacro` or via a xacro/launch parameter if you expose one.
2. **Permissions** – Ensure the user is in the `dialout` group (or has read/write access to the serial device):
   `sudo usermod -aG dialout $USER` (log out and back in).
3. **FTServo** – Real hardware requires the FTServo Linux library and the `so100_hardware_interface` package; see [so100_hardware_interface](../so100_hardware_interface/README.md).

## Controlling the arm

After bringup, send position commands to the arm and gripper controllers, e.g.:

```bash
# Arm (5 joints: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll)
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0]}"

# Gripper
ros2 topic pub --once /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}"
```

Or use the **rqt_controller_manager** (listed in package.xml) to load/switch controllers and send commands.
