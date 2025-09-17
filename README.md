# mvc01_control

ROS 2 package for controlling the **MVC01 BLV-R** motor driver via **Modbus RTU** using a joystick.  
The main node `joy_to_blvr` receives joystick input through `teleop_twist_joy` and forwards velocity commands to the motor driver.

---

## Requirements

- ROS 2 (Foxy, Humble, or compatible)
- External packages:
  - [`joy_linux`](https://index.ros.org/p/joy_linux/)
  - [`teleop_twist_joy`](https://index.ros.org/p/teleop_twist_joy/)
  - `libmodbus-dev` (system dependency for Modbus communication)

---

## Installation

Clone this package into your ROS 2 workspace and build:

```bash
cd ~/ros2_ws/src
git clone <repo-url> mvc01_control
cd ~/ros2_ws
colcon build --packages-select mvc01_control
source install/setup.bash
```

Make sure your joystick is connected and detected at `/dev/input/js0`.

---

## How to Run

> **Note:** Steps 1 and 2 must be executed in separate terminals before running step 3.

### 1. Run joystick driver
Open the first terminal:
```bash
ros2 run joy_linux joy_linux_node
```

### 2. Run teleop_twist_joy
Open the second terminal:
```bash
ros2 run teleop_twist_joy teleop_node \
  --ros-args \
  -p axis_linear.x:=1 \
  -p scale_linear.x:=40.0 \
  -p axis_angular.yaw:=0 \
  -p scale_angular.yaw:=70.0 \
  -p require_enable_button:=false
```

This maps joystick axes to velocity commands (`geometry_msgs/Twist`).

### 3. Run joy_to_blvr
Open the third terminal:
```bash
ros2 run mvc01_control joy_to_blvr
```

This node will:
- Read linear/angular velocity commands
- Send them to the motor driver via Modbus
- Publish driver feedback (odometry, IMU, supply voltage, temperatures, torque, etc.)

---

## Topics

**Subscribers**
- `/joy` – joystick input from `joy_linux`
- `/cmd_vel` – velocity command from `teleop_twist_joy`

**Publishers**
- `/odom_wheel`, `/odom_gyro`  
- `/imu`, `/imu_temperature`, `/controller_temperature`  
- `/supply_voltage`, `/power_on_count`, `/uptime_s`  
- `/command_speed`, `/feedback_speed`  
- `/torque_pct`

---

## License
Apache-2.0
