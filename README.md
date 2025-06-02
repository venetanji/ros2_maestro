# ROS 2 Maestro Motor Controller Node

This package provides a ROS 2 node that subscribes to the `/face_motors` topic and sends motor position commands to a Pololu Maestro 12-channel servo controller over serial. The node is implemented in Python and follows the design pattern of the provided `motor_controller.py`.

## Features
- Subscribes to `/face_motors` (expects an array of normalized motor positions, e.g., `std_msgs/msg/Float32MultiArray`)
- Sends commands to a Pololu Maestro 12-channel controller via serial
- Configurable serial port and YAML config for channel calibration
- Simulation mode if hardware is not available

## Requirements
- ROS 2 Humble or newer
- Python 3.8+
- `rclpy` and ROS 2 Python libraries
- `pyserial`, `numpy`, `PyYAML`

## Installation
1. Clone this repository into your ROS 2 workspace `src` folder.
2. Install Python dependencies:
   ```sh
   pip install -r requirements.txt
   ```
3. Build your ROS 2 workspace:
   ```sh
   colcon build
   ```

## Usage
Launch the node with:
```sh
ros2 launch ros2_maestro maestro_node.launch.py
```

## Configuration
- Edit `config/motor_ranges.yaml` to set up channel calibration.

## License
MIT
