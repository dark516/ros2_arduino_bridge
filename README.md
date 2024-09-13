<div align="center" style="text-align: center;">
  
# ROS2 to Arduino Communication Node
</div>
This ROS2 node provides bidirectional communication between a ROS2 system (e.g., running on a Raspberry Pi) and an Arduino, using a request-response protocol over a serial port. The node can send control commands (e.g., motor speeds) to the Arduino and receive sensor data or status updates from it, publishing the received data to relevant ROS2 topics.

---

## Features

- Sends motor control commands or other data to Arduino.
- Receives data packets from Arduino (e.g., sensor readings) and publishes them to appropriate ROS2 topics.
- Uses a request-response protocol over a serial connection to ensure reliable data exchange.

## Installation
### Prerequirements

- ROS2 Rolling (or your desired ROS2 distribution)
- A serial connection between the Raspberry Pi and the Arduino (via USB)

### Setting Up the Node

1. Clone this repository to your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/dark516/ros2_arduino_bridge
```
2.Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select ros2_arduino_bridge
```
3. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Running the Node

1. Connect the Arduino to the Raspberry Pi via USB
2. Check the port of the connected arduino:
```bash
ls /dev| grep ttyACM
```
3.  Run the node with an explicit indication of the port received in the previous step
```bash
ros2 run ros2_arduino_bridge arduino_bridge /dev/ttyACM0
```
