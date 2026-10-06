# RISBOT2
![image](https://github.com/user-attachments/assets/a6be4376-f133-4ab6-86f0-056b13f34615)


## Overview
- risbot2 folder: program for RISBOT2'PC
- risbot2_micro_controller folder: program for micro controller
- risbot2_tools folder: dependent for RISBOT2
- risbot2_docs folder: document for RISBOT2

## Hardware
- Jetson Orin NX 16G
- Hokuyo Lidar UST-20LX
- Realsense T265 (x2)
- ZED 2 Stereo Camera
- Livox MID-360
- 7inch screen
- Arduino Uno R3
- Stepping Motor (x2)
- TB6600 Stepper motor driver (x2)

## Software
- JetPack 6.1
- ROS2 Humble
- Realsense SDK 2.0 (v2.51.1), build with CUDA

## Setup

### 1. Install dependent RISBOT2 packages
- Install Cartographer:
```bash
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
```
- Install Navigation2:
```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```
### 2. Clone Repository
Clone the RISBot2 using the following commands:
```bash
mkdir -p ~/risbot2_ws/src
cd ~/risbot2_ws/src/
git clone https://github.com/TrH-Thang/risbot2.git
sudo apt install python3-colcon-common-extensions
cd ~/risbot2_ws
colcon build
echo 'source ~/risbot2_ws/install/setup.bash' >> ~/.bashrc
echo 'sudo chmod 666 /dev/ttyACM0' >> ~/.bashrc
source ~/.bashrc
```
### 3. Arduino Uno R3 setup
Code for Arduino Uno R3: /risbot2_ws/src/risbot2/risbot2_micro_controller/risbot2_arduino_uno.ino

## Launch

### 1. Bringup RISBOT2

Open a terminal and run following command:
```bash
ros2 launch risbot2_bringup robot.launch.py
```

or:
```bash
ros2 launch risbot2_bringup robot2.launch.py
```
### 2. SLAM RISBOT2

+ SLAM with a t265:
```bash
ros2 launch risbot2_bringup robot.launch.py
# Open a new terminal and run following command:
ros2 launch risbot2_cartographer cartographer.launch.py
```

+ SLAM with 2 T265s:
```bash
ros2 launch risbot2_bringup robot2.launch.py
# Open a new terminal and run following command:
ros2 launch risbot2_cartographer cartographer2.launch.py
```

+ Open a new terminal and run following command to controll RISBOT2:
```bash
ros2 run risbot2_teleop teleop_keyboard
```

### 3. Navigation RISBOT2

- Navigation with a t265:
```bash
ros2 launch risbot2_bringup robot.launch.py
# Open a new terminal and run following command:
ros2 launch risbot2_navigation2 navigation2.launch.py map:=/path/to/map.yaml
```

- Navigation with 2 t265s:
```bash
ros2 launch risbot2_bringup robot2.launch.py
# Open a new terminal and run following command:
ros2 launch risbot2_navigation2 navigation2.launch.py map:=/path/to/map.yaml
```
