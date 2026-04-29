# AutoDRIVE F1Tenth SLAM with ROS 2

This project implements a SLAM (Simultaneous Localization and Mapping) system 
for the AutoDRIVE F1Tenth simulator using ROS 2 Humble and slam_toolbox. 
The system maps the RoboRacer circuit through teleoperation.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- AutoDRIVE Simulator (Windows)
- VirtualBox
- slam_toolbox

## Installation

```bash
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-nav2-map-server
```

## Project Structure

- `odom_publisher.py` — Custom node that publishes odometry using IPS + IMU sensors
- `lidar_republisher.py` — Custom node that fixes LiDAR timestamp synchronization between Windows and Linux

## How to Run

### Terminal 1 — Bridge
```bash
cd ~/autodrive_ws
source /opt/ros/humble/setup.bash
source venv/bin/activate
export PYTHONPATH=~/autodrive_ws/venv/lib/python3.10/site-packages:$PYTHONPATH
source install/setup.bash
ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py
```

Connect the AutoDRIVE simulator on Windows.

### Terminal 2 — Odometry Publisher
```bash
cd ~/autodrive_ws
source /opt/ros/humble/setup.bash
source venv/bin/activate
export PYTHONPATH=~/autodrive_ws/venv/lib/python3.10/site-packages:$PYTHONPATH
source install/setup.bash
python3 odom_publisher.py
```

### Terminal 3 — LiDAR Republisher
```bash
cd ~/autodrive_ws
source /opt/ros/humble/setup.bash
source venv/bin/activate
export PYTHONPATH=~/autodrive_ws/venv/lib/python3.10/site-packages:$PYTHONPATH
source install/setup.bash
python3 lidar_republisher.py
```

### Terminal 4 — SLAM
```bash
cd ~/autodrive_ws
source /opt/ros/humble/setup.bash
source venv/bin/activate
export PYTHONPATH=~/autodrive_ws/venv/lib/python3.10/site-packages:$PYTHONPATH
source install/setup.bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p use_sim_time:=false \
  -p base_frame:=f1tenth_1 \
  -p odom_frame:=odom \
  -p map_frame:=map \
  -p transform_timeout:=5.0 \
  -p tf_buffer_duration:=60.0 \
  -r /scan:=/lidar_fixed
```

### Save the map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/mapa_autodrive
```

## TF Frame Tree
map
└── odom
└── f1tenth_1
├── lidar
├── front_camera
├── front_left_wheel
├── front_right_wheel
├── rear_left_wheel
├── rear_right_wheel
├── imu
├── ips
├── left_encoder
└── right_encoder

## Key Challenges Solved

1. **Timestamp synchronization** — The AutoDRIVE simulator runs on Windows while 
ROS 2 runs on Linux inside VirtualBox. This caused timestamp mismatches that 
prevented slam_toolbox from processing LiDAR data. Solved by creating the 
lidar_republisher node that republishes LiDAR data with fresh Linux timestamps.

2. **Odometry** — AutoDRIVE does not publish standard odometry. Solved by creating 
the odom_publisher node that combines IPS position data with IMU orientation to 
publish odometry on the /odom topic and broadcast the corresponding TF transform.

## Results

A clean occupancy map of the RoboRacer circuit was successfully generated through 
teleoperation and saved as mapa_autodrive.pgm and mapa_autodrive.yaml.

## Authors

- Juan Santana

## TF Frame Tree
