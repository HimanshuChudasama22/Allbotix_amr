# AMR SLAM Package

ROS2 package for mapping with your AMR robot (Arduino Mega + RMCS-2305 + YDLidar X2).

## Prerequisites

```bash
# Install dependencies (replace 'humble' with your ROS2 distro)
sudo apt install ros-humble-slam-toolbox \
                 ros-humble-nav2-map-server \
                 ros-humble-tf2-ros \
                 ros-humble-teleop-twist-keyboard

# Install pyserial
pip install pyserial

# YDLidar ROS2 driver (if not already installed)
cd ~/ros2_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
cd ~/ros2_ws && colcon build --packages-select ydlidar_ros2_driver
```

## Setup

### 1. Copy package into workspace
```bash
cp -r amr_slam ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select amr_slam
source install/setup.bash
```

### 2. Set up udev rules (recommended)

Create `/etc/udev/rules.d/99-amr.rules`:
```
# Arduino Mega
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", SYMLINK+="arduino", MODE="0666"
# YDLidar
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ydlidar", MODE="0666"
```
Then reload:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 3. Update Arduino firmware
Your Arduino code has `WHEELBASE_MM = 300.0f` but the actual wheelbase is **330mm**.
Update this line in your Arduino sketch:
```cpp
const float WHEELBASE_MM = 330.0f;   // corrected
```

## Usage

### Start mapping
```bash
ros2 launch amr_slam mapping.launch.py
```

With custom ports:
```bash
ros2 launch amr_slam mapping.launch.py serial_port:=/dev/arduino lidar_port:=/dev/ydlidar
```

Without RViz (headless on Pi):
```bash
ros2 launch amr_slam mapping.launch.py use_rviz:=false
```

### Publish sensor/odom data only (offboard processing)
Runs only robot data publishers (`/odom`, TF, `/scan`, optional `/scan_reliable`) with no SLAM/Nav2:
```bash
ros2 launch amr_slam offboard_sensors.launch.py
```

Disable scan relay to reduce CPU further:
```bash
ros2 launch amr_slam offboard_sensors.launch.py publish_reliable_scan:=false
```

For remote PC processing, keep both machines on the same network and set matching `ROS_DOMAIN_ID`.

### Localize and navigate on a saved map
```bash
ros2 launch amr_slam localization.launch.py
```

With a custom map yaml:
```bash
ros2 launch amr_slam localization.launch.py map:=/home/allbotix/ros2_ws/src/amr_slam/amr_slam/map/office_map.yaml
```

Notes:
- This launch auto-calls AMCL global localization at startup.
- In RViz, use the **Nav2 Goal** tool to send goals.
- If localization is uncertain at startup, rotate the robot slowly for a few seconds.

### Drive the robot
In a separate terminal:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use keys to drive around and build the map. **Drive slowly** for best results.

### Save the map
When you're satisfied with the map:
```bash
ros2 run amr_slam save_map
# or with a custom name:
ros2 run amr_slam save_map -- --map-name office_map
```
Maps are saved to `~/maps/` as `.pgm` + `.yaml` files.

## TF Tree
```
map -> odom -> base_link -> laser_frame
 (slam)  (odom)    (static: x=0.105m)
```

## Tuning Tips

- **Odometry drift**: If the robot drifts, check encoder signs and wheel diameter.
- **SLAM quality**: Drive slowly, make loops to trigger loop closures.
- **PWM / speed**: Adjust the Arduino PID gains if wheels don't track well.
- **Covariance**: Edit `pose.covariance` in `arduino_bridge.py` if SLAM doesn't trust odom enough.
