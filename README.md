# LIDAR Backpack System (ROS 2)

### End-to-End Setup, Deployment, and Colorized Mapping

## Overview

This repository documents the full pipeline for building and operating a wearable LiDAR mapping system capable of generating colorized 3D point clouds for underground environments.

The system integrates:

* Ouster OS1 LiDAR (primary geometry sensor)
* Four USB cameras (for colorization)
* ROS 2 (data handling and synchronization)
* TF tree (sensor alignment)
* Optional SLAM (for mapping and repeatability)

The final output supports:

* Colorized point clouds
* Corridor-scale mapping
* Change detection workflows (e.g., CloudCompare)

This system is designed for GPS-denied, visually repetitive environments such as underground tunnels, where consistent geometry capture is critical.

---

## System Architecture

Core data flow:

1. LiDAR publishes `/ouster/points`
2. Cameras publish `/cameraX/image_raw` and `/cameraX/camera_info`
3. Static TF defines all sensor relationships
4. Colorization node projects LiDAR points into camera frames
5. Output is a colored point cloud

Optional:

6. SLAM refines pose and reduces drift
7. Post-processing enables change detection

---

## Hardware Requirements

* Ouster OS1 LiDAR
* 4× USB cameras (USB 3.0 recommended)
* High-performance laptop or onboard computer
* Ethernet connection for LiDAR
* USB hub for cameras
* Rigid mounting frame (backpack or robot)

---

## Software Requirements

* Ubuntu 22.04
* ROS 2 Humble
* Python 3.10
* colcon build tools
* rosdep

Recommended:

* RViz2
* CloudCompare (post-processing)
* LidarView (optional SLAM validation)

---

## Step 0: Install ROS 2 Humble

```bash
sudo apt update && sudo apt upgrade -y

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update
sudo apt install curl -y

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -

sudo apt install ros-humble-desktop -y
```

Source ROS:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Install tools:

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep -y
rosdep init
rosdep update
```

---

## Step 1: Network Setup for Ouster LiDAR

You must configure Ethernet so your computer can communicate with the LiDAR.

Typical static setup:

* Computer IP: `192.168.90.1`
* LiDAR IP: `192.168.90.2`

Verify connection:

```bash
ping 192.168.90.2
```

Access LiDAR web interface:

```
http://192.168.90.2
```

Do not continue until this works.

---

## Step 2: Install Ouster ROS Driver

Clone into workspace:

```bash
mkdir -p ~/lidar_ws/src
cd ~/lidar_ws/src
git clone https://github.com/ouster-lidar/ouster-ros.git -b humble-devel
```

Install dependencies:

```bash
cd ~/lidar_ws
rosdep install --from-paths src -y --ignore-src
```

Build:

```bash
colcon build
source install/setup.bash
```

Run driver:

```bash
ros2 launch ouster_ros sensor.launch.py
```

Verify:

```bash
ros2 topic list
```

You should see:

```
/ouster/points
/tf
/tf_static
```

---

## Step 3: Install and Configure USB Cameras

Install package:

```bash
sudo apt install ros-humble-usb-cam
```

Check devices:

```bash
ls /dev/video*
```

Create 4 parameter files:

```
params_1.yaml
params_2.yaml
params_3.yaml
params_4.yaml
```

Each must define:

```yaml
video_device: "/dev/videoX"
image_width: 640
image_height: 480
framerate: 10
```

Important:

* Keep frame rate low (10 FPS) to avoid USB overload
* Ensure consistent device mapping

---

## Step 4: Create ROS 2 Workspace for Colorization

```bash
mkdir -p ~/colorization_ws/src
cd ~/colorization_ws/src
git clone <your_repo>
```

Install dependencies:

```bash
cd ~/colorization_ws
rosdep install --from-paths src -y --ignore-src
```

Build:

```bash
colcon build
source install/setup.bash
```

---

## Step 5: TF Tree Setup (Critical)

Your TF tree must match:

```
base_link
├── os_sensor
│   ├── os_lidar
│   └── os_imu
├── camera1
├── camera2
├── camera3
└── camera4
```

You must define:

* Translation (x, y, z in meters)
* Rotation (roll, pitch, yaw)

These come from:

* CAD model (preferred)
* Physical measurements

Run TF publisher:

```bash
ros2 run rig_tf static_tf_publisher
```

Verify:

```bash
ros2 run tf2_tools view_frames
```

Check:

* All frames connected
* No missing links
* No frame mismatches

---

## Step 6: Launch Cameras

Create launch file that starts all 4 cameras.

Verify topics:

```bash
ros2 topic list
```

Expected:

```
/camera1/image_raw
/camera2/image_raw
...
```

---

## Step 7: Validate System in RViz

Open RViz:

```bash
rviz2
```

Set:

* Fixed frame: `base_link`

Add:

* PointCloud2 → `/ouster/points`
* Image → `/cameraX/image_raw`
* TF

Ensure:

* LiDAR data visible
* Cameras publishing
* TF aligns correctly

---

## Step 8: Run Colorization

Standard node:

```bash
ros2 run lidar_camera_fusion colorize_node
```

Fast node:

```bash
ros2 run lidar_camera_fusion colorize_node_fast
```

Output:

* Colored point cloud topic

If misaligned:

* TF is wrong
* Camera calibration is wrong
* Frame IDs mismatch

---

## Step 9: (Optional) SLAM Integration

For mapping and repeatability:

* Use LiDAR SLAM (e.g., Kitware LidarSLAM)
* Tune parameters such as:

  * voxel size
  * keyframe spacing
  * loop closure thresholds

Goal:

* Reduce drift
* Enable repeatable scans for change detection

---

## Step 10: Post-Processing (CloudCompare)

Workflow:

1. Export point clouds
2. Align scans
3. Perform cloud-to-cloud distance

Used for:

* Detecting millimeter-level changes
* Monitoring structural deformation

---

## Key Dependencies for Accuracy

System performance depends on:

* Accurate TF calibration
* Stable mounting
* Consistent camera exposure
* LiDAR scan stability
* Minimal motion distortion

---

## Common Failure Points

If colorization fails:

* Wrong frame_id in camera topics
* Missing TF links
* Cameras not synchronized
* Incorrect intrinsic parameters
* CPU overload

Debug with:

```bash
ros2 topic echo /tf_static --once
```

---

## Limitations

* No hardware synchronization
* CPU-heavy processing
* No GPU acceleration
* Sensitive to calibration errors
* Limited performance in low-texture environments

---

## Future Improvements

* Hardware time synchronization
* GPU-based projection
* Automated calibration pipeline
* Improved SLAM integration
* Real-time change detection

---

If you want, I can next:

* Add a clean launch file structure
* Include camera calibration steps
* Add a troubleshooting flowchart
* Turn this into a polished GitHub-ready README with diagrams
