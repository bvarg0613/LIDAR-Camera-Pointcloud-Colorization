# LIDAR-Camera System Use

### End-to-End Setup, Deployment, and Colorized Mapping

## Overview

This repository documents the full pipeline for building and operating a LiDAR-based mapping system capable of generating three-dimensional point clouds in challenging environments. While the primary focus is on reliable geometry capture, the system also includes an initial implementation of point cloud colorization using multiple cameras. This colorization process is functional but remains computationally demanding and may perform slowly depending on the available hardware.

The system integrates an Ouster OS1 LiDAR as the primary sensing modality for geometric data acquisition, along with four USB cameras to provide visual information for colorization. ROS 2 is used as the backbone for data handling, communication, and synchronization, while a well-defined TF tree ensures proper spatial alignment between all sensors. In addition, SLAM is incorporated to improve mapping consistency and enable repeatable scans over time.

The resulting outputs support the generation of colorized point clouds, corridor-scale mapping, and post-processing workflows such as change detection using tools like CloudCompare. The system is specifically designed for GPS-denied and visually repetitive environments—such as underground tunnels—where maintaining consistent and accurate spatial representations is essential.


## System Architecture

The system operates through a structured data pipeline in which each sensor contributes to the final mapped output. The LiDAR continuously publishes point cloud data through ROS 2, while each of the four cameras publishes image streams along with their corresponding calibration information. A static TF tree defines the spatial relationships between the LiDAR, cameras, and base reference frame, ensuring that all data is expressed within a consistent coordinate system.

Colorization is performed by projecting LiDAR points into the image planes of the cameras using known extrinsic transformations and intrinsic camera parameters. This process produces a colored point cloud that enhances interpretability of the environment. To further improve spatial consistency, SLAM is used to refine pose estimation and reduce drift over time. Finally, the generated datasets can be exported and analyzed in post-processing tools to enable applications such as change detection and deformation analysis.


Hardware requirements include:

* Ouster OS1 LIDAR
* 4× USB cameras (USB 3.0 recommended)
* High-performance laptop or onboard computer
* Ethernet connection for LIDAR
* USB hub for cameras
* Rigid mounting frame (backpack or robot)


Software requirements include:

* Ubuntu 22.04
* ROS 2 Humble
* Python 3.10
* colcon build tools
* rosdep
* RViz2
* CloudCompare (post-processing)
* LidarView (optional SLAM validation)


## Step 0: ROS2 Installation

This repository was made using ROS2 Humble. Newer versions should work, but to mitigate any errors, it is reommended to use Humble. Below is the documentation to install it:
```bash
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
```

## Step 1: Network Setup for Ouster LIDAR
Before installing the LIDAR and SLAM drivers, it is important to ensure the LIDAR is working properly. This repository uses an Ouster OS1, but other Ouster models should work similarly.

Plug in your Ouster's interface box (power and ethernet) and check if the LIDAR connects to your computer. Locate the LIDAR's hostname (a 12-digit number usually located on the top of the physical sensor). Then navigate to this website:

```bash
os-############.local/
```

where ```############``` is your Ouster hostname. An example is shown below:
```bash
os-122208000461.local/
```

If the LIDAR is properly connected you should see the screen below:


If the LIDAR does not seem to connect, it is recommended to change the ethernet connection to "link local" as opposed to manual or DHCP. Refer to https://static.ouster.dev/sensor-docs/image_route1/image_route2/networking_guide/networking_guide.html#linux for more information on connecting your Ouster.

## Step 2: Install Ouster_ROS and LiDARSLAM Drivers

### Install dependencies
```bash
sudo apt install -y libeigen3-dev
sudo apt install -y libceres-dev
sudo apt install -y libpcl-dev
sudo apt install -y libboost-all-dev
sudo apt install -y libnanoflann-dev
```

### Install core slam ros2 wrapper
```bash
cmake -E make_directory colcon_ws && cd colcon_ws
git clone https://gitlab.kitware.com/keu-computervision/slam.git src/slam --recursive -b feat/ROS2
```

Hold off on building package until AFTER Ouster driver has been installed and built. That MUST be done first.

### Install ROS2 dependecies
Navigate to src directory
```bash
cd ~/workspaces/colcon_ws/src
```

Install each dependency
```bash
sudo apt install -y ros-$ROS_DISTRO-desktop-full
sudo apt install -y ros-$ROS_VERSION-pcl-ros
sudo apt install -y ros-$ROS_DISTRO-apriltag
sudo apt install -y ros-$ROS_DISTRO-libg2o

​sudo apt install -y         \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake                   \
    python3-colcon-common-extensions
```

### Clone Ouster repository
From inside your ```bash/colcon_ws/src``` directory:
```bash
git clone -b ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git

## Source ROS2 if needed
source /opt/ros/humble/setup.bash

## Build Ouster packages
cd ~/workspaces/colcon_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ouster_ros ouster_sensor_msgs

## Build slam package
colcon build --base-paths src/slam/ros2_wrapping --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### LiDARSLAM Live Usage
LiDARSLAM uses two different arguments to tell the node whether the LIDAR is indoors or outdoors. Use each one accordingly.
```bash
cd ~/workspaces/colcon_ws
source install/setup.bash

## Launch Ouster node
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=<sensor IP address>

## Run LiDARSLAM node on another terminal
ros2 launch lidar_slam slam_ouster.py replay:=false outdoor:=true  # outdoor use
ros2 launch lidar_slam slam_ouster.py replay:=false outdoor:=false  # indoor use
```

### LiDARSLAM ROSbag Replay
LiDARSLAM supports ROSbag replay. Ideally you should use the ```record.launch.xml``` node from the Ouster driver (as opposed to simply using ```ros2 bag record```). This saves all the topics better and allows for a smoother SLAM generation.

```bash
cd ~/workspaces/colcon_ws
source install/setup.bash

## Run Ouster rosbag replay launch file
ros2 launch ouster_ros replay.launch.xml bag_file:=/path/to/saved/rosbag

## Run LiDARSLAM node on another terminal
ros2 launch lidar_slam slam_ouster.py replay:=true outdoor:=true  # outdoor use
ros2 launch lidar_slam slam_ouster.py replay:=true outdoor:=false  # indoor use

## Commands to run for best SLAM map
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "command: 8"
```

### Saving SLAM Maps
You must publish a command to save the SLAM as a PCD. On a separate terminal, navigate to your colcon_ws and source:

```bash
cd ~/workspaces/colcon_ws
source install/setup.bash

## SAVE_KEYPOINTS_MAPS
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "{command: 16, string_arg: /path/to/maps/prefix}"

## SAVE_FILTERED_KEYPOINTS_MAP
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "{command: 17, string_arg: /path/to/maps_filtered/prefix}"
```

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

* LIDAR data visible
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

* Use LIDAR SLAM (e.g., Kitware LidarSLAM)
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
* LIDAR scan stability
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
