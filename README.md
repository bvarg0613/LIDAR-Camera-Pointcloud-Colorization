# LIDAR–Camera Pointcloud Colorization (ROS 2)

## Overview

This repository contains a ROS 2 workspace for basic pointcloud colorization using a 3D LiDAR and four USB cameras. The system projects camera images onto LiDAR points using known extrinsic transforms and simple pinhole camera models.

This project was developed for experimental and research use. It is not a fully optimized or production-ready colorization pipeline.

Key characteristics:

* Designed for ROS 2 Humble
* Tested with an Ouster LiDAR and four USB cameras
* Performs basic pointcloud colorization
* Computationally heavy and somewhat laggy
* Intended as a foundation for further optimization and development

Two colorization nodes are provided:

* A standard colorization node with denser output and slower performance
* A faster colorization node with reduced point density

This repository is meant to include everything needed to reproduce, understand, and extend the system.

---

## System Concept

The system assumes:

* A single LiDAR publishes a PointCloud2
* Four cameras publish synchronized image streams
* Static transforms define the spatial relationship between the LiDAR, cameras, and base frame
* Colorization occurs by projecting 3D points into camera image planes

The quality of the output depends heavily on:

* Accurate extrinsic calibration
* Camera resolution and frame rate
* CPU performance
* TF correctness

---

## Example System Setup

You can include photos, diagrams, and screenshots of your physical system here.

Create a folder called `images/` in the repository root and place your images inside it.

Example:

```
images/
├── system_overview.jpg
├── sensor_mounting.jpg
├── measurement_screenshot.png
```

Embed images in this README like this:

```markdown
![System Overview](images/system_overview.jpg)
![Sensor Mounting](images/sensor_mounting.jpg)
```

Suggested images:

* Full system overview
* Camera and LiDAR mounting
* Measurement references from CAD or physical measurements
* RViz screenshots showing colored pointcloud output

---

## Prerequisites

* Ubuntu 22.04
* ROS 2 Humble
* Working network interface for Ouster LiDAR
* Four USB cameras recognized by the OS
* `colcon`, `rosdep`, and standard ROS 2 build tools

---

## Step 1: Install and Verify Ouster LiDAR Driver

This project assumes the use of an Ouster LiDAR.

Clone and install the Ouster ROS 2 driver (Humble branch):

```
https://github.com/ouster-lidar/ouster-ros/tree/humble-devel
```

Follow the instructions in that repository exactly.

Before proceeding:

* Confirm the LiDAR is publishing a PointCloud2
* Confirm TFs from the LiDAR driver are available
* Verify visualization in RViz

Do not continue until the LiDAR is fully functional.

---

## Step 2: Install usb_cam for ROS 2

Install the `usb_cam` driver:

```
https://index.ros.org/p/usb_cam/
```

The package includes two example parameter files by default. This project requires four cameras, so you must create two additional parameter files.

### Camera Parameter Files

Create:

* `params_3.yaml`
* `params_4.yaml`

Each parameter file must specify the correct video device.

Check available devices:

```bash
ls /dev/video*
```

Example mapping:

* camera1 → /dev/video0
* camera2 → /dev/video2
* camera3 → /dev/video4
* camera4 → /dev/video6

Each YAML file must reflect the correct `video_device`.

---

## Step 3: Modify the Camera Launch File

The launch file must explicitly launch four camera nodes and point to the four parameter files.

This repository references a launch structure similar to the following, where cameras are appended and launched as a group. This file demonstrates how additional cameras are added and parameter files assigned .

Ensure that:

* All four cameras are listed
* Each camera has a unique name
* Each camera uses its corresponding parameter file
* Remappings and namespaces are consistent

---

## Step 4: Clone and Build This Repository

Clone this repository into a ROS 2 workspace:

```bash
mkdir -p ~/workspaces/colorization_ws/src
cd ~/workspaces/colorization_ws/src
git clone https://github.com/bvarg0613/LIDAR-Camera-Pointcloud-Colorization.git
```

Install dependencies:

```bash
cd ~/workspaces/colorization_ws
rosdep install --from-paths src -y --ignore-src
```

Build:

```bash
colcon build
source install/setup.bash
```

---

## Package Overview

### lidar_camera_fusion

Contains the pointcloud colorization logic.

Key nodes:

* `colorize_node.py`
  Slower, denser colorization output

* `colorize_node_fast.py`
  Faster execution with reduced point density

### rig_tf

Publishes all static transforms required for colorization.

This package is critical. Incorrect transforms will result in misaligned or incorrect coloring.

---

## Static Transforms and Calibration (rig_tf)

The `rig_tf` package defines:

* base_link → os_sensor
* base_link → each camera frame

These transforms must be edited for your physical system.

Distances and orientations should come from:

* A CAD model, or
* Careful physical measurements if CAD is unavailable

### Example Measurement Workflow

1. Define `base_link` as the reference origin
2. Measure each sensor position relative to `base_link`
3. Convert measurements to meters
4. Define roll, pitch, and yaw for each sensor
5. Update the static transform publisher accordingly

You can include a measurement screenshot like this:

```markdown
![Measurement Reference](images/measurement_screenshot.png)
```

Clearly label:

* Axes directions
* Frame origins
* Offsets used in the code

---

## RViz Configuration Notes

You must open two RViz panels:

* One for raw sensor verification
* One for colorized output

Important:

* The second RViz instance must subscribe to the correct colored pointcloud topic
* The fixed frame must be set to `base_link`

If the fixed frame is incorrect, colorization will appear distorted or invisible.

---

## Running the Colorization

After sourcing the workspace:

```bash
source install/setup.bash
```

Launch sensors and transforms first:

* Ouster LiDAR
* All four cameras
* `rig_tf` static transforms

Then run one of the colorization nodes.

Standard node:

```bash
ros2 run lidar_camera_fusion colorize_node
```

Fast node:

```bash
ros2 run lidar_camera_fusion colorize_node_fast
```

Use the fast node if performance becomes a bottleneck.

---

## Limitations and Future Work

* No synchronization guarantees between cameras and LiDAR
* CPU-bound colorization
* No GPU acceleration
* No dynamic calibration
* Minimal error handling

This repository is intended as a baseline for:

* Performance optimization
* Improved synchronization
* GPU-based colorization
* Advanced calibration pipelines

---

## Questions Before Finalizing

Before I refine this README further, I need the following:

1. Do you want launch files for the colorization nodes documented explicitly?
2. Should camera intrinsic calibration steps be added?
3. Do you want a recommended RViz config (.rviz file) included?
4. Should approximate expected topic names be hard-coded in the README?
5. Do you want a troubleshooting section for common TF and image issues?

Answering these will let me tighten this into a near-publication-quality research repo README.
