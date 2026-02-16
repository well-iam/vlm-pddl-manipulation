# A VLM-based Control Framework for Robotic Manipulation Tasks with Plan Verification

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python 3.10](https://img.shields.io/badge/Python-3.10-yellow.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

> **Project for Master Thesis in AI & Cognitive Robotics** <br/>
> *Implementation of a VLM-based high-level planner for long-horizon manipulation tasks using Franka Emika Research 3 (FR3).*

## 🎥 Demo

**Task:** *"The kids have forgotten the toys on the table, clean them"* <br/>
**Outcome:** The agent interprets the vague command, identifies objects via the Perception Server, and executes a multi-step cleanup using MoveIt.

<div align="center">
  <img src="https://github.com/well-iam/well-iam/blob/main/previews/vlm_control_framework.gif" alt="Robot Demo GIF" width="60%">
  <br>
  <a href="https://youtu.be/C8rL8y8n__4">
    <img src="https://img.youtube.com/vi/C8rL8y8n__4/0.jpg" alt="Watch full video" width="40%">
  </a>
  <br>
  <em>(Click to watch the full experiment)</em>
</div>


## 📖 Overview

This repository hosts a modular framework for **Zero-Shot Robotic Manipulation** driven by Vision-Language Models (VLMs). The system bridges the gap between semantic reasoning (Gemini API) and low-level control (ROS 2 / MoveIt), enabling the robot to understand unstructured commands and interact with open-world objects.

### Key Features
* **VLM-Centric Reasoning:** Utilizes a "Chain-of-Thought" prompting strategy (Inner Monologue) to decompose abstract commands into executable motion primitives.
* **Perception-Action Loop:** real-time integration of RGB-D data (Intel RealSense) with joint-space control.
* **Modular Architecture:**
    * `perception_server`: Handles 3D object detection, pose estimation, and point cloud processing.
    * `vlm_agent`: Core logic module encapsulating the LLM interface and task planning.
    * `real_franka_robot`: Hardware interface, motion planning (MoveIt), and action execution.

## 🏗️ Architecture

The system follows a modular ROS 2 design:

| Package | Type | Description |
| :--- | :--- | :--- |
| **`real_franka_robot`** | Control & Logic | Orchestrates the **Inner Monologue** node (LLM reasoning), maintains the TF tree, and interfaces with MoveIt for trajectory execution. |
| **`perception_server`** | Service Node | A dedicated ROS service that processes point clouds (from `realsense-ros`) to provide 3D object poses and scene semantics upon request. |
| **`vlm_agent`** | Library | Embedded Python module for Chain-of-Thought reasoning and Gemini API interaction. |

**External Dependencies:**
* `franka_ros2` (Hardware drivers)
* `realsense-ros` (Camera drivers)

## 🚀 Installation & Usage

### Prerequisites
* Ubuntu 22.04 LTS
* ROS 2 Humble
* Intel RealSense SDK 2.0
* `libfranka` & `franka_ros2`

### Setup
```bash
# Clone the repository
git clone https://github.com/well-iam/vlm-pddl-manipulation.git
cd vlm-pddl-manipulation
```

### Install dependencies
`rosdep install --from-paths src --ignore-src -r -y`

### Build the workspace
`colcon build --symlink-install`

# 🚀 Running the System (Modular Execution)
To allow for granular control and real-time debugging of each subsystem, the framework is launched across distinct terminals.

**1. Robot Hardware (High Priority)**:
Initializes the FR3 drivers and MoveIt planning pipeline.
```bash
launch franka_fr3_moveit_config fr3_moveit.launch.py robot_ip:=<robot-ip>
```
**2. Vision Drivers**
Starts the Intel RealSense camera stream.
```bash
ros2 launch realsense2_camera rs_launch.py clip_distance:=1.0 config_file:=$(ros2 pkg prefix real_franka_robot)/config/realsense_manipulation.yaml
```
**3. Static Transforms**
Publishes the calibrated camera-to-robot transform.
```bash
ros2 realsense_to_robot_static_transform.launch.py
```
**4. Perception Service**
Starts the object detection server.
```bash
ros2 launch perception_server get_scene_objects.launch.py
```
**5. Main Agent**
Runs the VLM reasoning loop. Ensure your API key is set.
```bash
export GEMINI_API_KEY="your_api_key"
ros2 run real_franka_robot main
```
Note: For manual interventions (e.g., error recovery or controller switching), a separate terminal is recommended for service calls.

# 🛠️ Hardware Setup
- Manipulator: Franka Emika Research 3 (FR3).
- Vision: Intel RealSense D435i (Eye-to-hand / Fixed configuration).
- Compute: Workstation with Real-Time Kernel patch.

## 📐 Sensor Calibration (Eye-to-hand)

To replicate the pick-and-place accuracy, an accurate extrinsic calibration between the camera (RealSense) and the robot base (FR3) is required.

### Calibration Target
The system uses a **ChArUco Board** with the following specifications:
* **Layout:** 5x5 Squares
* **Marker Size:** 50 px
* **Square Size:** 80 px
* **Margin Size:** 10 px
* **Marker Border:** 1 bits
* **Dictionary:** DICT_4X4_50

### Procedure
The extrinsic calibration routine is based on the [MoveIt's Hand-Eye calibration plugin](https://github.com/moveit/moveit_calibration).

1.  **Preparation:**
    * Dismount the Franka Hand (gripper).
    * Mount the ChArUco board rigidly in the workspace.
    * Switch the robot controller to "Migrated Profile" (Desk > Settings > End-Effector).

2.  **Launch Calibration Node:**
    Run the camera driver with calibration settings:
    ```bash
    ros2 launch realsense2_camera rs_launch.py config_file:=$(ros2 pkg prefix real_franka_robot)/config/realsense_calib.yaml
    ```

3.  **Execute MoveIt Calibration:**
    Launch the specialized RViz configuration for calibration:
    ```bash
    ros2 launch franka_fr3_moveit_config fr3_moveit.launch.py rviz_config:=calib.rviz
    ```
    *Follow the MoveIt GUI instructions to collect samples across different poses.*

4.  **Process & Apply Transform:**
    The MoveIt solver outputs the transform relative to the `camera_optical_frame`. To align this with the URDF physical model (`camera_link`), a frame conversion is required.
    
    * Write the solver outputs in helper script and run it:
        ```bash
        python3 src/real_franka_robot/calibration/camera_transform.py
        ```
    * Copy the corrected Translation and Quaternion values into:
        `launch/static_transform_publisher.launch.py`

# 🧪 Experiments & Sim-to-Real
This branch (main) contains the Real World deployment code.
For the simulation environment (CoppeliaSim) please refer to the `dev` branch.
