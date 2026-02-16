# A VLM-based Control Framework for Robotic Manipulation with PDDL Verification

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python 3.10](https://img.shields.io/badge/Python-3.10-yellow.svg)](https://www.python.org/)
[![YouTube](https://img.shields.io/badge/YouTube-Watch%20Full%20Demo-red?logo=youtube)](https://youtu.be/C8rL8y8n__4)

> **Master Thesis in AI & Cognitive Robotics** <br/>
> **University:** Università degli Studi di Napoli Federico II | **Lab:** PrismaLab <br/>
> **Supervisor:** Prof. Alberto Finzi | **Author:** William Notaro

## 📝 Abstract

Vision-Language Models (VLMs) have demonstrated impressive capabilities in high-level reasoning for robotic tasks. However, their deployment in real-world manipulation is often hindered by hallucinations, lack of physical grounding, and the generation of unsafe or geometrically infeasible plans. This thesis presents a hybrid neuro-symbolic framework for zero-shot robotic manipulation that bridges the gap between semantic reasoning and low-level control.

We introduce a VLM-based agent capable of interpreting unstructured, long-horizon commands (e.g., "clean the table") via a Chain-of-Thought. Crucially, to ensure reliability, we integrate a PDDL (Planning Domain Definition Language) verification layer. This acts as a logic filter, validating the VLM-generated actions against a symbolic world state before execution. The system is implemented on a Franka Emika Research 3 (FR3) robot, demonstrating the feasibility of using symbolic planning to ground generative models in physical constraints, providing a safeguard against hallucinations in open-world pick-and-place tasks.

---

## 🎥 Demo

**Task:** *"The kids have forgotten the toys on the table, clean them"* <br/>
**Challenge:** The command is abstract ("clean") and requires long-horizon planning with multiple objects.
> [!TIP]
> **Check the Full Video Demo:** [YouTube Video](https://youtu.be/C8rL8y8n__4)
<div align="center">
  <img src="https://github.com/well-iam/well-iam/blob/main/previews/vlm_control_framework.gif" alt="Robot Demo GIF" width="60%">
  <br>
  <em>The agent interprets the vague command, verifies actions via PDDL, and executes a multi-step cleanup.</em>
</div>

### Key Features
* **🧠 Neuro-Symbolic Architecture:** Combines the creativity of VLMs (Chain-of-Thought reasoning) with the safety guarantees of symbolic planning (PDDL).
* **🛡️ Plan Verification:** Automatically validates the VLM's output against a symbolic domain to prevent logical hallucinations or unsafe actions before execution.
* **🧩 Modular Design:**
    * `perception_server`: Handles 3D object detection and point cloud processing.
    * `vlm_agent`: Core logic module encapsulating the LLM interface and task planning.
    * `real_franka_robot`: Handles hardware, motion planning (MoveIt), and action execution.

## 🏗️ Architecture

The system follows a modular ROS 2 design:

| Package | Type | Description |
| :--- | :--- | :--- |
| **`real_franka_robot`** | Control & Logic | Orchestrates the main node, maintains the TF tree, and interfaces with MoveIt for execution. |
| **`perception_server`** | Service Node | Processes point clouds (RealSense) to provide 3D object poses and scene semantics. |
| **`vlm_agent`** | Library | **Core Intelligence.** Handles prompt engineering, PDDL state generation, and plan validation. |

**External Dependencies:** 
 * `franka_ros2` (Hardware)
 * `realsense-ros` (Vision)
 * `google-generativeai` (Reasoning)

## 🛠️ Installation & Usage

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
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Build the workspace
```bash
colcon build --symlink-install
```

## 🚀 Running the System (Modular Execution)
To allow for granular control and real-time debugging of each subsystem, the framework is launched across distinct terminals.

**1. Robot Hardware (High Priority)**:
Initializes the FR3 drivers and MoveIt planning pipeline.
```bash
ros2 launch franka_fr3_moveit_config fr3_moveit.launch.py robot_ip:=<robot-ip>
```
**2. Vision Drivers**
Starts the Intel RealSense camera stream.
```bash
ros2 launch realsense2_camera rs_launch.py clip_distance:=1.0 config_file:=$(ros2 pkg prefix real_franka_robot)/config/realsense_manipulation.yaml
```
**3. Static Transforms**
Publishes the calibrated camera-to-robot TF.
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

## 🛠️ Hardware Setup
- **Manipulator:** Franka Emika Research 3 (FR3).
- **Vision:** Intel RealSense D435i (Eye-to-hand / Fixed configuration).
- **Compute:** Workstation with Real-Time Kernel patch.

## 📐 Sensor Calibration (Eye-to-hand)
To ensure sub-centimeter pick-and-place accuracy, precise extrinsic calibration is performed.

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

## 🧪 Experiments & Sim-to-Real
- `main` branch: Real World deployment code (FR3 Hardware).
- `dev` branch: Simulation environment (CoppeliaSim) and experimental baselines.
