# VLM-PDDL Robotic Manipulation (Simulation)

[![Release](https://img.shields.io/badge/release-v1.1.0-blue.svg)](https://github.com/well-iam/vlm-pddl-manipulation/releases/tag/v1.1.0-sim-stable)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://www.python.org/)

## 🤖 Project Overview
This repository contains a hybrid architecture for robotic manipulation, integrating **Vision-Language Models (VLMs)** for high-level reasoning and **PDDL (Planning Domain Definition Language)** for symbolic verification.

The system is tested in a simulated environment using **CoppeliaSim** and a Franka Emika Panda robot.

> **Note:** This version (`v1.1.0`) is the stable release for the **Simulation Environment**.
> For the Real Robot implementation (ROS2), please check the `main` branch or upcoming `v2.0.0` releases.

### Key Features
* **VLM Agent:** Uses Google Gemini models for semantic reasoning and task planning.
* **PDDL Verification:** `PlanValidator` ensures generated plans are logically sound before execution.
* **Modular Architecture:** Decoupled `vlm_agent` package for easy Sim-to-Real transfer.
* **Simulation:** Full integration with CoppeliaSim via ZMQ Remote API.

---

## 🛠️ Installation

### 1. Prerequisites
* Python 3.8 or higher
* [CoppeliaSim](https://www.coppeliarobotics.com/) (Edu or Pro version)
* A Google Cloud API Key (for Gemini)

### 2. Clone the Repository
```bash
git clone [https://github.com/well-iam/vlm-pddl-manipulation.git](https://github.com/well-iam/vlm-pddl-manipulation.git)
cd vlm-pddl-manipulation
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

### 4. Configure API Key
Create a .env file in the root directory and add your key:
```Ini, TOML
GOOGLE_API_KEY=your_api_key_here
```

## 🚀 How to Run

### Step 1: Start the Simulator
 1. Open CoppeliaSim.
 2. Load the scene file located at: `simulation/scenes/scene_with_franka.ttt` (or your specific scene file).

### Step 2: Run the Agent
Open a terminal in the project root and execute:
```bash
# Replace 'src/main.py' with the actual path to your entry script
python src/main.py
```

### Step 3: Interact
The system will initialize the connection. You can interact via the console to assign tasks (e.g., "Pick up the red cube"). The agent will:
 1. Capture the scene.
 2. Generate a plan using Gemini.
 3. Validate it with PDDL.
 4. Execute the actions in the simulator.

