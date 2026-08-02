# 🚁 Obsidian Eye ROS 2

<p align="center">
  <img src="docs/images/architecture.png" width="900" alt="Obsidian Eye Architecture">
</p>

<p align="center">
Autonomous Drone Surveillance System built with <b>ROS 2</b>, <b>PX4</b>, <b>Gazebo</b>, <b>MAVSDK</b>, and <b>YOLOv8</b> for intelligent multi-sensor target verification.
</p>

---

# 📖 Overview

Obsidian Eye ROS 2 is an autonomous drone surveillance system designed to patrol critical infrastructure such as oil pipelines.

The system combines RF sensing, acoustic detection, and computer vision to verify aerial targets while minimizing false alarms through sensor fusion and autonomous decision-making.

---

# ✨ Features

- Multi-Sensor Fusion
- RF Signal Detection
- Acoustic Drone Detection
- YOLOv8 Object Detection
- Autonomous Decision Engine
- Mission Management
- Autonomous Target Tracking
- PX4 SITL Integration
- Gazebo Simulation
- ROS 2 Architecture

---

# 🏗️ System Architecture

<p align="center">
  <img src="docs/images/architecture.png" width="1000">
</p>

---

# 🔗 ROS 2 Communication Graph

The following graph illustrates the communication between ROS 2 nodes, topics, actions, and mission management components.

<p align="center">
  <img src="docs/images/rqt_graph.png" width="950">
</p>

---

# 🚁 Gazebo Simulation

The complete surveillance system is validated in Gazebo using a PX4 SITL drone flying over an industrial oil pipeline environment.

<p align="center">
  <img src="docs/images/gazebo.png" width="950">
</p>

---

# 🎯 YOLO Target Detection

YOLOv8 is used to verify suspicious targets before autonomous tracking begins.

<p align="center">
  <img src="docs/images/yolo_detection.png" width="900">
</p>

---

# 📂 Project Structure

```text
obsidian_eye_ros/
│
├── launch/
├── obsidian_eye_ros/
│   ├── mission_node.py
│   ├── decision_node.py
│   ├── camera_yolo_node.py
│   ├── tracking_action_server.py
│   ├── rf_node.py
│   ├── acoustic_node.py
│   └── flight_safety_node.py
│
├── meshes/
├── urdf/
├── resource/
├── docs/
│   └── images/
│
├── package.xml
├── setup.py
└── README.md
```

---

# 🛠️ Technology Stack

- ROS 2
- Python
- PX4 Autopilot
- MAVSDK
- Gazebo Sim
- OpenCV
- Ultralytics YOLOv8

---

# 🚀 Getting Started

```bash
colcon build

source install/setup.bash

ros2 launch obsidian_eye_ros obsidian_eye_bringup.launch.py
```

---

# 📌 Future Work

- Real Drone Deployment
- Hardware Sensor Integration
- Advanced Sensor Fusion
- AI Threat Classification
- Web Dashboard
- Remote Mission Monitoring

---

# 📄 License

This project is licensed under the MIT License.
