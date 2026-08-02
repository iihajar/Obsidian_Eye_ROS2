# 🚁 Obsidian Eye ROS 2
An autonomous drone surveillance system built with ROS 2, PX4, Gazebo, MAVSDK, and YOLO for intelligent multi-sensor target verification

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

<img width="1242" height="884" alt="WhatsApp Image 2026-08-03 at 2 39 26 AM" src="https://github.com/user-attachments/assets/39f776bc-4cd0-420e-9d31-90b5ff13e040" />

---

# 🔗 ROS 2 Communication Graph

The following graph illustrates the communication between ROS 2 nodes, topics, actions, and mission management components.

<img width="1600" height="900" alt="image" src="https://github.com/user-attachments/assets/a6fd4dbe-6a08-4125-809a-5c80e204d565" />

---

# 🚁 Gazebo Simulation

The complete surveillance system is validated in Gazebo using a PX4 SITL drone flying over an industrial oil pipeline environment.

<img width="1600" height="900" alt="WhatsApp Image 2026-06-24 at 3 42 52 AM" src="https://github.com/user-attachments/assets/30273c7a-92b4-4bae-ab9f-022a2e4c858c" />


---

# 🎯 YOLO Target Detection

YOLOv8 is used to verify suspicious targets before autonomous tracking begins.
<img width="1242" height="859" alt="yolo" src="https://github.com/user-attachments/assets/20fcc65f-407d-4545-a026-fad3df3ea286" />

<img width="1218" height="881" alt="dashboard" src="https://github.com/user-attachments/assets/e28d2700-9ba9-48aa-8f36-754f73392b89" />

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
