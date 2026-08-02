# 🚁 Obsidian Eye ROS 2

An autonomous drone surveillance system built with ROS 2, PX4, Gazebo, MAVSDK, and YOLO for intelligent multi-sensor target verification.

---

## 📖 Overview

Obsidian Eye is a ROS 2-based autonomous drone surveillance system designed to detect, verify, and track aerial targets using multi-sensor fusion.

The system combines RF sensing, acoustic analysis, and computer vision to reduce false alarms and support autonomous decision-making during surveillance missions.

---
<img width="1242" height="884" alt="WhatsApp Image 2026-08-03 at 2 39 26 AM" src="https://github.com/user-attachments/assets/d12c4b90-bc0e-40f3-a1aa-9f869624654a" />


## ✨ Features

- Multi-Sensor Fusion (RF + Acoustic + YOLO)
- Autonomous Decision Engine
- Mission Management
- Autonomous Target Tracking
- PX4 SITL Integration
- Gazebo Simulation
- ROS 2 Architecture
- YOLO-based Object Detection
<img width="1600" height="900" alt="image" src="https://github.com/user-attachments/assets/7aafd1ac-7285-40b7-a166-03a1f7dd00ac" />

---

## 🏗️ System Architecture

> Architecture diagram will be added soon.

---

## 📦 Project Structure

```text
obsidian_eye_ros/
├── launch/
├── obsidian_eye_ros/
├── meshes/
├── urdf/
├── resource/
├── package.xml
├── setup.py
└── README.md
```

---

## 🛠️ Technologies

- ROS 2
- PX4
- Gazebo
- MAVSDK
- Python
- OpenCV
- Ultralytics YOLO

---

## 🚀 Getting Started

```bash
colcon build
source install/setup.bash
ros2 launch obsidian_eye_ros obsidian_eye_bringup.launch.py
```

---

## 🔮 Future Improvements

- Real Drone Integration
- Web Dashboard
- Hardware Deployment
- Advanced Sensor Fusion
- AI-based Threat Assessment

---

## 📄 License

This project is licensed under the MIT License.
