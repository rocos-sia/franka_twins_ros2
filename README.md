# Franka Symphony: ROS 2 Dual-Arm Manipulation Framework

> A ROS 2-based control and coordination framework for dual Franka arms.


## ✨ Overview

**Franka Symphony** builds upon several excellent open-source projects to provide a dual-arm manipulation framework.  
This repository **integrates and extends** the capabilities of [franky](https://github.com/TimSchneider42/franky) to support coordinated control of two Franka Panda robots under ROS 2.

---

## 🤝 Acknowledgments

This project was inspired by and partially based on:

- [franky](https://github.com/TimSchneider42/franky): A high-level C++ and Python interface for real-time Cartesian impedance control of Franka Emika Panda arms.  
  We use franky as the core low-level control backend for motion primitives.
- [libfranka](https://github.com/frankaemika/libfranka): Official C++ interface to the Franka Control Interface.
- The ROS 2 and MoveIt communities.

Special thanks to Tim Schneider for developing franky and making it available under the MIT License.

---

## 📂 Repository Structure


---

## ⚙️ Requirements

- Ubuntu 22.04
- ROS 2 Humble 
- libfranka == 0.15.0
- Franka Control Interface (FCI)
- Franky

---

## 🚀 Quick Start

1️⃣ **Clone this repository**
```bash
git clone https://github.com/rocos-sia/franka_twins_ros2.git
cd franka_twins_ros2
```


2️⃣ **Install dependencies**
```bash
pip install franky-control
rosdep install -i --from-path src --rosdistro humble -y
```

3️⃣ **Build the package**
```bash
colcon build --symlink-install
source install/setup.bash
```
4️⃣ **Run the example**
```bash
#先启动这个程序，它会默认开始循环move()。
ros2 run franka_twins wave  
#用ros2 topic pub发布控制信号
ros2 topic pub /control_signal std_msgs/String "data: 'stop'"  
#就会停止运动。再发：
ros2 topic pub /control_signal std_msgs/String "data: 'start'"
#就重新启动。
```

```bash
#单夹爪
ros2 launch gripper gripper.launch.py
#用ros2 topic pub发布控制信号
ros2 topic pub /right/gripper/command tosor_msgs/msg/GripperDistance "{distance: 100}"

#双夹爪
ros2 launch gripper gripper_dual.launch.py
#用ros2 topic pub发布控制信号
ros2 topic pub /left/gripper/command tosor_msgs/msg/GripperDistance "{distance: 100}"
ros2 topic pub /right/gripper/command tosor_msgs/msg/GripperDistance "{distance: 100}"
```

```bash
#相机
pip install pyrealsense2
pip install mediapipe
pip install --upgrade pip
pip install "protobuf>=5.26.1,<6.0dev"
pip install "grpcio>=1.64.0"
pip install ultralytics

```
## 📄 License

This project is licensed under the Apache License 2.0.  
Note that components derived from franky remain subject to the MIT License.
