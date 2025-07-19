# 🐢 TurtleBot3 Line Follower with STOP Sign Detection

A ROS 2 Humble package for TurtleBot3 that enables:
- White line following using camera + OpenCV
- STOP sign detection using YOLOv8
- Autonomous rotation and resume behavior

## 🧠 Features

- Custom Gazebo worlds:
  - `line_follow_world.world`
  - `stop_sign.world`
- Line following logic using OpenCV moments
- Object detection using YOLOv8n
- State machine for STOP, ROTATE, and FOLLOW
- Launch files for single and multi-robot simulation

## 📂 Package Structure

turtlebot3_line_follower/              ← Your workspace root
├── src/
│   └── turtlebot3_line_follower/      ← Your ROS 2 package
│       ├── launch/                    ← ROS 2 launch files
│       │   ├── simple_line_follow.launch.py
│       │   ├── stop_sign_line_follow_world.launch.py
│       │   └── two_robots.launch.py
│       ├── worlds/                    ← Custom Gazebo world files
│       │   ├── line_follow_world.world
│       │   ├── stop_sign.world
│       │   └── wall_world.world
│       ├── turtlebot3_line_follower/  ← Python package (contains ROS 2 nodes)
│       │   ├── __init__.py
│       │   ├── line_follower_node.py
│       │   ├── stop_sign_follower_node.py
│       │   └── yolov8_detector.py
│       ├── yolov8n.pt                 ← YOLOv8 weights
│       ├── package.xml                ← ROS 2 package manifest
│       ├── setup.py                   ← Python setup script
│       ├── setup.cfg                  ← Python packaging config
│       ├── resource/                  ← ROS 2 resource index
│       │   └── turtlebot3_line_follower
│       ├── test/                      ← Optional test directory
│       ├── build/                     ← (ignored) colcon build dir
│       ├── install/                   ← (ignored) install dir
│       └── log/                       ← (ignored) logs



## 🚀 How to Run

### 1. Clone the repository:

```bash
cd ~/turtlebot3_line_follower/src
https://github.com/Ureed-Hussain/turtlebot3-line-follower-yolov8-stop-sign.git
cd ~/turtlebot3_line_follower
colcon build
source install/setup.bash
```
### 2. Launch Simulations:

Single Robot Line Following:
```
ros2 launch turtlebot3_line_follower simple_line_follow.launch.py
```
Two Robot Line Following:
```
ros2 launch turtlebot3_line_follower two_robot.launch.py
```
STOP Sign Detection:
```
ros2 launch turtlebot3_line_follower stop_sign_line_follow_world.launch.py
```

👁️ Visualization (RViz2)

To visualize the image processing and detection results:
```
rviz2
```
Then:

    Add → Image

    Set topic: /processed_image

    Encoding: rgb8

You will now see:

    Bounding box from YOLOv8 around the stop sign.

    Segmented line used for navigation.

🧠 Requirements

    ROS 2 Humble

    TurtleBot3 Packages

    ultralytics (YOLOv8)
```
pip install ultralytics opencv-python
```



