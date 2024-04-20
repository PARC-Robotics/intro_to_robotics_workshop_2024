# PARC 2024 Introduction to Robotics Workshop Repository

## Instructions:

### Clone the repository

Place this in the same workspace as the original PARC Engineers League repository

```
cd ~/ros2_ws/src
git clone https://github.com/PARC-Robotics/intro_to_robotics_workshop_2024.git

```

### Compile

```
cd ~/ros2_ws
colcon build
```

### Run

In a terminal, run the following commands to open Gazebo and RViz

```
source ~/ros2_ws/install/setup.bash
ros2 launch intro_to_robotics_workshop_2024 example1_launch.py

```

In a new terminal run this command to move the robot until it is 6m away from its start position

```
ros2 run intro_to_robotics_workshop_2024 example1_basic_node

```
