# ✨ PONYLINK Multi-Floor Navigation
*SPARO Lab & RCI Lab- Quadruped Robot Autonomous Navigation System*

---

## 📦 Clone
```bash
mkdir -p ${YOUR_WORKSPACE_PATH}/src && cd ${YOUR_WORKSPACE_PATH}/src
git clone --recursive https://github.com/sparolab/SPARO_Multi_Floor_Nav.git .
```

---

## 🐋 Docker Setup

### Option A: Build from Dockerfile
```bash
cd ${YOUR_WORKSPACE_PATH}
mv src/docker ./docker

cd docker

xhost +local:docker

chmod +x container_build.sh container_run.sh

./container_build.sh

./container_run.sh

# Attach to the running container
docker attach mfnav_container
# (From another terminal) access the same container
docker exec -it mfnav_container /bin/bash

cd /home/test_ws
```

### Option B: Pull Pre-built Image
```bash
docker pull sparolab/mfnav:latest

xhost +local:docker

docker run -it \
  --name mfnav_container \
  --gpus all \
  --privileged \
  --net=host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  sparolab/mfnav:latest

# (From another terminal) access the same container
docker exec -it mfnav_container /bin/bash

cd /home/test_ws
```
## 🌟 Navigation & SLAM Dependencies
---
```bash
# Navigation2
ros-humble-navigation2
ros-humble-nav2-bringup
ros-humble-turtlebot3-navigation2

# SLAM
ros-humble-slam-toolbox

# Robot state & URDF tools
ros-humble-xacro
ros-humble-robot-state-publisher
ros-humble-joint-state-publisher-gui

# ROS2 Control
ros-humble-control-toolbox
ros-humble-ros2-control
ros-humble-ros2-controllers
ros-humble-hardware-interface
ros-humble-controller-interface
ros-humble-controller-manager

# Sensor / Mapping
ros-humble-octomap-ros
ros-humble-octomap-rviz-plugins
ros-humble-velodyne-laserscan
```
---

## 🛠️ Build & Gazebo Path
```bash
colcon build --symlink-install
source install/setup.bash

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/test_ws/src/RCI_quadruped_robot_navigation/rl_sar/models/

export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/test_ws/install
```

---

## 🤖 Robot Launch
```bash
# Terminal 1: Launch Gazebo scene
ros2 launch rl_sar gazebo.launch.py rname:=b2

# Terminal 2: Start robot simulation node
ros2 run rl_sar rl_sim

# Terminal 3: Start action/command interface
ros2 run rl_action command
# (csuite) activation true
# (csuite) nav_mode true
```

---

## 🔄 Troubleshooting

### Teleport Robot in Gazebo
```bash
ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState \
  '{state: {name: "robot_model", pose: {position: {x: 20.0, y: -35.0, z: 6.0}, orientation: {w: 1.0}}}}'
```

---

## ⚙️ Demo Scenario Setup

### 1. Initialize Elevators
* Elevators
  * **Lift1 → Floor 1**
    ```bash
    ros2 topic pub /lift1/cmd_floor std_msgs/msg/Int32 "{data: 0}"
    ```

  * **Lift2 → Floor 2**
    ```bash
    ros2 topic pub /lift2/cmd_floor std_msgs/msg/Int32 "{data: 1}"
    ```
* Robot
  ```bash
  gz model -m robot_model -x 20.051 -y -35.023 -z 1.0 -R 0 -P 0 -Y -0.0
  ```

### 2. Spawn People in Lift1

<details>
<summary>Spawn People (Copy & Paste)</summary>

```bash
ros2 run gazebo_ros spawn_entity.py \
  -entity person_1 \
  -file /home/test_ws/src/.gazebo/models/person_standing/model.sdf \
  -x 20.5 \
  -y -19.25 \
  -z 0 \
  -R 0.0 \
  -P 0.0 \
  -Y 0

ros2 run gazebo_ros spawn_entity.py \
  -entity person_2 \
  -file /home/test_ws/src/.gazebo/models/person_standing/model.sdf \
  -x 19.75 \
  -y -19.25 \
  -z 0 \
  -R 0.0 \
  -P 0.0 \
  -Y 0

ros2 run gazebo_ros spawn_entity.py \
  -entity person_3 \
  -file /home/test_ws/src/.gazebo/models/person_standing/model.sdf \
  -x 19.0 \
  -y -19.25 \
  -z 0 \
  -R 0.0 \
  -P 0.0 \
  -Y 0

ros2 run gazebo_ros spawn_entity.py \
  -entity person_4 \
  -file /home/test_ws/src/.gazebo/models/person_standing/model.sdf \
  -x 19.5 \
  -y -18.5 \
  -z 0 \
  -R 0.0 \
  -P 0.0 \
  -Y 0

ros2 run gazebo_ros spawn_entity.py \
  -entity person_5 \
  -file /home/test_ws/src/.gazebo/models/person_standing/model.sdf \
  -x 20.25 \
  -y -18.5 \
  -z 0 \
  -R 0.0 \
  -P 0.0 \
  -Y 0
```
</details>

<details>
<summary>Delete People (Copy & Paste)</summary>

```bash
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'person_1'}"
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'person_2'}"
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'person_3'}"
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'person_4'}"
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'person_5'}"
```
</details>

### 3. Launch Navigation System

**Full BT (All Floors)**
```bash
ros2 launch sparo_navigation_bringup multi_bt_runners.launch.py
```

**Single Floor Test**
```bash
ros2 launch sparo_navigation_bringup test_navigation.launch.py
# 실행 후 터미널에서 층 선택 (1/2/3)
```

### 4. Launch YOLO Detection
```bash
ros2 launch yolo_bringup yolo.launch.py \
  use_3d:=True \
  input_image_topic:=/camera/image_raw \
  input_depth_topic:=/camera/depth/image_raw \
  depth_image_units_divisor:=1
```

### 5. Trigger Intruder Alert
```bash
ros2 topic pub --times 5 /intruder_alert geometry_msgs/PoseStamped "{
  header: {frame_id: 'L3'},
  pose: {position: {x: 14.25, y: -31.5, z: 10.0}}
}"
```
### 6. Launch Planner System

**Planner Mode (Text Command Navigation)**
```bash
ros2 launch sparo_navigation_bringup planner.launch.py
# 실행 후 터미널에서 시작 층 선택 (1/2/3)
```

**Planner Commands:**

*Navigation Commands* (키워드: 가, 가줘, 이동, go):
```bash
ros2 topic pub --once /planner/command std_msgs/String "data: '3층 복도 가줘'"
ros2 topic pub --once /planner/command std_msgs/String "data: '2층 엘리베이터 가'"
ros2 topic pub --once /planner/command std_msgs/String "data: '1층 계단 이동'"
ros2 topic pub --once /planner/command std_msgs/String "data: '3층 로비 가'"
```

*Patrol Commands* (키워드: 순찰, patrol, 돌아):
```bash
ros2 topic pub --once /planner/command std_msgs/String "data: '1층 순찰해'"
ros2 topic pub --once /planner/command std_msgs/String "data: '3층 순찰'"
```
---
