# ROS2_ControlTurtleBot_Aruco-Public
1. Download packages

usb cam:
```bash
git clone --branch ros2 https://github.com/ros-drivers/usb_cam.git
```

turtlebot:
```bash
sudo apt install ros-humble-turtlebot3*
```
2. Run
   
in your workspace:
```bash
colcon build
```
```bash
source intstall/setup.bash
```

run my node (camera usb is subscriber, turtlebot is publisher)
```bash
ros2 run camera_subscriber camera_node
```

run usb_cam:
```bash
ros2 run usb_cam usb_cam_node_exe
```

run turlebot:
```bash
export TURTLEBOT3_MODEL=burger
```

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:ros2 pkg \
prefix turtlebot3_gazebo \
/share/turtlebot3_gazebo/models/
```
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```
<img src="result.gif" alt="using a color picker" width="300" height="200">



