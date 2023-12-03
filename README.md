# ROS2_ControlTurtleBot_Aruco-Public
Detect and Movement

usb cam:
git clone --branch ros2 https://github.com/ros-drivers/usb_cam.git

turtlebot:
sudo apt install ros-humble-turtlebot3*


in your workspace:
colcon build
source intstall/setup.bash

run my node (camera usb is subscriber, turtlebot is publisher)
ros2 run camera_subscriber camera_node

run usb cam
ros2 run usb_cam usb_cam_node_exe

run turlebot
export TURTLEBOT3_MODEL=burger

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg \
prefix turtlebot3_gazebo \
`/share/turtlebot3_gazebo/models/
