# RL_HW3

## :hammer: Build

Clone this package in the `src` folder of your ROS 2 workspace. Check for missing dependencies
```
$ git clone https://github.com/MatteoRusso5/RL_HW3.git
```
```
$ rosdep install -i --from-path src --rosdistro humble -y
```
Build your new package

```
$ colcon build
```
Source the setup files

```
$ source install/setup.bash
```
## :white_check_mark: Usage 🤖
### 1. Run Rviz and Gazebo with the launch file by using the velocity interface

To run the world with aruco tag and velocity interface
⚙️
```
$ ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

To run the world with the sphere and velocity interface
```
$ ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_aruco:=false
```

## To execute tasks: positioning or look-at-point
### 1. Task positioning
In the first terminal
```
$ ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```
In the second terminal
```
$ ros2 run aruco_ros single --ros-args -r /image:=/videocamera -r /camera_info:=/camera_info -p marker_id:=201 -p marker_size:=0.1 -p reference_frame:=world -p marker_frame:=aruco_marker_frame -p camera_frame:=camera_link_optical
```
To run rqt_image_view in another terminal
```
$ ros2 run rqt_image_view rqt_image_view
```
To run the ros2_kdl_vision_control node in another terminal
```
$ ros2 run ros2_kdl_package ros2_kdl_vision_control
```
### 2. Task look-at-point
In the first terminal
```
$ ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```
In the second terminal
```
$ ros2 run aruco_ros single --ros-args -r /image:=/videocamera -r /camera_info:=/camera_info -p marker_id:=201 -p marker_size:=0.1 -p reference_frame:=camera_link_optical -p marker_frame:=aruco_marker_frame -p camera_frame:=camera_link_optical
```
To run rqt_image_view in another terminal
```
$ ros2 run rqt_image_view rqt_image_view
```
To run the ros2_kdl_vision_control node in another terminal
```
$ ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p task:="look-at-point"
```
## Point 2B
In the first terminal
```
$ ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller"
```
In the second terminal
```
$ ros2 run aruco_ros single --ros-args -r /image:=/videocamera -r /camera_info:=/camera_info -p marker_id:=201 -p marker_size:=0.1 -p reference_frame:=camera_link_optical -p marker_frame:=aruco_marker_frame -p camera_frame:=camera_link_optical
```
To run rqt_image_view in another terminal
```
$ ros2 run rqt_image_view rqt_image_view
```
To run ros2_kdl_vision_control node in another terminal
```
$ ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p task:="look-at-point" -p cmd_interface:=effort
```

