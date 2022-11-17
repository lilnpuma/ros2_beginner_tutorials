# ROS2 Publisher/Subscriber
Contains a publisher and subscriber that publishes and listens to a unique message respectively.
## Author
- Name: Manu Madhu Pillai
- UID: 117817928

## Dependancies
- ROS 2 Humble
- rclcpp (ROS Client Lib for C++)
- stdmsgs (Stores ROS message types)


## Build instructions

Open terminal from source directory of ROS2 workspace or navigate to the path using following command:
```
cd ~/ros2_ws/src
```
Clone GitHub repository:
```
git clone --recursive https://github.com/lilnpuma/ros2_beginner_tutorials.git
```
Build the ROS2 package by going back to workspace's root directory:
```
cd ~/ros2_ws
```
Checking for any missing dependencies before building:
```
rosdep install -i --from-path src --rosdistro humble -y
```
Build package:
```
colcon build --packages-select beginner_tutorials
```

## Run instructions

Next step is to run the program and for that open a new terminal and navigate to ROS2 workspace:
```
cd ~/ros2_ws
```
Source the setup files:
```
. install/setup.bash
```
Now run the talker node:
```
ros2 run beginner_tutorials talker
```
Open another terminal, source the setup files from inside ros2_ws again, and then start the listener node:
```
cd ~/ros2_ws
. install/setup.bash
ros2 run beginner_tutorials listener
```