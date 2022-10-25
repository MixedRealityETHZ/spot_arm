# spot_arm
Tele-operation of Spot Robot arm using Hololens 2

If you work on Windows with WSL2, make sure to follow [this tutorial](https://github.com/microsoft/mixed-reality-robot-interaction-demo/wiki/BuildingDemoFromSource#for-windows-installing-xserver) to have GUI apps (e.g. Gazebo, RVIZ) running.

## Spot simulation

[This repository](https://github.com/SoftServeSAG/spot_simulation/tree/spot_control) contains a Spot gazebo simulation. Before using it, make sure that you have necessary packages installed:
```
sudo apt-get install    ros-$(rosversion -d)-effort-controllers \
                        ros-$(rosversion -d)-joint-state-controller \
                        ros-$(rosversion -d)-joint-state-publisher-gui \
                        ros-$(rosversion -d)-interactive-marker-twist-server \ 
                        ros-$(rosversion -d)-robot-state-publisher \
                        ros-$(rosversion -d)-teleop-legged-robots
```
Clone the repository to your catkin workspace. If you work with Noetic ROS distro, you need to change the *spot_simulation/rs_description/launch/description.launch* file - replace *type="state_publisher"* to *type="robot_state_publisher"*. 
You can test the correctness of the setup by running two commands in separate terminals (make sure the HQ is ready before spawning the robot):
```
roslaunch rs_gazebo HQ.launch
```

```
roslaunch rs_gazebo robot.launch 
```
