# spot_arm
Tele-operation of Spot Robot arm using Hololens 2

## Prerequisities

* `spot_ros` at [https://github.com/willat343/spot_ros](https://github.com/willat343/spot_ros).

Make sure that the estop gentle command is in the terminal, ready to be called when the robot starts working improperly:

```bash
rosservice call /spot/estop/gentle
```

## Build

Clone this repository to your `catkin_ws`:
```bash
cd ~/catkin_ws/src
git clone git@github.com:MixedRealityETHZ/spot_arm.git
cd ~/catkin_ws
catkin build
```

## spot_arm_interface

This is a ROS package for controlling the Spot arm from a stream of `geometry_msgs/PoseStamped` messages, such as from a Hololens 2 or published by the user.

### Usage

First the `spot_ros` driver must be running on the robot, replacing PASSWORD with the spot password. Source the correct workspace and run the roslaunch.

```bash
source ws/devel/setup.bash
roslaunch spot_driver driver.launch hostname:=192.168.50.3 username:=admin password:=PASSWORD max_linear_velocity_x:=1.0 max_linear_velocity_y:=1.0 max_angular_velocity_z:=1.0
```

Also launch the tcp endpoint with default args:
```bash
source ws/devel/setup.bash
roslaunch ros_tcp_endpoint endpoint.launch
```

To configure your local machine to see Spot as the ROS master. Run `hostname -I` to get your ip address (e.g. `<local_ip_address> = 192.168.1.28`):
```bash
export ROS_MASTER_URI=http://192.168.1.243:11311
export ROS_IP=<local_ip_address>
```
To launch the camera compressed stream, run the following commands:
```bash
source ws/devel/setup.bash
roslaunch spot_arm_interface spot_arm_camera_republish.launch
```


To launch the arm interface, optionally turning `rviz` on:
```bash
source ws/devel/setup.bash
roslaunch spot_arm_interface spot_arm_interface.launch rviz:=false
```

To launch the Unity to ROS pose transformation node:
```bash
source ws/devel/setup.bash
rosrun unity_transform unity_to_ros_pose.py
```

### Troubleshooting

#### Spot body doesn't move

* Make sure you are launching with the max linear and max angular velocity variables set to non-zero values, such as 1.0.
* Check that `move_spot_body_enabled` is set to `true` in `default.yaml`
* Try `rosrun spot_move_interface spot_move_client` and publish a pose to `/spot_move_client/pose`.
* If any of the cameras are not working (error messages in driver terminal), then this may stop spot movement

## External Software

### Spot-Arm

#### Prerequisites

```bash
sudo apt install ros-noetic-moveit
```

#### Build

```bash
cd ~/catkin_ws/src
git clone git@github.com:estherRay/Spot-Arm.git
cd ~/catkin_ws
catkin build
```

#### Usage

We are still working out the usage for this package. To launch the demo with gazebo + rviz:
```bash
roslaunch spot_moveit_config demo_gazebo.launch
```

To launch with without gazebo:
```bash
roslaunch spot_moveit_config demo.launch
```

**Known Issue**: in the Gazebo version, executation fails with an error like:
```
[ERROR] [1666736964.727028347, 25.118000000]: Unable to identify any set of controllers that can actuate the specified joints: [ Rev10 Rev11 Rev3 Rev6 Rev8 Rev9 ]
[ERROR] [1666736964.727042432, 25.118000000]: Known controllers and their joints:

[ INFO] [1666736964.738207870, 25.130000000]: ABORTED: Solution found but controller failed during execution
```

### spot_simulation

#### Prerequisites



#### Install

[This repository](https://github.com/SoftServeSAG/spot_simulation/tree/spot_control) contains a Spot gazebo simulation. Before using it, make sure that you have necessary packages installed:
```bash
sudo apt install ros-$(rosversion -d)-effort-controllers \
                 ros-$(rosversion -d)-joint-state-controller \
                 ros-$(rosversion -d)-joint-state-publisher-gui \
                 ros-$(rosversion -d)-interactive-marker-twist-server \
                 ros-$(rosversion -d)-robot-state-publisher \
                 ros-$(rosversion -d)-teleop-legged-robots
```

Clone the repository to your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone git@github.com:SoftServeSAG/spot_simulation.git
cd spot_simulation
git checkout spot_control
cd ~/catkin_ws
catkin build
```

If you work with Noetic ROS distro, you need to patch the `spot_simulation/rs_description/launch/description.launch` file by replacing `type="state_publisher"` with `type="robot_state_publisher"`. 

#### Usage

To launch the simulation environment:
```bash
roslaunch rs_gazebo HQ.launch
```

Once HQ is ready, spawn robot with:
```bash
roslaunch rs_gazebo robot.launch
```

For various control operations, consult the [Control README Section](https://github.com/SoftServeSAG/spot_simulation/tree/spot_control#control). For example:
```bash
roslaunch rs_control talker.launch 
```

### WSL2 Setup

If you work on Windows with WSL2, make sure to follow [this tutorial](https://github.com/microsoft/mixed-reality-robot-interaction-demo/wiki/BuildingDemoFromSource#for-windows-installing-xserver) to have GUI apps (e.g. Gazebo, RVIZ) running. Also see [this video](https://youtu.be/DW7l9LHdK5c).

Synopsis:
1. Install X-Launch from https://github.com/microsoft/mixed-reality-robot-interaction-demo/wiki/BuildingDemoFromSource#for-windows-installing-xserver
2. Run X-Launch.
3. Run the following lines in WSL2:
```bash
export GAZEBO_IP=127.0.0.1
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0 
export LIBGL_ALWAYS_INDIRECT=0
```
