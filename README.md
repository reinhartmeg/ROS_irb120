# RBE501 - Final Term Project - ABB IRB120 Simulation

The repo here is branching off Diego Martín’s repo irb120_robotiq85 for the initial setup for an ABB IRB120 6-axis industrial robot in Gazebo. The code uploaded here has been modified to remove all references to the Robotiq 85 2-finger gripper. The simulation won’t work if copied directly from the referenced repo. However, it’s recommended to follow a similar setup that’s outlined there that’s only slightly modified from the example.

The forward_kinematics_mirror.py was added to copy the movements of the robot in the lab. It's included inside irb120_robotiq85_gazebo to run as directed and attatched to main for easier access.


## Dependencies and Installation ##

### ABB IRB 120 industrial robot ###

The ABB IRB120 Gazebo simulation is taken from the ROS-Industrial [ABB experimental metapackage](http://wiki.ros.org/abb_experimental). Clone it to your catkin_ws using the kinetic-devel branch:

```
cd catkin_ws/src
git clone https://github.com/ros-industrial/abb_experimental.git
```

The previous ABB experimental metapackage may depend on the main ROS-Industrial [ABB stack](http://wiki.ros.org/abb). Please clone its kinetic-devel branch:

```
git clone https://github.com/ros-industrial/abb.git
```

### Install this package ###

Once you have installed the previous packages, clone this one, build your catkin_ws and source the setup.bash: 

```
git clone https://github.com/reinhartmeg/ROS_irb120.git
cd ..
source devel/setup.bash
```

## Usage ##

### Gazebo simulation of the IRB120 + Moveit config ###

Launch the Gazebo simulation + MoveIt Commander:

```
roslaunch irb120_robotiq85_gazebo irb120_robotiq85_gazebo_moveit.launch
```

## Python Programs ##

First, launch the Gazebo simulation + MoveIt Commander. Then, there are four options: 

1. Forward kinematics plan and execution:
```
rosrun irb120_robotiq85_gazebo forward_kinematics.py
```

2. Inverse kinematics plan and execution (move joints):
```
rosrun irb120_robotiq85_gazebo IK_destination_pose.py
```

3. Cartesian path (IK) plan and execution (move linear):
```
rosrun irb120_robotiq85_gazebo IK_cartesian_path.py
```

4. Mirrored forward kinematics plan and execution:
```
rosrun irb120_robotiq85_gazebo forward_kinematics_mirror.py
```

Note: Modified to Python3 in this version. Ensure setup is compatible. Inside .bashrc file, recommend to include three lines: alias python="python3", alias pip="pip3", & export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages. Also ensure the permissions are set to executable (chmod +x) when running the python scripts.
