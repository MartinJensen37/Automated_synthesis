# UR5 Interface package
This readme describes programs that are contained within this ROS package as well as the dependencies that are needed to run the code.

### Dependencies

- Ubuntu 20.04 LTS and ROS Noetic
- Python 3.5+
- Universal robots driver that can be found in this [link](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) The instructions to install these packages can be seen in README.md in the link.
- Eclipse Moquitto Broker 3.1.1(NOT 5.0)
- Node-RED 2.1.0+ or Node-RED 1.6.0+ (for use with the UI see [Node-RED directory](https://github.com/MartinJensen37/Automated_synthesis/tree/main/node-red))

## UR Planner
This package is used to move TCP of the UR5 robot arm using [moveit!](https://github.com/ros-planning/moveit). The script for this is storing all the poses of the robot in a ```.csv``` file. The ```.csv``` file can then later be accessed using the ```ur_planner.py``` script in order to move the robot to a specified pose. This both works with the real robot and a Gazebo simulation of the robot. 

To run this code run the following set of  commands:

Enter the workspace.
    
    cd ~/path-to-workspace
    
Build the workspace.
    
    catkin build && source devel/setup.bash

The script can be run using ```rosrun``` for running the specific package.

    rosrun ur5_interface ur_planner.py

The package can also be run in conjunction with [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) using the ```roslaunch```.

    roslaunch ur5_interface ur_interface.launch
With either ```irl``` and ```sim``` as arguments depending on if the package has to be run in real-life or in simulation, respectively.

### Future improvements

Other features that has yet to be implemented are the following:

- Linear interpolation between to TCP poses.
- Adding via points to further constrain the trajectories of the robot to achieve a more desired path
- Better definition of the workspace such that the planner generates collision-free trajectories.



## UR ServiceController
This package is used for control of high level operations using a UR5 robot and the 