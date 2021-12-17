# Automated_synthesis

This repository is contains code and documentation for operating the automatic synthesis robot cell. This will contain both code for a real-life demonstration as well as a simulated environment in which the robot can operate.

## Use-case of the project

- Computerchemist discovers a possible synthesis that might result in a useful product.
- Computerchemist places the correct ingredients in the test tubes and gives the robot a certain recipe/set of instructions
- The robot moves follows the set of instructions. Currently the robot has this set of actions:
  - Put/get gripper
  - Put/get pipette
  - Fill/empty pipette
  - Move to small or large test tube
  - Move large test tube to/from heater

![image](https://user-images.githubusercontent.com/11269762/146534078-1a6d2434-7305-4252-a323-801507d97499.png)


## Communication Structure

The communication between the robot, computer and pipette is mainly done using MQTT and Robot Operating System(ROS). MQTT is a lightweight publish-subscribe network protocol that can transport messeges between devices. MQTT is usually used in IoT and IIoT for communication between a server and set of sensors. Currently, the UR5 robot used in this project is controlled using ROS while the smart pipette is controlled using MQTT. Power for the pipette is provided by the UR5 through the use of a [Kelvin Tool Changer](https://www.toolchanger.eu/). This can also be seen in the Figure below:
![image](https://user-images.githubusercontent.com/11269762/146416383-d93de111-cdac-4694-a24d-26503c61c69f.png)

![image](https://user-images.githubusercontent.com/11269762/146535781-8aff26d8-60b2-4458-90d7-c38f14d3208d.png)


## 3D models

![image](https://user-images.githubusercontent.com/11269762/146416910-898239d5-24cd-459c-b911-7d79153b5bc9.png)


![image](https://user-images.githubusercontent.com/11269762/146416038-44828104-0290-422c-b48c-804bb8728a21.png)
