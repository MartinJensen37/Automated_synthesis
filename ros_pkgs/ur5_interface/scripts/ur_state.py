from __future__ import print_function
#from six.moves import input

import sys
from csv import writer, reader
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory
#from ur_msgs.msg import SetIO
from ur_msgs.srv import *
from dataclasses import dataclass


# definitions for robot
@dataclass
class RobotJoints:
    SHOULDER_PAN_JOINT: int = 0
    SHOULDER_LIFT_JOINT: int = 1
    ELBOW_JOINT: int = 2
    WRIST_1_JOINT: int = 3
    WRIST_2_JOINT: int = 4
    WRIST_3_JOINT: int = 5



class URplanner():

    def __init__(self) -> None:
               
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ur_planner", anonymous=True)
        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        # kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()
        # Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the
        # surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface()

        # move_groups are defined in the srdf that can be found in the config of the selected ur robot. 
        # New groups can be defined as well as default positions.
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.move_group.set_planner_id("RRTstar")
        self.display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=20)
        #self.rqt_trajectory_publisher = rospy.Publisher("/pos_joint_traj_controller/command", JointTrajectory, queue_size=1)

        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()

        self.group_names = self.robot.get_group_names()
        
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")
        

    def moveit_joint(self, goal):
        pass

    def moveit_cartesian(self, goal):
        pass

    def moveit_tcp(self, desired_pose):

        move_group = self.move_group

        with open('/home/martin/Documents/repositories/bubble_ws/src/ur5_interface/scripts/robot_poses.csv', newline='') as pose_obj:
            pose_read = reader(pose_obj)
            data = list(pose_read)
        pose_dict = {item[0]: item[1:] for item in data}

        print(pose_dict[desired_pose])
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = float(pose_dict[desired_pose][0])
        pose_goal.position.y = float(pose_dict[desired_pose][1])
        pose_goal.position.z = float(pose_dict[desired_pose][2])
        pose_goal.orientation.x = float(pose_dict[desired_pose][3])
        pose_goal.orientation.y = float(pose_dict[desired_pose][4])
        pose_goal.orientation.z = float(pose_dict[desired_pose][5])
        pose_goal.orientation.w = float(pose_dict[desired_pose][6])
        

        print(pose_goal)
        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        #plan = move_group.plan()
        #display_trajectory = DisplayTrajectory()
        #display_trajectory.trajectory_start = self.robot.get_current_state()
        #display_trajectory.trajectory.append(plan)
        #self.display_trajectory_publisher.publish(display_trajectory)

        #move_group.execute(plan, wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()

        # It is always good to clear your targets after planning with poses.
        move_group.clear_pose_targets()

        # Check if the plan was executed
        if plan is True:
            return "Succesfully found and executed plan!"

        elif plan is False:
            return "Failed to get a plan! ABORTING."

    def ur_read(self, state):

        robot_pose = self.move_group.get_current_pose().pose
        print("This is the current pose: ", robot_pose)


        save = input("Would you like to save the pose please press 's' otherwise press any key to exit. \n")
        
        if save == 's':
            pose_name = input("Now please enter a name for the pose: ")
            status = self.save_pose(robot_pose, pose_name)
            return status
        else:
            return "Exited!"

    def rotate_joint():
        pass 


    def save_pose(self, pose, pose_name):

        pose_list = [pose_name, pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        with open('/home/martin/Documents/repositories/bubble_ws/src/ur5_interface/scripts/robot_poses.csv', 'a+', newline='') as write_obj:
            csv_writer = writer(write_obj)
            csv_writer.writerow(pose_list)
            
        return "Saved and exited!"

    def control_gripper(self, grip_state):
        set_io = rospy.ServiceProxy('ur_hardware_interface/set_io', SetIO)
        set_io(1, 16, grip_state)

        return "gripper_state changed"

def main():
    ur_planner = URplanner()

    command = input("===============MENU============= \n a: open/close gripper \n s: move tcp \n d: rotate joint \n")
    if command == "a":
        state = input("Please select a state for the gripper: \n 0: open gripper \n 1: close gripper \n input: ")
        status = ur_planner.ur_read(float(state))
        print(status)
    elif command == "s":
        desired_pose = input("Please write the name of the desired pose: ")
        status = ur_planner.moveit_tcp(desired_pose)


if __name__ == "__main__":
    main()