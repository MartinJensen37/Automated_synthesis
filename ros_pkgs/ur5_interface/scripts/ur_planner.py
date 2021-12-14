from __future__ import print_function
from six.moves import input
import sys
import math
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from biox_read.msg import IMU

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

        self.display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.rqt_trajectory_publisher = rospy.Publisher("/pos_joint_traj_controller/command", JointTrajectory, queue_size=1)

        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()

        self.group_names = self.robot.get_group_names()
        
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")
        
    def imu_read_cb(self, pose):

        quat = pose.Quaternion

        _x = quat.x
        _y = quat.y
        _z = quat.z
        _w = quat.w
        sqw = _w * _w
        sqx = _x * _x
        sqy = _y * _y
        sqz = _z * _z

        x = math.atan2(2.0 * (_x * _y + _z * _w), (sqx - sqy - sqz + sqw))
        y = math.asin(-2.0 * (_x * _z - _y * _w) / (sqx + sqy + sqz + sqw))
        z = math.atan2(2.0 * (_y * _z + _x * _w), (-sqx - sqy + sqz + sqw))
        print('This is the quat: ', quat, '\nThis is the rads: ', x, y, z)
        goal = [x, y, z, 0.0, 0.0, 0.0]
        self.rqt_ctrl_move(goal)

    def read(self):
        pass
        # IMU subscriber
        #self.imu_sub = rospy.Subscriber("/IMU", IMU, self.imu_read_cb, queue_size=1)
        #rospy.spin()

    def rqt_ctrl_move(self, goal):
        rqt_msg = JointTrajectory()
        point = JointTrajectoryPoint()
        print(self.move_group.get_active_joints())
        rqt_msg.joint_names = self.move_group.get_active_joints()
        point.positions = goal # example vector: [-1.82212373908208, -3.3876104167282426, 5.283185307179588, -2.566350443666414, -2.566382621182798, 12.566359000316584]
        point.time_from_start.secs = 1
        rqt_msg.points.append(point)
        self.rqt_trajectory_publisher.publish(rqt_msg)

    def moveit_joint(self, goal):
        pass

    def moveit_cartesian(self, goal):
        pass

    def moveit_tcp(self):

        move_group = self.move_group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.3
        pose_goal.position.z = 0.2

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)

        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()

        # It is always good to clear your targets after planning with poses.
        move_group.clear_pose_targets()

        # Check if the plan was executed
        if plan is True:
            return "Succesfully found and executed plan!"

        elif plan is False:
            return "Failed to get a plan! ABORTING."


def main():
    try:
        ur_planner = URplanner()

        command = input("===============MENU============= \n a: tcp control \n s: rqt control \n d: joint control \n")
        if command == "a":
            status = ur_planner.moveit_tcp()
            print(status)
        elif command == "s":
            #status = ur_planner.rqt_ctrl_move()
            ur_planner.read()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()