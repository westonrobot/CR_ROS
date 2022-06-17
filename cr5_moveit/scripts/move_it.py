#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function
from enum import auto
import time
from six.moves import input
from dobot_bringup.srv import OpenGripper, OpenGripperRequest, OpenGripperResponse

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

GRIPPER_OPEN = 1000
GRIPPER_CLOSE = 0

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "cr5_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        move_group.set_num_planning_attempts(10)

        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self,joint1, joint2, joint3, joint4, joint5, joint6):

        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = joint1
        joint_goal[1] = joint2
        joint_goal[2] = joint3
        joint_goal[3] = joint4
        joint_goal[4] = joint5
        joint_goal[5] = joint6
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

    def go_to_pose_goal(self,q_x,q_y,q_z,q_w,x,y,z):

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = q_x
        pose_goal.orientation.y = q_y
        pose_goal.orientation.z = q_z
        pose_goal.orientation.w = q_w
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        # print(self.move_group.get_current_pose(self.eef_link))
        self.move_group.set_pose_target(pose_goal,self.eef_link)

        ## Now, we call the planner to compute the plan and execute it.
        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface for Pick and Place")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")

        tutorial = MoveGroupPythonInterfaceTutorial()
        
        rospy.wait_for_service('/dobot_bringup/srv/OpenGripper')
        gripper=rospy.ServiceProxy('/dobot_bringup/srv/OpenGripper', OpenGripper)
        action = 0
        auto = False
        full_seq = [0,1,2,3,1,4,5,6,0]
        while True:
            if not auto:
                action=int(input("Input Action\nGoto Home: 0\nGoto pre-pickup: 1\nGoto pickup:2\nPickup:3\nGoto pre-dropoff:4\nGoto dropoff:5\nDropoff:6\n"))
                if action == 107:
                    auto = True
            else:
                action = full_seq[0]
                full_seq.pop(0)

            if action == 0:
                print("Going to home position")
                tutorial.go_to_joint_state(0,0,0,0,0,0)
                time.sleep(1)
            elif action == 1:
                print("Going to pre-pick up location")
                tutorial.go_to_joint_state(1.535576324783635, 0.6528407592387284, 1.8244286517350827, -0.9059237351286105, -1.5703469509015602, -0.03609090798175925)
                time.sleep(1)
            elif action == 2:
                print("Going to pick up location")
                tutorial.go_to_joint_state(1.5358207555052579, 1.2316446394547869, 1.6934242266459474, -1.3537928411456268, -1.5704520798429962, -0.035934519782196646)
                time.sleep(1)
            elif action == 3:
                print("Picking up")
                request = OpenGripperRequest(force=20,position=GRIPPER_CLOSE)
                response = OpenGripperResponse(res=-4004)
                while response.res != 0: 
                    response = gripper.call(request)
                    print(response.res)
                    time.sleep(1)
            elif action == 4:
                print("Going to pre-dropoff")
                tutorial.go_to_joint_state(-1.8806789241548443, 0.0014316995568454856, 1.0375469496416603, 0.5324535266447842, -1.5708003342934636, -0.310213436218866)
                time.sleep(1)
            elif action == 5:
                print("Going to dropoff")
                tutorial.go_to_joint_state(-1.8808554838217253, -0.11025035833231106, 1.5802745165258427, 0.10133884138242513, -1.5710548507487456, -0.3101776022004534)
                time.sleep(1)
            elif action ==6:
                print("Dropping off")
                request = OpenGripperRequest(force=20,position=GRIPPER_OPEN)
                response = OpenGripperResponse(res=-4004)
                while response.res != 0: 
                    response = gripper.call(request)
                    print(response.res)
                    time.sleep(1)
            elif action ==9:
                return

        print("============ Pick and place demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/noetic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL