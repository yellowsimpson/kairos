#!/usr/bin/env python3
import rospy, roslib, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler


class MoveItPlanningDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_demo")
        self.arm = moveit_commander.MoveGroupCommander("cobot/arm_group")
        self.gripper = moveit_commander.MoveGroupCommander("cobot/gripper_group")
        self.end_effector_link = self.arm.get_end_effector_link()
        self.reference_frame = "base"
        self.arm.set_pose_reference_frame(self.reference_frame)
        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)
        self.arm.home = self.arm.get_current_pose(end_effector_link='link6').pose
        self.pose_x = None
        self.pose_y = None
        self.pose_z = None

    def print_current_pose(self):
        print(self.arm.get_current_pose(end_effector_link='link6').pose)

    def run(self):
        self.print_current_pose()
        rospy.sleep(1)

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    o = MoveItPlanningDemo()
    o.run()
