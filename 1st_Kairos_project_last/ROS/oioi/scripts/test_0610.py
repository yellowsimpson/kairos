#!/usr/bin/env python3
import rospy, roslib, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler
from modbus_frame import create_modbus_frame, setup_serial
import time
from std_msgs.msg import Float32MultiArray

class MoveItPlanningDemo:
    def __init__(self):
        # Initialize moveit_commander and rospy
        #moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_demo")

        # Load URDF and SRDF parameters
        # self.load_robot_description()

        # Initialize the MoveGroupCommander for arm and gripper
        #self.arm = moveit_commander.MoveGroupCommander("arm_group",ns="/cobot")
        #self.gripper = moveit_commander.MoveGroupCommander("gripper_group",ns="/cobot")

        #self.end_effector_link = self.arm.get_end_effector_link()
        #self.reference_frame = "base"
        #self.arm.set_pose_reference_frame(self.reference_frame)
        #self.arm.allow_replanning(True)
        #self.arm.set_goal_position_tolerance(0.01)
        #self.arm.set_goal_orientation_tolerance(0.05)
        #self.arm.home = self.arm.get_current_pose(end_effector_link='link6').pose

        # Initialize pose and PLC serial
        self.pose_x = None
        self.pose_y = None
        self.pose_z = None
        #self.ser_plc = setup_serial()
        
        # Initialize AGV position and orientation variables
        self.agv_x = 0.0
        self.agv_y = 0.0
        self.agv_theta = 0

        self.pub_init()

    def load_robot_description(self):
        # Load URDF
        urdf_path = rospy.get_param('~urdf_path', '/home/jsy/jsy_ws/src/mycobot_ros/mycobot_description/urdf/mycobot_320_gripper.urdf')
        with open(urdf_path, 'r') as urdf_file:
            urdf_string = urdf_file.read()
        rospy.set_param('/robot_description', urdf_string)

        # Load SRDF
        srdf_path = rospy.get_param('~srdf_path', '/home/jsy/jsy_ws/src/mycobot_ros/mycobot_320/mycobot_320_gripper_moveit/config/firefighter.srdf')
        with open(srdf_path, 'r') as srdf_file:
            srdf_string = srdf_file.read()
        rospy.set_param('/robot_description_semantic', srdf_string)

    def moving(self, pose_x, pose_y, pose_z):
        # Set the target position for Cartesian motion
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = pose_x
        target_pose.pose.position.y = pose_y
        target_pose.pose.position.z = pose_z
        q = quaternion_from_euler(0.0, 0.0, 90.0)
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        # Set the target position for Cartesian motion
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        print(self.arm.get_current_pose(end_effector_link='link6').pose)
        plan = self.arm.plan()
        self.arm.execute(plan[1])  # Modified line
        rospy.sleep(1)
        
    def start(self):
        self.moving(-0.134, -0.185, 0.338)
        rospy.sleep(1)
        
    def start_2(self):
        self.moving(-0.134, -0.2, 0.398)
        rospy.sleep(1)
        
    #def plc_motor_ctrl(self):
    #    # 펑션코드 03 정지, 05 역방향, 06 정방향
    #    frame = create_modbus_frame([0x3A, '01', '03', "00010008", 0x0D, 0x0A], 'lrc')
    #    self.ser_plc.write(frame)
    #    time.sleep(3)
        
    def pub_init(self):
        # Publisher for AGV position and orientation
        self.pub_agv = rospy.Publisher('agv_position_orientation', Float32MultiArray, queue_size=1)
        
    def pub_coordinates(self):
        # Create and publish the AGV position and orientation message
        agv_msg = Float32MultiArray()
        agv_msg.data = [self.agv_x, self.agv_y, self.agv_theta]
        self.pub_agv.publish(agv_msg)
        rospy.loginfo(f"AGV position and orientation: x={self.agv_x}, y={self.agv_y}, theta={self.agv_theta}")


if __name__ == "__main__":
    o = MoveItPlanningDemo()
    rospy.loginfo("agv 3")
    o.agv_x = 0.0
    o.agv_y = 0.0
    o.agv_theta = 3.14
    o.pub_coordinates()
    rospy.sleep(1)

