#!/usr/bin/python3
import sys
import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32MultiArray  # Import the necessary message types


def agv_callback(data):
    # Parse the AGV position and orientation
    agv_x = data.data[0]
    agv_y = data.data[1]
    agv_theta = data.data[2]
    
    print(f"Received AGV position and orientation: x={agv_x}, y={agv_y}, theta={agv_theta}")
    
    move_goal(agv_x, agv_y, agv_theta)
    print('----Moving to new position----')

def update_init_pose(x, y, theta):
    init_pose.header.stamp = rospy.Time.now()
    init_pose.pose.pose.position.x = x
    init_pose.pose.pose.position.y = y
    q = quaternion_from_euler(0.0, 0.0, theta)
    init_pose.pose.pose.orientation.x = q[0]
    init_pose.pose.pose.orientation.y = q[1]
    init_pose.pose.pose.orientation.z = q[2]
    init_pose.pose.pose.orientation.w = q[3]

def send_goal(x, y, theta):
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    q = quaternion_from_euler(0.0, 0.0, theta)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    client.send_goal(goal)

def move_goal(a, b, theta):
    send_goal(a, b, theta)
    wait = client.wait_for_result()
    if not wait:
        print('Error')
    else:
        print(client.get_result())

rospy.init_node('init_pose')

# Subscriber to listen for the AGV position and orientation
rospy.Subscriber('agv_position_orientation', Float32MultiArray, agv_callback)
rospy.loginfo("Waiting MyAGV")

pub_init = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)

init_pose = PoseWithCovarianceStamped()
init_pose.header.frame_id = "map"
init_pose.header.stamp = rospy.Time.now()
init_pose.pose.pose.position.x = 0.0
init_pose.pose.pose.position.y = 0.0
init_pose.pose.pose.orientation.w = 1.0
update_init_pose(0.0, 0.0, 1)
pub_init.publish(init_pose)

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"

# Wait for messages indefinitely
rospy.spin()
