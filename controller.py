#!/usr/bin/env python2
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot
from std_msgs.msg import String
import math

# Use to transform between frames
tf_buffer = None
listener = None

# The exploration simple action client
goal_client = None
# The collision avoidance service client
control_client = None
# The velocity command publisher
pub = None

# The robots frame
robot_frame_id = "base_link"

# Max linear velocity (m/s)
max_linear_velocity = 0.5
# Max angular velocity (rad/s)
max_angular_velocity = 1.0


def move(path):
    global control_client, robot_frame_id, pub, tf_buffer

    rate = rospy.Rate(10.0)
    # Call service client with path

    while path.poses:
        # Transform Setpoint from service client
        res = control_client(path)
        setpoint = res.setpoint
        new_path = res.new_path
        # Create Twist message from the transformed Setpoint

        transform = tf_buffer.lookup_transform(robot_frame_id,setpoint.header.frame_id,rospy.Time(0))
        transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform)

        # Publish Twist -1 <= y <= 1
        mess_twist = Twist()
        angle_v = atan2(transformed_setpoint.point.y, transformed_setpoint.point.x)
        mess_twist.angular.z =  max(min(angle_v,max_angular_velocity), -max_angular_velocity)

        if -0.98 > angle_v or angle_v > 0.98 :
            mess_twist.linear.x = 0 
        else :
            mess_twist.linear.x = min(transformed_setpoint.point.x,max_linear_velocity)
        

        # Call service client again if the returned path is not empty and do stuff again
        pub.publish(mess_twist)
        
        path = new_path

        rate.sleep()

    # Send 0 control Twist to stop robot
    mess_twist.angular.z = 0
    mess_twist.linear.x = 0
    pub.publish(mess_twist)
    # Get new path from action server


def get_path():
    global goal_client

    while True:
        # Get path from action server
        goal_client.wait_for_server()
        goal = irob_assignment_1.msg.GetNextGoalAction()
        # Call move with path from action server
        goal_client.send_goal(goal)
        goal_client.wait_for_result()

        road = goal_client.get_result().path
        reward = goal_client.get_result().gain

        if not road.poses :
            break 

        move(goal_client.get_result().path)




if __name__ == "__main__":
    # Init node
    rospy.init_node('controller')

    # Init publisher
    pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)

    # Init simple action client
    goal_client = actionlib.SimpleActionClient('get_next_goal',irob_assignment_1.msg.GetNextGoalAction)

    # Init service client
    control_client = rospy.ServiceProxy('get_setpoint',GetSetpoint)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Call get path
    get_path()

    # Spin
    rospy.spin()
