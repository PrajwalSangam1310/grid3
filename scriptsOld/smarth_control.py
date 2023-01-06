#!/usr/bin/env python

import rospy
import roslib
from nav_msgs.msg import  Odometry
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion
from controller_latest_smart import SmartController

my_pose = Pose2D()
my_command = Twist()

# initialize trajectory only once
my_trajectory = [Pose2D() for i in range(4)]

my_trajectory[0] = Pose2D(1,1,0)
my_trajectory[1] = Pose2D(1,2,0)
my_trajectory[2] = Pose2D(1,3,0)
my_trajectory[3] = Pose2D(1,4,0)

trajecotry_index = 0
my_controller = SmartController()
my_controller.update_trajectory(my_trajectory)

def update_trajectory():
    pass

def get_robot_position(data):
    global my_pose
    my_pose.x = data.pose.pose.position.x
    my_pose.y = data.pose.pose.position.y
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    _,_,theta = euler_from_quaternion([x,y,z,w])
    my_pose.theta = theta
    # print(my_pose)

def update_smarth_command():
    global my_command
    global my_pose

    my_controller.update_robot_position(my_pose)
    my_controller.calculate_command()
    command = my_controller.get_command()
    my_command.angular.z = command[0]
    my_command.linear.x = command[1]

    # We will update my_command.linear.x and my_command.angular.z
    # pass bot position and angle



if __name__=="__main__":
    rospy.init_node("smarth_controller", anonymous=True)
    pub = rospy.Publisher("/robot1/cmd_vel", Twist,queue_size=10)
    rospy.Subscriber("/robot1/odom", Odometry,get_robot_position)
    r = rospy.Rate(25)
    while not rospy.is_shutdown():
        update_smarth_command()
        pub.publish(my_command)
        r.sleep()
    print("not exe")
    pub.publish(Twist())
