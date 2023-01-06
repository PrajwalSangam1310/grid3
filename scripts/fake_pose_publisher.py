#!/usr/bin/env python3

import rospy
from nav_msgs.msg import  Odometry
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion
from grid3.msg import botPoses
import numpy as np
from visualization_msgs.msg import Marker
#region Initialization
robot1_pose = Pose2D()
robot2_pose = Pose2D()
robot3_pose = Pose2D()
robot4_pose = Pose2D()
#endregion

#region subuscriber callbacks
def get_robot1_position(data):
    global robot1_pose
    robot1_pose.x = data.pose.pose.position.x
    robot1_pose.y = data.pose.pose.position.y
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    _,_,theta = euler_from_quaternion([x,y,z,w])
    robot1_pose.theta = theta
    # print(my_pose)

def get_robot2_position(data):
    global robot2_pose
    robot2_pose.x = data.pose.pose.position.x
    robot2_pose.y = data.pose.pose.position.y
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    _,_,theta = euler_from_quaternion([x,y,z,w])
    robot2_pose.theta = theta
    # print(my_pose)

def get_robot3_position(data):
    global robot3_pose
    robot3_pose.x = data.pose.pose.position.x
    robot3_pose.y = data.pose.pose.position.y
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    _,_,theta = euler_from_quaternion([x,y,z,w])
    robot3_pose.theta = theta
    # print(my_pose)

def get_robot4_position(data):
    global robot4_pose
    robot4_pose.x = data.pose.pose.position.x
    robot4_pose.y = data.pose.pose.position.y
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    _,_,theta = euler_from_quaternion([x,y,z,w])
    robot4_pose.theta = theta
    # print(my_pose)
#endregion

def xy2ij(x,y):
    factor = 1600/2.4384
    # offset = 1.2192
    offset = 0
    i = int(round(factor*(x+offset)))
    j = int(round(factor*(y+offset))) 
    return (i,(1600-j))

def xy2rc(x,y):
    factor = 0.1524 #6 inches in meter
    offset = 0
    # offset = 1.2192

    r = int((x+offset)//factor + 1)
    c = int((y+offset)//factor + 1)
    return (r,c)

def publishRobotPoses():
    xy1 = xy2ij(robot1_pose.x,robot1_pose.y)
    xy2 = xy2ij(robot2_pose.x,robot2_pose.y)
    xy3 = xy2ij(robot3_pose.x,robot3_pose.y)
    xy4 = xy2ij(robot4_pose.x,robot4_pose.y)

    allRobotPoses.xs[0] = xy1[0]
    allRobotPoses.ys[0] = xy1[1]
    allRobotPoses.thetas[0] = robot1_pose.theta

    allRobotPoses.xs[1] = xy2[0]
    allRobotPoses.ys[1] = xy2[1]
    allRobotPoses.thetas[1] = robot2_pose.theta

    allRobotPoses.xs[2] = xy3[0]
    allRobotPoses.ys[2] = xy3[1]
    allRobotPoses.thetas[2] = robot3_pose.theta

    allRobotPoses.xs[3] = xy4[0]
    allRobotPoses.ys[3] = xy4[1]
    allRobotPoses.thetas[3] = robot4_pose.theta

    allRobotPosesPub.publish(allRobotPoses)

def publishRobotPosesRC():
    xy1 = xy2rc(robot1_pose.x,robot1_pose.y)
    xy2 = xy2rc(robot2_pose.x,robot2_pose.y)
    xy3 = xy2rc(robot3_pose.x,robot3_pose.y)
    xy4 = xy2rc(robot4_pose.x,robot4_pose.y)

    allRobotPosesRC.xs[0] = xy1[0]
    allRobotPosesRC.ys[0] = xy1[1]
    allRobotPosesRC.thetas[0] = robot1_pose.theta

    allRobotPosesRC.xs[1] = xy2[0]
    allRobotPosesRC.ys[1] = xy2[1]
    allRobotPosesRC.thetas[1] = robot2_pose.theta

    allRobotPosesRC.xs[2] = xy3[0]
    allRobotPosesRC.ys[2] = xy3[1]
    allRobotPosesRC.thetas[2] = robot3_pose.theta

    allRobotPosesRC.xs[3] = xy4[0]
    allRobotPosesRC.ys[3] = xy4[1]
    allRobotPosesRC.thetas[3] = robot4_pose.theta

    allRobotPosesPubRC.publish(allRobotPosesRC)

#region Main Call
if __name__=="__main__":
    rospy.init_node("smarth_controller", anonymous=True)
    robot1_pub = rospy.Publisher("/robot1/cmd_vel", Twist,queue_size=10)
    robot2_pub = rospy.Publisher("/robot2/cmd_vel", Twist,queue_size=10)
    robot3_pub = rospy.Publisher("/robot3/cmd_vel", Twist,queue_size=10)
    robot4_pub = rospy.Publisher("/robot4/cmd_vel", Twist,queue_size=10)
    
    allRobotPosesPub = rospy.Publisher("grid3/allRobots/poses", botPoses, queue_size=10)
    allRobotPosesPubRC = rospy.Publisher("grid3/allRobots/posesRC", botPoses, queue_size=10)

    rospy.Subscriber("/robot1/odom", Odometry,get_robot1_position)
    rospy.Subscriber("/robot2/odom", Odometry,get_robot2_position)
    rospy.Subscriber("/robot3/odom", Odometry,get_robot3_position)
    rospy.Subscriber("/robot4/odom", Odometry,get_robot4_position)

    allRobotPoses = botPoses()
    allRobotPosesRC = botPoses()

    for i in range(4):
        allRobotPoses.ids = [i for i in range(4)]
        allRobotPoses.xs = [0 for i in range(4)]
        allRobotPoses.ys = [0 for i in range(4)]
        allRobotPoses.thetas = [0.0 for i in range(4)]

        allRobotPosesRC.ids = [i for i in range(4)]
        allRobotPosesRC.xs = [0 for i in range(4)]
        allRobotPosesRC.ys = [0 for i in range(4)]
        allRobotPosesRC.thetas = [0.0 for i in range(4)]

    m = Marker()
    m.type = Marker.LINE_STRIP
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        publishRobotPoses()
        publishRobotPosesRC()

        rospy.loginfo("publishing the poses")
        r.sleep()