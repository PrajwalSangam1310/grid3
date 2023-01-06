#!/usr/bin/env python

import rospy
import roslib
from nav_msgs.msg import  Odometry
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion
from controller_latest_smart2 import SmartController
import numpy as np

#region Initialization
robot1_pose = Pose2D()
robot2_pose = Pose2D()
robot3_pose = Pose2D()
robot4_pose = Pose2D()

robot4_start_pose = Pose2D(-0.685,0.208,0)
robot3_start_pose = Pose2D(-0.685,0.0455,0)
robot2_start_pose = Pose2D(-0.685,-0.1,0)
robot1_start_pose = Pose2D(-0.685,-0.25,0)

robot1_command = Twist()
robot2_command = Twist()
robot3_command = Twist()
robot4_command = Twist()
#endregion

#region trajectory template
#     #Robot1
#         0,0 start 
#         0.2032,0 Lturn(8*6 inches)
#         0.2032,0.1778 end(7*6 inches)
#         0.2032,0 Lturn(8 inches)
#         0,0
#     #Robot2
#         0,0 start 
#         0.2286,0 Lturn (9 inches)
#         0.2032,0.2286 end (8 inches)
#         0.2032,0 Lturn (8 inches)
#         0,0
#     #Robot3
#         0,0 start 
#         0.2032,0 Lturn (8 inches)
#         0.2032,-0.1778 end (8 inches)
#         0.2032,0 Lturn (8 inches)
#         0,0
#     #Robot4
#         0,0 start 
#         0.2032,0 Lturn (8 inches)
#         0.2032,-0.2032 end (8 inches)
#         0.2032,0 Lturn (8 inches)
#         0,0
#endregion

#region Trajectory
#Trajectory1
robot1_trajectory = [Pose2D() for i in range(5)]

robot1_trajectory[0] = Pose2D(0,0,0) #start
robot1_trajectory[1] = Pose2D(0.2032*6,0,0) #Lturn
robot1_trajectory[2] = Pose2D(0.2032*6,-0.1778*6,0) #end
robot1_trajectory[3] = Pose2D(0.2032*6,0,0) #Lturn
robot1_trajectory[4] = Pose2D(0,0,0) #start

samples = [8,7,7,8]
inter_polated_trajectory = []
print(len(robot1_trajectory))
for i in range(len(robot1_trajectory)-1):
    print(i)
    xpoints = np.linspace(robot1_trajectory[i].x, robot1_trajectory[i+1].x,num=samples[i],endpoint=False)
    ypoints = np.linspace(robot1_trajectory[i].y, robot1_trajectory[i+1].y,num=samples[i],endpoint=False)
    for j in range(len(xpoints)):
        inter_polated_trajectory.append(Pose2D(xpoints[j],ypoints[j],0))
inter_polated_trajectory.append(robot1_trajectory[4])
robot1_trajectory = inter_polated_trajectory
print(inter_polated_trajectory)
#Trajectory2
robot2_trajectory = [Pose2D() for i in range(5)]

robot2_trajectory[0] = Pose2D(0,0,0)
robot2_trajectory[1] = Pose2D(0.2286*6,0,0)
robot2_trajectory[2] = Pose2D(0.2286*6,-0.2032*6,0)
robot2_trajectory[3] = Pose2D(0.2286*6,0,0)
robot2_trajectory[4] = Pose2D(0,0,0)

#Trajectory3
robot3_trajectory = [Pose2D() for i in range(5)]

robot3_trajectory[0] = Pose2D(0,0,0)
robot3_trajectory[1] = Pose2D( 0.2286*6,0 ,0)
robot3_trajectory[2] = Pose2D(0.2286*6,0.2032*6,0)
robot3_trajectory[3] = Pose2D( 0.2286*6,0 ,0)
robot3_trajectory[4] = Pose2D(0,0,0)

#Trajectory4
robot4_trajectory = [Pose2D() for i in range(5)]

robot4_trajectory[0] = Pose2D(0,0,0)
robot4_trajectory[1] = Pose2D( 0.2032*6,0 ,0)
robot4_trajectory[2] = Pose2D(0.2032*6,0.1778*6,0)
robot4_trajectory[3] = Pose2D( 0.2032*6,0 ,0)
robot4_trajectory[4] = Pose2D(0,0,0)
#endregion

#region Controllers
#Controller1

trajecotry_index = 0
robot1_controller = SmartController()
robot1_controller.update_trajectory(robot1_trajectory)

#Controller2

trajecotry_index = 0
robot2_controller = SmartController()
robot2_controller.update_trajectory(robot2_trajectory)

#Controller3


trajecotry_index = 0
robot3_controller = SmartController()
robot3_controller.update_trajectory(robot3_trajectory)

#Controller4


trajecotry_index = 0
robot4_controller = SmartController()
robot4_controller.update_trajectory(robot4_trajectory)
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

#region Smart Controller Interface
def update_smarth_command():
    global my_commands
    global robot1_pose
    global robot2_pose
    global robot3_pose
    global robot4_pose
    global robot1_controller
    global robot2_controller
    global robot3_controller
    global robot4_controller

    if not robot1_controller.execution_completed:
        robot1_controller.update_robot_position(robot1_pose)
        #
        robot1_controller.calculate_command()
        command = robot1_controller.get_command()
        robot1_command.angular.z = command[0]
        robot1_command.linear.x = command[1]

    # elif not robot2_controller.execution_completed:
    #     robot2_controller.update_robot_position(robot2_pose)
    #     robot2_controller.calculate_command()
    #     command = robot2_controller.get_command()
    #     robot2_command.angular.z = command[0]
    #     robot2_command.linear.x = command[1]

    # elif not robot3_controller.execution_completed:
    #     robot3_controller.update_robot_position(robot3_pose)
    #     robot3_controller.calculate_command()
    #     command = robot3_controller.get_command()
    #     robot3_command.angular.z = command[0]
    #     robot3_command.linear.x = command[1]

    # elif not robot4_controller.execution_completed:
    #     robot4_controller.update_robot_position(robot4_pose)
    #     robot4_controller.calculate_command()
    #     command = robot4_controller.get_command()
    #     robot4_command.angular.z = command[0]
    #     robot4_command.linear.x = command[1]

#endregion

#region util functions
def set_trajectories():
    global robot1_start_pose
    global robot2_start_pose
    global robot3_start_pose
    global robot4_start_pose
    global robot1_trajectory
    global robot2_trajectory
    global robot3_trajectory
    global robot4_trajectory

    for i in range(len(robot1_trajectory)):
        robot1_trajectory[i].x += robot1_start_pose.x
        robot1_trajectory[i].y += robot1_start_pose.y

    for i in range(len(robot2_trajectory)):
        robot2_trajectory[i].x += robot2_start_pose.x
        robot2_trajectory[i].y += robot2_start_pose.y

    for i in range(len(robot3_trajectory)):
        robot3_trajectory[i].x += robot3_start_pose.x
        robot3_trajectory[i].y += robot3_start_pose.y

    for i in range(len(robot4_trajectory)):
        robot4_trajectory[i].x += robot4_start_pose.x
        robot4_trajectory[i].y += robot4_start_pose.y
    

#endregion

#region Main Call
if __name__=="__main__":
    rospy.init_node("smarth_controller", anonymous=True)
    robot1_pub = rospy.Publisher("/robot1/cmd_vel", Twist,queue_size=10)
    robot2_pub = rospy.Publisher("/robot2/cmd_vel", Twist,queue_size=10)
    robot3_pub = rospy.Publisher("/robot3/cmd_vel", Twist,queue_size=10)
    robot4_pub = rospy.Publisher("/robot4/cmd_vel", Twist,queue_size=10)

    rospy.Subscriber("/robot1/odom", Odometry,get_robot1_position)
    rospy.Subscriber("/robot2/odom", Odometry,get_robot2_position)
    rospy.Subscriber("/robot3/odom", Odometry,get_robot3_position)
    rospy.Subscriber("/robot4/odom", Odometry,get_robot4_position)

    set_trajectories()
    r = rospy.Rate(25)
    while not rospy.is_shutdown():
        update_smarth_command()
        robot1_pub.publish(robot1_command)
        robot2_pub.publish(robot2_command)
        robot3_pub.publish(robot3_command)
        robot4_pub.publish(robot4_command)
        r.sleep()
    print("not exe")
    robot1_pub.publish(Twist())
#endregion