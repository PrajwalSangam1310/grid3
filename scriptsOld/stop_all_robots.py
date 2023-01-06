#!/usr/bin/env python

import rospy
import roslib
from nav_msgs.msg import  Odometry
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion
from controller_latest_smart import SmartController

#region Initialization

#region Main Call
if __name__=="__main__":
    rospy.init_node("smarth_stopper", anonymous=True)
    robot1_pub = rospy.Publisher("/robot1/cmd_vel", Twist,queue_size=10)
    robot2_pub = rospy.Publisher("/robot2/cmd_vel", Twist,queue_size=10)
    robot3_pub = rospy.Publisher("/robot3/cmd_vel", Twist,queue_size=10)
    robot4_pub = rospy.Publisher("/robot4/cmd_vel", Twist,queue_size=10)
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        robot1_pub.publish(Twist())
        robot2_pub.publish(Twist())
        robot3_pub.publish(Twist())
        robot4_pub.publish(Twist())
        print("stopiing")
        r.sleep()
    print("not exe")
#endregion