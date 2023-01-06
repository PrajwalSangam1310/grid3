#!/usr/bin/env python3

import rospy
from rospy.rostime import Time
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from grid3.msg import botPoses, botCommands
from gridIdentifier import utilfunctions
from tf.transformations import quaternion_from_euler
from tf import TransformBroadcaster
from geometry_msgs.msg import TwistStamped


class robotVisualizer:
    def __init__(self):
        self.N = 4
        self.robotMarkers = MarkerArray()
        self.robotMarkers.markers = [Marker() for _ in range(4)]
        self.initializeMarkers()
        self.robotsSubscriber = rospy.Subscriber("grid3/allRobots/poses", botPoses, self.botPosesSubscriberCb)
        self.commandSubscriber = rospy.Subscriber("grid3/controller/cmd", botCommands, self.commandSubscriberCb)

        self.markersPublisher = rospy.Publisher("grid3/displayrobot",MarkerArray, queue_size=10)
        self.myutils = utilfunctions()
        self.br = TransformBroadcaster()
        self.frameNames = ['robot1', 'robot2', 'robot3','robot4']
        self.parentFrameName = 'grid/odom'
        self.twists = [TwistStamped() for i in range(4)] 
        self.twistPublishers = [rospy.Publisher('grid3/visualize/robot'+str(i+1)+'/cmd', TwistStamped, queue_size = 10) for i in range(4)]
        # self.twistPublisher1 = rospy.Publisher('grid3/visualize/robo1/cmd', TwistStamped, queue_size = 10) 
        
        self.rotationScale = 0.1
        self.linearScale = 0.15
        self.initializeTwists()



    def initializeMarkers(self):
        for i in range(4):
            self.robotMarkers.markers[i].type = Marker.CUBE
            self.robotMarkers.markers[i].points = []
            self.robotMarkers.markers[i].header.frame_id = 'grid/odom'
            self.robotMarkers.markers[i].header.stamp = Time.now()
            self.robotMarkers.markers[i].ns = 'grid3'
            self.robotMarkers.markers[i].color.a = 1.0
            self.robotMarkers.markers[i].id = i
            self.robotMarkers.markers[i].scale.x = 0.127
            self.robotMarkers.markers[i].scale.y = 0.127
            self.robotMarkers.markers[i].scale.z = 0.01

        self.robotMarkers.markers[0].color.r = 1.0
        self.robotMarkers.markers[1].color.g = 1.0
        self.robotMarkers.markers[2].color.b = 1.0
        self.robotMarkers.markers[3].color.b = 1.0
        self.robotMarkers.markers[3].color.g = 1.0
    
    def initializeTwists(self):
        self.seq = 0
        for i in range(4):
            self.twists[i].header.frame_id = self.frameNames[i]
            self.twists[i].header.stamp = rospy.Time.now()

    def botPosesSubscriberCb(self, data):
        # print("in callback")
        for i in range(self.N):
            (tempx,tempy) = self.myutils.ij2xy(data.xs[i], data.ys[i])
            self.robotMarkers.markers[i].pose.position.x = tempx
            self.robotMarkers.markers[i].pose.position.y = tempy
            temp = quaternion_from_euler(0,0,data.thetas[i])
            # print(temp)
            self.robotMarkers.markers[i].pose.orientation.x = temp[0]
            self.robotMarkers.markers[i].pose.orientation.y = temp[1]
            self.robotMarkers.markers[i].pose.orientation.z = temp[2]
            self.robotMarkers.markers[i].pose.orientation.w = temp[3]

            self.robotMarkers.markers[i].header.stamp = rospy.Time.now()
            self.br.sendTransform((tempx, tempy,0.03),temp,rospy.Time.now(),self.frameNames[i],self.parentFrameName)

        self.markersPublisher.publish(self.robotMarkers)    

    def commandSubscriberCb(self, data):
        # print("in command subscriber")
        for i in range(4):
            if data.instructions[i] == 4:
                self.twists[i].twist.linear.x = self.linearScale
                if data.forwardSpeeds[i] == 0:
                    self.twists[i].twist.linear.x = 0
            else:
                self.twists[i].twist.linear.x = 0

            if data.rotationalSpeeds[i] != 0:
                if data.instructions[i] == 5:
                    self.twists[i].twist.angular.z = -self.rotationScale 
                elif data.instructions[i] == 7:
                    self.twists[i].twist.angular.z = self.rotationScale
                else:
                    self.twists[i].twist.angular.z = 0

            else:
                self.twists[i].twist.angular.z = 0

            if data.instructions[i] == 1 or data.instructions[i] == 0:
                self.twists[i].twist.angular.z = 0
                self.twists[i].twist.linear.x = 0
            
            if i == 1:
                self.twists[i].twist.angular.z = -self.twists[i].twist.angular.z
            # print(self.twists[i])
            self.twists[i].header.stamp = rospy.Time.now()
            self.twists[i].header.seq = self.seq
            self.twistPublishers[i].publish(self.twists[i])
        self.seq += 1
    
if __name__ == '__main__':
    rospy.init_node("show_robots", anonymous=True)
    visualizer = robotVisualizer()
    print("Node started and object created")
    rospy.spin()
    






