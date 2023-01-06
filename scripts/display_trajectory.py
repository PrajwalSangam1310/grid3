#!/usr/bin/env python3

import rospy
from rospy.rostime import Time
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from grid3.msg import botTrajectories,trajectory
from gridIdentifier import utilfunctions

class trajectoryVisualizer:
    def __init__(self):
        self.N = 4
        self.trajectoryMarkers = MarkerArray()
        self.trajectoryMarkers.markers = [Marker() for _ in range(4)]
        self.initializeMarkers()
        self.trajectorySubscriber = rospy.Subscriber("grid3/robotTrajectories", botTrajectories, self.trajectorySubscriberCb)
        self.markersPublisher = rospy.Publisher("grid3/displayTrajectory",MarkerArray, queue_size=10)


    def initializeMarkers(self):
        for i in range(4):
            self.trajectoryMarkers.markers[i].type = Marker.LINE_STRIP
            self.trajectoryMarkers.markers[i].points = []
            self.trajectoryMarkers.markers[i].header.frame_id = 'grid/odom'
            self.trajectoryMarkers.markers[i].header.stamp = Time.now()
            self.trajectoryMarkers.markers[i].ns = 'grid3'
            self.trajectoryMarkers.markers[i].color.a = 1.0
            self.trajectoryMarkers.markers[i].id = i
            self.trajectoryMarkers.markers[i].scale.x = 0.01
            self.trajectoryMarkers.markers[i].scale.y = 0.1
            self.trajectoryMarkers.markers[i].scale.z = 0.1

        self.trajectoryMarkers.markers[0].color.r = 1.0
        self.trajectoryMarkers.markers[1].color.g = 1.0
        self.trajectoryMarkers.markers[2].color.b = 1.0
        self.trajectoryMarkers.markers[3].color.b = 1.0
        self.trajectoryMarkers.markers[3].color.g = 1.0



    
    def ij2xy(self,i,j):
        factor = 2.4384/1600
        x = (i)*factor
        y = (1600- j)*factor
        return (x,y)

    def trajectorySubscriberCb(self, data):
        # print("in callback")
        for i in range(self.N):
            self.trajectoryMarkers.markers[i].points = []
            for j in range(len(data.trajectories[i].points)):
                (tempx,tempy) = self.ij2xy(data.trajectories[i].points[j].i, data.trajectories[i].points[j].j)
                self.trajectoryMarkers.markers[i].points.append(Point(tempx,tempy, 0))
                self.trajectoryMarkers.markers[i].header.stamp = Time.now()
            if len(self.trajectoryMarkers.markers[i].points) == 1:
                temp = self.trajectoryMarkers.markers[i].points[0]
                self.trajectoryMarkers.markers[i].points.append(Point(temp.x+0.01, temp.y+0.01, 0))
        self.markersPublisher.publish(self.trajectoryMarkers)     

if __name__ == '__main__':
    rospy.init_node("test_node", anonymous=True)
    visualizer = trajectoryVisualizer()
    print("Node started and object created")
    rospy.spin()
    



if False:
    rospy.init_node("test_node", anonymous=True)
    temp = MarkerArray()
    temp.markers = [Marker() for _ in range(4)]  
    for i in range(4):
        temp.markers[i].type = Marker.LINE_STRIP
        temp.markers[i].points = []
        temp.markers[i].header.frame_id = 'map'
        temp.markers[i].header.stamp = Time.now()
        temp.markers[i].ns = 'grid3'
        temp.markers[i].color.a = 1.0
        temp.markers[i].color.r = 1.0


        temp.markers[i].id = i
        temp.markers[i].scale.x = 0.1
        temp.markers[i].scale.y = 0.1
        temp.markers[i].scale.z = 0.1

    
    for i in range(4):
        for j in range(4):
            temp.markers[i].points.append(Point(i,j,0))
    
    # print(temp)
    markerPublisher = rospy.Publisher("grid3/displayAllTrajectory", MarkerArray, queue_size=10)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        for i in range(4):
            temp.markers[i].header.stamp = Time.now()
        markerPublisher.publish(temp)
        # print("published")
        r.sleep()




