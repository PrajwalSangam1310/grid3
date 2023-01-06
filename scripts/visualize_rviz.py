#!/usr/bin/env python3

import rospy
from rospy.rostime import Time
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header

if __name__=="__main__":
    rospy.init_node("test_texture_node", anonymous=True)
    pub = rospy.Publisher("/grid3/mat", MarkerArray, queue_size=10)
    grid = MarkerArray()
    grid.markers = [Marker() for i in range(23)]
    delta = 0.1524
    base = 2.1336
    dest = 0.3048

    for i in range(3):
        for j in range(3):
            grid.markers[i*3+j].type = Marker.CUBE
            # grid.markers[i*3+j].points = [Point((i+1)*dest,(j+1)*dest,0)]

            grid.markers[i*3+j].points = []
            grid.markers[i*3+j].pose.position.x = (2*i+1.5)*dest + delta
            grid.markers[i*3+j].pose.position.y = (2*j+1.5)*dest + delta
            grid.markers[i*3+j].pose.position.z = -0.02


            grid.markers[i*3+j].header.frame_id = 'grid/odom'
            grid.markers[i*3+j].header.stamp = Time.now()
            grid.markers[i*3+j].ns = 'grid3'
            grid.markers[i*3+j].color.a = 1
            grid.markers[i*3+j].color.r = 1
            grid.markers[i*3+j].color.g = 1
            grid.markers[i*3+j].id = i*3 + j
            grid.markers[i*3+j].scale.x = dest
            grid.markers[i*3+j].scale.y = dest
            grid.markers[i*3+j].scale.z = 0.01
    
    grid.markers[9].type = Marker.CUBE
    grid.markers[9].pose.position.x = base/2 + delta
    grid.markers[9].pose.position.y = base/2 + delta
    grid.markers[9].pose.position.z = -0.03
    grid.markers[9].header.frame_id = 'grid/odom'
    grid.markers[9].header.stamp = Time.now()
    grid.markers[9].ns = 'grid3'
    grid.markers[9].color.a = 1
    grid.markers[9].color.r = 1
    grid.markers[9].color.b = 1
    grid.markers[9].color.g = 1

    grid.markers[9].id = 9
    grid.markers[9].scale.x = base
    grid.markers[9].scale.y = base
    grid.markers[9].scale.z = 0.01
    
    induction_points = [(5,15),(10,15)]

    for i in range(2):
        grid.markers[10+i].type = Marker.CUBE
        # grid.markers[i*3+j].points = [Point((i+1)*dest,(j+1)*dest,0)]

        grid.markers[10+i].points = []
        grid.markers[10+i].pose.position.x = (induction_points[i][1] + 0.5)*delta 
        grid.markers[10+i].pose.position.y = (induction_points[i][0] + 0.5)*delta 
        grid.markers[10+i].pose.position.z = -0.01
        grid.markers[10+i].header.frame_id = 'grid/odom'
        grid.markers[10+i].header.stamp = Time.now()
        grid.markers[10+i].ns = 'grid3'
        grid.markers[10+i].color.a = 1
        grid.markers[10+i].color.b = 0.4
        grid.markers[10+i].id = 10+i
        grid.markers[10+i].scale.x = delta
        grid.markers[10+i].scale.y = delta
        grid.markers[10+i].scale.z = 0.01

        desNames = ['Mumbai', 'Delhi', 'Kolkata', 'Chennai','Bengaluru','Hyderbad','Pune','Ahemdabad','Jaipur']

    for i in range(3):
        for j in range(3):
            grid.markers[12+i*3+j].type = Marker.TEXT_VIEW_FACING
            # grid.markers[i*3+j].points = [Point((i+1)*dest,(j+1)*dest,0)]

            grid.markers[12+i*3+j].points = []
            grid.markers[12+i*3+j].pose.position.x = (2*j+1.5)*dest + delta
            grid.markers[12+i*3+j].pose.position.y = (2*(2-i)+1.5)*dest + delta
            grid.markers[12+i*3+j].pose.position.z = 0.02
            grid.markers[12+i*3+j].text = desNames[i*3+j]

            grid.markers[12+i*3+j].header.frame_id = 'grid/odom'
            grid.markers[12+i*3+j].header.stamp = Time.now()
            grid.markers[12+i*3+j].ns = 'grid3'
            grid.markers[12+i*3+j].color.a = 1
            grid.markers[12+i*3+j].color.r = 1
            grid.markers[12+i*3+j].color.g = 0
            grid.markers[12+i*3+j].id = 12+i*3 + j
            grid.markers[12+i*3+j].scale.x = 0.03
            grid.markers[12+i*3+j].scale.y = 0.0
            grid.markers[12+i*3+j].scale.z = 0.07

    # pub.publish(grid)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        for i in range(len(grid.markers)):
            grid.markers[i].header.stamp = Time.now()
        pub.publish(grid)
        # print("published new data")
        r.sleep()