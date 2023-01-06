#!/usr/bin/env python3

import rospy
from rospy.rostime import switch_to_wallclock
from sensor_msgs.msg import Imu
from grid3.msg import simpleImu

class robot:
    def __init__(self):
        self.simpleImu = simpleImu()
        self.imu = Imu()
        self.trajectory = []
        self.controller = None
        self.controllerState = None # speed, direction etc
        self.controllerCommand = None # goal command
        self.controllerFeedback = None #executing failed reached etc
        self.imuPublisher = rospy.Publisher("/grid3/robot1/imu", Imu, queue_size= 10)

    def imuInit(self):
        self.imu.orientation.x = 0
        self.imu.orientation.y = 0
        self.imu.orientation.z = 0
        self.imu.orientation.w = 0

        self.imu.angular_velocity.x =  0
        self.imu.angular_velocity.y =  0  
        self.imu.angular_velocity.z =  0  

        self.imu.linear_acceleration.x =  0
        self.imu.linear_acceleration.y =  0  
        self.imu.linear_acceleration.z =  0  

        self.imu.angular_velocity_covariance = [0.0 for i in range(9)]
        self.imu.linear_acceleration_covariance = [0.0 for i in range(9)]
        self.imu.orientation_covariance = [0.0 for i in range(9)]

    def updateSimpleImu(self, tempImu):
        self.simpleImu.wx =  tempImu.wx
        self.simpleImu.wy =  tempImu.wy
        self.simpleImu.wz =  tempImu.wz
        self.simpleImu.ax =  tempImu.ax
        self.simpleImu.ay =  tempImu.ay
        self.simpleImu.az =  tempImu.az

        self.imu.angular_velocity.x = tempImu.wx
        self.imu.angular_velocity.y = tempImu.wy
        self.imu.angular_velocity.z = tempImu.wz

        self.imu.linear_acceleration.x = tempImu.ax
        self.imu.linear_acceleration.y = tempImu.ay
        self.imu.linear_acceleration.z = tempImu.az

    def publishImu(self):
        self.imuPublisher.publish(self.imu)
        rospy.loginfo("Published the imu values")

    def controllerUpdateCmd(self):
        self.controller.setGoal()
        self.controller.setCurrPos()
        self.controller.getCommand()
        pass


class allRobots:
    def __init__(self):
        self.robots = [robot() for i in range(4)]
        self.controller = None
        self.pathUpdate = None
        self.poistionUpdate = None
        self.subscribeTopic = "arduino/imu"

    def updateCommand(self):
        pass

    def subscriberInit(self):
        self.sub = rospy.Subscriber(self.subscribeTopic, simpleImu, self.subscriberCb) 

    def publishAllImu(self):
        for oneRobot in self.robots:
            oneRobot.publishImu()


    def subscriberCb(self, data):
        self.robots[data.botId].updateSimpleImu(data)

    def allCmdPublisher(self):
        for robot in self.robots:
            


