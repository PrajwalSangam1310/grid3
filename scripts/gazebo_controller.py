#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from grid3.msg import botCommands


class GazeboController:
    def __init__(self):
        self.robot1Topic = "robot1/cmd_vel"
        self.robot2Topic = "robot2/cmd_vel"
        self.robot3Topic = "robot3/cmd_vel"
        self.robot4Topic = "robot4/cmd_vel"

        self.robot1Publisher = rospy.Publisher(self.robot1Topic, Twist, queue_size= 10)
        self.robot2Publisher = rospy.Publisher(self.robot2Topic, Twist, queue_size= 10)
        self.robot3Publisher = rospy.Publisher(self.robot3Topic, Twist, queue_size= 10)
        self.robot4Publisher = rospy.Publisher(self.robot4Topic, Twist, queue_size= 10)

        self.cmdSubscriber = rospy.Subscriber("grid3/controller/cmd", botCommands, self.cmdSubscriberCb)
        self.cmds = botCommands()
        self.cmds.botIds = [i for i in range(4)]
        self.cmds.instructions = [0 for i in range(4)]
        self.cmds.rotationalSpeeds = [0 for i in range(4)]
        self.cmds.forwardSpeeds = [0 for i in range(4)]

        self.linearSpeed = 0.04
        self.angularSpeed = 0.35


        self.robotCmds = [Twist() for i in range(4)]
        for i in range(4):
            self.robotCmds[i].linear = Vector3()
            self.robotCmds[i].angular = Vector3()
        
        self.rotationalSpeedFactor = 0.02
        self.forwardSpeedFactor = 0.1



    def cmdSubscriberCb(self, data):
        self.cmds.botIds  = data.botIds
        self.cmds.instructions  = data.instructions
        self.cmds.rotationalSpeeds  = data.rotationalSpeeds
        self.cmds.forwardSpeeds  = data.forwardSpeeds

        for i in range(4):
            self.robotCmds[i].angular.z = self.cmds.rotationalSpeeds[i]/32*self.rotationalSpeedFactor
            self.robotCmds[i].linear.x = self.cmds.forwardSpeeds[i]/32*self.forwardSpeedFactor
            # if self.robotCmds[i].linear.x > 0.05:
            #     self.robotCmds[i].linear.x = 0.03
            # if self.robotCmds[i].angular.z > 0.05:
            #     self.robotCmds[i].angular.z = 0.05
            if self.cmds.instructions[i] == 4:
                self.robotCmds[i].linear.x = self.linearSpeed
                self.robotCmds[i].angular.z = 0

            elif self.cmds.instructions[i] == 7:
                self.robotCmds[i].linear.x = 0
                self.robotCmds[i].angular.z = -self.angularSpeed
                if i == 2 or i == 3:
                    self.robotCmds[i].angular.z = self.angularSpeed
            elif data.instructions[i] == 5:
                    
                self.robotCmds[i].angular.z = self.angularSpeed
                self.robotCmds[i].linear.x = 0
                if i == 2 or i == 0 or i == 3:
                    self.robotCmds[i].angular.z = -self.angularSpeed
            else:
                self.robotCmds[i].linear.x = 0
                self.robotCmds[i].angular.z = 0


            # self.robotCmds[i].angular.z = -self.robotCmds[i].angular.z
            
            
    
    def publishAllCmds(self):
        for i in range(4):
            print(self.robotCmds[i].linear.x, self.robotCmds[i].angular.z)

        self.robot1Publisher.publish(self.robotCmds[0])
        self.robot2Publisher.publish(self.robotCmds[1])
        self.robot3Publisher.publish(self.robotCmds[2])
        self.robot4Publisher.publish(self.robotCmds[3])

    
if __name__ == '__main__':
    rospy.init_node("gazebo_controller")
    temp = GazeboController()
    r = rospy.Rate(15)
    while not rospy.is_shutdown():
        temp.publishAllCmds()
        print("published commands")
        r.sleep()

        


    






