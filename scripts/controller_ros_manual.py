#!/usr/bin/env python3
from queue import Empty
from typing import ForwardRef
import rospy
from controller_latest_smart2 import SmartController
from grid3.msg import botCommands, botPoses, plannerStatus, botTrajectories, trajectory, imagePoint, botCommand
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Empty
from gridIdentifier import utilfunctions
from colorama import Fore, Back
import math

class controllerRosInterface:
    def __init__(self):
        self.N = 4
        self.controllers = [SmartController() for i in range(self.N)]

        # command publisher
        self.commands = botCommands()
        self.commands.botIds = [i for i in range(self.N)]
        self.commands.forwardSpeeds = [0 for i in range(self.N)]
        self.commands.rotationalSpeeds = [0 for i in range(self.N)]
        self.commands.instructions = [0 for i in range(self.N)]
        self.command = botCommand()
        self.commandsPublisher = rospy.Publisher("grid3/controller/cmd", botCommands, queue_size=10)
        self.commandsBroadcaster = rospy.Publisher("grid3/controller/cmdBroadcast", botCommand, queue_size=10)

        # self.commandPublisher1 = rospy.Publisher("grid3/controller/cmd1", botCommand, queue_size=10)
        # self.commandPublisher2 = rospy.Publisher("grid3/controller/cmd2", botCommand, queue_size=10)
        # self.commandPublisher3 = rospy.Publisher("grid3/controller/cmd3", botCommand, queue_size=10)
        # self.commandPublisher4 = rospy.Publisher("grid3/controller/cmd4", botCommand, queue_size=10)



        # subscribers
        self.curPoseSubscriber = rospy.Subscriber("grid3/allRobots/poses", botPoses, self.curPoseSubscriberCb)
        self.trajectorySubscriber = rospy.Subscriber("grid3/robotTrajectories", botTrajectories, self.trajectorySubscriberCb)
        self.plannerStatusSubscriber = rospy.Subscriber("grid3/planner/status", plannerStatus, self.trajectoryPlannerStatusCb)
        self.manualCommandSubscriber = rospy.Subscriber("grid3/robot1/cmd_vel", Twist, self.cb)
        self.dropSubscriber = rospy.Subscriber("grid3/drop", Empty, self.drop)
        self.dropStopSubscriber = rospy.Subscriber("grid3/dropStop", Empty, self.dropStop)

        self.drop = False
        #attributes to store the subscriber data
        self.curPoses = botPoses()
        self.curPoses.ids = [i for i in range(self.N)]
        self.curPoses.xs = [0 for i in range(self.N)]
        self.curPoses.ys = [0 for j in range(self.N)]
        self.curPoses.thetas = [0.0 for i in range(self.N)]

        self.curTrajectories = botTrajectories()
        self.curTrajectories.botIds = [i for i in range(self.N)] 
        self.curTrajectories.trajectories = [trajectory() for i in range(self.N)]
        for i in range(self.N):
            self.curTrajectories.trajectories[i].points = []
        #class variables
        self.commandTrajectories = [[] for i in range(self.N)]
        self.plannerStatus = plannerStatus()
        self.plannerStatus.botIds = [i for i in range(4)]
        self.plannerStatus.status = [1 for i in range(4)]
        self.utilfunctions = utilfunctions()

        self.rate2 = rospy.Rate(8)

        self.droppingIterations = [0 for i in range(4)]
        self.droppingInProgress = [False for i in range(4)]

        self.dropAngleThreshold = 10/180*math.pi
        self.droplinearThreshold = 20

        self.cur_last_point = [None for i in range(4)]
        self.distance_reached = [False for i in range(4)]
        self.start_turning = [False for i in range(4)]
        #instructions
        # 4 - move Forward
        # 5 - turn right
        # 0 -  halt
        # 7 - turn left
    def drop(self, data):
        self.drop = True
        print("changed dropping status")
        i = 1
        self.commands.instructions[i] = 1

    def dropStop(self, data):
        self.drop = False
        print("changed dropping status")
        i = 1
        self.commands.instructions[i] = 0



    def cb(self, data):

            rotationalSpeed = -data.angular.z 
            forwardSpeed = data.linear.x
            print("in cb")
            i  = 1
            self.commands.botIds[i] = i
            if rotationalSpeed > 1:
                rotationalSpeed = 1
            if forwardSpeed > 1:
                forwardSpeed = 1
            
            # self.commands.rotationalSpeeds[i] = abs(int(rotationalSpeed*32))
            self.commands.rotationalSpeeds[i] = 4
            if i == 2:
                self.commands.rotationalSpeeds[i] = 4

            self.commands.forwardSpeeds[i] = 7
            # self.commands.forwardSpeeds[i] = abs(int(forwardSpeed*32))

            # set the instructions
            if forwardSpeed == 0:
                if rotationalSpeed == 0:
                    self.commands.instructions[i] = 0
                elif rotationalSpeed > 0:
                    # turn left
                    self.commands.instructions[i] = 7
                elif rotationalSpeed < 0:
                    self.commands.instructions[i] = 5
            else:
                self.commands.instructions[i] = 4

            if i == 1:
                if self.commands.instructions[i] == 7:
                    self.commands.instructions[i] = 5
                elif self.commands.instructions[i] == 5:
                    self.commands.instructions[i] = 7

            if self.drop:
                self.commands.instructions[i] = 1

            print(self.commands.instructions[i])
            print(self.commands.forwardSpeeds[i])
            print(self.commands.rotationalSpeeds[i])
            print(self.commands.botIds[i])




    def curPoseSubscriberCb(self, data):
        for i in range(self.N):
            self.curPoses.ids[i] = data.ids[i]
            self.curPoses.xs[i] = data.xs[i]
            self.curPoses.ys[i] = data.ys[i]
            self.curPoses.thetas[i] = data.thetas[i]
        self.updateRobotPoses()
        
    
    def trajectorySubscriberCb(self, data):
        for i in range(self.N):
            self.curTrajectories.botIds[i] = data.botIds[i]
            self.curTrajectories.trajectories[i] = data.trajectories[i]
        self.updateCommandTrajectories()
        for i in range(4):
            # self.controllers[i].clear_trajectory()
            self.controllers[i].update_trajectory(self.commandTrajectories[i])
        
    
    def trajectoryPlannerStatusCb(self, data):
        self.plannerStatus.botIds = data.botIds
        self.plannerStatus.status = data.status
        # print("in planner status callback")
        # print(self.plannerStatus)

    def publishCommands(self):
        self.commandsPublisher.publish(self.commands)
        for i in range(4):
            self.command.botId = self.commands.botIds[i]
            self.command.instruction = self.commands.instructions[i]
            self.command.forwardSpeed = self.commands.forwardSpeeds[i]
            self.command.rotationalSpeed = self.commands.rotationalSpeeds[i]
            self.commandsBroadcaster.publish(self.command)

            # if i == 0:
            #     self.commandPublisher1.publish(self.command)
            # elif i==1:
            #     self.commandPublisher2.publish(self.command)
            # elif i==2:    
            #     self.commandPublisher3.publish(self.command)
            # elif i==3:
            #     self.commandPublisher4.publish(self.command)


    def updateCommandTrajectories(self):
        self.commandTrajectories = [[] for i in range(4)]
        # print(self.curTrajectories.trajectories[0])
        for i in range(len(self.curTrajectories.trajectories)):
            for j in range(len(self.curTrajectories.trajectories[i].points)):
                self.commandTrajectories[i].append(
                    Pose2D(self.curTrajectories.trajectories[i].points[j].i,\
                     self.curTrajectories.trajectories[i].points[j].j,\
                    0))
                if len(self.curTrajectories.trajectories[i].points) == 1:
                    self.cur_last_point[i] = Pose2D(self.curTrajectories.trajectories[i].points[j].i,\
                                                    self.curTrajectories.trajectories[i].points[j].j,\
                                                    0)

    def updateRobotPoses(self):
        for i in range(self.N):
            self.controllers[i].update_robot_position(\
                Pose2D(self.curPoses.xs[i], self.curPoses.ys[i], self.curPoses.thetas[i]
                ))

    def updateCommands(self):
        for i in range(self.N):
            computeCommand = False
            print(Fore.GREEN+'\n----------- caluculating for bot:', i,'---------')
            print(Fore.RESET)
            
            if self.plannerStatus.status[i] == 1 or self.droppingInProgress[i]:
                dir =  self.controllers[i].isAtDestination()
                print(f"[Controller, {i}]: At destination")
                print(f"[Controller, {i}]: Direction is {dir}")
                # print("####################################################################################################33\n################################################################################################################################333###########################################3n\n################################\n")
                if dir and not self.distance_reached[i]:
                    print(f"[Controller, {i}]: Cur last point")
                    print(self.cur_last_point[i])
                    if self.cur_last_point[i] == None:
                        continue
                    distance_error = self.controllers[i].euclidean_distance(self.curPoses.xs[i],self.curPoses.ys[i],\
                                                                            self.cur_last_point[i].x, self.cur_last_point[i].y)
                    print(f'[Controller,{i}] Minimizing reach error, reach error:{distance_error}')
                    if(not distance_error < self.droplinearThreshold):
                            self.controllers[i].update_trajectory([self.cur_last_point[i]])
                            computeCommand = True
                            self.distance_reached[i] = False
                            self.start_turning[i] = False
                    else:
                        self.distance_reached[i] = True
                        self.start_turning[i] = True
                        print(f"[Controller,{i}] Goal Reached,Starting turning")
                                    
                elif dir and self.start_turning[i]:
                    self.droppingInProgress[i] = True
                    dir = self.get_theta(dir)
                    cur_angle = self.curPoses.thetas[i]
                    if self.curPoses.thetas[i] < 0:
                        cur_angle += 2*math.pi 

                    drop_angle_error = dir - cur_angle
                    if drop_angle_error > math.pi:
                        angle_error = -2*math.pi + angle_error

                    elif drop_angle_error < -math.pi:
                        drop_angle_error += 2*math.pi
                    print(f"[Controller, {i}]: Drop angle error {drop_angle_error}")

                    if abs(drop_angle_error)< self.dropAngleThreshold:
                        print(f"[Controller, {i}]: Turining Completed")
                        print(f"[Controller, {i}]: dumping initiated")
                        self.commands.botIds[i] = i
                        self.commands.instructions[i] = 1
                        self.commands.rotationalSpeeds[i] = 0
                        self.commands.forwardSpeeds[i] = 0
                        self.droppingIterations[i] += 1
                        if self.droppingIterations[i] > 10:
                            self.droppingInProgress[i] = False
                            self.distance_reached[i] = False
                            self.droppingIterations[i] = 0
                            self.start_turning[i] = False
                        continue
                    else:
                        computeCommand = True
                        print(f"[Controller, {i}] Turning to Dump Point")
                        self.controllers[i].update_trajectory([Pose2D(self.curPoses.xs[i], self.curPoses.ys[i], dir)])  

            if self.plannerStatus.status[i] != 0 and not computeCommand:
                
                print(f"[Controller, {i}] Planner status {self.plannerStatus.status[i]}")
                print(f"[Controller, {i}] didnt calculate")
                self.commands.botIds[i] = i
                self.commands.instructions[i] = 0
                self.commands.rotationalSpeeds[i] = 0
                self.commands.forwardSpeeds[i] = 0
                continue
            
            
            self.controllers[i].calculate_command()
            [rotationalSpeed, forwardSpeed] = self.controllers[i].get_command()
            # print(forwardSpeed, rotationalSpeed)
            #map the forward speed and rotational speed to int
            
            self.commands.botIds[i] = i
            if rotationalSpeed > 1:
                rotationalSpeed = 1
            if forwardSpeed > 1:
                forwardSpeed = 1
            
            # self.commands.rotationalSpeeds[i] = abs(int(rotationalSpeed*32))
            self.commands.rotationalSpeeds[i] = 4
            if i == 2:
                self.commands.rotationalSpeeds[i] = 4

            self.commands.forwardSpeeds[i] = 7
            # self.commands.forwardSpeeds[i] = abs(int(forwardSpeed*32))

            # set the instructions
            if forwardSpeed == 0:
                if rotationalSpeed == 0:
                    self.commands.instructions[i] = 0
                elif rotationalSpeed > 0:
                    # turn left
                    self.commands.instructions[i] = 7
                elif rotationalSpeed < 0:
                    self.commands.instructions[i] = 5
            else:
                self.commands.instructions[i] = 4

            if i == 1:
                if self.commands.instructions[i] == 7:
                    self.commands.instructions[i] = 5
                elif self.commands.instructions[i] == 5:
                    self.commands.instructions[i] = 7
                
            #map the tempCommand to our required Command
    def get_theta(self, dir):
        if dir == 'N':
            return math.pi/2
        elif dir =='S':
            return 3*math.pi/2
        elif dir == 'W':
            return math.pi
        elif dir == 'E':
            return 0.0
        

        
if __name__ =='__main__':
    rospy.init_node("controller_ros_interface", anonymous=True)
    testInterface = controllerRosInterface()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        # testInterface.updateCommands()
        
        testInterface.publishCommands()
        print("in loop")
        rate.sleep()