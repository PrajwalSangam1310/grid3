#!/usr/bin/env python


from collections import deque
import math
from geometry_msgs.msg import Pose2D
from gridIdentifier import utilfunctions
import numpy as np

class SmartController:
    utilfunctions = utilfunctions()
    def __init__(self, target_points = (), radial_gain_constant = 1, linear_gain_constant = 0.2, distance_threshold = 0.01, angle_threshold = 0.05):
        self.target_points = deque(target_points)
        self.LINEAR_GAIN_CONSTANT = 16/1080/3
        self.RADIAL_GAIN_CONSTANT = 15/math.pi
        self.DISTANCE_THRESHOLD = 30
        self.SUB_DISTANCE_THRESHOLD = 0.15
        self.ANGLE_THRESHOLD = angle_threshold
        self.new_point = True
        self.next_turn = False 
        self.angular_velocity = 0
        self.linear_velocity = 0
        self.current_position = Pose2D()
        self.MIN_LINEAR_VELOCITY = 0
        self.MAX_LINEAR_VELOCITY = 1
        self.MIN_ANGULAR_VELOCITY = 0.0
        self.MAX_ANGULAR_VELOCITY = 1
        self.MAX_ANGULAR_THRESHOLD = 15/180*math.pi
        self.MIN_ANGULAR_THRESHOLD = 10/180*math.pi


        self.inLinearState = True
        self.inRotationstate = True
        self.execution_completed = False
        self.destinations = {}
        self.initialize_destination_dict()

    def initialize_destination_dict(self):
        self.destinations = {}
        c = [2,5,6,9,10,13]
        r = [3,4,7,8,11,12]
        east = [2,6,10]
        west = [5,9,13]
        north = [2,6,10]
        south = [5,9,13]
        for i in c:
            for j in r:
                self.destinations[(j,i)] = True
                self.destinations[(i,j)] = True
        
        for k in self.destinations.keys():
            if k[0] in north:
                self.destinations[k] = 'S'
            elif k[0] in south:
                self.destinations[k] = 'N'
            elif k[1] in east:
                self.destinations[k] = 'W'
            elif k[1] in west:
                self.destinations[k] = 'E'
        

        print(self.destinations)

    def calculate_command(self):
        if len(self.target_points) == 0:
            print("No More Target Points!")
            self.execution_completed = True 
            return -1
        current_target = self.target_points[0]


        # display section
        print("current_position:\nx:{} y:{} theta:{}".format(self.current_position.x, self.current_position.y, self.current_position.theta))

        print("goal_position:\nx:{} y:{} theta:{}".format(current_target.x, current_target.y, current_target.theta))
        
        distance_error = self.euclidean_distance(self.current_position.x,self.current_position.y,current_target.x,current_target.y)
        print("Distance Error: {}".format(distance_error))
        

        if abs(current_target.theta) != 0:
            angle_ref = current_target.theta
        else:
            angle_ref = math.atan2(-current_target.y + self.current_position.y,current_target.x - self.current_position.x)
        
        if(angle_ref < 0):
            angle_ref += 2*math.pi

        if(self.current_position.theta < 0):
            self.current_position.theta += 2*math.pi
            # if angle_ref>170*3.14/180 or angle_ref < -170*3.14/180:
            #     angle_ref = abs(angle_ref)
        print("ref angle is {}".format(angle_ref*180/3.14))
        print("cur angle is {}".format(self.current_position.theta*180/3.14))
        
        angle_error = angle_ref - self.current_position.theta
        # print("old Angle Error: {}".format(angle_error*180/3.14))


        if angle_error > math.pi:
            angle_error = -2*math.pi + angle_error

        elif angle_error < -math.pi:
            angle_error += 2*math.pi
        print("new Angle Error: {}".format(angle_error*180/3.14))


        if self.inLinearState and abs(angle_error) > self.MAX_ANGULAR_THRESHOLD:
            self.inRotationstate = True
            self.inLinearState = False

        elif self.inRotationstate and  abs(angle_error) < self.MIN_ANGULAR_THRESHOLD:
            self.inLinearState = True
            self.inRotationstate = False

        if self.inLinearState:
            self.linearCommand(distance_error)
        elif self.inRotationstate:
            self.rotationCommand(angle_error)

    def isAtDestination(self):
        curPos = self.ij2rc(self.current_position.y, self.current_position.x)
        # if self.target_points!=None and len(self.target_points) > 0:

        #     current_target = self.target_points[0]
        #     distance_error = self.euclidean_distance(self.current_position.x,self.current_position.y,current_target.x,current_target.y)
        #     print(f"[controller smart] distance error: {distance_error}")
        #     if distance_error > self.DISTANCE_THRESHOLD:
        #         print(f"[controller smart] returning false, distance error more then the threshold")
        #         return False

        if curPos in self.destinations:
            return self.destinations[curPos]
        else:
            return False

    def linearCommand(self, distance_error):
        self.linear_velocity = min(1,distance_error*self.LINEAR_GAIN_CONSTANT)
        self.angular_velocity = 0

    def rotationCommand(self, angle_error):
        self.angular_velocity = min(1,angle_error*self.RADIAL_GAIN_CONSTANT)
        self.linear_velocity = 0

    def update_robot_position(self,position):
        self.current_position.x = position.x
        self.current_position.y = position.y
        self.current_position.theta = position.theta 


    def get_command(self):
        # return (self.angular_velocity,0)
        # if self.execution_completed:
        #     return(0,0)
        # else:
        print("linear velocity: {}\nangualar velocity: {}".format(self.linear_velocity,self.angular_velocity))
        return (self.angular_velocity, self.linear_velocity)

    def update_trajectory(self, target_points):
        self.target_points = target_points

    def clear_trajectory(self):
        self.target_points = deque()
    
    def _update_target_point(self,distance_error):
        if distance_error < self.DISTANCE_THRESHOLD:
            if(len(self.target_points) < 2):
                self.target_points.popleft()
                return
            present = self.target_points[0]
            present_next = self.target_points[1]
            present_next_next = self.target_points[2]
            angle2 = np.arctan2(present_next_next.y - present_next.y, present_next_next.x - present_next.x)
            angle1 = np.arctan2(present_next.y - present.y, present_next.x - present.x)
            #abs dalna hai

            angle_diff = angle2-angle1
            print(abs(angle_diff))
            if(abs(angle_diff) > 1.5):
                self.next_turn = True
            else:
                self.next_turn = False 

            self.new_point = True
            self.target_points.popleft()

    @staticmethod
    def euclidean_distance(x1,y1,x2,y2):
        return ((x1-x2)**2 + (y1-y2)**2)**0.5

    @staticmethod
    def ij2rc(i,j):
        # print(i,j)
        delta = 1600/16
        initialI = delta/2
        initialJ = delta/2
        r = int((i-initialI+delta/2)//delta)
        c = 15-int((j-initialJ+delta/2)//delta)
        return(r,c)
    #make distnace error like angle error