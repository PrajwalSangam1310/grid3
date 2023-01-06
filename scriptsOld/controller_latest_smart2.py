#!/usr/bin/env python


from collections import deque
import math
from geometry_msgs.msg import Pose2D
import numpy as np

class SmartController:

    def __init__(self, target_points = (), radial_gain_constant = 0.5, linear_gain_constant = 0.2, distance_threshold = 0.01, angle_threshold = 0.05):
        self.target_points = deque(target_points)
        self.LINEAR_GAIN_CONSTANT = linear_gain_constant
        self.RADIAL_GAIN_CONSTANT = radial_gain_constant
        self.DISTANCE_THRESHOLD = distance_threshold
        self.SUB_DISTANCE_THRESHOLD = 0.15
        self.ANGLE_THRESHOLD = angle_threshold
        self.new_point = True
        self.next_turn = False 
        self.anular_velocity = 0
        self.linear_velocity = 0
        self.current_position = Pose2D()
        self.MIN_LINEAR_VELOCITY = 0.01
        self.MAX_LINEAR_VELOCITY = 0.2
        self.MIN_ANGULAR_VELOCITY = 0.0
        self.execution_completed = False


    
    def calculate_command(self):
        if len(self.target_points) == 0:
            print("No More Target Points!")
            self.execution_completed = True 
            return -1
        current_target = self.target_points[0]


        # display section
        print("current_position: \n{}".format(self.current_position))
    
        print("goal_position: \n{}".format(current_target))
        print("goal_index: {}".format(5-len(self.target_points)))

        distance_error = self.euclidean_distance(self.current_position.x,self.current_position.y,current_target.x,current_target.y)
        print("Distance Error: ", distance_error)

        angle_error = math.atan2(current_target.y - self.current_position.y,current_target.x - self.current_position.x) - self.current_position.theta
        if angle_error > math.pi:
            angle_error -= 2*math.pi
        elif angle_error < -math.pi:
            angle_error += 2*math.pi
        print("Angle Error: ", angle_error)

        self.angular_velocity = self.RADIAL_GAIN_CONSTANT * angle_error
        
        if self.new_point:
            print("\nturning\n")
            if abs(angle_error) < self.ANGLE_THRESHOLD:
                self.new_point = False
            else:
                self.linear_velocity = 0
                return
        else:
            print("\nno turn\n")
        
        self._update_target_point(distance_error)
        #linear velocity calculation
        c = 40
        m = 0.1666
        stop_error = 45.0/180*3.14
        pass_error = 15.0/180*3.14
        smart_error = 1
        if abs(angle_error) > stop_error:
            smart_error  = 0
        elif abs(angle_error) < pass_error:
            smart_error = 1
        else:
            smart_error = (stop_error - abs(angle_error))/(stop_error-pass_error)
        # sig_error = 1/(1+math.e**(c*(abs(angle_error)/math.pi-m)))
        # lin_error = 1 - angle_error/math.pi
        
        if abs(angle_error) > self.ANGLE_THRESHOLD:
            self.linear_velocity = self.LINEAR_GAIN_CONSTANT * (distance_error) * (smart_error)
            return

        if not self.next_turn:
            self.linear_velocity = 0.05
        else:
            self.linear_velocity = self.LINEAR_GAIN_CONSTANT * (distance_error) * (smart_error)
            return

        if abs(self.linear_velocity) < self.MIN_LINEAR_VELOCITY:
            self.linear_velocity = np.sign(self.linear_velocity)*self.MIN_LINEAR_VELOCITY
        if abs(self.linear_velocity) > self.MAX_LINEAR_VELOCITY:
            self.linear_velocity = self.MAX_LINEAR_VELOCITY
        

        # return (self.RADIAL_GAIN_CONSTANT * angle_error, self.LINEAR_GAIN_CONSTANT * distance_error)
    
    def update_robot_position(self,position):
        self.current_position.x = position.x
        self.current_position.y = position.y
        self.current_position.theta = position.theta 


    def get_command(self):
        # return (self.angular_velocity,0)
        if self.execution_completed:
            return(0,0)
        else:
            return (self.angular_velocity, self.linear_velocity)

    def update_trajectory(self, target_points):
        for point in target_points:
            self.target_points.append(point)

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

    #make distnace error like angle error