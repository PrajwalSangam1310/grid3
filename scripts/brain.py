#!/usr/bin/env python3

from collections import deque
from typing import List, Union
import rospy
from grid3.msg import commenceDropping, plannerStatus, botPoses, gridGoals, packageLoaded
from gridIdentifier import utilfunctions
import time

class Brain:
    '''
    This class gives high level instructions to other components.
    '''
    
    def __init__(self, number_of_robots, package_list_0, package_list_1, pickup_points = ((5,15),(10,15))) -> None:
        self.number_of_robots: int = number_of_robots
        self.package_list = [deque(package_list_0), deque(package_list_1)]
        self.pickup_points = pickup_points
        self.pickup_point_availablity = [True] * len(pickup_points)
        self.robots: List[Robot] = [] 
        self.utilFunctions = utilfunctions()
        self.N = 4
        for index in range(self.number_of_robots):
            self.robots.append(
                Robot()
            )
        # Trajectory Status Variable
        self.planner_status = plannerStatus()
        self.planner_status.botIds = [i for i in range(4)]
        self.planner_status.status = [1 for _ in range(4)] 

        self.curPoses= botPoses()
        self.curPoses.ids = [i for i in range(self.N)]
        self.curPoses.xs = [0 for i in range(self.N)]
        self.curPoses.ys = [0 for i in range(self.N)]
        self.curPoses.thetas = [0.0 for i in range(self.N)]

        self.positionSubscriber = rospy.Subscriber("grid3/allRobots/poses", botPoses, self.robotCbPoses)
        self.packageLodedSubscriber = rospy.Subscriber("grid3/packageLoaded", packageLoaded, self.packageLoadedSubscriberCb)
        self.planner_status_subscriber = rospy.Subscriber("grid3/planner/status", plannerStatus, self.robotPlannerStatusCb)

        self.commenceDropping = commenceDropping()
        self.commenceDropping.status = [False]*4
        self.droppingStartTimes = [0.0 for i in range(4)]
        self.packageLoaded = packageLoaded()
        self.packageLoaded.status = [False]*4

        self.commenceDroppingPub = rospy.Publisher("grid3/commenceDropping", commenceDropping, queue_size=10)

        self.gridGoals = gridGoals()
        self.gridGoals.ids = [i for i in range(4)]
        self.gridGoals.goals = [10 for i in range(4)]
        self.goalPublisher = rospy.Publisher("grid3/allRobots/goals", gridGoals, queue_size=10)      
    
        self.started = [False]*4
        self.droppingPeriod = 10

        self.des_names = ['Mumbai', 'Delhi', 'Kolkata', 'Chennai','Bengaluru','Hyderbad','Pune','Ahemdabad','Jaipur']
        

    def publishTrajectoryPlanningCommand(self, robot_index, location):
        '''
        Should publish the new location for the bot to trajectory planner to start planning for robot_index to goto location
        '''
        # change the goal
        if(location != 10):
            if location != 0:
                print(f"[Brain, {robot_index}]: Publishing new goal at {location}. {self.des_names[location-1]}")
        else:
            print(f"[Brain, {robot_index}]: Returning to induction point!")
        self.gridGoals.goals[robot_index] = location
        self.goalPublisher.publish(self.gridGoals)

    def publishDropPackageCommand(self, robot_index, flag):
        '''
        Should publish command to communicator to drop package
        '''
        self.commenceDropping.status[robot_index] = flag
        self.commenceDroppingPub.publish(self.commenceDropping)
        print(f"[Brain, {i}]: Published Drop command")

    def get_current_location(self, robot_index):
        '''
        Get the i,j value of robot_index and convert to rc and then return
        '''
        return self.utilFunctions.ij2rc(self.curPoses.ys[robot_index], self.curPoses.xs[robot_index])
        

    def robotCbPoses(self,data):
        for i in range(4):
            self.curPoses.xs[i] = data.xs[i]
            self.curPoses.ys[i] = data.ys[i] 
            self.curPoses.thetas[i] = data.thetas[i] 

    def robotPlannerStatusCb(self, data):
        for i in range(4):
            self.planner_status.status[i] = data.status[i]

    def packageLoadedSubscriberCb(self, data):
        self.packageLoaded.status = data.status

    def at_pickup_point(self, robot_index):
        robot_location = self.get_current_location(robot_index)
        for i in range(len(self.pickup_points)):
            if self.pickup_points[i] == robot_location:
                return True
        return False
    
    def get_pickup_point_number(self, robot_index):
        robot_location = self.get_current_location(robot_index)
        for i in range(len(self.pickup_points)):
            if self.pickup_points[i] == robot_location:
                return i+1
        return 0

    def next_goal_point(self, i):
        if self.planner_status.status[i] == 0 or self.planner_status.status[i] == 2:
            # No additional commands need to given to planner or communicator
            return
        if self.robots[i].isDropping:

            if time.time() - self.droppingPeriod > self.droppingStartTimes[i]:
                print(f"[Brain, {i}]: Dropping completed!")
                self.robots[i].isDropping = False
                self.robots[i].package_loaded = False
                self.commenceDropping.status[i] = False
                self.publishDropPackageCommand(i, False)
                # allot_new_pickup_point
                self.publishTrajectoryPlanningCommand(i, 10)
            else:
                #dropping in progress
                print(f"[Brain, {i}]: Dropping in progress!")
                return
                
        elif not self.at_pickup_point(i):
            # Robot has reached drop location
            print(f"[Brain, {i}]: At dropping point...")
            self.publishDropPackageCommand(i, True)
            self.robots[i].isDropping =  True
            self.droppingStartTimes[i] = time.time()
            
        else:
            print(f"[Brain, {i}]:At pickup point...")
            if self.is_loaded(i):
                print(f"[Brain, {i}]: Loaded")
                self.assign_new_goal(i)
            # Robot has reached pickup point
            # Do nothing wait for loading to occur
    
    def is_loaded(self, i ):
        '''
        Returns whether the robot i has been loaded
        '''
        if self.packageLoaded.status[i]:
            return True
        else:
            return False
    
    def assign_new_goal(self, i):
        # Get current pickup point
        pickup_point = self.get_pickup_point_number(i)
        # Check corresponding package list for new goal
        # print(pickup_point)

        if pickup_point != 0:
            if self.package_list[pickup_point- 1]:
                new_goal_point = self.package_list[pickup_point-1].popleft()
                self.publishTrajectoryPlanningCommand(i, new_goal_point)
                self.robots[i].allotedGoalPoint = new_goal_point
                return True
            else:
                print(f"[Brain, {i}]: No goals to assign!")
                return False
        return False

        


class Robot:
    def __init__(self):
        self.package_loaded: bool = False
        self.allotedGoalPoint: Union[bool, int] = False
        self.isDropping: bool = False

if __name__ == '__main__':
    rospy.init_node("brain", anonymous=True)
    # packages = [9,9,9,8,9,3,2,1,6]
    induction_packages_0 = [2,3,9,8,1,9,7,5,7,8,1,8,6,4,8,7,7,7,8,4,5,1,8,2,8,6,1,3,6,8,9,1,4,7,1,6,6,8,7,3,2,8,7,2,1,1,1,8,7,2,5,9,6,5,5,8,7,2,4,8,9,4,5,5,3,9,4,6,8,8,8,3,4,7,8,7,4,6,1,4,3,5,3,4,6,2,2,7,7,8,9,2,3,2,1,8,7,1,3,7,7,9,2,7,7,4,8,9,3,5,9,8,9,6,2,6,2,5,2,4,2,6,8,6,3,1,5,2,5,5,2,9,3,6,6,5,5,7,2,8,7]
    induction_packages_1 = [2,5,7,8,8,5,2,6,4,6,2,4,5,5,2,5,5,4,6,9,8,1,9,4,6,8,2,2,9,7,2,7,4,9,4,1,9,6,9,5,4,3,5,5,3,6,4,7,8,8,9,6,4,4,6,7,3,9,6,5,3,9,9,8,7,8,4,9,6,4,2,1,8,1,9,1,3,8,2,1,1,2,1,2,5,8,9,3,6,3,6,3,6,7,6,5,5,8,3,3,8,1,2,9,9,5,5,1,5,6,8,5,3,4,2,4,2,1,8,7,9,5,5,4,1,8,3,6,7,3,3,3,4,6,7,8,1,9,9,8,6,1,2,9,9,5,2,3,7,4,5,6,5,2,5,5,8,9,2]
    brain = Brain(4, package_list_0=induction_packages_0, package_list_1=induction_packages_1)
    print("[Brain] Started")
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        for i in range(4):
            #if brain.keepWorking()
            # continue
            if not brain.started[i]:
                if brain.is_loaded(i):
                    if brain.assign_new_goal(i):
                        print(f"[Brain, {i}] Initialized with package!") 
                        brain.started[i] = True
                    else:
                        # brain not started
                        continue
            else:
                brain.next_goal_point(i)

        r.sleep()