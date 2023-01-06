#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D
from grid3.msg import botPoses, gridGoals, botTrajectories, trajectory, imagePoint, plannerStatus,controllerStatus
from std_msgs.msg import UInt8
from trajectory_planner_v3 import TrajectoryPlanner
from gridIdentifier import utilfunctions
import math

# subscribe to the one of 9 goals
# calculate the trajectories and publish them on the topic

class TrajectoryPlannerRosInterface:
    def __init__(self):
        self.N = 4
        #inputs to the trajectory planenr
        self.positionSubscriber = rospy.Subscriber("grid3/allRobots/poses", botPoses, self.robotCbPoses)
        self.goalSubscriber = rospy.Subscriber("grid3/allRobots/goals", gridGoals, self.robotCbGoals)
        
        #output of the traejctoy planner
        self.trajectoryPublisher = rospy.Publisher("grid3/robotTrajectories", botTrajectories, queue_size=10)
        
        #flags
        # publisher
        self.plannerStatusPublisher = rospy.Publisher("grid3/planner/status", plannerStatus, queue_size=10)
        
        #subscriber
        self.controllerStatusSub = rospy.Subscriber("grid3/controller/status", controllerStatus, self.controllerStatusCb)
        self.controllerStatus = controllerStatus()
        self.controllerStatus.botIds = [i for i in range(self.N)]
        self.controllerStatus.status = [False for i in range(self.N)]

        #initializint the current poses
        self.curPoses= botPoses()
        self.curPoses.ids = [i for i in range(self.N)]
        self.curPoses.xs = [0 for i in range(self.N)]
        self.curPoses.ys = [0 for i in range(self.N)]
        self.curPoses.thetas = [0.0 for i in range(self.N)]
    
        self.oldPose = [(-1,-1)]*4

        # initializing the bot goals
        self.robotGoals = gridGoals()
        self.robotGoals.ids =[i for i in range(4)] 
        self.robotGoals.goals =[0 for i in range(4)] 

        self.oldRobotGoals = gridGoals()
        self.oldRobotGoals.ids =[i for i in range(4)] 
        self.oldRobotGoals.goals =[10 for i in range(4)] 
        # self.planner = TrajectoryPlanner()
        self.botTrajectories = botTrajectories()
        self.plannerOutput = [[] for i in range(self.N)]

        # planner attribute
        self.plannerStatus = plannerStatus()
        self.plannerStatus.botIds = [i for i in range(self.N)]
        self.plannerStatus.status = [1 for _ in range(self.N)]
        # 2 for waiting for path to clear
        # 1 for the robot has changed the destination or induction point
        # 0 for ok and keep moving 
        #util objects
        self.utils = utilfunctions()
        # self.utils.initializeParams()
        self.updateGrid()

    def updateGrid(self):
        self.GRID = [
                [-1]*16,
                [-1]+[0]*14+[-1],
                [-1]+[0]*14+[-1],
                [-1,0,0,-2,-2,0,0,-2,-2,0,0,-2,-2,0,0,-1],
                [-1,0,0,-2,-2,0,0,-2,-2,0,0,-2,-2,0,0,-1],
                [-1]+[0]*14+[0],
                [-1]+[0]*14+[-1],
                [-1,0,0,-2,-2,0,0,-2,-2,0,0,-2,-2,0,0,-1],
                [-1,0,0,-2,-2,0,0,-2,-2,0,0,-2,-2,0,0,-1],
                [-1]+[0]*14+[-1],
                [-1]+[0]*14+[0],
                [-1,0,0,-2,-2,0,0,-2,-2,0,0,-2,-2,0,0,-1],
                [-1,0,0,-2,-2,0,0,-2,-2,0,0,-2,-2,0,0,-1],
                [-1]+[0]*14+[-1],
                [-1]+[0]*14+[-1],
                [-1]*16
            ]
        self.planner = TrajectoryPlanner(grid=self.GRID, robot_start_locations=[(5,15),(10,15),(0,0),(0,0)], robot_directions=['W','W','W','W'])


    def robotCbPoses(self,data):
        for i in range(4):
            self.curPoses.xs[i] = data.xs[i]
            self.curPoses.ys[i] = data.ys[i] 
            self.curPoses.thetas[i] = data.thetas[i] 
        # print("in callback")
        # print(self.curPoses)    

    def robotCbGoals(self, data):
        for i in range(4):
            # if self.data.goals[i] != self.robotGoals.goals[i]:
                # self.oldRobotGoals.ids[i] = self.robotGoals.ids[i]
                # self.oldRobotGoals.goals[i] = self.robotGoals.goals[i]
                self.robotGoals.ids[i] = data.ids[i]
                self.robotGoals.goals[i] = data.goals[i]
        print(self.robotGoals.goals)
        # self.updateTrajectory()
        # self.publishTrajectory()


    def controllerStatusCb(self, data):
        for i in range(self.N):
            self.controllerStatus.botIds[i] = data.botIds[i]
            self.controllerStatus.status[i] = data.status[i]

    
    def updateTrajectory(self):
        #update the trajectory
        #compelete this  part
        for i in range(4):
            tempPose = self.utils.ij2rc(self.curPoses.ys[i],self.curPoses.xs[i])
            tempDir = self.getDirection(self.curPoses.thetas[i])
            # tempDir = 'W'
            # print("new position",tempPose)
            # print("direction",tempDir)
            # print("new direction:" + tempDir)
            #call the trajectory planner function and store the trajectory in the object attribute
            

            if self.replanningRequired(i, tempPose):
            # if self.is_position_changed(i,tempPose, tempDir) or self.plannerStatus.status[i] == 2:
                # print("in replanning:", i)
                self.oldPose[i] = tempPose

                self.planner.force_update_location(i,tempPose,tempDir)
                # print(self.planner.robots[i].current_trajectory)
                # print(self.planner.robots[i].location)
                # print(self.planner.robots[i].orientation)
                print(f"[Planner, {i}]: Replanning Trajectory")
                # print(i,"goal debug",self.planner.dl[self.robotGoals.goals[i]])
                # print(self.planner.robots[i].location)
                tempPlannerOut  = self.planner.find_trajectory(i, self.planner.dl[self.robotGoals.goals[i]])
                # print("planner output :",tempPlannerOut)
                if tempPlannerOut == -1:
                    # no path available, and robot should wait
                    # not reached yet, no paths available
                    print(f"[Planner, {i}]: No paths available!")
                    self.plannerStatus.status[i] = 2
                    self.plannerOutput[i] = []
                elif len(tempPlannerOut) == 0:
                    # publish flag to make robot wait
                    print(f"[Planner, {i}]: Finished trajectory")
                    self.plannerStatus.status[i] = 1
                    self.oldRobotGoals.goals[i] = self.robotGoals.goals[i]
                    self.plannerOutput[i] = []
                else:
                    if len(tempPlannerOut[0]) != 2:
                        self.plannerStatus.status[i] = 2
                    else:
                        self.plannerStatus.status[i] = 0
                    self.plannerOutput[i] = list(tempPlannerOut)
                    # print(self.plannerOutput[i])
                    self.oldRobotGoals.goals[i] = self.robotGoals.goals[i]

                    # print(f"[Planner, {i}] planner status :",self.plannerStatus.status[i])
                    
                    print(f"[Planner, {i}]: New trajectory:\n {self.plannerOutput[i]}")
        self.plannerStatusPublisher.publish(self.plannerStatus)


    def replanningRequired(self,i, tempPose):
        # print(f"[Planner, {i}] is goal changed :",self.is_goal_changed(i))
        # print(f"[Planner, {i}] is position changed:",self.is_position_changed(i, tempPose))
        # print(f"[Debug][Planner, {i}] planner status :",self.plannerStatus.status[i])

        return self.is_goal_changed(i) or self.is_position_changed(i, tempPose) or self.plannerStatus.status[i] == 2
        
        #goal changed OR position changed OR waiting for path to clear
    
    def is_goal_changed(self,i):
        if self.oldRobotGoals.goals[i] != self.robotGoals.goals[i]:
            # self.plannerStatus.status[i] = 0
            print(f"[Planner, {i}]: is Goal changed to {self.robotGoals.goals[i]}")
            return True
        else:
            # print(f"[Debug][Planner, {i}]: is Goal changed:{False}")
            return False

    def is_position_changed(self, i, tempPose):
        if self.oldPose[i] != tempPose:
            print(f"[Planner, {i}]: Position changed to {tempPose}")
            return True
        else:
            # print(f"[Debug][Planner, {i}]: is position changed: {False}")
            return False


    def getDirection(self, angle):
        if angle < math.pi/4 and angle > -math.pi/4:
            return 'E'
        if angle > math.pi/4 and angle < 3*math.pi/4:
            return 'N'
        if angle > 3*math.pi/4 or angle < -3*math.pi/4:
            return 'W'
        if angle < -math.pi/4 and angle > -3*math.pi/4:
            return 'S'

    def publishTrajectory(self):
        self.botTrajectories.botIds = [i for i in range(self.N)]
        self.botTrajectories.trajectories = [trajectory() for i in range(self.N)]

        for i in range(self.N):
            self.botTrajectories.botIds[i] = i
            self.botTrajectories.trajectories[i] = trajectory()
            for rc in self.plannerOutput[i]:
                temp = imagePoint()
                # print(rc)
                [temp.j, temp.i] = self.utils.rc2ij(rc[0],rc[1])
                self.botTrajectories.trajectories[i].points.append(temp)

        self.trajectoryPublisher.publish(self.botTrajectories)



if __name__ == '__main__':
    rospy.init_node("trajectory_planner_ros", anonymous=True)
    myPlanner = TrajectoryPlannerRosInterface()
    print(f"[Planner] Started!")
    rate = rospy.Rate(5)
    # rospy.spin()
    while not rospy.is_shutdown():
        myPlanner.updateTrajectory()
        myPlanner.publishTrajectory()
        rate.sleep()
    rospy.loginfo("ended")

