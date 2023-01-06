#import serial
import math
import numpy as np
from final_function import BotFinder
from shortest_path_input import *
import enum
import cv2
import threading
from collections import deque
import time
import json

class pose2D:
    def __init__(self, x =0,y=0, theta = 0):
        self.x = x
        self.y = y
        self.theta = theta

    def distance(self, p2):
        return math.sqrt((self.x-p2.x)**2 + (self.y-p2.y)**2)

class Command(enum.Enum):
    #right rotate
    Clock_wise = (1,-1)
    #left rotate
    Anti_clock_wise = (-1,1)
    #move forward
    Forward = (1,1)
    #stop
    Stop = (0,0)
    #reached
    Reached = (2,2)

    

class Status(enum.Enum):
    Not_executed = 0
    Executing = 1
    Executed = 2

class bot:
    def __init__(self):
        self.id = str(1010)
        self.pose = pose2D()
        self.goal_pose = pose2D()
        self.command = command()

class controller:
    def __init__(self):
        self.goal = pose2D()
        self.pose = pose2D()
        self.command = Command
        self.angle_threshold = 0.01
        self.distance_threshold = 0.1
        self.controller_mode = 0 #0 for rotate and move, 1 for pid etc...

    def set_goal(self,goal):
        self.goal = goal
    
    def set_pose(self,pose):
        self.pose = pose

    def update_command(self):
        if self.controller_mode == 0:
            pose_theta = self.pose.theta
            goal_theta = math.atan2(self.goal.y - self.pose.y, self.goal.x - self.pose.x) 
            error_theta = goal_theta - pose_theta
            distance_error = self.pose.distance(self.goal)
            print("distance_error is:", distance_error)
            print("angle_error is:", error_theta)
            if error_theta > 2*math.pi:
                error_theta = error_theta - 2*math.pi
            elif error_theta < -2*math.pi:
                error_theta += 2*math.pi

            if abs(error_theta) > self.angle_threshold:     
                if error_theta < 0:
                    self.command = Command.Clock_wise
                else:
                    self.command = Command.Anti_clock_wise

            elif distance_error > self.distance_threshold:
                self.command = Command.Forward
            else:
                self.command = Command.Reached
    
    def get_command(self):
        return self.command

class central_monitor:
    def __init__(self):
        self.serial_communicator = None
        self.bot_identifier = BotFinder()
        self.my_controller = controller()
        self.stage_finder = matrix_builder()
        # self.bots = [bot() for i in range(4)]
        # self.cmaeraFeed
        # self.matrixStage
        self.acknowledgements = [True, True, True,True]
        self.bot_ids = ['0122','2010','2210','2121']
        self.bot_commands = {}
        self.execution_order = {0:'2210',1:'0122',2:'2010',3:'2121'}
        self.status = {self.bot_ids[0]:Status.Not_executed, self.bot_ids[1]:Status.Not_executed, self.bot_ids[2]:Status.Not_executed, self.bot_ids[3]:Status.Not_executed}
        self.end_process = False

        #image
        self.visualization_image = None
        self.camerafeed = None 
        self.point_list = []

    def initialize_monitor(self):
        self.start_serial_communicator('COM3', 115200)
        self.initialize_trajectories()
        #still need to be completed
        pass

    def start_serial_communicator(self,com_port,baudrate):
        self.serial_communicator = serial.Serial(com_port, baudrate)
        

    def update_image(self, image):
        self.camerafeed = image

    def read_image(self,addr):
        self.camerafeed = cv2.imread(addr)


    def initialize_trajectories(self):
        traj4 = deque([(8,9), (7,9), (6,9),(5,9),(4,9),(3,9),(2,9),(1,9), (1,10), (1,11), (1,12), (1,13), (1,14), (1,15)])
        traj3 = deque([(8,8), (7,8), (6,8),(5,8),(4,8),(3,8),(2,8),(1,8),(0,8),(0,9),(0,10), (0,11), (0,12), (0,13), (0,14), (0,15)])
        traj2 = deque([(8,7), (7,7), (6,7),(5,7),(4,7),(3,7),(2,7),(1,7),(0,7),(0,6),(0,5), (0,4), (0,3), (0,2), (0,1), (0,0)])
        traj1 = deque([(8,6), (7,6), (6,6),(5,6),(4,6),(3,9),(2,6),(1,6), (1,5), (1,4), (1,3), (1,2), (1,1), (1,0)])
        traj4.extend(list(traj4)[len(traj4)::-1])
        traj3.extend(list(traj3)[len(traj3)::-1])
        traj2.extend(list(traj2)[len(traj2)::-1])
        traj1.extend(list(traj1)[len(traj1)::-1])

        self.trajectories = {self.bot_ids[0]: traj1, self.bot_ids[1]:traj2, self.bot_ids[2]:traj3, self.bot_ids[3]:traj4}
        self.temp_trajectories = {self.bot_ids[0]: traj1, self.bot_ids[1]:traj2, self.bot_ids[2]:traj3, self.bot_ids[3]:traj4}


    def load_manual_trajectories(self, bot_id):
        with open("trajectory.txt","r") as fp:
            manual_trajectory = json.load(fp)
        # manual_trajectory_length = len(manual_trajectory)

        # for i in range(len(self.trajectories[bot_id])):
        #     if i > manual_trajectory_length  - 1:
        #         x,y = self.trajectories[bot_id][i]
        #         self.matrixStage[x][y] = [0,0,0]
        #     else:
        #         x,y = self.trajectories[bot_id][i]
        #         self.matrixStage[x][y] = manual_trajectory[i]

        self.temp_trajectories[bot_id] = deque(self.point_list)
        

    def click_event(self,event, x, y, flags, params):

        # checking for left mouse clicks
        if event == cv2.EVENT_LBUTTONDOWN:

            # displaying the coordinates
            # on the Shell
            print(x, ' ', y)    
            # displaying the coordinates
            # on the image window
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(self.camerafeed, str(x) + ',' +str(y), (x,y), font,1, (255, 0, 0), 2)
            self.point_list += [(x,y,0)]
            cv2.imshow('image', self.camerafeed)


    def set_manual_trajectories(self):
        # displaying the image
        cv2.imshow('image', self.camerafeed)
        # setting mouse hadler for the image
        # and calling the click_event() function
        cv2.setMouseCallback('image', self.click_event)
        # wait for a key to be pressed to exit
        cv2.waitKey(0)
        # close the window
        print(self.point_list)
        with open("trajectory.txt","w") as fp:
            json.dump(self.point_list, fp)
        with open("trajectory.txt","r") as fp:
            temp = json.load(fp)

        cv2.destroyAllWindows()
        print("read data is:",temp)

    #position functions
    def update_robot_poses(self, image_feed):
        self.bot_identifier.update_image(image_feed)
        self.bot_identifier.detect_bots()
        self.bots = self.bot_identifier.get_bot_poses()

    def visualize(self):
        # draw circles on idntified robots
        # represent the orientation of the robots with the help of a stick
        # draw a arrow to goal
        # draw a trajectory
        # display command and id

        self.visualization_image  = self.camerafeed
        # drawing trajectory
        self.show_trajectories()
        self.show_bots()
        self.show_lines_to_goal()

        # self.visualization_image = cv2.resize(self.visualization_image, (0,0),fx = 0.5, fy = 0.5)
        cv2.imshow('visualization', self.visualization_image)
        cv2.waitKey(0)

    def show_trajectories(self):
        for i in self.bot_ids:
            if self.status[i] == Status.Executing:
                self.show_trajectory(i)

    def show_trajectory(self, bot_id):
        color = (0,255,0)
        thickness = 5
        temp_trajectory = self.trajectories[bot_id]
        start_point = temp_trajectory.popleft()

        for present_point in temp_trajectory:
            #temp = (self.matrixStage[start_point[0]][start_point[1]])    
            start_xy = (int(start_point[0]),int(start_point[1]))    

            #temp = (self.matrixStage[present_point[0]][present_point[1]])    
            present_xy = (int(present_point[0]),int(present_point[1]))   

            cv2.line(self.visualization_image,start_xy,present_xy,color, thickness)

            start_point  = present_point
        
        self.show_goal(bot_id)
    
    def show_goal(self, bot_id):
        # goal = self.get_bot_goal(bot_id)
        goal = self.get_temp_bot_goal(bot_id)

        print("this is the goal",goal)
        cv2.circle(self.visualization_image,(int(goal[0]), int(goal[1])), radius = 40, color = (0,0,0), thickness = 2)


    def show_bots(self):
        for i in self.bot_ids:
                self.show_bot(i)
    
    def show_bot(self, bot_id):
        bot_xytheta = self.bots[bot_id]
        xy_start = (int(bot_xytheta[1]), int(bot_xytheta[0]))
        theta = math.radians(bot_xytheta[2])

        #parameters
        line_length = 100 #20 pixels
        thickness = 5
        color1 = (0,0,255)
        color2 = (255,0,0)
        radius = 50 # 10 pixels
        offset_x = 50
        offset_y = 50
        org1 = (xy_start[0] - offset_x, xy_start[1] + offset_y)
        org2 = (xy_start[0] + offset_x, xy_start[1] + offset_y)
        org3 = (xy_start[0] - offset_x, xy_start[1] + offset_y+ 30)
        print(math.cos(theta), math.sin(theta), theta)
        xy_end = (int(xy_start[0] + line_length*math.cos(theta)),int(xy_start[1] - line_length*math.sin(theta)))
        status_value = self.status[bot_id].value
        # print("status value is ",status_value)
        if status_value == 0:
            status_text = "Not_executed"
        elif status_value == 1:
            status_text = "executing"
        else:
            status_text = "executed"
        #command text
        temp_command = self.bot_commands[bot_id]
        if temp_command == Command.Forward:
            command_text = "Forward"
        if temp_command == Command.Stop:
            command_text = "Stop"
        if temp_command == Command.Clock_wise:
            command_text = "Clock_wise"
        if temp_command == Command.Anti_clock_wise:
            command_text = "Anti_clock_wise"
    

        # print(xy_start, xy_end )
        cv2.line(self.visualization_image,xy_start, xy_end,color1,thickness = 2)
        cv2.circle(self.visualization_image, xy_start, radius = radius, color = color1, thickness = 3)
        cv2.putText(self.visualization_image, str(bot_id),  org1, cv2.FONT_HERSHEY_SIMPLEX, 1, color2, thickness= 2)
        cv2.putText(self.visualization_image, status_text, org2, cv2.FONT_HERSHEY_SIMPLEX,1 ,color2, thickness = 2)
        cv2.putText(self.visualization_image, command_text, org3, cv2.FONT_HERSHEY_SIMPLEX,1 , color2, thickness = 2 )


    def show_lines_to_goal(self):
        for i in self.bot_ids:
            if self.status[i] == Status.Executing:
                self.show_line_to_goal(i)

    def show_line_to_goal1(self, bot_id):
        goal = (self.trajectories[bot_id][0])
        print("goal is", goal)
        goal_xy = (int(self.matrixStage[goal[0]][goal[1]][0]),int(self.matrixStage[goal[0]][goal[1]][1]))
        print("goal indices are", goal_xy)
        start_xy = (int(self.bots[bot_id][1]), int(self.bots[bot_id][0]))
        print("start indices are", start_xy)

        cv2.line(self.visualization_image, start_xy, goal_xy, color = (255,0,0), thickness = 5)
    
    def show_line_to_goal(self, bot_id):
        goal = (self.temp_trajectories[bot_id][0])
        print("goal is", goal)
        #goal_xy = (int(self.matrixStage[goal[0]][goal[1]][0]),int(self.matrixStage[goal[0]][goal[1]][1]))
        goal_xy = (goal[0],goal[1])
        print("goal indices are", goal_xy)
        start_xy = (int(self.bots[bot_id][1]), int(self.bots[bot_id][0]))
        print("start indices are", start_xy)

        cv2.line(self.visualization_image, start_xy, goal_xy, color = (255,0,0), thickness = 5)

    #executed once at the start of the process
    def update_stage_coordinates(self, image):
        self.stage_finder.update_image(image)
        self.stage_finder.detect_grid()
        self.matrixStage = self.stage_finder.get_matrix()

    def get_bot_pose(self, bot_id):
        print(self.bots)
        return(self.bots[bot_id])

    #command functions
    def get_bot_goal(self,bot_id):
        traj = self.trajectories[bot_id]
        (i,j) = traj[0] #use queue and dequeue logic
 
        (x,y,_) = self.matrixStage[i][j]
        # print(bot_id, (x,y))
        return (x,y)
    
    def get_temp_bot_goal(self, bot_id):
        traj = self.temp_trajectories[bot_id]
        print(traj[0])
        print(traj)
        (x,y,_) = traj[0] #use queue and dequeue logic
        # print(bot_id, (x,y))
        return (x,y)
        
    #calculate the commands for bots
    def get_bot_command(self,bot_id, bot_pose):
        #check if its executed if yess return (0,0)
        print("computing for bot:", bot_id)
        if self.status[bot_id] == Status.Executed:
            print((bot_id,"execution complete"))
            return Command.Stop
        
        #if its not executed check if previous robots have completed there execution
        elif self.status[bot_id] == Status.Not_executed:
            for i in range(4):
                print("status value of the ith robot",self.status[self.execution_order[i]] )
                print(self.execution_order[i], bot_id)
                if self.execution_order[i] == bot_id:
                    break
                if self.status[self.execution_order[i]].value != Status.Executed.value:
                    print(bot_id,"not a correct time to execute trajectory returning 0\n\n")
                    return Command.Stop

        self.status[bot_id] = Status.Executing
        print("Executing bot is ", bot_id, "\n\n")
        # print(bot_id,"executing trajectory")
        #if it is turn of bot_id to perform execution then compute the control command
        goal = self.get_temp_bot_goal(bot_id)
        # goal = self.get_bot_goal(bot_id)

        bot_pose = self.get_bot_pose(bot_id)
        theta = bot_pose[2]
        self.my_controller.set_goal(pose2D(goal[0],goal[1]))
        self.my_controller.set_pose(pose2D(bot_pose[0],bot_pose[1],bot_pose[2]))
        self.my_controller.update_command()
        cmd = self.my_controller.get_command()
        if cmd == Command.Reached:
            if len(self.trajectories[bot_id]) == 0:
                #execution completed
                #set the status variable to executed
                #return (0,0) to stop the wheels
                self.status[bot_id] = Status.Executed
                return (0,0)
            # self.trajectories[bot_id].popleft()
            self.temp_trajectories[bot_id].popleft()
            cmd = Command.Stop

        return cmd

    #update the commands in class variable
    def update_bot_commands(self):
        for i in range(4):
            # print(i)
            bot_goal = self.get_temp_bot_goal(self.bot_ids[i])
            # bot_goal = self.get_bot_goal(self.bot_ids[i])

            self.bot_commands[self.bot_ids[i]] = self.get_bot_command(self.bot_ids[i], self.get_bot_pose(self.bot_ids[i]))

    def send_msg(self):
        self.serial_communicator.write(self.msg)
        print("{}: msg sent".format(self.msg))
        
    def send_msgs_thread(self):
        print("in the send message thread\n")
        while True:
            if self.end_process:
                print("ending the sender thread")
                return
            self.wait_for_msgs()
            print("waiting completed sending msgs")
            self.send_msg()
    
    def pack_commands(self):
        # commmad strucutres
        # "11213241"
        bot_ids = self.bot_commands.keys()
        msg = ""
        default_msg = "11213141"
        count = 1
        for i in bot_ids:
            msg +=str(count)
            temp_cmd = 1
            if self.bot_commands[i] == Command.Stop:
                temp_cmd = 1
            if self.bot_commands[i] == Command.Clock_wise:
                temp_cmd = 2
            if self.bot_commands[i] == Command.Anti_clock_wise:
                temp_cmd = 3
            if self.bot_commands[i] == Command.Forward:
                temp_cmd = 4
            msg +=(str(temp_cmd))
        if len(msg) < 8:
            print("incorrect message generate :({}) ".format(msg))
            self.msg = default_msg
            return
        # msg += "\n"
        self.msg = bytes(msg,'utf-8')
    
    def wait_for_msgs(self):
        print("waiting for msg from arduino")
        ack1 = self.serial_communicator.readline()
        print("recieved message is {}".format(ack1))


test = True #for video
main = False #main block

if test:
    start_time = time.time()
    url = ""
    cap = cv2.VideoCapture(1)

    my_monitor = central_monitor()

    ret,temp_image = cap.read()
    cv2.imshow("cam",temp_image)
    cv2.waitKey(0)
    #temp_image = cv2.rotate(temp_image, cv2.cv2.ROTATE_90_COUNTERCLOCKWISE)
    # temp_image = cv2.resize(temp_image, (0,0),fx = 0.5, fy = 0.5)

    my_monitor.camerafeed = temp_image
    #my_monitor.update_stage_coordinates(temp_image)
    my_monitor.initialize_trajectories()
    print("length of the trajectory of 2210 is:", len(my_monitor.trajectories['2210']))

    #resizing so that we could set manual trajectory, brcause orginal size is more than the screen size.
    #temp_image = cv2.resize(temp_image, (0,0),fx = 0.5, fy = 0.5)
    my_monitor.camerafeed = temp_image

    #setting test flags
    test_trajectories = True
    test_manual_trajectories = True

    if test_manual_trajectories:
        my_monitor.set_manual_trajectories()
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        my_monitor.load_manual_trajectories('2010')#loads mual trajectories for bot wit this id
        my_monitor.load_manual_trajectories('2121')#loads mual trajectories for bot wit this id
        my_monitor.load_manual_trajectories('2210')#loads mual trajectories for bot wit this id
        my_monitor.load_manual_trajectories('0122')#loads mual trajectories for bot wit this id


    if test_trajectories:
        my_monitor.status['2210'] = Status.Executing
        my_monitor.visualization_image = temp_image
        my_monitor.show_trajectory('2210')
        # my_monitor.visualization_image = cv2.resize(my_monitor.visualization_image, (0,0),fx = 0.5, fy = 0.5)
        cv2.imshow('visualization', my_monitor.visualization_image)
        cv2.waitKey(0)

    while True:
        print('start of loop')
        # present_time = time.time()
        # delta_time  = present_time - start_time + offset_time
        ret, frame = cap.read()
        #frame = cv2.rotate(frame, cv2.cv2.ROTATE_90_COUNTERCLOCKWISE)
        #frame = cv2.resize(frame, (0,0),fx = 0.5, fy = 0.5)
        my_monitor.update_image(frame)
        my_monitor.update_robot_poses(my_monitor.camerafeed)
        print(my_monitor.bots)
        my_monitor.update_bot_commands()
        # print(my_monitor.bot_commands)
        my_monitor.pack_commands()
        # print(my_monitor.bot_commands)
        my_monitor.visualize()

# if __name__=='__main__':
if main:
    # initialize monitor class
    stage_image_path = ""
    stage_image = cv2.imread(stage_image_path)
    
    my_monitor = central_monitor()
    my_monitor.update_stage_coordinates(stage_image)
    my_monitor.initialize_trajectories()
    my_monitor.start_serial_communicator(4, 9600)

    sender_thread = threading.Thread(target=my_monitor.send_msgs_thread)
    sender_thread.start()
    
    while(True):
        try:
            my_monitor.update_image() #camera feed
            my_monitor.update_robot_poses()
            my_monitor.update_bot_commands()
            my_monitor.pack_commands()
        except KeyboardInterrupt:
            my_monitor.end_process = True
            time.sleep(0.1)
            sender_thread.join()
            break
        else:
            my_monitor.end_process = True
            time.sleep(0.1)
            sender_thread.join()            






