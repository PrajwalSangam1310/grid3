import serial
import math
import numpy as np
# from final_function import BotFinder
# from shortest_path_input import *
import enum
import cv2
import threading
from collections import deque
import time
import json
import datetime 
import multiprocessing
from Smarth_CNN.shape_identifier import ShapeIdentifier
# from angle_only import BotFinder as bf2
from all_in_one import BotFinder as bf2
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
    #dump action
    Dump  = (3,3)

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
        self.angle_threshold = 0.5
        self.distance_threshold = 100
        self.controller_mode = 0 #0 for rotate and move, 1 for pid etc...

    def set_goal(self,goal):
        self.goal = goal
    
    def set_pose(self,pose):
        self.pose = pose

    def update_command(self):
        if self.controller_mode == 0:
            pose_theta = math.radians(self.pose.theta)
            goal_theta = math.atan2(-self.goal.y + self.pose.y, self.goal.x - self.pose.x)# based on the angle for image the signs are changed 
            error_theta = goal_theta - pose_theta
            distance_error = self.pose.distance(self.goal)
            print("bot poistion is:", self.pose.x, self.pose.y)
            print("goal poistion is:", self.goal.x, self.goal.y)
            print("bot_angle is:", pose_theta)
            print("goal_angle is:", goal_theta)
            print("distance_error is:", distance_error)
            if error_theta > math.pi:
                error_theta = error_theta - 2*math.pi
            elif error_theta < -math.pi:
                error_theta += 2*math.pi
            print("angle_error is:", error_theta)
            command_text = "Nun"
            if abs(error_theta) > self.angle_threshold:     
                if error_theta < 0:
                    self.command = Command.Clock_wise
                    command_text = 'clock_wise'
                else:
                    self.command = Command.Anti_clock_wise
                    command_text = 'Anti_clock_wise'


            elif distance_error > self.distance_threshold:
                self.command = Command.Forward
                command_text = 'Forward'

            else:
                self.command = Command.Reached
                command_text = 'Reached'
            
            print(f"Command generated is: {command_text}")
    
    def get_command(self):
        return self.command

class central_monitor:
    def __init__(self):
        self.serial_communicator = None
        self.bot_identifier = bf2()
        # self.simple_finder = bf2()
        self.my_controller = controller()
        # self.stage_finder = matrix_builder()
        self.time_in_string = ''
        # self.bots = [bot() for i in range(4)]
        # self.cmaeraFeed
        # self.matrixStage
        self.acknowledgements = [True, True, True,True]
        self.bot_ids = ['0122','2010','2210','2121']
        self.debug_bot_ids = []
        self.bot_commands = {'0122':Command.Stop,'2010':Command.Stop,'2210':Command.Stop,'2121':Command.Stop}
        self.bots = {'0122':[0,0,0],'2010':[0,0,0],'2210':[0,0,0],'2121':[0,0,0]}
        self.execution_order = {0:'2210',1:'0122',2:'2010',3:'2121'}
        self.status = {self.bot_ids[0]:Status.Not_executed, self.bot_ids[1]:Status.Not_executed, self.bot_ids[2]:Status.Not_executed, self.bot_ids[3]:Status.Not_executed}
        self.end_process = False

        #image
        self.visualization_image = None
        self.camerafeed = None 
        self.point_list = {}

    def initialize_monitor(self):
        self.start_serial_communicator('COM3', 9600)
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
            manual_trajectories = json.load(fp)
        # manual_trajectory_length = len(manual_trajectory)

        # for i in range(len(self.trajectories[bot_id])):
        #     if i > manual_trajectory_length  - 1:
        #         x,y = self.trajectories[bot_id][i]
        #         self.matrixStage[x][y] = [0,0,0]
        #     else:
        #         x,y = self.trajectories[bot_id][i]
        #         self.matrixStage[x][y] = manual_trajectory[i]
        print(manual_trajectories)
        self.temp_trajectories[bot_id] = deque(manual_trajectories[bot_id])
        
    def click_event(self,event, x, y, flags, params):

        # checking for left mouse clicks
        if event == cv2.EVENT_LBUTTONDOWN:
            # print(params[0])
            bot_id = params[0]
            # displaying the coordinates
            # on the Shell
            print(x, ' ', y)    
            # displaying the coordinates
            # on the image window
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(self.visualization_image, str(x) + ',' +str(y), (x,y), font,0.5, (255, 0, 0), 1)
            if bot_id in self.point_list.keys():
                self.point_list[bot_id] += [(2*x,2*y,0)]
            else:
                self.point_list[bot_id] = [(2*x,2*y,0)]
            cv2.imshow('image', self.visualization_image)
        if event == cv2.EVENT_RBUTTONDOWN:
            # print(params[0])
            bot_id = params[0]
            # displaying the coordinates
            # on the Shell
            print(x, ' ', y)    
            # displaying the coordinates
            # on the image window
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(self.visualization_image, str(x) + ',' +str(y), (x,y), font,0.5, (0, 255, 0), 1)
            if bot_id in self.point_list.keys():
                self.point_list[bot_id] += [(2*x,2*y,1)]
            else:
                self.point_list[bot_id] = [(2*x,2*y,1)]
            cv2.imshow('image', self.visualization_image)

    def set_manual_trajectories(self):
        self.point_dict = {}
        self.camerafeed = cv2.resize(self.camerafeed, (0,0),fx = 0.5, fy = 0.5)
        bot_ids = ['2210', '0122', '2010', '2121']
        for i in range(4):
            self.visualization_image = self.camerafeed.copy()
            cv2.putText(self.visualization_image, 'set trajectory for bot_id:' + bot_ids[i], (200,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),thickness = 2)
            cv2.imshow('image', self.visualization_image)
            cv2.setMouseCallback('image', self.click_event, param = [bot_ids[i]])
            if cv2.waitKey(0) == ord('q'):
                # self.point_dict[bot_ids[i]] = self.point_list
                cv2.destroyAllWindows()
                # cv2.imshow('set_trajectory', self.visualization_image)
        with open("trajectory.txt","w") as fp:
            json.dump(self.point_list, fp)
        with open("trajectory.txt","r") as fp:
            temp = json.load(fp)
        print("read data is:",temp)
        cv2.destroyAllWindows()

    #position functions
    def update_robot_poses_original(self, image_feed):
        self.bot_identifier.update_image(image_feed)
        self.bot_identifier.detect_bots()
        self.bots = self.bot_identifier.get_bot_poses()

    def update_robot_poses(self, image_feed):
        self.bot_identifier.give_frame(image_feed)
        temp_bots = self.bot_identifier.ret()
        print("identified bots are:", temp_bots)
        temp_keys = list(temp_bots.keys())
        key = temp_keys[0]
        temp_bots['2210'] = temp_bots[key]
        print('new temp bots:', temp_bots)
        self.debug_bot_ids = []
        for key in temp_bots.keys():
            if key in self.bots:
                self.debug_bot_ids.append(key)
                for i in range(len(temp_bots[key])):
                    self.bots[key][i] = temp_bots[key][i]
    
    def visualize(self):
        # draw circles on idntified robots
        # represent the orientation of the robots with the help of a stick
        # draw a arrow to goal
        # draw a trajectory
        # display command and id
        self.visualization_image  = self.camerafeed.copy()
        # drawing trajectory
        self.show_trajectories()
        self.show_bots()
        self.show_lines_to_goal()
        self.show_time()
        self.visualization_image = cv2.resize(self.visualization_image, (0,0),fx = 0.5, fy = 0.5)
        # cv2.imshow('visualization', self.visualization_image)
        # cv2.waitKey(0)

    def visualize_thread(self, image):
        count = 0
        while True:
            count += 1
            if self.end_process:
                break
            # print("test_count is",self.test_count)
            # temp = self.visualization_image.copy()
            temp_image = image
            cv2.imshow('visualization Thread', temp_image)
            cv2.waitKey(100)

    def show_time(self):
        org = [100,100]
        # print(my_monitor.time_in_string,"\n\n\n")
        cv2.putText(self.visualization_image, self.time_in_string,  org, cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255), thickness= 2)

    def show_trajectories(self):
        for i in self.bot_ids:
            if self.status[i] == Status.Executing:
                self.show_trajectory(i)
        # self.show_trajectory('2210')

    def show_trajectory(self, bot_id):
        color = (0,255,0)
        thickness = 2
        temp_trajectory = deque(self.temp_trajectories[bot_id])
        # print(bot_id,temp_trajectory)
        start_point = temp_trajectory.popleft()
        for present_point in temp_trajectory:
            #temp = (self.matrixStage[start_point[0]][start_point[1]])    
            start_xy = (int(start_point[0]),int(start_point[1]))    

            #temp = (self.matrixStage[present_point[0]][present_point[1]])    
            present_xy = (int(present_point[0]),int(present_point[1]))   

            cv2.line(self.visualization_image,start_xy,present_xy,color, thickness)
            if present_point[2] == 1:
                cv2.circle(self.visualization_image, present_xy, radius = 20, color = (0,0,255), thickness = -1)


            start_point  = present_point
        
        self.show_goal(bot_id)
    
    def show_goal(self, bot_id):
        # goal = self.get_bot_goal(bot_id)
        goal = self.get_temp_bot_goal(bot_id)

        # print("this is the goal",goal)
        cv2.circle(self.visualization_image,(int(goal[0]), int(goal[1])), radius = 20, color = (0,0,0), thickness = 1)

    def show_bots(self):
        for i in self.bot_ids:
                self.show_bot(i)
    
    def show_bot(self, bot_id):
        bot_xytheta = self.bots[bot_id]
        # print(bot_xytheta)
        xy_start = (int(bot_xytheta[1]), int(bot_xytheta[0]))
        theta = math.radians(bot_xytheta[2])
        theta_degree =  bot_xytheta[2] 
        if bot_id in self.debug_bot_ids:
            flag = True
        else:
            flag = False
        #parameters
        line_length = 60 #20 pixels
        thickness = 5
        #bgr format
        color1 = (0,0,255)
        color2 = (255,0,0)
        if flag:
            color3 = (0,255,0)
        else:
            color3 = (0,0,255)

        radius = 40 # 10 pixels
        offset_x = 50
        offset_y = 40
        org1 = (xy_start[0] - offset_x, xy_start[1] + offset_y)
        org2 = (xy_start[0] + offset_x, xy_start[1] + offset_y)
        org3 = (xy_start[0] + offset_x, xy_start[1] + offset_y+ 40)
        org4 = (xy_start[0] - offset_x, xy_start[1] + offset_y+ 50)

        
        # print(math.cos(theta), math.sin(theta), theta)
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
        elif temp_command == Command.Stop:
            command_text = "Stop"
        elif temp_command == Command.Clock_wise:
            command_text = "Clock_wise"
        elif temp_command == Command.Anti_clock_wise:
            command_text = "Anti_clock_wise"
        elif temp_command == Command.Reached:
            command_text = "Reached"
        elif temp_command == Command.Dump:
            command_text = "Dumping"
        else:
            print(temp_command, "is the reading", bot_id)
            command_text = 'error'
    

        # print(xy_start, xy_end )
        cv2.line(self.visualization_image,xy_start, xy_end,color1,thickness = 2)
        cv2.circle(self.visualization_image, xy_start, radius = radius, color = color1, thickness = 2)
        cv2.putText(self.visualization_image, str(bot_id),  org1, cv2.FONT_HERSHEY_SIMPLEX, 1, color3, thickness= 2)
        cv2.putText(self.visualization_image, status_text, org2, cv2.FONT_HERSHEY_SIMPLEX,1 ,color2, thickness = 2)
        cv2.putText(self.visualization_image, command_text, org3, cv2.FONT_HERSHEY_SIMPLEX,1 , color2, thickness = 2)
        # cv2.putText(self.visualization_image, str(theta_degree), org4, cv2.FONT_HERSHEY_SIMPLEX,0.5 , color2, thickness = 1)

    def show_lines_to_goal(self):
        for i in self.bot_ids:
            if self.status[i] == Status.Executing:
                self.show_line_to_goal(i)

    def show_line_to_goal1(self, bot_id):
        goal = (self.trajectories[bot_id][0])
        # print("goal is", goal)
        goal_xy = (int(self.matrixStage[goal[0]][goal[1]][0]),int(self.matrixStage[goal[0]][goal[1]][1]))
        # print("goal points are", goal_xy)
        start_xy = (int(self.bots[bot_id][1]), int(self.bots[bot_id][0]))
        # print("start start are", start_xy)

        cv2.line(self.visualization_image, start_xy, goal_xy, color = (255,0,0), thickness = 1)
    
    def show_line_to_goal(self, bot_id):
        goal = (self.temp_trajectories[bot_id][0])
        # print("goal is", goal)
        #goal_xy = (int(self.matrixStage[goal[0]][goal[1]][0]),int(self.matrixStage[goal[0]][goal[1]][1]))
        goal_xy = (goal[0], goal[1])
        # print("goal indices are", goal_xy)
        start_xy = (int(self.bots[bot_id][1]), int(self.bots[bot_id][0]))
        # print("start indices are", start_xy)

        cv2.line(self.visualization_image, start_xy, goal_xy, color = (255,0,0), thickness = 2)

    #executed once at the start of the process
    def update_stage_coordinates(self, image):
        self.stage_finder.update_image(image)
        self.stage_finder.detect_grid()
        self.matrixStage = self.stage_finder.get_matrix()

    def get_bot_pose(self, bot_id):
        # print(self.bots)
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
        # print("goal is:",traj[0])
        # print(traj)
        (x,y,_) = traj[0] #use queue and dequeue logic
        # print(bot_id, (x,y))
        return (x,y)
        
    #calculate the commands for bots
    def get_bot_command(self,bot_id, bot_pose):
        #check if its executed if yess return (0,0)
        # print("in get command function")
        print("Computing command for bot:", bot_id)
        if self.status[bot_id] == Status.Executed:
            print((bot_id,"has completed its execution"))
            return Command.Stop
        
        #if its not executed check if previous robots have completed there execution
        elif self.status[bot_id] == Status.Not_executed:
            for i in range(4):
                # print(f"Status of the bot {self.execution_order[i]} is {self.status[self.execution_order[i]]}" )
                if self.execution_order[i] == bot_id:
                    break
                if self.status[self.execution_order[i]].value != Status.Executed.value:
                    print(f"not a correct time to execute trajectory for {bot_id}, returning 0")
                    print("--------------")
                    return Command.Stop

        self.status[bot_id] = Status.Executing
        print("Executing bot is ", bot_id, "\n","<<<<---------------------------->>>>")
        print(bot_id,"executing trajectory")
        #if it is turn of bot_id to perform execution then compute the control command
        goal = self.get_temp_bot_goal(bot_id)
        # goal = self.get_bot_goal(bot_id)

        bot_pose = self.get_bot_pose(bot_id)
        theta = bot_pose[2]
        self.my_controller.set_goal(pose2D(goal[0],goal[1]))
        self.my_controller.set_pose(pose2D(bot_pose[1],bot_pose[0],bot_pose[2]))
        self.my_controller.update_command()
        cmd = self.my_controller.get_command()
        if cmd == Command.Reached:
            print("recieved command reached, assigning new goal")
            self.temp_trajectories[bot_id].popleft()

            #check if it has to perform dumping
            if self.temp_trajectories[bot_id][0][2] == 1:
                #perform dumping
                self.temp_trajectories[bot_id][0][2] = 0 
                return Command.Dump
                
            if len(self.temp_trajectories[bot_id]) == 0:
                #execution completed
                #set the status variable to executed
                #return (0,0) to stop the wheels
                self.status[bot_id] = Status.Executed
                print("execution of",bot_id,"completed")
                return Command.Reached

            cmd = Command.Reached

        return cmd

    #update the commands in class variable
    def update_bot_commands(self):
        for i in range(4):
            #update those commands whose execution is till ongoing
            if self.status[self.bot_ids[i]] != Status.Executed:
                bot_goal = self.get_temp_bot_goal(self.bot_ids[i])
                self.bot_commands[self.bot_ids[i]] = self.get_bot_command(self.bot_ids[i], self.get_bot_pose(self.bot_ids[i]))

    def send_msg(self):
        # self.msg = self.msg.encode('utf-8')
        self.serial_communicator.write(self.msg)
        print("{}: msg sent".format(self.msg))
        
    def send_msgs_thread(self):
        # print("in the send message thread\n")
        while True:
            if self.end_process:
                # print("ending the sender thread")
                return
            self.wait_for_msgs()
            print("waiting completed sending msgs")
            self.send_msg()
    
    def wait_for_msgs(self):
        print("waiting for msg from arduino")
        ack1 = self.serial_communicator.readline()
        print("recieved message is {}".format(ack1))

    def pack_commands2(self):
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
    
    def pack_commands(self):
        for id in self.bot_ids:
            if self.status[id] != Status.Executing:
                continue
            #chande the id_int accordingly
            if id == '2210':
                id_int = 1
            if id == '2010':
                id_int = 2
            if id == '0122':
                id_int = 3
            if id == '2121':
                id_int = 4
            command_string = ''
            if self.bot_commands[id] == Command.Stop or self.bot_commands[id] == Command.Reached:
                command_string = '111111'
            elif self.bot_commands[id] == Command.Anti_clock_wise:
                command_string = '111011'
            elif self.bot_commands[id] == Command.Clock_wise:
                command_string = '110111'
            elif self.bot_commands[id] == Command.Forward:
                command_string = '011111'
            elif self.bot_commands[id] == Command.Dump:
                command_string = '111101'
            command_string = command_string + str(id_int)
            self.msg = command_string
            self.msg = bytes(self.msg,'utf-8')

def visualize_thread(pqueue):
    try:
        count = 0
        print('inside thread')
        while True:
            count += 1
            temp_image = pqueue.get()
            # print("test_count is",temp_image)
            # temp = self.visualization_image.copy()
            cv2.imshow('visualization Thread', temp_image)
            cv2.waitKey(10)
    except:
        print('thread failed')

test = False #for video
main = False #main bloc

test2 = True

if __name__ == '__main__':
    print("start of code")
    start_time = time.time()
    offset_time = 0
    # image_path = r"D:\Robotics\FlipkartD2C\grid 3.0\test_run\photo12.jpg"
    # image_path = r"D:\Robotics\FlipkartD2C\grid 3.0\ohm_scripts\bot_det\cam_feed2.jpeg"

    stage_image = r"D:\Robotics\FlipkartD2C\grid 3.0\ohm_scripts\bot_det\cam_feed1.jpeg"
    #video_path = 1
    video_path = r"D:\Robotics\FlipkartD2C\grid 3.0\test_run\test_small_car1.mp4"
    cap = cv2.VideoCapture(video_path)
    cap.set(3,1920)
    cap.set(4,1080)

    my_monitor = central_monitor()
    shape_identifier = ShapeIdentifier()

    # my_monitor.serial_communicator = serial.Serial("COM10", 9600)

    ret,temp_image = cap.read()
    # cv2.waitKey(5000)
    ret,temp_image = cap.read()

    print("after cap.read")
    cv2.imshow("Test display", temp_image)
    cv2.waitKey(1000)
    my_monitor.camerafeed = temp_image
    # my_monitor.update_stage_coordinates(temp_image)
    my_monitor.initialize_trajectories()

    build_manual_trajectories = True
    visualize_trajectories = True

    if build_manual_trajectories:
        flag = False
        if flag:
            my_monitor.set_manual_trajectories()
        cv2.waitKey(10)
        cv2.destroyAllWindows()
        my_monitor.load_manual_trajectories('2010')#loads mual trajectories for bot wit this id
        my_monitor.load_manual_trajectories('2121')#loads mual trajectories for bot wit this id
        my_monitor.load_manual_trajectories('2210')#loads mual trajectories for bot wit this id
        my_monitor.load_manual_trajectories('0122')#loads mual trajectories for bot wit this id
        # for i in self.bot_ids:
        #     print(my_monitor.temp_trajectories[i])
        # exit()

    if visualize_trajectories:
        my_monitor.status['2210'] = Status.Executing
        my_monitor.visualization_image = temp_image
        for i in my_monitor.bot_ids:
            my_monitor.show_trajectory(i)

        my_monitor.visualization_image = cv2.resize(my_monitor.visualization_image, (0,0),fx = 0.5, fy = 0.5)
        cv2.imshow('trajectory visualization', my_monitor.visualization_image)
        cv2.waitKey(0)

    count = 0
    index = 0
    loop_iteration = 0
    past_time = time.time()
    start_time = time.time()


    # q = multiprocessing.Queue()
    # image = None
    # thread = multiprocessing.Process(target = my_monitor.visualize_thread, args = (q,))
    # thread = multiprocessing.Process(target = visualize_thread, args = ((q),))


    # thread.start()
    count = 0
    while True:
        # my_monitor.wait_for_msgs()
        loop_iteration += 1
        present_time = time.time()
        delta_time  = present_time - past_time
        my_monitor.time_in_string = str(datetime.timedelta(seconds = int(present_time - start_time)))
        past_time = present_time
        print("<<<<-------------start loop--------------->>>>")
        print(f"Start of the loop number: {loop_iteration}")
        print('Time taken is:' ,delta_time,"\nTotal running time is:", present_time - start_time)
        # index += 1
        # if index > 500:
        #     print("processing complete")
        #     cap.release()
        #     out.release()
        #     cv2.destroyAllWindows()
        #     exit()
        ret, frame = cap.read()

        my_monitor.update_image(frame)
        lets_try = False
        if lets_try:
            try:
                # print(index)
                my_monitor.update_robot_poses(frame)
                print("update robot poses successful")
                # print(my_monitor.bots)
            except Exception as e:
                # print("error occured in update_robot_poses")
                # print(str(e))
                cv2.putText(frame, str(e), (300, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
                # cv2.imshow("Error in this image",frame)
                # cv2.waitKey(100)
                # out.write(frame)
                print("update robot poses failed, error is:", str(e))
                continue
        else:
            my_monitor.update_robot_poses(frame)

        my_monitor.update_bot_commands()
        # print(my_monitor.bot_commands)

        my_monitor.pack_commands()
        # print(my_monitor.bot_commands)
        print(my_monitor.msg)

        my_monitor.visualize()
        cv2.imshow('visulize', my_monitor.visualization_image)
        cv2.waitKey(5)
        # my_monitor.send_msg()
        # q.put(my_monitor.visualization_image)
        # q.put(my_monitor.visualization_image.copy())
        # out.write(my_monitor.visualization_image)
        try:
            pass
        except KeyboardInterrupt:
            my_monitor.end_process = True
            time.sleep(2)
            print('KeyboardInterrupt 729')
            # thread.join()
            break
        count += 1
        print("<<<<<<<<<<<<---------------end loop---------------->>>>>>>>>")
test_communication = False
if test_communication:
    flag = True
    my_monitor = central_monitor()
    my_monitor.serial_communicator = serial.Serial("COM3", 9600)
    my_monitor.bot_commands['2210'] = Command.Forward
    my_monitor.status['2210'] = Status.Executing
    count = 1
    temp_command = Command.Forward
    while flag:
        count = count%20
        if count < 5:
            temp_command = Command.Forward
        elif count < 10:
            temp_command = Command.Clock_wise
        elif count < 15:
            temp_command = Command.Anti_clock_wise
        elif count < 20:
            temp_command = Command.Reached
        
        my_monitor.bot_commands['2210'] = temp_command
        print("Looping, Command is:",temp_command.value)
        my_monitor.wait_for_msgs()
        my_monitor.pack_commands()
        my_monitor.send_msg()
        count +=1
         






