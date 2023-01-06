import serial
import math
import numpy as np
from final_function import BotFinder
from shortest_path_input import *
import enum
import cv2
import threading
from collections import deque
import time


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
    #Reached
    Stop = (0,0)

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
        self.serial_communicator = serial.Serial("COM4",9600)
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
        self.execution_status = []
        self.end_process = False

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


    #position functions
    def update_robot_poses(self, image_feed):
        self.bot_identifier.update_image(image_feed)
        self.bot_identifier.detect_bots()
        self.bots = self.bot_identifier.get_bot_poses()

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
    
    #calculate the commands for bots
    def get_bot_command(self,bot_id, bot_pose):
        #check if its executed if yess return (0,0)
        if self.status[bot_id] == Status.Executed:
            # print((bot_id,"execution complete"))
            return Command.Stop
        
        #if its not executed check if previous robots have completed there execution
        elif self.status[bot_id] == Status.Not_executed:
            for i in self.execution_order.keys():
                if self.execution_order[i] == bot_id:
                    break
                if self.status[self.execution_order[i]] != Status.Executed:
                    # print(bot_id,"not a correct time to execute trajectory")
                    return Command.Stop
                self.status[bot_id] = Status.Executing
                break
        # print(bot_id,"executing trajectory")
        #if it is turn of bot_id to perform execution then compute the control command
        goal = self.get_bot_goal(bot_id)
        bot_pose = self.get_bot_pose(bot_id)
        theta = bot_pose[2]
        self.my_controller.set_goal(pose2D(goal[0],goal[1]))
        self.my_controller.set_pose(pose2D(bot_pose[0],bot_pose[1],bot_pose[2]))
        self.my_controller.update_command()
        cmd = self.my_controller.get_command()
        if cmd == (0,0):
            if len(self.trajectories[bot_id]) == 0:
                #execution completed
                #set the status variable to executed
                #return (0,0) to stop the wheels
                self.status[bot_id] = Status.Executed
                return (0,0)
            self.trajectories[bot_id].popleft()
        return cmd

    #update the commands in class variable
    def update_bot_commands(self):
        for i in range(4):
            # print(i)
            bot_goal = self.get_bot_goal(self.bot_ids[i])
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


test = False #for image
test2 = True #for video
main = False #main block

if test:
    image_path = r"D:\Robotics\FlipkartD2C\grid 3.0\ohm_scripts\bot_det\cam_feed2.jpeg"
    stage_image = r"D:\Robotics\FlipkartD2C\grid 3.0\ohm_scripts\bot_det\cam_feed1.jpeg"
    # image1 = cv2.imread(image_path)
    my_monitor = central_monitor()
    my_monitor.update_stage_coordinates(my_monitor.camerafeed)
    my_monitor.initialize_trajectories()
    my_monitor.read_image(image_path)
    # print(my_monitor.camerafeed)
    # cv2.imshow('llal',my_monitor.camerafeed)
    my_monitor.update_robot_poses(my_monitor.camerafeed)
    # print(my_monitor.bots)
    my_monitor.update_bot_commands()
    # print(my_monitor.bot_commands)
    # print(my_monitor.bots)

if test2:
    start_time = time.time()
    offset_time = 27
    image_path = r"D:\Robotics\FlipkartD2C\grid 3.0\ohm_scripts\bot_det\cam_feed2.jpeg"
    stage_image = r"D:\Robotics\FlipkartD2C\grid 3.0\ohm_scripts\bot_det\cam_feed1.jpeg"
    video_path = r"D:\Robotics\FlipkartD2C\grid 3.0\test_run\test_video.mp4"
    cap = cv2.VideoCapture(video_path)
    # image1 = cv2.imread(image_path)
    my_monitor = central_monitor()
    my_monitor.read_image(image_path)
    my_monitor.update_stage_coordinates(my_monitor.camerafeed)
    my_monitor.initialize_trajectories()
    total_frames = cap.get(cv2.CAP_PROP_FRAME_COUNT) 
    fps = cap.get(cv2.CAP_PROP_FPS)
    sender_thread = threading.Thread(target=my_monitor.send_msgs_thread)
    sender_thread.start()
    # duration = total_frames/fps
    # print(duration)
    # index = round(present_time*fps)
    while True:
        print('start of loop')
        present_time = time.time()
        delta_time  = present_time - start_time + offset_time
        index = round(delta_time*fps)
        
        # print(round(delta_time*fps))
        cap.set(cv2.CAP_PROP_POS_FRAMES, index)
        ret, frame = cap.read()
        frame = cv2.rotate(frame, cv2.cv2.ROTATE_90_COUNTERCLOCKWISE)
        # cv2.imshow('video',frame)
        # print(cap.get(cv2.CAP_PROP_POS_MSEC))
        # cv2.waitKey(1000)
        my_monitor.update_image(frame)
        my_monitor.update_robot_poses(my_monitor.camerafeed)
        # print(delta_time)
        # print(my_monitor.bots)
        my_monitor.update_bot_commands()
        # print(my_monitor.bot_commands)
        my_monitor.pack_commands()
        # my_monitor.update_bot_commands()
        # print(my_monitor.bot_commands)


# if __name__=='__main__':
if main:
    # initialize monitor class
    stage_image_path = ""
    stage_image = cv2.imread(stage_image_path)
    
    my_monitor = central_monitor()
    my_monitor.update_stage_coordinates(stage_image)
    my_monitor.initialize_trajectories()
    my_monitor.start_communication()

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






