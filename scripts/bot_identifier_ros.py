#!/usr/bin/env python3
import error_finder_m4 as efm
import rospy
from grid3.msg import botPoses
from gridIdentifier import utilfunctions
import cv2
import math
import numpy as np
import time

if __name__ == '__main__':

    # region ros
    rospy.init_node("botIdentifier", anonymous=True)

    allRobotPoses = botPoses()
    allRobotPosesRC = botPoses()

    for i in range(4):
        allRobotPoses.ids = [i for i in range(4)]
        allRobotPoses.xs = [0 for i in range(4)]
        allRobotPoses.ys = [0 for i in range(4)]
        allRobotPoses.thetas = [0.0 for i in range(4)]

        allRobotPosesRC.ids = [i for i in range(4)]
        allRobotPosesRC.xs = [0 for i in range(4)]
        allRobotPosesRC.ys = [0 for i in range(4)]
        allRobotPosesRC.thetas = [0.0 for i in range(4)]

    posePub = rospy.Publisher("grid3/allRobots/poses", botPoses, queue_size=10)
    poseRcPub = rospy.Publisher("grid3/allRobots/posesRC", botPoses, queue_size=10)

    # endregion

    myutil = utilfunctions()

    BF = efm.BotFinder()
    pts = [[570,150],[1340,190],[1360,990],[500,970]] # img24.jpg
    # pts_plus = [[520,95],[1390,140],[1420,1050],[440,1030]] # img24 with startpts
    pts_plus = [[150,100],[1300,50],[1300,1055],[250,1040]] # img24.jpg


    # region video capture
    # vdo = cv2.VideoCapture('/home/prajwal/Downloads/grid/vdo.mp4')
    # print(vdo)
    # print("after video")

    #IN CASE CAMERA FLICKERS
    # v4l2-ctl --list-devices
    # v4l2-ctl -d /dev/video3 --set-ctrl=exposure_auto=1
    # v4l2-ctl --device /dev/video2 --set-ctrl=exposure_absolute=400



    vdo = cv2.VideoCapture(2)
    vdo.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    vdo.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    desX = [400, 800, 1200, 400,750,1150,400,700,1200]
    desY = [400,400, 400, 800,800,800,1200,1200,1200]
    desNames = ['Mumbai', 'Delhi', 'Kolkata', 'Chennai','Bengaluru','Hyderbad','Pune','Ahemdabad','Jaipur']

    while True:
        # start = time.time()
        ret,img = vdo.read()
        # cv2.imshow("windowname",img)
        # cv2.waitKey()
        if ret is False:
                print("breaking")
                break 
        
        # img_aligned= img_align(img,pts,1400,1400)
        img_aligned_plus= efm.img_align(img,pts_plus,1600,1600)            
        img_copy = img_aligned_plus.copy()
        # cv2.imshow("plot_img",img_copy)
        
        BF.give_frame(img_aligned_plus)
        bot = BF.ret()

        for i in range(9):
            cv2.putText(img_copy, str(desNames[i]), (int(desX[i]-50), int(desY[i])),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3)

        for i in bot.keys():
            # extract info from bot
            cY,cX,angle = bot[i]
            print(bot)
            #draw circle and show bot ID  
            cv2.putText(img_copy, str(i), (int(cX-10), int(cY)),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3)
            cv2.circle(img_copy, (int(cX),int(cY)), 30, (0,255,255),thickness = 2)
            # draw line of orientation
            efm.orientation_line_from_angle(img_copy, angle, (cX,cY))
            for j in range(4):
                tempxy = myutil.ij2rc(cX,cY)
                if allRobotPoses.ids[j] == i:
                    allRobotPoses.xs[j] = cX
                    allRobotPoses.ys[j] = cY
                    allRobotPoses.thetas[j] = math.radians(angle)
                
                    allRobotPosesRC.xs[j] = tempxy[0]
                    allRobotPosesRC.ys[j] = tempxy[1]
                    allRobotPosesRC.thetas[j] = math.radians(angle)
        
        poseRcPub.publish(allRobotPosesRC)
        posePub.publish(allRobotPoses)
        cv2.imshow("plot_img",efm.size_mod(img_copy,0.3))
        key = cv2.waitKey(15)
        if key == 27:
            break

        
        # end = time.time()
        # print(f'''{1/(end-start)}Hz''')    

    print("Executiion completed")


    # endregion


    # region image processing
   
    # img = cv2.imread('C:/Users/OHM/Desktop/pytthon/Resources/image/test1/img (24).jpg')
    # pts = [[570,150],[1340,190],[1360,990],[500,970]] # img24.jpg
    # pts_plus = [[520,95],[1390,140],[1420,1050],[440,1030]] # img24 with startpts

    # img_aligned= img_align(img,pts,1400,1400)
    # img_aligned_plus= img_align(img,pts_plus,1600,1600)
    # img_copy = img_aligned_plus.copy()
          
    # BF.give_frame(img_aligned_plus)
    # bot = BF.ret()
    # print(bot)

    # for i in bot.keys():
    #     # extract info from bot
    #     cY,cX,angle = bot[i]
    #     #draw circle and show bot ID  
    #     cv2.putText(img_copy, str(i), (int(cX-10), int(cY)),
    #         cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3)
    #     cv2.circle(img_copy, (int(cX),int(cY)), 30, (0,255,255),thickness = 2)

    #     # draw line of orientation
    #     orientation_line_from_angle(img_copy, angle, (cX,cY))

    # cv2.imshow("plot_img",size_mod(img_copy,0.5))
    # # cv2.waitKey(2)
    # cv2.waitKey()
    # endregion