# region lib
import cv2 
import time
import imutils
import numpy as np
import argparse
import matplotlib.pyplot as plt
from skimage.transform import (hough_line,hough_line_peaks) 
import math


def size_mod(img,times):
    h,w = img.shape[:2]
    dim = (int(w*times), int(h*times))
    return cv2.resize(img, dim)

def nothing(x):
    pass

def orientation_line_from_angle(img,angle,cur_location):
    cX, cY = cur_location
    rad = angle*math.pi/180
    radius = 75
    dy = radius*math.sin(rad)
    dx = radius*math.cos(rad)       
    Y = int(cY - dy)
    X = int(cX + dx)
    cv2.line(img,(int(cX),int(cY)),(X,Y),(0,0,255),2)

def min_dist(coord_1,coord_2):
    dist = math.sqrt(  (coord_1[0] - coord_2[0])**2  + (coord_1[1] - coord_2[1])**2  )
    return dist
        
class Angle:
    def __init__(self,img):
        self.ret_degree = 0 # orientation found
        self.img = img # image to be rotated
        H,W = self.img.shape[:2] # original image height 
        copy_img = self.img.copy() # copy of original image

        # change image format
        # idk why i used rgb format 
        rgb = cv2.cvtColor(copy_img, cv2.COLOR_BGR2RGB) 
        hsv = cv2.cvtColor(copy_img, cv2.COLOR_BGR2HSV)
        # cv2.imshow('rgb', rgb)
        # cv2.imshow('hsv', hsv)
        # cv2.imshow('img1', self.img)
        
        # white mask of image 
        mask = cv2.inRange(hsv, (0, 0, 254), (255, 255,255))
        # cv2.imshow('mask_angle', mask)
        
        # cv2.waitKey()
        # cv2.destroyAllWindows()
        
        # find contour
        (cnts, _) = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
        
        
        try:
            # find center of biggest contour
            c = max(cnts, key = cv2.contourArea)
            x,y,w,h = cv2.boundingRect(c)
            location = [int(y+ h/2), int(x+ w/2)]

            # assgin value of orientation in degree
            self.angle_b([H,W],location) 

            
            # draw line on bot's provided image
            copy_img = cv2.line(copy_img, (int(W/2),int(H/2)), 
                (location[1],location[0]), (255,0,0))
            # cv2.imshow('copy_img', copy_img)
            # cv2.waitKey()

        except:
            print("failed in angle detection")
            self.ret_degree = 90
            pass

    def angle_b(self,img_size,position):
        self.img_size = img_size # image size (H,W)
        self.position = position # contour center location in image
        H,W = self.img_size
        h,w = self.position

        # x,y are loaction of contour center w.r.t image center
        x = w - (W/2)
        y = (H/2) - h
        
        # rad and degree are orientation w.r.t to +ve x-axis
        rad = math.atan2(y , x)
        degree = rad*180/math.pi
        
        # assign global degre variable
        self.ret_degree = degree
        print("the angle of the roo=bot is",self.ret_degree)
        # print(f'initial_angle :: {self.ret_degree}')

    def ret(self):      
        temp =  self.ret_degree -180
        if temp < -180:
            temp += 360
        elif temp > 180:
            temp = temp - 360
        return (temp)

class ExtractContour:
    def __init__(self,img):
        self.img = img
        H,W = self.img.shape[:2]
        self.bot_img = [] # list of bot extracted images
        
        #purple_mask
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        pruple_mask = cv2.inRange(hsv,(160, 50, 100), (180, 255,255))
        # cv2.imshow('purple mask',size_mod(pruple_mask,0.5))
        # cv2.waitKey()

        kernel = np.ones((5,5),np.uint8)
        opening = cv2.morphologyEx(pruple_mask, cv2.MORPH_OPEN, kernel)
        blur = cv2.blur(opening, (3,3))
        eroded = cv2.erode(pruple_mask, kernel)
        # cv2.imshow('opening', opening)
        # cv2.imshow('blur', blur)
        # cv2.imshow('eroded_img', eroded)
        # cv2.waitKey()

        #canny edges
        edge = cv2.Canny(opening, 10,250)
        # cv2.imshow('egde', size_mod(edge, 0.5))
        # cv2.waitKey()

        # find external contour of egde  
        (cnts, _) = cv2.findContours(edge.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
        # cv2.drawContours(self.img, cnts, -1, (0, 255, 0), 2)
        
        # cv2.imshow("extrea",size_mod(self.img,0.5))
        # cv2.waitKey()

        self.bot_location = [] # list of bot location
        for c in cnts:
            # print(f'{cv2.contourArea(c)} |||  W : { 0.02*0.02*H*W}') 
            # find the bound rectangle of bot
            x,y,w,h = cv2.boundingRect(c) 
            # area = cv2.contourArea(c)
            if w*h > 1000 and w*h < 10000: 
            # if w> (0.02*W) and h> (0.02*W): 
                location = [int(y+ h/2), int(x+ w/2)]
                # add loaction of current bot 
                self.bot_location.append(location)
                # crop the bot out of image
                new_img=self.img[y-5:y+h+5,x-5:x+w+5] 

                # cv2.circle(self.img, (location[1],location[0]), 20, (0,255,255),thickness = 5)
                # cv2.imshow('img_ext',size_mod(self.img,0.5))
                # cv2.waitKey()
                # add image of current bot 
                self.bot_img.append(new_img)
                # cv2.imshow(f'{idx}bot_img_extract_contour',new_img)
        
        # print number of bots
        # print('number of bots: '+ str(len(self.bot_img)))
                
    def ret(self):
        # print(f'''botlocatiom {self.bot_location}''')
        return self.bot_img, self.bot_location

class BotFinder:
    def __init__(self):#,f_no):
        # ID = bot_ID()# intialize PCA bot ID finder 
        self.bot_ID_all = []# all bot's ID are stored here
        self.rotated_img = 0 # temp. rotated image
        # pre defined center
        init1 = (1550,1040)
        init2 = (1550,550)

        self.b1 = [(1550,1050),1]
        self.b2 = [(1550,550),2]
        # self.b3 = [(1550,450),3]
        # self.b4 = [(1550,1150),4]

        self.funx = lambda x: x + 0.0423*(800-x)
        self.funy = lambda y: y + 0.0436*(800-y)

        self.last_Bot = {}
        # self.last_bot[0] = []

        # original 
        # self.last_Bot[0] = [1550,1050 , -180] # for bot 1
        # self.last_Bot[1] = [1550,550  , -180]  # for bot 2
        # self.last_Bot[2] = [1550,450  , -180]  # for bot 3
        # self.last_Bot[3] = [1550,1150 , -180] # for bot 4
        
        # for testing
        # self.last_Bot[0] = [1550,1050 , -180] # for bot 1
        # self.last_Bot[1] = [1550,1050, -180]  # for bot 2
        # self.last_Bot[2] = [1550,550 , -180]  # for bot 3
        # self.last_Bot[3] = [1550,450 , -180]  # for bot 4
    

    def give_frame(self,img):# provide current frame as img
        self.img = img
        self.Bot = {} # contain value as (X,Y,Theta) with key as bot ID
        self.AC_orientation = []# all bot's Angle are stored here

        self.bot_location = []# location of all bots in (X,Y)

        ext = ExtractContour(self.img)# process frame
        bot_img, self.bot_location = ext.ret()# gives croped_img, croped_img_location
        # print(f'''location {self.bot_location}''')
        #iterate for 4 croped images        
        for i in range(len(bot_img)):
            
            # show extracted image
            # cv2.imshow('bot_img', bot_img[i])
            # cv2.waitKey()

            #resize image 4 times original image
            h,w = bot_img[i].shape[:2]
            dim = ( int(w*4) , int(h*4) )
            # print(dim)
            bot_img[i] = cv2.resize(bot_img[i], dim)            
   
            # find angle of alignment of bot with +ve x-aixs 'degree'
            Agl = Angle(bot_img[i])# feed croped image 
            # return bot allignment in degree and rotated croped image
            degree = Agl.ret()
            # add the orientation to self.AC_orientation list
            self.AC_orientation.append(degree)

            # initialize temp bot id as 'self.curr_bot_id'
            self.curr_bot_id = 1




            #region crosscheck the code
            # curr_x,curr_y = self.bot_location[i]
            # curr_angle = degree
            # print(i)
            # try:
            #     last_x,last_y,last_angle = self.last_Bot[i]    
            # except:
            #     print(f'''error in {i} bot ID''')
            #     cv2.waitKey()

            # # cross check
            # location_diff = min_dist((curr_x,curr_y), (last_x,last_y))
            # angle_diff = abs(last_angle - curr_angle)

            # valid_input = (location_diff < 50) and (angle_diff < 15)
            # if valid_input :
            #     # add the temp bot ID to main list of bot_ID 
            #     self.bot_ID_all.append(self.curr_bot_id)    
            #     # add bot location and orientation to self.Bot dict with ID string as key

                
            #     self.Bot[self.curr_bot_id] = [int(self.funx(self.bot_location[i][0])), int(self.funy(self.bot_location[i][1]))
            #                         , self.AC_orientation[i]]
            # else:
            #     self.bot_ID_all.append(self.curr_bot_id)    
            #     # add bot location and orientation to self.Bot dict with ID string as key
            #     self.Bot[self.curr_bot_id] = self.last_Bot[self.curr_bot_id]

            #endregion    


            # region old cross check 
            #cross check bot_id with predefined bot id 
            # curr_y,curr_x = self.bot_location[i]
            # b1_sep = min_dist((curr_x,curr_y), self.b1[0])
            # print(b1_sep)

            curr_y,curr_x = self.bot_location[i]

            b1_sep = min_dist((curr_x,curr_y), self.b1[0])
            if b1_sep < 50 :
                self.curr_bot_id = 0
                self.b1[0] = (curr_x,curr_y)

            b2_sep = min_dist((curr_x,curr_y), self.b2[0])
            if b2_sep < 50 :
                self.curr_bot_id = 1
                self.b2[0] = (curr_x,curr_y)

            # b3_sep = min_dist((curr_x,curr_y), self.b3[0])
            # if b3_sep < 50 :
            #     self.curr_bot_id = 2
            #     self.b3[0] = (curr_x,curr_y)
            # b4_sep = min_dist((curr_x,curr_y), self.b4[0])
            
            # if b4_sep < 50 :
            #     self.curr_bot_id = 3
            #     self.b4[0] = (curr_x,curr_y)


            # add the temp bot ID to main list of bot_ID 
            self.bot_ID_all.append(self.curr_bot_id)    
            # add bot location and orientation to self.Bot dict with ID string as key

            self.Bot[self.curr_bot_id] = [self.bot_location[i][0] ,self.bot_location[i][1], self.AC_orientation[i]]

            # self.Bot[self.curr_bot_id] = [int(self.funx(self.bot_location[i][0])), int(self.funy(self.bot_location[i][1]))
            #                                 , self.AC_orientation[i]]

    def ret(self):    
        # count = 0
        # for i in self.Bot.keys():
        #     temp = self.Bot[i]
        #     self.Bot[i] = [temp[0],temp[1], self.AC_orientation[count]]
        #     count = count + 1
        # print(self.Bot)
        return self.Bot

# region PCA tester
# class bot_ID:
#     def __init__(self):
#         self.bot_ID = ''
#         filename = 'finalized_model.sav'
#         loaded_model = pickle.load(open(filename, 'rb'))
#         # classifier = loaded_model.score(x_test, y_test)

#         # initialise loaded_model here

#     def Predict(self,img):
#         self.img = img
#         gray = cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)
#         flat = np.ravel(gray)
#         flat = flat.T
#         flat = pca.transform(flat)
#         ID = loaded_model.predict(flat)
#         if ID == 1:
#             self.bot_ID = '0200'
#         elif ID == 2:
#             self.bot_ID = '2010'
#         elif ID == 3:
#             self.bot_ID = '0110'
#         elif ID == 4:
#             self.bot_ID = '0102'
#         return self.bot_ID 
# endregion

def orientation_line_from_angle(img,angle,cur_location):
    cX, cY = cur_location
    rad = angle*math.pi/180
    radius = 75
    dy = radius*math.sin(rad)
    dx = radius*math.cos(rad)       
    Y = int(cY - dy)
    X = int(cX + dx)
    cv2.line(img,(int(cX),int(cY)),(X,Y),(0,0,255),2)


def img_align(img,pts,size_x,size_y):
    img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    rows,cols,ch = img_gray.shape
    pts1 = np.float32(pts)
    pts2 = np.float32([[0,0],[size_x,0],[size_x,size_y],[0,size_y]])
    M = cv2.getPerspectiveTransform(pts1,pts2)
    dst = cv2.warpPerspective(img,M,(size_x,size_y))
    return dst


if __name__ == '__main__':


    # initialiser
    BF = BotFinder()
    # pts = [[570,150],[1340,190],[1360,990],[500,970]] # img24.jpg
    # pts = [[305,100],[1340,20],[1350,1025],[400,1040]] # img24.jpg

    # pts_plus = [[520,95],[1390,140],[1420,1050],[440,1030]] # img24 with startpts
    pts_plus = [[305,100],[1340,20],[1350,1025],[400,1040]] # img24.jpg
    pts = [[305,100],[1340,20],[1350,1025],[400,1040]] # img24.jpg 

    # region video capture

    # vdo = cv2.VideoCapture('C:/Users/OHM/Desktop/pytthon/Resources/image/test1/vdo.mp4')
    vdo = cv2.VideoCapture(2)
    vdo.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    vdo.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)


    ret,img = vdo.read()    
    img_aligned_plus= img_align(img,pts_plus,1600,1600)  
    # print(img.shape[:2])
    # cv2.imshow('img1',img)          
    # cv2.imshow('img',img_aligned_plus)
    cv2.waitKey()

    ret,saved_img = vdo.read()
    filename = 'abcd.jpg'

    while True:
        start = time.time()
        ret,img = vdo.read()
        if ret is False:break 
        
        # img_aligned= img_align(img,pts,1400,1400)
        img_aligned_plus= img_align(img,pts_plus,1600,1600)            
        img_copy = img_aligned_plus.copy()
        # cv2.imshow("plot_img",size_mod(img_copy,0.5))

        # cv2.imshow("plot_img",img_copy)
        
        BF.give_frame(img_aligned_plus)
        bot = BF.ret()

        for i in bot.keys():
            # extract info from bot
            cY,cX,angle = bot[i]
            print(bot)
            #draw circle and show bot ID  
            cv2.putText(img_copy, str(i), (int(cX-10), int(cY)),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3)
            cv2.circle(img_copy, (int(cX),int(cY)), 30, (0,255,255),thickness = 2)

            # draw line of orientation
            orientation_line_from_angle(img_copy, angle, (cX,cY))
        
        cv2.imshow("plot_img",size_mod(img_copy,0.5))
        key = cv2.waitKey(20)
        if key == 27:break
        end = time.time()
        print(f'''{1/(end-start)}Hz''')    


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