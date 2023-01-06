import cv2 
import time
import imutils
import numpy as np
import argparse
import matplotlib.pyplot as plt
import math
# from new_final_function import BotFinder
from angle_only import BotFinder
# img = cv2.imread(f'C:/Users/OHM/Desktop/pytthon/error_imgs/64.jpg')


def size_mod(img,times):
    h,w = img.shape[:2]
    dim = (int(w*times), int(h*times))
    return cv2.resize(img, dim)

#region for all functions
# BF = BotFinder()

# BF.give_frame(img)

# bot = BF.ret()
# print(bot)
#endregion

#region just orientation and loaction
# BF = BotFinder()

# BF.give_frame(img)
# print(BF.ret())
#endregion

#region video 
cap = cv2.VideoCapture(1)
cap.set(3,1920)
cap.set(4,1080)
ret,frame = cap.read()
cv2.waitKey(3000)

BF = BotFinder()
# cv2.imshow('frame',frame)
print(frame.shape)
error = 0
while True:
    ret,frame = cap.read()
    if ret is False:break 
    try:
        BF.give_frame(frame)
        angle, location = BF.ret()
    except:error+=1
    print(error)
    cY,cX = location[0]
    cv2.circle(frame, (int(cX),int(cY)), 40, (255,0,255),thickness=2)

    rad = angle*math.pi/180
    radius = 75
    dy = radius*math.sin(rad)
    dx = radius*math.cos(rad)       
    Y = int(cY - dy)
    X = int(cX + dx)
    cv2.line(frame,(int(cX),int(cY)),(X,Y),(0,0,255),2)

    cv2.imshow('frame',size_mod(frame,0.5))

    key = cv2.waitKey(1)
    if key == 27:break
#endregion
