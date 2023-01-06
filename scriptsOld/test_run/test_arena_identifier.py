import cv2
from final_function import BotFinder
import time
import math
import numpy as np
import pandas as pd
import matplotlib.image as mpimg


def min_dist(coord_1,coord_2):
    dist = math.sqrt(  (coord_1[0] - coord_2[0])**2  + (coord_1[1] - coord_2[1])**2  )
    return dist

def test(img,name):
    print(str(name))
    cv2.imshow("test",img)
    cv2.waitKey()


if __name__ =="__main__":
    img = cv2.imread('C:/Users/OHM/Desktop/pytthon/Resources/arena_high.png')
    empty = cv2.imread('C:/Users/OHM/Desktop/pytthon/Resources/empty_area.jpg')
    cam_feed1 = cv2.imread('Resources/cam_feed_1.jpeg')
    cam_feed2 = cv2.imread('Resources/cam_feed_2.jpeg')
    cam_feed3 = cv2.imread(r"D:\Robotics\FlipkartD2C\grid 3.0\ohm_scripts\bot_det\cam_feed1.jpeg")


#h,w = self.image.shape[:2]
#dim = (int(w/2), int(h/2))
#self.image = cv2.resize(self.image, dim)
class matrix_builder:
    def __init__(self,image=None):
        self.image = image
        self.matrix = []
    def update_image(self, image1):
        self.image = image1 
    
    def detect_grid(self):
        s = time.time()
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0,0,130), (175,36,255))
        #test(mask,'mask')

        blur = cv2.GaussianBlur(mask, (5,5), 0)
        #test(blur,'blur')
        thresh = cv2.adaptiveThreshold(blur, 255, 1, 1, 11, 2)
        #test(thresh,'thresh')
        blur = cv2.GaussianBlur(thresh, (5,5), 0)
        #test(blur,'blur')

        kernel = np.ones((5,5), np.uint8)
        opening = cv2.morphologyEx(blur, cv2.MORPH_OPEN, kernel)
        #img_erosion = cv2.erode(blur, kernel, iterations=1)
        #img_dilation = cv2.dilate(blur, kernel, iterations=5)
        #print("dilated inage")
        #
        #test(opening,'img_dilation')

        contours, _ = cv2.findContours(opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        area = self.image.shape[:2][1]*self.image.shape[:2][0]
        min_area = area/(10*20)
        print('start')
        i = 0
        boxes = {}
        coord = []
        for c in contours:

            cnt_area = cv2.contourArea(c)
            #if cnt_area > min_area/5 and cnt_area < 10* min_area:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)
            if cnt_area > 2250 and cnt_area < 6300 and len(approx) == 4:
                i +=1
                #cv2.drawContours(self.image, c, -1, (0,255,0),thickness=5)
                #print(f'{area} || {min_area} || {cnt_area} ')

                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                center_location = (cX,cY)
                boxes[i] = center_location
                coord.append(center_location)
                # draw the contour and center of the shape on the image
                cv2.drawContours(self.image, [c], -1, (0, 255, 0), 2)
                cv2.circle(self.image, (cX, cY), 30, (255, 255, 0), 4)
                cv2.circle(self.image, (cX, cY), 4, (255, 0, 255), -1)
                cv2.putText(self.image, str(i), (cX - 10, cY - 10),
            	cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 2)


        matrix_x = np.zeros((9,16))
        matrix_y = np.zeros((9,16))
        value = np.zeros((9,16))
        row = 8
        key = 1
        for i in range(7):
            col = 6
            for j in range(4):
                post = boxes[key]
                matrix_x[row,col] = post[0]
                matrix_y[row,col] = post[1]
                value[row,col] = True

                #print(f'{post[0]} : {post[1]} :: {post}')
                #print(f'{matrix_x[row,col]} : {matrix_y[row,col]} :: {post}')

                #print(matrix_x)
                #print(matrix_y)
                col +=1
            key +=1
            row -=1
        row = 1
        for i in range(2):
            col = 0
            for i in range(16):
                post = boxes[key]
                #print(post)
                matrix_x[row,col] =post[0]
                matrix_y[row,col] = post[1]
                value[row,col] = True
                col +=1
                key +=1
            row -=1
        final = list(zip(matrix_x, matrix_y,value))
        i=0
        a = final.copy()
        for i in range(len(matrix_x)):
            final[i] = list(zip(matrix_x[i], matrix_y[i],value[i]))
        df = pd.DataFrame(final)

        block_sepearation = min_dist(df[0][0], df[0][1])
        self.matrix = final

    def get_matrix(self):
        return self.matrix
# print(len(df),len(df[1]),len(df[0]))
# print(f'{df[0][0]} : {df[0][1]} --> {block_sepearation}')
# e = time.time()
# print(e-s)
#test(self.image,'empty')

# bot_position is (x,y)
# bot_position = [700,700]
# min_sep = block_sepearation*0.1
# bot_x = bot_position[0]
# bot_y = bot_position[1]

