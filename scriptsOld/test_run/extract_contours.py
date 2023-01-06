import cv2
import numpy as np 
import imutils

def size_mod(img,times):
    h,w = img.shape[:2]
    dim = (int(w*times), int(h*times))
    return cv2.resize(img, dim)

class ExtractContour:
    def __init__(self,img):
        self.img = img
        H,W = self.img.shape[:2]
        #purple_mask
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        pruple_mask = cv2.inRange(hsv,(130, 100, 80), (180, 255,255))
        # pruple_mask = cv2.inRange(hsv,(165,100, 80), (175, 200,255))
        
        #for analysis only
        #comment at end
        #dim = (int(W/2), int(H/2))
        #pruple_mask= cv2.resize(pruple_mask, dim)
        
        kernel = np.ones((5,5),np.uint8)
        eroded = cv2.morphologyEx(pruple_mask, cv2.MORPH_OPEN, kernel)
        blur = cv2.blur(eroded, (3,3))
        eroded = cv2.erode(pruple_mask, kernel)
        
        # cv2.imshow('eroded_img', eroded)
        # cv2.imshow('blur', blur)
        # cv2.waitKey()
        # cv2.destroyAllWindows()
        #vdo_tbar(self.img)
        #canny edges
        
        edge = cv2.Canny(blur, 10,250)
        small_img = size_mod(edge, 0.5)
        
        # cv2.imshow('egde', small_img)
        # cv2.waitKey()
        
        ##after morph open these are not required
        #kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        #edge = cv2.morphologyEx(edge, cv2.MORPH_CLOSE, kernel)
        # cv2.imshow('ExtractCoutour_pruple_mask',pruple_mask)
        # cv2.imshow('edge', edge)
        # cv2.waitKey()
        # cv2.destroyAllWindows()
        # cv2.imshow('1',edge[0:int(H/2),0:int(W/2)])
        # cv2.imshow('2',edge[0:int(H/2),int(W/2):W])
        # cv2.imshow('3',edge[int(H/2):H,0:int(W/2)])
        # cv2.imshow('4',edge[int(H/2):H,int(W/2):W])
        # cv2.waitKey()
        # cv2.destroyAllWindows()

        (cnts, _) = cv2.findContours(edge.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
        
        idx = 0         
        self.bot_location = []
        sort_cnts = sorted(cnts,key=cv2.contourArea,reverse=True)
        for c in cnts:
            # print(f'video_size = {H,W} || extract_size = {cv2.contourArea(c)} |||  threshold_size : { 0.02*0.02*H*W}') 
            x,y,w,h = cv2.boundingRect(c) 
            if w> (0.02*W) and h> (0.02*W): 
                idx+=1 
                location = [y+ h/2, x+ w/2]
                self.bot_location.append(location)
                new_img=self.img[y-30:y+h+30,x-30:x+w+30] 
                self.bot_img = new_img
                # cv2.imshow('small',self.bot_img)        
        #         cv2.imshow(f'{idx}bot_img_extract_contour',new_img)
        # print('number of bots: '+ str(len(self.bot_img)))
        # cv2.waitKey()
        # cv2.destroyAllWindows()
                
    def ret(self):
        return self.bot_img, self.bot_location