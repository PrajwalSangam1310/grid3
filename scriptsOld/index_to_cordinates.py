#!/usr/bin/env python

import numpy as np
import math
from geometry_msgs.msgs import Pose2D


class point:
    def __init__(self,x=0,y=0):
        self.x = x
        self.y = y


class CordinateConvertor:
    def __init__(self):
        self.START = point(-1.1454, 1.16544) 
        self.STEP = 0.0254*6
    def get_coordinate(self,i,j):
        tempx = self.START.x + i*self.STEP
        tempy = self.START.y + j*self.STEP 
        return(tempx, tempy)
