## First initialization ##

## lib ##
import numpy as np
import math
import random
import Parameters 


def Initialization(XR,XE,MAP,Map_points_camera):
   
   dii = math.sqrt(pow(XR[0]-MAP[0], 2) + pow(XR[1]-MAP[1], 2) + pow(XR[2]-MAP[2], 2)) + random.gauss(0,.2)
   AN =  np.dot(Parameters.R_tot,np.array([[Map_points_camera[0]],[Map_points_camera[1]],[Parameters.focal]]))
   ti = math.atan2(AN[1],AN[0])
   fi = math.atan2(AN[2],math.sqrt(pow(AN[0], 2) + pow(AN[1], 2)))           
   VM = np.array([[math.cos(fi)*math.cos(ti)],[math.cos(fi)*math.sin(ti)],[math.sin(fi)]])   
   LAND = XE + dii * VM
   LAND = np.transpose(LAND)
   
   return LAND