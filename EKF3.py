## Estimation a posteriori (Camera) ##

## lib ##
import numpy as np
from scipy import linalg

## Modules ##
import Parameters 
import Camera 

def EKF_correction_Camera(XE,XLAND,Z,P,i,tam):
    
    Pro = Camera.model_camera(XE[0:3],XLAND)
    ME = np.vstack([Pro[0], Pro[1]])
    
    du1 = 0
    du2 = float(-Parameters.focal/(XE[2]-XLAND[2]))
    du3 = float((Parameters.focal*(XE[1]-XLAND[1]))/(pow(XE[2]-XLAND[2],2)))
    du4 = 0
    du5 = float(Parameters.focal/(XE[2]-XLAND[2]))
    du6 = float(-(Parameters.focal*(XE[1]-XLAND[1]))/(pow(XE[2]-XLAND[2],2)))
    
    dv1 = float(-Parameters.focal/(XE[2]-XLAND[2]))
    dv2 = 0
    dv3 = float((Parameters.focal*(XE[0]-XLAND[0]))/(pow(XE[2]-XLAND[2],2)))
    dv4 = float(Parameters.focal/(XE[2]-XLAND[2]))
    dv5 = 0
    dv6 = float(-(Parameters.focal*(XE[0]-XLAND[0]))/(pow(XE[2]-XLAND[2],2)))
    
    C1 = np.hstack([np.array([[du1,du2,du3,0,0,0]]),np.zeros((1,3*(i-1))),np.array([[du4,du5,du6]]),np.zeros((1,3*(tam-i)))]) 
    C2 = np.hstack([np.array([[dv1,dv2,dv3,0,0,0]]),np.zeros((1,3*(i-1))),np.array([[dv4,dv5,dv6]]),np.zeros((1,3*(tam-i)))]) 
    C = np.vstack([C1,C2])
    K = np.dot(np.dot(P,np.transpose(C)),np.linalg.inv(np.dot(np.dot(C,P),np.transpose(C)) + Parameters.R_CAM))

    XE = XE + np.dot(K,(Z-ME))
    
    PP = np.dot(np.identity(6+(tam*3)) - np.dot(K,C),P)

    return XE,PP
