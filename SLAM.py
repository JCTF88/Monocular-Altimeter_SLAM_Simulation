## Main ##

## lib ##
import numpy as np

## Modules ##
import Parameters 
import Map
import Dynamic
import Graphics
import EKF1
import EKF2
import EKF3
import Altimeter
import Camera 
import Initialization1

# Read Map
Map_t = Map.MAPP

# Initialization
XE = Parameters.XP
PP = Parameters.PP
XLAND = np.empty((0,3)) 
XRR = np.empty((0, 3))
XEE = np.empty((0, 3))
Map_not_Visible = Map_t
Map_Visible = np.empty((0,3))
Final_MAP_E = np.empty((0,3))
Final_MAP = np.empty((0,3))

tim = 0
# Principal cycle
while tim <= Parameters.time_simu * (1/Parameters.dtt):

    # Dynamics of the UAV
    XR = Dynamic.dynamic_UAV(tim * Parameters.dtt)
    XRR = np.vstack([XRR, XR])
    
    # Altimeter Measurement
    XR_Altimeter = Altimeter.measurement_Altimeter(XR)
         
    # not Visible Map Projection  
    Map_projection_not_Visible = np.empty((0,2))       
    if np.shape(Map_not_Visible)[0] >= 1: 
       for i in range(0,np.shape(Map_not_Visible)[0],1): 
           Pro = Camera.model_camera(XR.T,np.transpose(np.array([Map_not_Visible[i][0:3]])))
           Map_projection_not_Visible = np.vstack([Map_projection_not_Visible, np.hstack([Pro[0], Pro[1]])])    
       
    # Visible Map Projection
    Map_projection_Visible = np.empty((0,2))
    if np.shape(Map_Visible)[0] >= 1: 
       for i in range(0,np.shape(Map_Visible)[0],1): 
           Pro = Camera.model_camera(XR.T,np.transpose(np.array([Map_Visible[i][0:3]])))
           Map_projection_Visible = np.vstack([Map_projection_Visible, np.hstack([Pro[0], Pro[1]])])  
             
    # Inicialization of Landmarks   
    if np.shape(XLAND)[0] < Parameters.size_MAP and np.shape(Map_projection_not_Visible)[0] >= 1:  
       bor = np.empty((0,1)) 
       XXLAND = np.empty((0,3)) 
       for i in range(0,np.shape(Map_projection_not_Visible)[0],1): 
           if abs(Map_projection_not_Visible[i][0]) <= Parameters.size_img and abs(Map_projection_not_Visible[i][1]) <= Parameters.size_img and np.shape(XLAND)[0] < Parameters.size_MAP:
               LAND = Initialization1.Initialization(XR.T,XE[0:3],Map_not_Visible[i][0:3],Map_projection_not_Visible[i][0:2])
               XXLAND = np.vstack([XXLAND,LAND])
               MLAND = np.array([[Map_not_Visible[i][0],Map_not_Visible[i][1],Map_not_Visible[i][2]]])
               Map_Visible = np.vstack([Map_Visible,MLAND])     
               PLAND = np.array([[Map_projection_not_Visible[i][0],Map_projection_not_Visible[i][1]]])
               Map_projection_Visible = np.vstack([Map_projection_Visible,PLAND])  
               bor = np.vstack([bor,i])  
       if np.shape(bor)[0] >= 1 and np.shape(Map_not_Visible)[0] >= 1: 
          Map_not_Visible = np.delete(Map_not_Visible,bor.astype(int)[0:],axis=0)
          AUM = np.identity(3*np.shape(bor)[0]) * Parameters.inc_ini
          P1 = np.hstack([PP,np.zeros((np.shape(PP)[0],np.shape(bor)[0]*3))])
          P2 = np.hstack([np.zeros((np.shape(bor)[0]*3,np.shape(PP)[0])),AUM])
          PP = np.vstack([P1,P2])
          for i in range(0,np.shape(XXLAND)[0],1):  
              XLAND = np.vstack([XLAND,np.array([XXLAND[i][0:3]])])
              
    # Join
    if np.shape(XLAND)[0] >= 1:
       for i in range(0,np.shape(XLAND)[0],1):  
           XE = np.vstack([XE,np.transpose(np.array([XLAND[i][0:3]]))])          
               
    # Estimation a priori
    EE = EKF1.EKF_prediction_apriori(XE,PP,np.shape(XLAND)[0])
    XE = EE[0]  
    PP = EE[1]      
                   
    # Estimation a posteriori (Camera)
    if np.shape(XLAND)[0] >= 1:
       for i in range(0,np.shape(XLAND)[0],1):  
           EE = EKF3.EKF_correction_Camera(XE,np.transpose(np.array([XLAND[i][0:3]])),np.transpose(np.array([Map_projection_Visible[i][0:2]])),PP,i+1,np.shape(XLAND)[0])
           XE = EE[0]
           PP = EE[1]        
                 
    # Estimation a posteriori (Altimeter)  
    EE = EKF2.EKF_correction_Altimeter(XE,XR_Altimeter,PP,np.shape(XLAND)[0])
    XE = EE[0]
    PP = EE[1]
    
    # Separation
    if np.shape(XLAND)[0] >= 1:
       XLAND = np.empty((0,3)) 
       for i in range(0,np.shape(Map_Visible)[0],1):
           XLAND = np.vstack([XLAND,np.transpose(XE[6+(i*3):6+(i*3)+3])])
       XE = XE[0:6]
    
    # Delete landmarks
    if np.shape(XLAND)[0] >= 1:
        bor = np.empty((0,1)) 
        for i in range(0,np.shape(Map_projection_Visible)[0],1):
            if abs(Map_projection_Visible[i][0]) > Parameters.size_img or abs(Map_projection_Visible[i][1]) > Parameters.size_img:
                bor = np.vstack([bor,i]) 
                Final_MAP_E = np.vstack([Final_MAP_E,np.array([XLAND[i][0:3]])])
                Final_MAP = np.vstack([Final_MAP,np.array([Map_Visible[i][0:3]])])
        if np.shape(bor)[0] >= 1:
            Map_Visible = np.delete(Map_Visible,bor.astype(int)[0:],axis=0)
            XLAND = np.delete(XLAND,bor.astype(int)[0:],axis=0)
            for i in range(0,np.shape(bor)[0],1): 
                re = int(bor[i] - i) 
                a = list(range(6+re*3,6+re*3+3))
                PP = np.delete(PP,a,axis=0)
                PP = np.delete(PP,a,axis=1)
                
    # Save estimated data
    XE_g = np.transpose(XE[0:3])
    XEE = np.vstack([XEE,XE_g])
    
    # Increase cycle
    tim = tim + 1

# Print Graphics
Graphics.graphic_slam(XRR,XEE,Final_MAP_E)
