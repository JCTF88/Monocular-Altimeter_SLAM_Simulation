## Estimation a priori ##

## lib ##
import numpy as np

## Modules ##
import Parameters 


def EKF_prediction_apriori(XP,PP,tam):

    xxx = XP[0] + Parameters.dtt * XP[3]                                     
    yyy = XP[1] + Parameters.dtt * XP[4]                          
    zzz = XP[2] + Parameters.dtt * XP[5]

    xxxp = XP[3]
    yyyp = XP[4]
    zzzp = XP[5]

    XE = np.vstack([xxx,yyy,zzz,xxxp,yyyp,zzzp,XP[6:6+tam*3]])
    
    A1 = np.hstack([Parameters.A,np.zeros((6,3*tam))])
    A2 = np.hstack([np.zeros((3*tam,6)),np.identity(3*tam)])
    A = np.vstack([A1,A2])
    
    Q1 = np.hstack([Parameters.Q,np.zeros((6,3*tam))])
    Q2 = np.hstack([np.zeros((3*tam,6)),np.identity(3*tam)*Parameters.Q2])
    Q = np.vstack([Q1,Q2]) 
    
    P = np.dot(np.dot(A,PP),np.transpose(A)) + Q

    return XE,P

