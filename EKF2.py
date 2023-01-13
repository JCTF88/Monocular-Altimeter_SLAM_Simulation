## Estimation a posteriori (Altimeter) ##

## lib ##
import numpy as np
from scipy import linalg

## Modules ##
import Parameters 

def EKF_correction_Altimeter(XE,Z,P,tam):
 
    ME = XE[2]

    C1 = np.array([[0,0,1,0,0,0]])
    C = np.hstack([C1,np.zeros((1,3*tam))])
    
    K = np.dot(np.dot(P,np.transpose(C)),linalg.inv(np.dot(np.dot(C,P),np.transpose(C)) + Parameters.R_Altimeter))
    
    XE = XE + np.dot(K,(Z-ME))
    
    PP = np.dot(np.identity(6+(tam*3)) - np.dot(K,C),P)

    return XE,PP