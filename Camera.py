## Camera model ##

## lib ##
import numpy as np
import random

## Modules ##
import Parameters 

def model_camera(Pos,Point):

    As = np.dot(np.transpose(Parameters.R_tot),(Point - Pos))
          
    u = (Parameters.focal * As[0] / As[2]) + random.gauss(0,1)
    v = (Parameters.focal * As[1] / As[2]) + random.gauss(0,1)

    return u,v