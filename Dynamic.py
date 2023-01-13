## Dynamic of the UAV ##

## lib ##
import math
import numpy as np

## Modules ##
import Parameters 

def dynamic_UAV(time_simulation):

    xr = - Parameters.radio_tra * math.cos(time_simulation * Parameters.fre_tra) + Parameters.radio_tra

    yr = Parameters.radio_tra * math.sin(time_simulation * Parameters.fre_tra)

    zr = (math.sqrt(xr*xr+yr*yr) / 20) + Parameters.alt_tra 

    XR = np.array([[xr,yr,zr]])

    return XR