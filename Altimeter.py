# Altimeter measurement

## lib ##
import numpy as np
import random

def measurement_Altimeter(XR):
  
    zzz = XR[0,2] + random.gauss(0,.1)

    zzz = np.array([[zzz]])

    return zzz