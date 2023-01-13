## Map ##

## lib ##
import math
import numpy as np
import random

MAPP = np.zeros((50*75,3))

for i_1 in range(0,50):
    for i_2 in range(0,75):
         
        x = -5 + i_2 + (random.random() - .5) * 3
        y = -5 + i_1 + (random.random() - .5) * 3
        z = (math.sqrt(x*x+y*y) / 20) + (random.random() - .5) * 2

        MAPP[i_2+75*i_1][0] = x
        MAPP[i_2+75*i_1][1] = y
        MAPP[i_2+75*i_1][2] = z
 
