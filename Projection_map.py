## Map points projected on the camera ##

## lib ##
import numpy as np

## Modules ##
import Camera 

def points_map_projection(MAP,XR):
    
    Pro_map = np.empty((1,2))
    size_map = np.shape(MAP)

    for i in range(0,size_map[0],1):

        Pro = Camera.model_camera(XR,np.transpose(np.array([MAP[i][0:3]])))
        Pro_map = np.vstack([Pro_map, np.hstack([Pro[0], Pro[1]])])       

    return Pro_map     

 
