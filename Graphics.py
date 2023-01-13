## Grafics ##

## lib ##
import matplotlib.pyplot as plt

def graphic_slam(XRR,XEE,Final_MAP_E):
    
    fig = plt.figure(figsize=(10,8))
    ax1 = fig.add_subplot(111, projection='3d')
    ax1.scatter(XRR.T[0], XRR.T[1], XRR.T[2],linestyle='-')
    ax1.scatter(XEE.T[0], XEE.T[1], XEE.T[2],linestyle='-')
    ax1.scatter(Final_MAP_E.T[0], Final_MAP_E.T[1], Final_MAP_E.T[2],marker='.')
    ax1.legend(['Real Trajectory','Estimated Trajectory','Estimated Landmarks'])
    plt.show()

