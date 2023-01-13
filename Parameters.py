## Parameters ##

## lib ##
import numpy as np

# Camera rotation matrix
R_tot = np.array([[0,1,0],[1,0,0],[0,0,-1]])

# Sampling time
dtt = .1

# Focal lenght
focal = 300.1

# Simulation time
time_simu = 200

# Trajectory radius
radio_tra = 30

# Trajectory frecuency 
fre_tra = .015

# Trajectory altitude
alt_tra = 8

# Estimated initial conditions
XP = np.array([[0],[0],[8],[0],[0],[0]])

# Matrix A
A = np.array([[1,0,0,dtt,0,0],[0,1,0,0,dtt,0],[0,0,1,0,0,dtt],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])

# Matrix Q
Q = np.identity(6) * .01

# Matrix Q2
Q2 = .0001

# Matrix R for Altimeter
R_Altimeter =  .1

# Matrix R for Camera
R_CAM = np.identity(2) * 2

# Estimated initial conditions for P

PP = np.identity(6) * .00001

# Matrix I
I = np.identity(6)

# Size of landmarks on the map

size_MAP = 25

# Size of image

size_img = 100

# Initial uncertainty

inc_ini = .2

