import numpy as np
from math import *
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

x0 = np.array([0,0,0])
x1 = np.array([0,100,0])
x2 = np.array([100,0,0])
x3 = np.array([100,100,0])
x4 = np.array([202.9,0,0])
x5 = np.array([202.9,100,0])
l1 = 100
l2 = 102.9

#Hip Pitch & Roll
theta = 0
LHipPitch = np.array([[cos(theta),0,sin(theta)],[0,1,0],[-sin(theta),0,cos(theta)]])
theta = 0
LHipRoll = np.array([[cos(theta),-sin(theta),0],[sin(theta),cos(theta),0],[0,0,1]])
theta = 0
RHipPitch = np.array([[cos(theta),0,sin(theta)],[0,1,0],[-sin(theta),0,cos(theta)]])
theta = 0
RHipRoll = np.array([[cos(theta),-sin(theta),0],[sin(theta),cos(theta),0],[0,0,1]])

#Knee Pitch
theta = 0
LKneePitch = np.array([[cos(theta),0,sin(theta)],[0,1,0],[-sin(theta),0,cos(theta)]])
theta = 0
RKneePitch = np.array([[cos(theta),0,sin(theta)],[0,1,0],[-sin(theta),0,cos(theta)]])

#update joints
x2 = np.dot(RHipRoll,np.dot(RHipPitch,x2))
x3 = np.dot(LHipRoll,np.dot(LHipPitch,x3 - x1)) + x1

x4 = np.dot(RHipRoll,np.dot(RHipPitch,x4)) - x2
x4 = x2 + np.dot(RKneePitch,x4)

x5 = np.dot(LHipRoll,np.dot(LHipPitch,x5 - x1)) + x1 - x3
x5 = x3 + np.dot(LKneePitch,x5)

#display results
print(x2)
print(x3)
print(x4)
print(x5)


#get equation
control = 210
p = np.array([control, 50, 300])

# These two vectors are in the plane
v1 = x4 - p
v2 = x5 - p

# the cross product is a vector normal to the plane
cp = np.cross(v1, v2)
cp = cp/np.linalg.norm(cp)
a, b, c = cp

if a <= 0:
    a = -a
    b = -b
    c = -c

# This evaluates a * x3 + b * y3 + c * z3 which equals d
d = np.dot(cp, p)

print('The equation is {0}x + {1}y + {2}z = {3}'.format(a, b, c, d))
vL = x5 - x3
thetaLPitch = -atan(c/a) + atan(vL[2]/(vL[0]+0.0001))
thetaLRoll = atan(b/sqrt(a**2+c**2)) - atan(vL[1]/(sqrt(vL[0]**2+vL[2]**2)))
LPitch = np.array([[cos(thetaLPitch),0,sin(thetaLPitch)],[0,1,0],[-sin(thetaLPitch),0,cos(thetaLPitch)]])
LRoll = np.array([[cos(thetaLRoll),-sin(thetaLRoll),0],[sin(thetaLRoll),cos(thetaLRoll),0],[0,0,1]])
newVL = np.dot(LRoll,np.dot(LPitch,vL))
newVL = newVL/np.linalg.norm(newVL)
print(newVL)

#plot results
ax = plt.figure().gca(projection='3d')

x = [x4[0],x2[0],x0[0],x1[0],x3[0],x5[0]]
y = [x4[1],x2[1],x0[1],x1[1],x3[1],x5[1]]
z = [x4[2],x2[2],x0[2],x1[2],x3[2],x5[2]]
ax.plot(x,y,z)
ax.scatter(p[0],p[1],p[2])
X = np.arange(-200, 200, 1)
Y = np.arange(-200, 200, 1)
X, Y = np.meshgrid(X, Y)
Z = (d - a*X - b*Y)/c
ax.plot_surface(X,Y,Z)

plt.show()
