import numpy as np
from math import *
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from convertCod import *
from LowerHalfTrans import *
from calcCoM import calcCom
from rayCast import rayCast
import os

#parameters
control = 212.6
the = 0.3

x0 = np.array([0,0,0])
x1 = np.array([0,100,0])
x2 = np.array([100,0,0])
x3 = np.array([100,100,0])
x4 = np.array([202.9,0,0])
x5 = np.array([202.9,100,0])
x4_1 = np.array([202.9, -25, 50])
x4_2 = np.array([202.9, -25, -0])
x5_1 = np.array([202.9, 125, 50])
x5_2 = np.array([202.9, 125, -0])
print(np.sqrt(np.sum(x5_1 - x5_2)**2))
l1 = 100
l2 = 102.9
x = np.stack((x0,x1,x2,x3,x4,x5,x4_1,x4_2,x5_1,x5_2))
Theta = np.array([the,0,0,0,0,0])
x = lowerHalfTrans(x, Theta)
x0 = x[0]
x1 = x[1]
x2 = x[2]
x3 = x[3]
x4 = x[4]
x5 = x[5]
x4_1 = x[6]
x4_2 = x[7]
x5_1 = x[8]
x5_2 = x[9]
#display results
print(x2)
print(x3)
print(x4)
print(x5)
print(x4_1,x4_2,x5_1,x5_2)
print(np.sqrt(np.sum((x5_1 - x5_2)**2)))
#get equation

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

vR = x4 - x2
thetaRPitch = -atan(c/a) + atan(vR[2]/(vR[0]+0.0001))
thetaRRoll = atan(b/sqrt(a**2+c**2)) - atan(vR[1]/(sqrt(vR[0]**2+vR[2]**2)))
RPitch = np.array([[cos(thetaRPitch),0,sin(thetaRPitch)],[0,1,0],[-sin(thetaRPitch),0,cos(thetaRPitch)]])
RRoll = np.array([[cos(thetaRRoll),-sin(thetaRRoll),0],[sin(thetaRRoll),cos(thetaRRoll),0],[0,0,1]])
newVR = np.dot(RRoll,np.dot(RPitch,vR))
newVR = newVR/np.linalg.norm(newVR)
print(newVR)

print('&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&')
print(thetaLRoll, thetaLPitch, thetaRRoll, thetaRPitch)
#plot results
if 0:
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

#get Com of the robot
Com = calcCom(the)
M = Com[3]
Com = Com[0:3]
print(Com)
Com = convCod2book(Com)
print(Com)

#project Com to ground plane
ComP = Com - np.dot((Com - p), cp)*cp
print(ComP)

#judge whether the projection is in vertex
#project 4 points to plane (not strict)
print(x4_1,x4_2,x5_1,x5_2)
print(np.sqrt(np.sum((x4_1 - x4_2)**2)))
print(np.sqrt(np.sum((x5_1 - x5_2)**2)))

x5_1 = np.dot(LRoll,np.dot(LPitch,x5_1-x5))+x5
x5_2 = np.dot(LRoll,np.dot(LPitch,x5_2-x5))+x5

x4_1 = np.dot(RRoll,np.dot(RPitch,x4_1-x4))+x4
x4_2 = np.dot(RRoll,np.dot(RPitch,x4_2-x4))+x4
print(x4_1,x4_2,x5_1,x5_2)
print(np.sqrt(np.sum((x4_1 - x4_2)**2)))
print(np.sqrt(np.sum((x5_1 - x5_2)**2)))

if 0:
    ax = plt.figure().gca(projection='3d')

    x = [x4_1[0],x4_2[0],x5_2[0],x5_1[0]]
    y = [x4_1[1],x4_2[1],x5_2[1],x5_1[1]]
    z = [x4_1[2],x4_2[2],x5_2[2],x5_1[2]]
    ax.plot(x,y,z)
    ax.scatter(ComP[0],ComP[1],ComP[2])

    plt.show()


x4_1 = x4_1 - np.dot((x4_1 - p), cp)*cp
x4_2 = x4_2 - np.dot((x4_2 - p), cp)*cp
x5_1 = x5_1 - np.dot((x5_1 - p), cp)*cp
x5_2 = x5_2 - np.dot((x5_2 - p), cp)*cp
print('Foot after project:', x4_1,x4_2,x5_1,x5_2)
PFar = np.array([2000,2000,2000])
PFar = PFar - np.dot((PFar - p), cp)*cp
xJudge = np.stack((x4_1,x4_2,x5_2,x5_1,ComP,PFar))
IN = rayCast(xJudge)
print(IN)
#plot relation
if 1:
    ax = plt.figure().gca(projection='3d')

    x = [x4_1[0],x4_2[0],x5_2[0],x5_1[0]]
    y = [x4_1[1],x4_2[1],x5_2[1],x5_1[1]]
    z = [x4_1[2],x4_2[2],x5_2[2],x5_1[2]]
    ax.plot(x,y,z)
    ax.scatter(ComP[0],ComP[1],ComP[2])

    plt.show()

