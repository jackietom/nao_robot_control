import numpy as np
from math import *
from calcCoM import *
from rayCast import *
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def judge(param,Theta):
    a = param[0]
    b = param[1]
    c = param[2]
    thetaLPitch = param[3]
    thetaLRoll = param[4]
    thetaRPitch = param[5]
    thetaRRoll = param[6]
    x = param[7:]

    vL = x[5] - x[3]
    thetaLPitch = -atan(c/a) + atan(vL[2]/(vL[0]+0.0001))
    thetaLRoll = atan(b/sqrt(a**2+c**2)) - atan(vL[1]/(sqrt(vL[0]**2+vL[2]**2)))
    LPitch = np.array([[cos(thetaLPitch),0,sin(thetaLPitch)],[0,1,0],[-sin(thetaLPitch),0,cos(thetaLPitch)]])
    LRoll = np.array([[cos(thetaLRoll),-sin(thetaLRoll),0],[sin(thetaLRoll),cos(thetaLRoll),0],[0,0,1]])

    vR = x[4] - x[2]
    thetaRPitch = -atan(c/a) + atan(vR[2]/(vR[0]+0.0001))
    thetaRRoll = atan(b/sqrt(a**2+c**2)) - atan(vR[1]/(sqrt(vR[0]**2+vR[2]**2)))
    RPitch = np.array([[cos(thetaRPitch),0,sin(thetaRPitch)],[0,1,0],[-sin(thetaRPitch),0,cos(thetaRPitch)]])
    RRoll = np.array([[cos(thetaRRoll),-sin(thetaRRoll),0],[sin(thetaRRoll),cos(thetaRRoll),0],[0,0,1]])
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
    Com = calcCom(Theta)
    M = Com[3]
    Com = Com[0:3]
    Com = convCod2book(Com)


    p = x[6]
    cp = np.array([a,b,c])
    #project Com to ground plane
    ComP = Com - np.dot((Com - p), cp)*cp

    #judge whether the projection is in vertex
    #project 4 points to plane (not strict)

    x4_1 = x[6]
    x4_2 = x[7]
    x5_1 = x[8]
    x5_2 = x[9]
    x5 = x[5]
    x4 = x[4]

    x5_1 = np.dot(LRoll,np.dot(LPitch,x5_1-x5))+x5
    x5_2 = np.dot(LRoll,np.dot(LPitch,x5_2-x5))+x5
    x4_1 = np.dot(RRoll,np.dot(RPitch,x4_1-x4))+x4
    x4_2 = np.dot(RRoll,np.dot(RPitch,x4_2-x4))+x4


    x4_1 = x4_1 - np.dot((x4_1 - p), cp)*cp
    x4_2 = x4_2 - np.dot((x4_2 - p), cp)*cp
    x5_1 = x5_1 - np.dot((x5_1 - p), cp)*cp
    x5_2 = x5_2 - np.dot((x5_2 - p), cp)*cp

    PFar = np.array([2000,2000,2000])
    PFar = PFar - np.dot((PFar - p), cp)*cp
    xJudge = np.stack((x4_1,x4_2,x5_2,x5_1,ComP,PFar))
    IN = rayCast(xJudge)

    #plot relation
    if 1:
        ax = plt.figure().gca(projection='3d')

        x = [x4_1[0],x4_2[0],x5_2[0],x5_1[0]]
        y = [x4_1[1],x4_2[1],x5_2[1],x5_1[1]]
        z = [x4_1[2],x4_2[2],x5_2[2],x5_1[2]]
        ax.plot(x,y,z)
        ax.scatter(ComP[0],ComP[1],ComP[2])
        #ax.scatter(PFar[0], PFar[1], PFar[2])

        plt.show()


    result = IN
    return result
