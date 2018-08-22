import numpy as np
from math import *

def lowerHalfTrans(x,Theta):
    #Hip Pitch & Roll
    theta = Theta[0]
    LHipPitch = np.array([[cos(theta),0,sin(theta)],[0,1,0],[-sin(theta),0,cos(theta)]])
    theta = Theta[1]
    LHipRoll = np.array([[cos(theta),-sin(theta),0],[sin(theta),cos(theta),0],[0,0,1]])
    theta = Theta[2]
    RHipPitch = np.array([[cos(theta),0,sin(theta)],[0,1,0],[-sin(theta),0,cos(theta)]])
    theta = Theta[3]
    RHipRoll = np.array([[cos(theta),-sin(theta),0],[sin(theta),cos(theta),0],[0,0,1]])

    #Knee Pitch
    theta = Theta[4]
    LKneePitch = np.array([[cos(theta),0,sin(theta)],[0,1,0],[-sin(theta),0,cos(theta)]])
    theta = Theta[5]
    RKneePitch = np.array([[cos(theta),0,sin(theta)],[0,1,0],[-sin(theta),0,cos(theta)]])

    #update joints
    x[2] = np.dot(RHipRoll,np.dot(RHipPitch,x[2]))
    x[3] = np.dot(LHipRoll,np.dot(LHipPitch,x[3] - x[1])) + x[1]

    x[4] = np.dot(RHipRoll,np.dot(RHipPitch,x[4])) - x[2]
    x[4] = x[2] + np.dot(RKneePitch,x[4])
    x[6] = np.dot(RHipRoll,np.dot(RHipPitch,x[6])) - x[2]
    x[6] = x[2] + np.dot(RKneePitch,x[6])
    x[7] = np.dot(RHipRoll,np.dot(RHipPitch,x[7])) - x[2]
    x[7] = x[2] + np.dot(RKneePitch,x[7])

    x[5] = np.dot(LHipRoll,np.dot(LHipPitch,x[5] - x[1])) + x[1] - x[3]
    x[5] = x[3] + np.dot(LKneePitch,x[5])
    x[8] = np.dot(LHipRoll,np.dot(LHipPitch,x[8] - x[1])) + x[1] - x[3]
    x[9] = np.dot(LHipRoll,np.dot(LHipPitch,x[9] - x[1])) + x[1] - x[3]
    x[8] = x[3] + np.dot(LKneePitch,x[8])
    x[9] = x[3] + np.dot(LKneePitch,x[9])

    return x

def lowerHalfComTrans(x,Theta):
    #Hip Pitch & Roll
    theta = Theta[0]
    LHipPitch = np.array([[cos(theta),0,sin(theta)],[0,1,0],[-sin(theta),0,cos(theta)]])
    theta = Theta[1]
    LHipRoll = np.array([[cos(theta),-sin(theta),0],[sin(theta),cos(theta),0],[0,0,1]])
    theta = Theta[2]
    RHipPitch = np.array([[cos(theta),0,sin(theta)],[0,1,0],[-sin(theta),0,cos(theta)]])
    theta = Theta[3]
    RHipRoll = np.array([[cos(theta),-sin(theta),0],[sin(theta),cos(theta),0],[0,0,1]])

    #Knee Pitch
    theta = Theta[4]
    LKneePitch = np.array([[cos(theta),0,sin(theta)],[0,1,0],[-sin(theta),0,cos(theta)]])
    theta = Theta[5]
    RKneePitch = np.array([[cos(theta),0,sin(theta)],[0,1,0],[-sin(theta),0,cos(theta)]])

    #update joints
    #x[0]:RPelvis, x[1]:LPevis, x[2]:RHip, x[3]:LHip,x[4]:RThigh, x[5]:LThigh --> Hip
    #x[6]:RTibia, x[7]:LTibia, x[8]:RAnkle, x[9]:LAnkle --> Knee
    #x[10]:RFoot, x[11]:LFoot --> Ankle (Knee instead for convenience)
    #x[12] --> x1(LHip), x[13] --> x2(RKnee), x[14] --> x3(LKnee)
    x[0] = np.dot(RHipRoll,np.dot(RHipPitch,x[0]))
    x[2] = np.dot(RHipRoll,np.dot(RHipPitch,x[2]))
    x[4] = np.dot(RHipRoll,np.dot(RHipPitch,x[4]))
    x[1] = np.dot(LHipRoll,np.dot(LHipPitch,x[1] - x[12])) + x[12]
    x[3] = np.dot(LHipRoll,np.dot(LHipPitch,x[3] - x[12])) + x[12]
    x[5] = np.dot(LHipRoll,np.dot(LHipPitch,x[5] - x[12])) + x[12]

    #x[13] and x[14] need to be updated
    #x[13] = np.dot(RHipRoll,np.dot(RHipPitch,x[13]))
    #x[14] = np.dot(LHipRoll,np.dot(LHipPitch,x[14] - x[12])) + x[12]


    x[6] = np.dot(RHipRoll,np.dot(RHipPitch,x[6])) - x[13]
    x[6] = x[13] + np.dot(RKneePitch,x[6])
    x[8] = np.dot(RHipRoll,np.dot(RHipPitch,x[8])) - x[13]
    x[8] = x[13] + np.dot(RKneePitch,x[8])
    x[10] = np.dot(RHipRoll,np.dot(RHipPitch,x[10])) - x[13]
    x[10] = x[13] + np.dot(RKneePitch,x[10])

    x[7] = np.dot(LHipRoll,np.dot(LHipPitch,x[7] - x[12])) + x[12] - x[14]
    x[7] = x[14] + np.dot(LKneePitch,x[7])
    x[9] = np.dot(LHipRoll,np.dot(LHipPitch,x[9] - x[12])) + x[12] - x[14]
    x[9] = x[14] + np.dot(LKneePitch,x[9])
    x[11] = np.dot(LHipRoll,np.dot(LHipPitch,x[11] - x[12])) + x[12] - x[14]
    x[11] = x[14] + np.dot(LKneePitch,x[11])

    return x
