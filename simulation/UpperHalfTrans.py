import numpy as np
from math import *

def UpperHalfTrans(x,Theta):
    #joint angles
    LShoulderPitch = Theta[0]
    LShoulderRoll = Theta[1]
    RShoulderPitch = Theta[2]
    RShoulderRoll = Theta[3]
    LElbowYaw = Theta[4]
    LElbowRoll = Theta[5]
    RElbowYaw = Theta[6]
    RElbowRoll = Theta[7]

    #Shoulder:x[0]: right x[1]: left
    #Elbow:x[2]: right, x[3]: left
    #hand:x[4]: right, x[5]: left
    #Rotation matrix
    LShoulderPitchM = np.array([[cos(LShoulderPitch), 0, sin(LShoulderPitch)],
                                [0, 1, 0],
                                [-sin(LShoulderPitch), 0, cos(LShoulderPitch)]])

    LShoulderRollM = np.array([[1, 0, 0],
                               [0, cos(LShoulderRoll), sin(LShoulderRoll)],
                               [0, -sin(LShoulderRoll), cos(LShoulderRoll)]])

    RShoulderPitchM = np.array([[cos(RShoulderPitch), 0, sin(RShoulderPitch)],
                                [0, 1, 0],
                                [-sin(RShoulderPitch), 0, cos(RShoulderPitch)]])

    RShoulderRollM = np.array([[1, 0, 0],
                               [0, cos(RShoulderRoll), sin(RShoulderRoll)],
                               [0, -sin(RShoulderRoll), cos(RShoulderRoll)]])

    LElbowYawM = np.array([[cos(LElbowYaw), -sin(LElbowYaw), 0],
                           [sin(LElbowYaw), cos(LElbowYaw), 0],
                           [0, 0, 1]])

    LElbowRollM = np.array([[cos(LElbowRoll), 0, sin(LElbowRoll)],
                                [0, 1, 0],
                                [-sin(LElbowRoll), 0, cos(LElbowRoll)]])

    RElbowYawM = np.array([[cos(RElbowYaw), -sin(RElbowYaw), 0],
                           [sin(RElbowYaw), cos(RElbowYaw), 0],
                           [0, 0, 1]])

    RElbowRollM = np.array([[cos(RElbowRoll), 0, sin(RElbowRoll)],
                                [0, 1, 0],
                                [-sin(RElbowRoll), 0, cos(RElbowRoll)]])
    #x[2]
    x[2] = np.dot(RShoulderRollM, np.dot(RShoulderPitchM, (x[2] - x[0]))) + x[0]
    #x[3]
    x[3] = np.dot(LShoulderRollM, np.dot(LShoulderPitchM, (x[3] - x[1]))) + x[1]
    #x[4]
    x[4] = np.dot(RShoulderRollM, np.dot(RShoulderPitchM, (x[4] - x[0]))) + x[0]
    x[4] = np.dot(RElbowRollM, np.dot(RElbowYawM, x[4] - x[2])) + x[2]
    #x[5]
    x[5] = np.dot(LShoulderRollM, np.dot(LShoulderPitchM, (x[5] - x[1]))) + x[1]
    x[5] = np.dot(LElbowRollM, np.dot(LElbowYawM, x[5] - x[3])) + x[3]

    return x

def UpperHalfComTrans(x, Theta):
    #joint angles
    LShoulderPitch = Theta[0]
    LShoulderRoll = Theta[1]
    RShoulderPitch = Theta[2]
    RShoulderRoll = Theta[3]
    LElbowYaw = Theta[4]
    LElbowRoll = Theta[5]
    RElbowYaw = Theta[6]
    RElbowRoll = Theta[7]

    #Rotation matrix
    LShoulderPitchM = np.array([[cos(LShoulderPitch), 0, sin(LShoulderPitch)],
                                [0, 1, 0],
                                [-sin(LShoulderPitch), 0, cos(LShoulderPitch)]])

    LShoulderRollM = np.array([[1, 0, 0],
                               [0, cos(LShoulderRoll), sin(LShoulderRoll)],
                               [0, -sin(LShoulderRoll), cos(LShoulderRoll)]])

    RShoulderPitchM = np.array([[cos(RShoulderPitch), 0, sin(RShoulderPitch)],
                                [0, 1, 0],
                                [-sin(RShoulderPitch), 0, cos(RShoulderPitch)]])

    RShoulderRollM = np.array([[1, 0, 0],
                               [0, cos(RShoulderRoll), sin(RShoulderRoll)],
                               [0, -sin(RShoulderRoll), cos(RShoulderRoll)]])

    LElbowYawM = np.array([[cos(LElbowYaw), -sin(LElbowYaw), 0],
                           [sin(LElbowYaw), cos(LElbowYaw), 0],
                           [0, 0, 1]])

    LElbowRollM = np.array([[cos(LElbowRoll), 0, sin(LElbowRoll)],
                                [0, 1, 0],
                                [-sin(LElbowRoll), 0, cos(LElbowRoll)]])

    RElbowYawM = np.array([[cos(RElbowYaw), -sin(RElbowYaw), 0],
                           [sin(RElbowYaw), cos(RElbowYaw), 0],
                           [0, 0, 1]])

    RElbowRollM = np.array([[cos(RElbowRoll), 0, sin(RElbowRoll)],
                                [0, 1, 0],
                                [-sin(RElbowRoll), 0, cos(RElbowRoll)]])

    #Transformation
    #x[0]: Torso, x[1]:Neck, x[2]:Head, x[3]:LShoulder, x[4]:RShoulder,
    #x[5]:LBiceps, x[6]:RBiceps, x[7]:LElbow, x[8]:RElbow, x[9]:LForeArm,
    #x[10]:RForeArm, x[11]:LHand, x[12]:RHand
    #x[13]:left shoulder, x[14]:right shoulder, x[15]:left elbow, x[16]: right elbow
    #x[4]
    x[4] = np.dot(RShoulderRollM, np.dot(RShoulderPitchM, (x[4] - x[14]))) + x[14]
    #x[3]
    x[3] = np.dot(LShoulderRollM, np.dot(LShoulderPitchM, (x[3] - x[13]))) + x[13]
    #x[5]
    x[5] = np.dot(LShoulderRollM, np.dot(LShoulderPitchM, (x[5] - x[13]))) + x[13]
    #x[6]
    x[6] = np.dot(RShoulderRollM, np.dot(RShoulderPitchM, (x[6] - x[14]))) + x[14]
    #x[7]
    x[7] = np.dot(LShoulderRollM, np.dot(LShoulderPitchM, (x[7] - x[13]))) + x[13]
    #x[8]
    x[8] = np.dot(RShoulderRollM, np.dot(RShoulderPitchM, (x[8] - x[14]))) + x[14]

    #x[15] and x[16] needed to be updated
    x[15] = np.dot(LShoulderRollM, np.dot(LShoulderPitchM, (x[15] - x[13]))) + x[13]
    x[16] = np.dot(RShoulderRollM, np.dot(RShoulderPitchM, (x[16] - x[14]))) + x[14]

    #x[9]
    x[9] = np.dot(LShoulderRollM, np.dot(LShoulderPitchM, (x[9] - x[13]))) + x[13]
    x[9] = np.dot(LElbowRollM, np.dot(LElbowYawM, x[9] - x[15])) + x[15]
    #x[10]
    x[10] = np.dot(RShoulderRollM, np.dot(RShoulderPitchM, (x[10] - x[14]))) + x[14]
    x[10] = np.dot(RElbowRollM, np.dot(RElbowYawM, x[10] - x[16])) + x[16]
    #x[11]
    x[11] = np.dot(LShoulderRollM, np.dot(LShoulderPitchM, (x[11] - x[13]))) + x[13]
    x[11] = np.dot(LElbowRollM, np.dot(LElbowYawM, x[11] - x[15])) + x[15]
    #x[12]
    x[12] = np.dot(RShoulderRollM, np.dot(RShoulderPitchM, (x[12] - x[14]))) + x[14]
    x[12] = np.dot(RElbowRollM, np.dot(RElbowYawM, x[12] - x[16])) + x[16]

    return x
