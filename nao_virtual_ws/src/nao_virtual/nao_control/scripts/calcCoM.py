import numpy as np
from LowerHalfTrans import lowerHalfComTrans
from convertCod import *
def calcCom(Theta):
    #center of mass of different parts in local coordinates
    #reference http://doc.aldebaran.com/2-1/family/nao_h25/masses_h25_v4.html#h25-masses-v4
    ComTorso = np.array([-0.00413, 0, 0.04342])
    MTorso = 1.0496
    ComNeck = np.array([-10**(-5), 0, -0.02742])
    MNeck = 0.06442
    ComHead = np.array([-0.00112, 0, 0.05258])
    MHead = 0.60533
    ComRShoulder = np.array([-0.00165, 0.02663,0.00014])
    MRShoulder = 0.07504
    ComLShoulder = np.array([-0.00165, -0.02663,0.00014])
    MLShoulder = 0.07504
    ComRBiceps = np.array([0.02455, -0.00563, 0.0033])
    MRBiceps = 0.15777
    ComLBiceps = np.array([0.02455, 0.00563, 0.0033])
    MLBiceps = 0.15777
    ComRElbow = np.array([-0.02744, 0, -0.00014])
    MRElbow = 0.06483
    ComLElbow = np.array([-0.02744, 0, -0.00014])
    MLElbow = 0.06483
    ComRForeArm = np.array([0.02556, -0.00281, 0.00076])
    MRForeArm = 0.07761
    ComLForeArm = np.array([0.02556, 0.00281, 0.00076])
    MLForeArm = 0.07761
    ComRHand = np.array([0.03434, 0.00088, 0.00308])
    MRHand = 0.18533
    ComLHand = np.array([0.03434, -0.00088, 0.00308])
    MLHand = 0.18533
    ComRPelvis = np.array([-0.00781, 0.01114, 0.02661])
    MRPelvis = 0.06981
    ComLPelvis = np.array([-0.00781, -0.01114, 0.02661])
    MLPelvis = 0.06981
    ComRHip = np.array([-0.01549, -0.00029, -0.00515])
    MRHip = 0.13053
    ComLHip = np.array([-0.01549, 0.00029, -0.00515])
    MLHip = 0.13053
    ComRThigh = np.array([0.00138, -0.00221, -0.05373])
    MRThigh = 0.38968
    ComLThigh = np.array([0.00138, 0.00221, -0.05373])
    MLThigh = 0.38968
    ComRTibia = np.array([0.00453, -0.00225,-0.04936])
    MRTibia = 0.29142
    ComLTibia = np.array([0.00453, 0.00225,-0.04936])
    MLTibia = 0.29142
    ComRAnkle = np.array([0.00045, -0.00029, 0.00685])
    MRAnkle = 0.13416
    ComLAnkle = np.array([0.00045, 0.00029, 0.00685])
    MLAnkle = 0.13416
    ComRFeet = np.array([0.02542, -0.0033, -0.03239])
    MRFeet = 0.16184
    ComLFeet = np.array([0.02542, 0.0033, -0.03239])
    MLFeet = 0.16184

    #*****************************************************
    #Joint location
    JTorso = np.array([0,0,0])
    JHead = np.array([0,0,126.5])
    JLShoulder = np.array([0, 98, 100])
    JRShoulder = np.array([0,-98, 100])
    JLElbow = np.array([105, 15, 0]) + JLShoulder
    JRElbow = np.array([105, -15, 0]) + JRShoulder
    JLHand = np.array([55.95, 0, 0]) + JLElbow
    JRHand = np.array([55.95, 0, 0]) + JRElbow
    JLHip = np.array([0, 50, -85])
    JRHip = np.array([0,-50, -85])
    JLKnee = np.array([0, 0, -100]) + JLHip
    JRKnee = np.array([0, 0, -100]) + JRHip
    JLAnkle = np.array([0, 0, -102.9]) + JLKnee
    JRAnkle = np.array([0, 0, -102.9]) + JRKnee

    #Com of Upper body
    MU = MTorso + MNeck + MHead + MLShoulder + MRShoulder + MLBiceps + MRBiceps + MLElbow + MRElbow \
        + MLForeArm + MRForeArm + MLHand + MRHand

    ComU = (ComTorso+JTorso)*MTorso/MU + (ComNeck + JHead)*MNeck/MU \
        + (ComHead + JHead)*MHead/MU + (ComLShoulder + JTorso)*MLShoulder/MU \
        + (ComRShoulder + JTorso)*MRShoulder/MU + (ComLBiceps + JLShoulder)*MLBiceps/MU \
        + (ComRBiceps + JRShoulder)*MRBiceps/MU + (ComLElbow + JLShoulder)*MLElbow/MU \
        + (ComRElbow + JRShoulder)*MRElbow/MU + (ComLForeArm + JLShoulder)*MLForeArm/MU \
        + (ComRForeArm + JRShoulder)*MRForeArm/MU + (ComLHand + JLElbow)*MLHand/MU \
        +(ComRHand + JRElbow)*MRHand/MU

    MU = np.array([MU])

    #Com of lower half
    ML = MLPelvis + MRPelvis + MLHip + MRHip + MLThigh + MRThigh + MLTibia + MRTibia + MLAnkle + MRAnkle\
    + MLFeet + MRFeet

    #ComL = (ComLPelvis + JLHip)*MLPelvis/ML + (ComRPelvis + JRHip)*MRPelvis/ML\
    #+(ComLHip + JLHip)*MLHip/ML + (ComRHip + JRHip)*MRHip/ML + (ComLThigh + JLHip)*MLThigh/ML \
    #+(ComRThigh + JRHip)*MRThigh/ML + (ComLTibia + JLKnee)*MLTibia/ML + (ComRTibia + JRKnee)*MRTibia/ML \
    #+(ComLAnkle + JLKnee)*MLAnkle/ML + (ComRAnkle + JRKnee)*MRAnkle/ML + (ComLFeet + JLAnkle)*MLFeet/ML \
    #+(ComRFeet + JRAnkle)*MRFeet/ML

    #x[0]:RPelvis, x[1]:LPevis, x[2]:RHip, x[3]:LHip,x[4]:RThigh, x[5]:LThigh --> Hip
    #x[6]:RTibia, x[7]:LTibia, x[8]:RAnkle, x[9]:LAnkle --> Knee
    #x[10]:RFoot, x[11]:LFoot --> Ankle (Knee instead for convenience)
    #x[12] --> x1(LHip), x[13] --> x2(RKnee), x[14] --> x3(LKnee)
    x = np.array([ComRPelvis + JRHip, ComLPelvis + JLHip, ComRHip + JRHip, ComLHip + JLHip, ComRThigh + JRHip \
                  , ComLThigh + JLHip, ComLTibia + JLKnee, ComRTibia + JRKnee, ComRAnkle + JRKnee, \
                  ComLAnkle + JLKnee, ComRFeet + JRAnkle, ComLFeet + JLAnkle, JLHip, JRKnee, JLKnee])

    x2 = x
    for i in range(np.size(x,0)):
        x[i] = convCod2book(x[i])

    x = lowerHalfComTrans(x, Theta)
    for i in range(np.size(x,0)):
        x[i] = convCod2doc(x[i])


    ComL = x[1]*MLPelvis/ML + x[0]*MRPelvis/ML\
    +x[3]*MLHip/ML + x[2]*MRHip/ML + x[5]*MLThigh/ML \
    +x[4]*MRThigh/ML + x[7]*MLTibia/ML + x[6]*MRTibia/ML \
    +x[9]*MLAnkle/ML + x[8]*MRAnkle/ML + x[11]*MLFeet/ML \
    +x[10]*MRFeet/ML

    M = ML + MU
    Com = ComU*MU/M + ComL*ML/M

    return np.concatenate((Com,M))

#test function
#a = calcCom(np.array([0,0,0,0,0,0])
#print(a)


