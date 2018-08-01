import numpy as np
from LowerHalfTrans import *
from calcCoM import *
def calcAnkleLine(ThetaL, ThetaU):
    x0 = np.array([0,0,0])
    x1 = np.array([0,100,0])
    x2 = np.array([100,0,0])
    x3 = np.array([100,100,0])
    x4 = np.array([202.9,0,0])
    x5 = np.array([202.9,100,0])
    x4_1 = np.array([202.9, -25, 25])
    x4_2 = np.array([202.9, -25, -10])
    x5_1 = np.array([202.9, 125, 25])
    x5_2 = np.array([202.9, 125, -10])

    l1 = 100
    l2 = 102.9
    x = np.stack((x0,x1,x2,x3,x4,x5,x4_1,x4_2,x5_1,x5_2))
    x = lowerHalfTrans(x, ThetaL)
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
    #get equation
    Com = calcCom(ThetaL, ThetaU)
    M = Com[3]
    Com = Com[0:3]
    Com = convCod2book(Com)

    cp = np.cross(Com - x4_1, x5_1 - x4_1)
    cp  = np.cross(cp, x5_1 - x4_1)
    cp = cp/np.linalg.norm(cp)
    a, b, c = cp

    if a <= 0:
        a = -a
        b = -b
        c = -c

    # This evaluates a * x3 + b * y3 + c * z3 which equals d

    vL = x5 - x3
    thetaLPitch = -atan(c/a) + atan(vL[2]/(vL[0]+0.0001))
    thetaLRoll = atan(b/sqrt(a**2+c**2)) - atan(vL[1]/(sqrt(vL[0]**2+vL[2]**2)))

    vR = x4 - x2
    thetaRPitch = -atan(c/a) + atan(vR[2]/(vR[0]+0.0001))
    thetaRRoll = atan(b/sqrt(a**2+c**2)) - atan(vR[1]/(sqrt(vR[0]**2+vR[2]**2)))

    result = [a,b,c,thetaLPitch, thetaLRoll, thetaRPitch, thetaRRoll, \
                       x0,x1,x2,x3,x4,x5,x4_1,x4_2,x5_1,x5_2]

    print(thetaRRoll, thetaLRoll, thetaLPitch, thetaRPitch)
    return result
