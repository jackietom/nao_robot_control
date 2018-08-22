from calcAnkle import *
from judge import *
from calcAnkleLine import *
import os

#parameters
#Lower part
roLHipPitch = 0
roLHipRoll = 0
roRHipPitch = 0
roRHipRoll = 0
roLKnee = 0
roRKnee = 0
#Upper part
roLShoulderPitch = 0
roLShoulderRoll = 0
roRShoulderPitch = 0
roRShoulderRoll = 0
roLElbowYaw = 0
roLElbowRoll = 0
roRElbowYaw = 0
roRElbowRoll = -1


ThetaL = np.array([roLHipPitch,roLHipRoll,roRHipPitch,roRHipRoll,roLKnee,roRKnee])
ThetaU = np.array([roLShoulderPitch, roLShoulderRoll, roRShoulderPitch, roRShoulderRoll,
                   roLElbowYaw, roLElbowRoll, roRElbowYaw, roRElbowRoll])
result = calcAnkleLine(ThetaL, ThetaU)
result = judge(result,ThetaL, ThetaU)

print(result)




