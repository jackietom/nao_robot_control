from calcAnkle import *
from judge import *
from calcAnkleLine import *
import os

#parameters
roLHipPitch = 0.6
roLHipRoll = 0.1
roRHipPitch = 0.6
roRHipRoll = 0.1
roLKnee = 0
roRKnee = 0

Theta = np.array([roLHipPitch,roLHipRoll,roRHipPitch,roRHipRoll,roLKnee,roRKnee])
result = calcAnkleLine(Theta)
result = judge(result,Theta)

print(result)




