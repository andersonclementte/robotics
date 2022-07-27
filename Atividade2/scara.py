'''IMPORTS E CONSTANTES'''
import roboticstoolbox as rtb
import math, matplotlib as plt
pi = math.pi

robot = rtb.models.DH.Puma560()
print(robot)
robot.plot([0,0,0,0,0,0])