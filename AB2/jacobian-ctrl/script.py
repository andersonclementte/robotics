# simRemoteApi.start(19999)
try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import numpy as np

print ('Program started\n')
sim.simxFinish(-1) # just in case, close all opened connections

clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
    print ('Connected to remote API server\n')

    '''Obter posição e orientação relativa ao cenário'''
    returnCode, dummyHandle = sim.simxGetObjectHandle(clientID, '/MTB/suctionPad/LoopClosureDummy2', sim.simx_opmode_blocking)
    returnCode, dummyPosition = sim.simxGetObjectPosition(clientID, dummyHandle, -1, sim.simx_opmode_blocking)
    returnCode, dummyOrientation = sim.simxGetObjectOrientation(clientID, dummyHandle, -1, sim.simx_opmode_blocking)
    dummyPose = [x for x in dummyPosition+dummyOrientation]
    print(f'x: {dummyPose[0]:.4f}, y: {dummyPose[1]:.4f}, z: {dummyPose[2]:.4f}\nalfa: {dummyPose[3]:.2f}, beta: {dummyPose[4]:.2f}, gama: {dummyPose[5]:.2f}')

    '''Obter eixos do Robo'''
    returnCode, axis1 = sim.simxGetObjectHandle(clientID, '/MTB/axis', sim.simx_opmode_blocking)
    returnCode, axis2 = sim.simxGetObjectHandle(clientID, 'MTB_axis2', sim.simx_opmode_blocking)
    returnCode, axis3 = sim.simxGetObjectHandle(clientID, 'MTB_axis3', sim.simx_opmode_blocking)

    # '''Valores para movimentação de eixos'''
    # q1 = 1 * np.pi/2
    # q2 = (-1) * np.pi/2
    # q3 = 0.05
    # '''aplica alterações'''
    # returnCode = sim.simxSetJointPosition(clientID, axis1, q1, sim.simx_opmode_oneshot)
    # returnCode = sim.simxSetJointPosition(clientID, axis2, q2, sim.simx_opmode_oneshot)
    # returnCode = sim.simxSetJointPosition(clientID, axis3, q3, sim.simx_opmode_oneshot)
    
    # '''Define posição'''
    # returnCode = sim.simxSetObjectPosition(clientID, dummyHandle, -1, [0.2, 0.1,-0.015,], sim.simx_opmode_oneshot)
    
else:
    print ('Failed connecting to remote API server\n')
print ('\nProgram ended\n')
