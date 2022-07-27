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
import math
import time

PI = math.pi
TS = 0.05

print ('Program started\n')
sim.simxFinish(-1) # just in case, close all opened connections

clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
    print ('Connected to remote API server\n')

    def cineD(dhMatrix, lista):
        auxMatrix = dhMatrix
        for i in range(len(lista)):
            auxMatrix[i][0] = lista[i]

        listT = []
        for i in range(len(auxMatrix)):
            thetaI = auxMatrix[i][0]
            dI = auxMatrix[i][1]
            aI = auxMatrix[i][2]
            alphaI = auxMatrix[i][3]
            auxT = [
                    [math.cos(thetaI), (-1)*math.sin(thetaI)*math.cos(alphaI),      math.sin(thetaI)*math.sin(alphaI), aI*math.cos(thetaI)],
                    [math.sin(thetaI),      math.cos(thetaI)*math.cos(alphaI), (-1)*math.cos(thetaI)*math.sin(alphaI), aI*math.sin(thetaI)],
                    [               0,                       math.sin(alphaI),                       math.cos(alphaI),                  dI],
                    [               0,                                      0,                                      0,                   1]
            ]
            listT.append(auxT)
        T0_i = np.identity(4)
        for matrix in listT:
            T0_i = np.matmul(T0_i, np.array(matrix))

        np.set_printoptions(precision=3, suppress=True)
        return T0_i, listT

    def jacobian(dhParams, q):
        Jacobian = np.zeros((6, len(q)))
        T_0_4, TransformationsMatrixes = cineD(dhParams, q)
        point_end = T_0_4[0:3, 3]
        T_0_i = np.identity(4)

        for i in range(4):
            if i == 0:
                T_0_i = T_0_i
            else:
                T = TransformationsMatrixes[i-1]
                T_0_i = np.matmul(T_0_i, T)

            z_i = T_0_i[0:3, 2]
            p_i = T_0_i[0:3, 3]
            r = point_end - p_i
            Jacobian[0:3, i] = np.cross(z_i, r)
            Jacobian[3:6, i] = z_i
        return Jacobian
    
    def x_m(dhMatrix, q):
        T_0_4, TransformationsMatrixes = cineD(dhMatrix, q)
        xyz = T_0_4[0:3, 3]
        row = math.atan(T_0_4[2, 1]/T_0_4[2, 2])
        pitch = math.atan((-1)*T_0_4[2, 0]/math.sqrt(T_0_4[2, 1]**2 + T_0_4[2, 2]**2))
        yaw = math.atan(T_0_4[1, 0]/T_0_4[0, 0])
        return np.array([xyz[0], xyz[1], xyz[2], row, pitch, yaw])

    
    returnCode, dummy_handle = sim.simxGetObjectHandle(clientID, '/Dummy', sim.simx_opmode_blocking)
    returnCode, dummy_position = sim.simxGetObjectPosition(clientID, dummy_handle, -1, sim.simx_opmode_blocking)
    returnCode, dummy_orientation = sim.simxGetObjectOrientation(clientID, dummy_handle, -1, sim.simx_opmode_blocking)
    xd = np.array([dummy_position[0], dummy_position[1], dummy_position[2], dummy_orientation[2], dummy_orientation[1], dummy_orientation[0]])

    '''Obter eixos do Robo'''
    returnCode, hip = sim.simxGetObjectHandle(clientID, '/hip', sim.simx_opmode_blocking)
    returnCode, shoulder = sim.simxGetObjectHandle(clientID, '/shoulder', sim.simx_opmode_blocking)
    returnCode, elbow = sim.simxGetObjectHandle(clientID, '/elbow', sim.simx_opmode_blocking)
    returnCode, wrist = sim.simxGetObjectHandle(clientID, '/wrist', sim.simx_opmode_blocking)
    # print(hip, shoulder, elbow, wrist)
    
    '''Obter posição de juntas no startup no startup'''
    def get_joint_positions():
        returnCode, joint1 = sim.simxGetJointPosition(clientID, hip, sim.simx_opmode_blocking)
        returnCode, joint2 = sim.simxGetJointPosition(clientID, shoulder, sim.simx_opmode_blocking)
        returnCode, joint3 = sim.simxGetJointPosition(clientID, wrist, sim.simx_opmode_blocking)
        returnCode, joint4 = sim.simxGetJointPosition(clientID, elbow, sim.simx_opmode_blocking)
        # print('Joints positions: ', joint1, joint2, joint3, joint4)
        return np.array([joint1, joint2, joint3, joint4])

    q = get_joint_positions()
    dh = [
      # [theta,    d,     a,    alpha] 
        [q[0],  0.5,   0.1,   PI/2],
        [q[1],  0.4,   0.1,      0],
        [q[2],  0.4,       0,     0],
        [q[3],  0.05,       0,     0]
    ]

    # sim.simxSetJointTargetPosition(clientID, hip, PI, sim.simx_opmode_blocking)

    while (np.linalg.norm((xd - x_m(dh, q))[0:3]) > 0.001):
    # for i in range(2):
        # print(q)
        xm = x_m(dh, q)
        xdot = xd - xm
        print(f'erro: {xdot}    posição das juntas: {q}')
        Jacobian = jacobian(dh, q)
        Jt = np.linalg.pinv(Jacobian)
        qdot = np.matmul(Jt, xdot)
        qdot.reshape(4, 1)
        # print('qdot: ', qdot)
        q = q + qdot*TS
        # print('q: ', q)

        sim.simxSetJointTargetPosition(clientID, hip, q[0], sim.simx_opmode_blocking)
        sim.simxSetJointTargetPosition(clientID, shoulder, q[1], sim.simx_opmode_blocking)
        sim.simxSetJointTargetPosition(clientID, elbow, q[2], sim.simx_opmode_blocking)
        sim.simxSetJointTargetPosition(clientID, wrist, q[3], sim.simx_opmode_blocking)

        time.sleep(TS)

        q = get_joint_positions()

        
        
        



else:
    print ('Failed connecting to remote API server\n')
print ('\nProgram ended\n')
