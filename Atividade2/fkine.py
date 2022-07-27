import numpy as np
import math

PI = math.pi

def cineD(dhMatrix, lista):
    auxMatrix = dhMatrix
    for i in range(len(lista)-1):
        auxMatrix[i][0] = lista[i]
    auxMatrix[2][1] = lista[-1]
    print(auxMatrix)

    

    listT = []
    for i in range(len(auxMatrix)):
        thetaI = auxMatrix[i][0]
        dI = auxMatrix[i][1]
        aI = auxMatrix[i][2]
        alphaI = auxMatrix[i][3]
        auxT = [
                [math.cos(thetaI), (-1)*math.sin(thetaI)*math.cos(alphaI),
                    math.sin(thetaI)*math.sin(alphaI), aI*math.cos(thetaI)],
     
                [math.sin(thetaI), math.cos(thetaI)*math.cos(alphaI),
                    (-1)*math.cos(thetaI)*math.sin(alphaI), aI*math.sin(thetaI)],
                [0, math.sin(alphaI), math.cos(alphaI), dI],
                [0, 0, 0, 1]
        ]
        listT.append(auxT)
    T = np.identity(4)
    for matrix in listT:
        T = np.matmul(T, np.array(matrix))

    np.set_printoptions(precision=3, suppress=True)
    print(T)


    


dh = [
   # [theta, d, a, alpha] 
    [0, 0, 0.475, 0],
    [0, 0, 0.4, 0],
    [0, 0, 0, PI],
    [0, 0.1, 0, 0]
]

#theha1, thetha2, theta3, d
a = [0, 0, 0, 0]
b = [PI/2, -PI/2, 0, 0]
c = [PI/2, -PI/2, 0, 0.05]


cineD(dh, c)
