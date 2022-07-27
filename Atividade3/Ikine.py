import math
PI = math.pi
def cineI(listaParam):
    x = listaParam[0]
    y = listaParam[1]
    z = (-1)*listaParam[2]
    PHI = listaParam[3]
    l1 = 0.475
    l2 = 0.4

    C2 = ((x**2)+(y**2)-(l1**2)-(l2**2))/(2*l1*l2)
    if C2 > 1 or C2 < -1:
        return "Não alcançável."
    else:
        theta2 = math.atan2(math.sqrt(1-(C2**2)), C2)
        theta1 = math.atan(x/y) - math.atan((l2*math.sin(theta2)) / (l1+(l2*math.cos(theta2))))
        theta3 = PHI - theta1 - theta2
        return [theta1, theta2, theta3, z]

print(f'Entrada 1: {cineI([0.2, 0.1, -0.015, PI/4])}')
print(f'Entrada 2: {cineI([0.5, 0.1, -0.015, PI/4])}')
print(f'Entrada 3: {cineI([0.15, 0.15, 0, PI])}')