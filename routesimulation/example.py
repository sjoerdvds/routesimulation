from routesimulation.model import *
from routesimulation.plot import *
A = Point(3.975766,52.060835)
B = Point(2.0,52.8474)
C = Point(-0.05,53.9866)
D = Point(-1.8250, 57.0212)
E = Point(6.5664,57.9774)

AB = Edge(A,B)
BC = Edge(B,C)
CD = Edge(C,D)
DE = Edge(D,E)

#p = Path([AB, BC, CD, DE], VehicleGenerator(totalVehicles = 100, totalDuration = 1000))
edgesDict = [([AB],100), ([BC],10), ([CD],74), ([DE],25)]
simulation = Simulation(edgesDict = edgesDict, runTime = 1000, maxSpeed = 12, acceleration = 2, deceleration = 2.5, noiseFactor = 5, storeFrames = True)
simulation.run()
Controller(simulation)

#C
#|\
#| \
#|__\
#A   B
# d
# BC = a - dx
# BC' = 10 - dx
# 10 -dx = 0
# dx = 10
# x = 10/d
# AB = 10/d
# AC = 10
# tan ACB = AB/AC
# tan ACB = 10/d/10
# tan ACB = 1/d

# s= 0.5*v*t^2
# v= given
# t= 1/d * v
# s= 0.5*v*(v/d)^2

#----Vector stuff----
#
#   /
#  /
# o
#
#
