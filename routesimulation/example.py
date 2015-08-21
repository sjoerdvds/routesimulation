from routesimulation.model import *
A = Point(3.975766,52.060835)
B = Point(2.0,52.8474)
C = Point(-0.05,53.9866)
D = Point(-1.8250, 57.0212)
E = Point(6.5664,57.9774)

AB = Edge(A,B)
BC = Edge(B,C)
CD = Edge(C,D)
DE = Edge(D,E)

p = Path([AB, BC, CD, DE], VehicleGenerator(totalVehicles = 100, totalDuration = 1000))