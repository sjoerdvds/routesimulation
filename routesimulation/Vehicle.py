# -*- coding: utf-8 -*-
"""
Created on Fri Aug 21 13:24:10 2015

@author: DobrkovicA
"""

from routesimulation.model import *
import numpy as np
import matplotlib.pyplot as plt

class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
    # creates vector from two points (touples)
    def fromPoints(a, b):
        return Vector(b[0] - a[0], b[1] - a[1])

    # creates vector from two class (routesimulation) points        
    def fromSimPoints(a, b):
        return Vector(b.lon - a.lon, b.lat - a.lat)

    #creates random vector with given scale (random identity vector * scale factor)    
    def getRandom(scale):
        tempVector = Vector(random.random() - 0.5, random.random() - 0.5)
        return tempVector.getIdentity() * scale
    
    
        
    def __str__(self):
        return "Vector[%f,%f]"%(self.x, self.y)

    def __repr__(self):
        return str(self)
    
    def __add__(self, vec2):
        return Vector(self.x + vec2.x, self.y + vec2.y)
        
    def __sub__(self, vec2):
        return Vector(self.x - vec2.x, self.y - vec2.y)
        
    def __mul__(self, val):
        return Vector(self.x * val, self.y * val)
        
    def __div__(self, val):
        return Vector(self.x / val, self.y / val)
        
        
        
    def getNorm(self):
        return np.sqrt(self.x ** 2 + self.y ** 2)
        
    def getIdentity(self):
        norm = self.getNorm()
        return Vector(self.x / norm, self.y / norm)
        
        
        
        
    def getDotProduct(vec1, vec2):
        return (vec1.x * vec2.x + vec1.y * vec2.y)
        
    # calculates angle between two vectors (in radians)
    def getAngleRad(vec1, vec2):
        """
        dot(A,B) = |A| * |B| * cos(angle) 
        which can be rearranged to 
        angle = arccos(dot(A,B) / (|A|* |B|))
        """
        return np.arccos(Vector.getDotProduct(vec1, vec2) / (vec1.getNorm() * vec2.getNorm()))
        
    # calculates angle between two vectors (in degrees)
    def getAngle(vec1, vec2):
        return Vector.getAngleRad(vec1,vec2) * 180 / np.pi
        
    
    def translateSimPoint(self, pt):
        return Point(pt.lon + self.x, pt.lat + self.y)
        
        
        
class Vehicle:    
    def __init__(self, vehicleId, path, maxSpeed, acceleration, deceleration, noiseFactor):
        self.vehicleId = vehicleId
        self.maxSpeed = maxSpeed
        self.acceleration = acceleration
        self.deceleration = deceleration
        
        self.noiseFactor = noiseFactor
        
        self.speed = 0
        self.speedStatus = "accelerate"
    
        self.points = path.getPoints()
        self.position = self.points[0]
        if len(self.points) > 1:
            self.destinationPoint = 1
            self.destination = self.points[self.destinationPoint]
            self.status = "ready"
        else:
            self.status = "arrived"
            
    def update(self):
        # update vessel position (and status) if ready or sailing
        if (self.status == "ready") or (self.status == "sailing"):
            # get direction and random vector
            direction = Vector.fromSimPoints(self.position, self.destination)
            noise = Vector.getRandom(self.noiseFactor)
            
            """
            # check if the vessel enters "final" deceleration stage
            if self.speedStatus == "decelerate":
                self.speed -= self.deceleration
                if self.speed < 0:
                    self.speed = 0
                    self.status = "arrived"
                    print ("speed:" + str(self.speed))
                    print (self.status)
            else:
                if self.destinationPoint == len(self.points) - 1:
                    magnitude = direction.getNorm()
                    decPoint = (magnitude ** 2) / (2 * self.deceleration)
                    if decPoint > magnitude:
                        #self.speed -= self.deceleration
                        self.speedStatus = "decelerate"
            """
            
            
            # check if within destination distance (note: we use intensity of direction vector and compare it with "speed" (also intensity)
            if direction.getNorm() <= self.speed:
                # adjust position and acquire new destination point
                self.position = self.destination
                self.destinationPoint += 1
                # check if this is the end of the route
                if self.destinationPoint >= len(self.points):
                    self.status = "arrived"
                else:
                    self.destination = self.points[self.destinationPoint]
            # calculate new position by summing direction and random vector; direction vectory (identity) is multiplied by "speed" (intensity)
            else:
                # !!!! correct speed later !!!!
                # add call to function that decides if acceleration or deceleration is needed
                if self.speed <= self.maxSpeed - self.acceleration:
                    self.speed += self.acceleration
                #self.adjustSpeed()
            
                movement = direction.getIdentity() * self.speed + noise
                self.position = movement.translateSimPoint(self.position)
                self.status = "sailing"
                
    def adjustSpeed(self):
        if self.speedStatus == "accelerate":
            self.speed += self.acceleration
            if self.speed > self.maxSpeed:
                self.speed = self.maxSpeed
                self.speedStatus = "cruise"
                
    def getDecelerationPoint(self, direction):
        p = direction.getIdentity ^2 / (2 * self.deceleration)
        return p
                
                
    def debugPlot(self, color):
        x = self.position.lon
        y = self.position.lat
        plt.plot(x, y, 'o', markerfacecolor=color, markeredgecolor='k', markersize=4)
        
        



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
p2 = Path([BC,CD], VehicleGenerator(totalVehicles = 50, totalDuration = 500))

V1 = Vehicle(1, p, 0.8, 0.01, 0.2, 0.01)
V2 = Vehicle(2, p2, 0.2, 0.01, 0.2, 0.01)

plt.plot(A.lon, A.lat, 'o', markerfacecolor='k', markeredgecolor='k', markersize=8)
plt.plot(B.lon, B.lat, 'o', markerfacecolor='k', markeredgecolor='k', markersize=8)
plt.plot(C.lon, C.lat, 'o', markerfacecolor='k', markeredgecolor='k', markersize=8)
plt.plot(D.lon, D.lat, 'o', markerfacecolor='k', markeredgecolor='k', markersize=8)
plt.plot(E.lon, E.lat, 'o', markerfacecolor='k', markeredgecolor='k', markersize=8)

V1.debugPlot('r')
#while (V1.status != "arrived"):
for i in range(0,100):
    V1.update()
    V1.debugPlot('r')

"""    
V2.debugPlot('b')
while (V2.status != "arrived"):
    V2.update()
    V2.debugPlot('b')
"""    
