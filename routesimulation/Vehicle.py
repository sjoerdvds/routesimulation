# -*- coding: utf-8 -*-
"""
Created on Fri Aug 21 13:24:10 2015

@author: DobrkovicA
"""

from routesimulation.model import *
import numpy as np

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
    
        self.points = path.getPoints()
        self.position = self.points[0]
        if len(self.points) > 1:
            self.destinationPoint = 1
            self.destination = self.points[desinationPoint]
            self.status = "ready"
        else:
            self.status = "arrived"
            
    def update(self):
        # update vessel position (and status) if ready or sailing
        if (self.status == "ready") or (self.status == "sailing"):
            # get direction and random vector
            direction = Vector.fromSimPoints(position, destination)
            noise = Vector.getRandom(self.noiseFactor)
            #self.calculate_vectors()
            #self.randomize_vector(self.random_factor)
            
            # check if within destination distance (note: we use intensity of vecnorm and compare it with "speed" (also intensity)
            
            if self.vec_norm <= self.speed:
                self.status = "arrived"
                self.position = self.end
            # calculate new position by summing direction and random vector; direction vectory (identity) is multiplied by "speed" (intensity)
            else:
                # !!!! correct speed later !!!!
                self.speed = self.maxSpeed
                movement = direction.getIdentity() * self.speed + noise
                self.position = movement.translateSimPoint(self.position)
                self.status = "sailing"
                
                #newX = self.position.lon + self.
                #newX = self.position[0] + self.vec_dir[0] * self.speed + self.ran_vec[0]
                #newY = self.position[1] + self.vec_dir[1] * self.speed + self.ran_vec[1]
                #self.position = (newX, newY)
                #self.status = "sailing"
        
        
"""
    # update all vectors in order to get ship directional vectory (used to calculate nex position)
    def calculate_vectors(self):
        # create trajectory vector from current position to desired end point
        self.vec_trip = (self.end[0] - self.position[0],    self.end[1] - self.position[1])
        # normalize trajectory vector ...
        self.vec_norm = np.sqrt(self.vec_trip[0] ** 2 + self.vec_trip[1] ** 2)
        # .. and create directional vectory that is identity vector of the trajectory vector
        self.vec_dir = (self.vec_trip[0] / self.vec_norm,   self.vec_trip[1] / self.vec_norm)

    # random vector used to displace new position by random factor
    def randomize_vector(self, scale):
        # get x, y values in range -0.5 to 0.5
        x = random.random() - 0.5
        y = random.random() - 0.5
        # normalize to get identity vector
        vec_norm = np.sqrt(x ** 2   +   y ** 2)
        self.ran_vec = (x / vec_norm * scale,  y / vec_norm * scale)
    
        
    def update(self):
        # update vessel position (and status) if ready or sailing
        if (self.status == "ready") or (self.status == "sailing"):
            # get direction and random vector
            self.calculate_vectors()
            self.randomize_vector(self.random_factor)
            
            # check if within destination distance (note: we use intensity of vecnorm and compare it with "speed" (also intensity)
            if self.vec_norm <= self.speed:
                self.status = "arrived"
                self.position = self.end
            # calculate new position by summing direction and random vector; direction vectory (identity) is multiplied by "speed" (intensity)
            else:
                newX = self.position[0] + self.vec_dir[0] * self.speed + self.ran_vec[0]
                newY = self.position[1] + self.vec_dir[1] * self.speed + self.ran_vec[1]
                self.position = (newX, newY)
                self.status = "sailing"
        
    def plot(self, m):
        x,y = m(self.position[0], self.position[1])
        plt.plot(x, y, 'o', markerfacecolor=self.color, markeredgecolor='k', markersize=4)
        
"""

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

V1 = Vehicle(1, p, 10, 1, 1, 3)

a = Vector(5,6)

A = (5,4)
B = (3,4)
b = Vector.fromPoints(A,B)

C = p.getPoints()[1]
D = p.getPoints()[2]
c= Vector.fromSimPoints(C,D)