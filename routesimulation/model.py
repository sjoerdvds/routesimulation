"""
.. module:: model
   :platform: Unix, Windows
   :synopsis: Data model for route simulation

.. moduleauthor:: Sjoerd van der Spoel <s.j.vanderspoel@utwente.nl>
.. moduleauthor:: Andrej Dobrkovic <a.dobrkovic@utwente.nl>
"""
from math import radians, sin, cos, sqrt, asin, fabs, atan2, pi, tan
import random
import operator

class Edge:
    """A directed edge between two points

    :param start: Starting point of the edge
    :type start: :class:`model.Point` 
    :param end: End point of the edge.
    :type end: :class:`model.Point` 

    """
    def __init__(self, start, end):
        self.start = start
        self.end = end

    def __eq__(self, other):
        return isinstance(other, Edge) and other.start == self.start and other.end == self.end

    def __hash__(self):
        return hash(str(self.start) + str(self.end))

    def __repr__(self):
        return repr(self.start) + "->" + repr(self.end)

    
    def bearing(self):
        """Gives the bearing of this Edge from start to end in radians

        :returns: float -- The Edge bearing in radians 
        """
        start = self.start.toRadians()
        end = self.end.toRadians()
        lat1 = start.lat
        lon1 = start.lon
        lat2 = end.lat
        lon2 = end.lon

        return atan2(sin(lon2-lon1) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1))

    
    def length(self):
        """Gives the length of this Edge in meters, i.e. the Haversine distance between its start and end points

        :returns: float -- The length of this Edge in meters
        """
        return self.start.distanceTo(self.end)

    def toVector(self):
        pass

    
    def pointAtDistance(self, distance, R = 6378137):
        """Gives a :class:`model.Point` on this Edge at the specified distance from start

        :param distance: The distance in meters from the start of the edge
        :type distance: float
        :param R: The circumference of the Earth at the coordinates of the edge. 
        :type R: float
        :returns: :class:`model.Point` -- A Point p on this Edge such that ``p.distanceTo(self.start)==distance``. If distance is greater than ``self.length()``, returns ``self.end``.
        """
        if self.length() > distance:
            #print(self, "Calculating point at distance", distance)
            b = self.bearing()
            d = distance/R
            start = self.start.toRadians()
            
            lon = start.lon
            lat = start.lat
            
            lat2 = asin(sin(lat) * cos(d) + cos(lat) * sin(d) * cos(b))
            lon2 = lon + atan2(sin(b) * sin(d) * cos(lat), cos(d) - sin(lat) * sin(lat2))
            return Point(lon2, lat2).toDegrees()
        else:
            return self.end

class Trip:
    """A sequence of :class:`model.Point` s

    :param points: An ordered list of :class:`model.Point` s
    :type points: list

    """
    def __init__(self, points):
        self.points = points

   
class Point:
    """A point in two-dimensional space

    :param lon: Longitude (x)
    :type lon: float
    :param lat: Latitude (y)
    :type lat: float
    """
    def __init__(self,lon, lat):
        self.lon = lon;
        self.lat = lat;

    def longitude(self):
        """The longitude (x-axis) of this Point

        :returns: float -- The Point longitude
        """
        return self.lon;

    def latitude(self):
        """The latitude (y-axis) of this Point

        :returns: float -- The Point latitude
        """
        return self.lat;

    def degreeRadianConversion(self, degreesToRadians = True):
        """Create a **copy** of this Point converted to degrees (range -180:180) or radians (range -pi:pi)

        :param degreesToRadians: If ``True``, create a copy with radians assuming this point uses degrees, or vice versa otherwise.
        :type degreesToRadians: bool
        :returns: :class:`model.Point`--A converted Point copy
        """
        conv = pi/180 if degreesToRadians else 180/pi
        lon = self.lon * conv
        lat = self.lat * conv
        return Point(lon, lat)

    def toRadians(self):
        """Create a **copy** of this Point converted to radians

        :returns: :class:`model.Point`--A converted Point copy
        """
        return self.degreeRadianConversion(degreesToRadians = True)

    def toDegrees(self):
        """Create a **copy** of this Point converted to degrees

        :returns: :class:`model.Point`--A converted Point copy
        """
        return self.degreeRadianConversion(degreesToRadians = False)

    def toCartesian(self, R = 6378137):
        """Calculate the position of this Point in Cartesian space

        :param R: The circumference of the Earth at the coordinates of this point.
        :type R: float
        :returns: list -- x, y, and z coordinates of this Point in Cartesian space
        """
        self = self.toRadians()
        x = R * cos(self.lat) * cos(self.lon)
        y = R * cos(self.lat) * sin(self.lon)
        z = R * sin(self.lat)
        return [x, y, z]


    def fromCartesian(vector, R = 6378137):
        """Create a two-dimensional Point from a point in Cartesian space

        :param vector: x, y and z coordinates of a point in Cartesian space
        :type vector: list
        :param R: The circumference of the Earth at the coordinates of this point. 
        :type R: float
        :returns: :class:`model.Point` -- A Point in two-dimensional space
        """
        x = vector[0]
        y = vector[1]
        z = vector[2]
        lat = asin(z/R) * 180/pi
        lon = atan2(y, x) * 180/pi
        return Point(lon, lat)

    def north(self, distance):
        """Create a Point at the specified distance due north of this Point

        :param distance: Distance in meters
        :type distance: float
        :returns: :class:`model.Point` -- A Point north of this point
        """
        return self.pointAtDistance(distance, bearing = 0)

    def south(self, distance):
        """Create a Point at the specified distance due sourth of this Point

        :param distance: Distance in meters
        :type distance: float
        :returns: :class:`model.Point` -- A Point south of this point
        """
        return self.pointAtDistance(distance, bearing = pi)

    def east(self, distance):
        """Create a Point at the specified distance due east of this Point

        :param distance: Distance in meters
        :type distance: float
        :returns: :class:`model.Point` -- A Point east of this point
        """
        return self.pointAtDistance(distance, bearing = 0.5*pi)        

    def west(self, distance):
        """Create a Point at the specified distance due west of this Point

        :param distance: Distance in meters
        :type distance: float
        :returns: :class:`model.Point` -- A Point west of this point
        """
        return self.pointAtDistance(distance, bearing = 1.5 * pi)

    def distanceTo(self, other, R = 6378137):
        """Calculate the Haversine distance between this Point and another Point

        :param other: The other Point
        :type other: :class:`model.Point`
        :param R: The circumference of the Earth at the coordinates of this point. 
        :type R: float
        :returns: float -- The distance in meters
        """
        dLat = radians(other.lat - self.lat)
        dLon = radians(other.lon - self.lon)
        lat0 = radians(self.lat)
        lat1 = radians(other.lat)

        a = sin(dLat/2)**2 + cos(lat0)*cos(lat1)*sin(dLon/2)**2
        c = 2*asin(sqrt(a))

        return R * c

    def pointAtDistance(self, distance, bearing, R = 6378137):
        """Create a Point at a specified distance and bearing from this Point

        :param distance: Distance in meters
        :type distance: float
        :param bearing: Bearing in radians
        :type bearing: float
        :param R: The circumference of the Earth at the coordinates of this point. 
        :type R: float
        :returns: :class:`model.Point` -- A Point at the specified distance and bearing from this Point
        """
        d = distance/R
        b = bearing
        point = self.toRadians()
            
        lon =  point.lon
        lat =  point.lat
            
        lat2 = asin(sin(lat) * cos(d) + cos(lat) * sin(d) * cos(b))
        lon2 = lon + atan2(sin(b) * sin(d) * cos(lat), cos(d) - sin(lat) * sin(lat2))
        return Point(lon2, lat2).toDegrees()

    def __str__(self):
        return "Point(x = %f y = %f)"%(self.lon, self.lat)

    def __repr__(self):
        return str(self)

    def __eq__(self, other):
        return isinstance(other, Point) and other.lon == self.lon and other.lat == self.lat
  
    def __hash__(self):
        return hash(str(self))


class Graph:
    """A Graph of Edges

    :param edges: A list of edges
    :type edges: list
    """
    def __init__(self, edges):
        self.edges = edges
        self.updateNodes()
        self.randomizeEdgeLengths()


    def updateNodes(self):
        """Determine what the nodes are of this Graph. 

        This method should be called after changing the Graph's list of edges
        """
        nodes = set()
        for edge in self.edges:
            nodes.add(edge.start)
            nodes.add(edge.end)

        self.nodes = nodes

    def randomizeEdgeLengths(self):
        lengths = {}
        for edge in self.edges:
            lengths[edge] = random.randrange(1,100)

        self.lengths = lengths

    def randomPath(self, start, end):
        self.randomizeEdgeLengths()
        return self.shortestPath(start, end)

    def nodeOutDegrees(self):
        degrees = dict(zip(self.nodes, [0] * len(self.nodes)))
        for edge in self.edges:
            outDegree = degrees[edge.start]
            outDegree = outDegree + 1
            degrees[edge.start] = outDegree
        return degrees

    def shortestPath(self, start, end):
        unvisited = []
        dist = {}
        prev = {}

        dist[start] = 0
        for node in self.getNodes():
            if node is not start:
                dist[node] = float("inf")
                prev[node] = None

            unvisited.append(node)

        while len(unvisited) > 0:
            # Sort the distances
            unvisited_dist = dict([(node, dist[node]) for node in unvisited if node in dist])
            dist_sorted = sorted(unvisited_dist.items(), key = operator.itemgetter(1))

            minDistNode = dist_sorted[0][0]
            
            unvisited.remove(minDistNode)

            for neighbor in self.neighbors(minDistNode):
                if neighbor in unvisited:
                    alt = dist[minDistNode] + self.length(minDistNode, neighbor)
                    if alt < dist[neighbor]:
                        dist[neighbor] = alt
                        prev[neighbor] = minDistNode

        path = []
        node = end
        #print(prev)
        while node is not None:
            path = [node] + path
            if node in prev.keys():
                node = prev[node]
            else:
                node = None
        # now make the path into edges
        pathEdges = []
        for i in range(0, len(path)-1):
            pathEdges.append(Edge(path[i], path[i+1]))
        return pathEdges


    def length(self, start, end):
        """Determine the length of the Edge between start and end

        :param start: A start point
        :type start: :class:`model.Point`
        :param end: An end point
        :type end: :class:`model.Point`
        :returns: float -- Length of the shortest edge if start and end are on an edge in this Graph, or ``None`` otherwise.
        """
        edgeLengths = []
        edges = [Edge(start,end), Edge(end, start)]

        for edge in edges:
            if edge in self.lengths.keys():
                edgeLengths.append(self.lengths[edge])

        if len(edgeLengths) > 0:
            return min(edgeLengths)
        else:
            return None    
        

    def connectedEdges(self, node):
        connected = set()
        for edge in self.edges:
            if edge.start == node or edge.end == node:
                connected.add(edge)
        return connected

    def neighbors(self, node):
        connectedEdges = self.connectedEdges(node)
        neighbors = set()
        for edge in connectedEdges:
            neighbors.add(edge.start)
            neighbors.add(edge.end)
        # The node itself is not its neighbor.
        neighbors.remove(node)
        return neighbors

    def getNodes(self):
        return self.nodes

class Simulation:
    # Create a new simulation object
    def __init__(self, edgesDict, runTime, maxSpeed, acceleration, deceleration, noiseFactor, storeFrames):
        """
        :param edgesDict: Dictionary with a list of edges as key and a number of trucks on those edges as value
        :type edgesDict: dict
        :param runTime: Total running time of the simulation
        :type runTime: int
        :param maxSpeed: Maximum speed of vehicles in the simulation
        :param acceleration: Maximum acceleration of vehicles in the simulation
        :param deceleration: Maximum deceleration of vehicles in the simulation
        :param noiseFactor: Factor of noise to be added to vehicle movement
        :param storeFrames: Should all frames (snapshots of vehicle positions) be stored?
        """
        self.runTime = runTime
        self.vehicles = []
        self.paths = []
        self.maxSpeed = maxSpeed
        self.acceleration = acceleration
        self.deceleration = deceleration
        self.noiseFactor = noiseFactor
        
        self.storeFrames = storeFrames
        self.frames = {}
        for edges, number in edgesDict:
            vg = VehicleGenerator(number, runTime)
            path = Path(edges, vg)
            self.paths.add(path)

    def run(self):
        """
        Run the simulation
        """
        for time in range(0, self.runTime):
            # Go through all paths to check for new vehicles
            for path in self.paths:
                # Check if there are any new vehicles on this path
                for i in range(path.vehicleGenerator.vehicles[time]):
                    vehicle = Vehicle(id = len(self.vehicles), path = path, maxSpeed = self.maxSpeed, acceleration = self.acceleration, deceleration = self.deceleration, noiseFactor = self.noiseFactor)
                    self.vehicles.append(vehicle)

            # Should snapshots be stored?
            if self.storeFrames:
                frame = {}
            
            # Update ALL vehicles
            for vehicle in self.vehicles:
                vehicle.update()
                # If snapshots are to be stored, store the position of the vehicle
                if self.storeFrames:
                    frame[vehicle] = vehicle.position
            if self.storeFrames:
                frames.append(frame)

class Path:
    # edges: Edges that make up this Path
    # frequency: number of vehicles that travel this path per time period
    def __init__(self, edges, vehicleGenerator):
        self.edges = edges
        self.vehicleGenerator = vehicleGenerator

    # Returns a sequential list of the points on the edges of this path
    def getPoints(self):
        """Determine the subsequent points that are on the edges of this path

        :returns: list -- A list of the Points connecting the edges of this path
        """
        points = []
        if len(self.edges) > 0:
            points.append(self.edges[0].start)

        for edge in self.edges:
            points.append(edge.end)
        return points

class VehicleGenerator:
    # totalVehicles: total number of vehicles to generate, uniformly distributes (same amount of time between vehicles)
    # totalDuration: total time of simulation
    def __init__(self, totalVehicles, totalDuration):
        self.totalVehicles = totalVehicles
        self.totalDuration = totalDuration
        self.vehicles = self.generate()
        
    def generate(self):
        vehicles = []
        generated = 0
        for t in range(1,self.totalDuration+1):
            toGenerate = max(0,int(self.cdf(t) * self.totalVehicles - generated))
            vehicles.append(toGenerate)
            generated += toGenerate
            
        return vehicles

    def cdf(self,t):
        return t/self.totalDuration
