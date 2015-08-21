from math import radians, sin, cos, sqrt, asin, fabs, atan2, pi, tan
import random
import operator

class Edge:
    def __init__(self, start, end):
        self.start = start
        self.end = end

    def __eq__(self, other):
        return isinstance(other, Edge) and other.start == self.start and other.end == self.end

    def __hash__(self):
        return hash(str(self.start) + str(self.end))

    def __repr__(self):
        return repr(self.start) + "->" + repr(self.end)

    """Gives the bearing of this Edge from start to end in radians"""
    def bearing(self):
        start = self.start.toRadians()
        end = self.end.toRadians()
        lat1 = start.lat
        lon1 = start.lon
        lat2 = end.lat
        lon2 = end.lon

        return atan2(sin(lon2-lon1) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1))

    """Gives the length of this Edge in meters"""
    def length(self):
        return self.start.distanceTo(self.end)

    def toVector(self):
        pass

    """Gives a Point on this Edge at the specified distance from start"""
    def pointAtDistance(self, distance, R = 6378137):
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
    def __init__(self, points):
        self.points = points

   
class Point:
    def __init__(self,lon, lat):
        self.lon = lon;
        self.lat = lat;

    def longitude(self):
        return self.lon;

    def latitude(self):
        return self.lat;

    def degreeRadianConversion(self, degreesToRadians = True):
        conv = pi/180 if degreesToRadians else 180/pi
        lon = self.lon * conv
        lat = self.lat * conv
        return Point(lon, lat)

    def toRadians(self):
        return self.degreeRadianConversion(degreesToRadians = True)

    def toDegrees(self):
        return self.degreeRadianConversion(degreesToRadians = False)

    def toCartesian(self, R = 6378137):
        self = self.toRadians()
        x = R * cos(self.lat) * cos(self.lon)
        y = R * cos(self.lat) * sin(self.lon)
        z = R * sin(self.lat)
        return [x, y, z]

    def fromCartesian(vector, R = 6378137):
        x = vector[0]
        y = vector[1]
        z = vector[2]
        lat = asin(z/R) * 180/pi
        lon = atan2(y, x) * 180/pi
        return Point(lon, lat)

    def north(self, distance):
        return self.pointAtDistance(distance, bearing = 0)

    def south(self, distance):
        return self.pointAtDistance(distance, bearing = pi)

    def east(self, distance):
        return self.pointAtDistance(distance, bearing = 0.5*pi)        

    def west(self, distance):
        return self.pointAtDistance(distance, bearing = 1.5 * pi)

    def distanceTo(self, other, R = 6378137):
        dLat = radians(other.lat - self.lat)
        dLon = radians(other.lon - self.lon)
        lat0 = radians(self.lat)
        lat1 = radians(other.lat)

        a = sin(dLat/2)**2 + cos(lat0)*cos(lat1)*sin(dLon/2)**2
        c = 2*asin(sqrt(a))

        return R * c

    def pointAtDistance(self, distance, bearing, R = 6378137):
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
    
    def __init__(self, edges):
        self.edges = edges
        self.updateNodes()
        self.randomizeEdgeLengths()

    def updateNodes(self):
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
    #
    def __init__(self, pathDict):
        self.pathDict = pathDict

class Path:
    # edges: Edges that make up this Path
    # frequency: number of vehicles that travel this path per time period
    def __init__(self, edges, vehicleGenerator):
        self.edges = edges
        self.vehicleGenerator = vehicleGenerator

    # Returns a sequential list of the points on the edges of this path
    def getPoints(self):
        points = []
        if len(edges) > 0:
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
