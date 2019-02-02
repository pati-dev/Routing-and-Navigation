#!/bin/python3
# route.py : Routing problem
# Ankit Mathur & Nitesh Jaswal, September 2018

# Test cases and times taken:
# Below is the list of a few test routes and corresponding times taken as tested on the CS Linux machines:
# Short routes:
# python3 route.py Bloomington,_Indiana Royalton,_Indiana bfs distance : < 3 sec
# python3 route.py Bloomington,_Indiana Royalton,_Indiana ids distance : < 1 sec
# python3 route.py Bloomington,_Indiana Royalton,_Indiana uniform distance : < 2 sec
# python3 route.py Bloomington,_Indiana Royalton,_Indiana astar distance : < 2 sec
# Long routes:
# python3 route.py Bloomington,_Indiana Chicago,_Illinois astar time : < 10 sec
# python3 route.py Bloomington,_Indiana Chicago,_Illinois astar distance : < 3 sec
# python3 route.py Bloomington,_Indiana Nashville,_Tennessee astar distance : < 4 sec
# python3 route.py Bloomington,_Indiana Nashville,_Tennessee astar time : < 20 sec
# python3 route.py Bloomington,_Indiana Detroit,_Michigan astar distance : < 70 sec
# python3 route.py Chicago,_Illinois Detroit,_Michigan astar distance : < 60 sec
# python3 route.py Chicago,_Illinois Detroit,_Michigan astar time : < 35 sec

# Formulation of the search problem:
# (1) State space: The state space, essentially, is a combination of the following variables
# along the route taken from the start city to its successor cities:
# --> Current city on the route
# --> List of cities along the route
# --> Total distance travelled so far along the route
# --> Total travel time so far along the route
# --> Total number of segments ("turns") taken along the route
# The aforementioned variables form the core of a state irrespective of the routing
# algorithm being used. In case of Uniform-cost and A* search algorithms, an additional
# variable is added to the state to account for cost and heuristic respectively.
# (2) Successor function: The successor function uses the "road-segments.txt" file to expand
# the current state into all possible successor states by adding a new city to the current
# route such that:
# --> the new city and the current city are connected via a road segment as per the txt file
# --> the new city is not already present in the current route
# (3) Edge weights: As expected, edge weights, or costs, depend on the user input. While BFS, DFS, and IDS
# assume number of segments taken to reach a node as the cost, Uniform-cost search and A* search algorithms
# call one of the 3 cost functions defined in the code to account for the cost function provided by the user.
# (4) Goal state: Trivially, the program returns a node as the goal node by checking if the current city is the end city.
# The program uses Search Algorithm 2 to traverse the graph and hence always checks for goal before expanding a node,
# thereby ensuring that the most optimized route to the end city is found.
# (5) Heuristic function: The heuristic for every successor state is calculated, in case of A* search, by first calculating
# the straight line distance (SLD) between the current city (i.e. the child city) and the end city.
# In case distance is passed as the cost function, this SLD is directly passed as the heuristic and is added to the cost
# which, in this case, is the distance travelled to reach the child city.
# In case time is passed as the cost function, the SLD is divided by the maximum speed on any highway (65mph as per the dataset)
# and is added to the cost which, in this case, is the time taken to reach the child city.
# In case of segments, the SLD is not calculated to begin with and each child is given a heuristic of 1 which, in turn, is added
# to the total number of segments taken to reach the child city.

# Handling the noise in data
# The program assumes that the road segments are error free and always represent the accurate road distance between two cities.
# In case of A* search with distance or time as costs, the heuristic has been designed to proactively check for
# noise in the GPS data of the child node and still underestimate the cost to reach the goal, thereby keeping it consistent.
# This is done in the following manner:
# Case 1 - Unavailable GPS location: In case the GPS location of the end city is not available, the program prompts the user
# that it will use Uniform-cost search instead of A* because it will never be able to estimate the heuristic at any node.
# In case the GPS location of end city is available but that of any city on the route is not available, the program
# backtracks the route from the child city and calculates the SLD between the most immediate parent city with available GPS coordinates
# and the end city. This SLD is then subtracted by the total road distance travelled to reach the child city from this parent city
# assuming that the current route is exactly on the straight line path from this parent city to the goal city, thereby underestimating the heuristic.
# In the edge case when none of the parent cities of the child city has a GPS location, the heuristic is passed as the SLD between child city
# and end city. In case neither the parents nor the child cities have an available GPS location, the heuristic is passed as zero for the child node.
# Case 2 - Incorrect GPS location: Every time the program computes the SLD from child city to the end city, it also checks if its SLD
# from its (first geographically available) parent city is more than the total road distance travelled from the said parent to the child city.
# In this case, minimum of the parent and child cities' SLD to end city is passed as the heuristic, thereby ensuring that heuristic is
# underestimating the cost to goal.

# Handling the noise in argument vars:
# Case 1 - Incorrect cities: The program works if both the start and end cities are available in the road-segments text file. If
# either of them is not available in the file, the program terminates prompting the user to enter the correct city name(s).
# Case 2 - Incorrect routing algorithm/cost function: In this case, the program terminates prompting the user to enter the
# correct value of routing algo/cost function respectively.


from queue import PriorityQueue
from math import sin, cos, sqrt, atan2, radians, floor
import sys

# Citation: Following function, sld_between_cities, uses a part of the code posted at stackoverflow (link below)
# https://stackoverflow.com/questions/19412462/getting-distance-between-two-points-based-on-latitude-longitude
def sld_between_cities(city1, city2):
    global city_gps
    # Unpack lats and longs for the cities
    (x1, y1), (x2, y2) = city_gps[city1], city_gps[city2]
    # Convert degrees to radians
    lat1 = radians(x1)
    lon1 = radians(y1)
    lat2 = radians(x2)
    lon2 = radians(y2)

    ### Begin cited code
    # approximate radius of earth in km
    R = 6373.0

    dlon = lon1 - lon2
    dlat = lat1 - lat2

    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    # Calculate distance and convert it into miles
    distance = (R * c) / 1.60934
    ### End cited code

    return floor(distance)

def road_dist(node):
    # Unpack the route from the node
    route = node[1]
    # Backtrack the route and identify the first parent with available GPS location
    parent = ''
    reverse_route = list( reversed( route.split() ) )
    for city in reverse_route[1:]:
        if city in city_gps.keys():
            parent = city
            break
    # If not parent on the route has a GPS location, return zero
    if not len(parent):
        return 0
    # Else, return the total road distance on the route from parent city to child city
    else:
        dist_parent_to_child = 0
        route_parent_to_child = route.split()[ route.split().index(parent) :]
        for i in range(0, len(route_parent_to_child) - 1 ):
            for key, value in road_seg.items():
                if (route_parent_to_child[i] in key) and (route_parent_to_child[i+1] in key):
                    dist_parent_to_child += value[0]

        return dist_parent_to_child

def sld_between_nodes(node1, node2):
    global end_city
    # Unpack routes from the nodes
    route1, route2 = node1[1], node2[1]
    # Initialize city1 and city2
    city1 = ''
    city2 = ''
    # Track each city on route1 in reverse order and return sld between the first available city and goal city
    for city in reversed( route1.split() ):
        if city in city_gps.keys():
            city1 = city
            break
    # Track each city on route1 in reverse order and return sld between the first available city and goal city
    for city in reversed( route2.split() ):
        if city in city_gps.keys():
            city2 = city
            break
    # Assign goal city as the corresponding city if no city on route1 or route2 has a GPS location
    if not len(city1):
        city1 = end_city
    if not len(city2):
        city2 = end_city

    return sld_between_cities(city1, city2)

def sld_to_goal(node):
    global end_city
    # Unpack route from the node
    route = node[1]
    # Track each city on route in reverse order and return sld between the first available city and goal city
    for city in reversed( route.split() ):
        if city in city_gps.keys():
            return sld_between_cities(city, end_city)
    # Return zero if no city on the route has a GPS location
    return 0

def segments(child_node, algo):
    # Unpack child node
    city, route, dist, time, seg = child_node
    g = seg
    h = 1 if algo == 'astar' else 0
    return g + h

def travel_distance(child_node, parent_node, algo):
    global city_gps
    # Pass distance to child city as the cost
    g = child_node[2]
    # Evaluate heuristic as per the routing algorithm
    if algo == 'uniform':
        h = 0
    else:
        # For A* search, check for inaccuracy in child city's GPS location
        if ( sld_between_nodes(parent_node, child_node) > road_dist(child_node) ) or ( not sld_between_nodes(parent_node, child_node) ):
            inaccuracy_factor = road_dist(child_node)
            h = min( sld_to_goal(child_node), max(0, ( sld_to_goal(parent_node) -  inaccuracy_factor ) ) )
        else:
            h = sld_to_goal(child_node)

    return g + h

def travel_time(child_node, parent_node, algo):
    global city_gps
    # Use 65 mph as the max speed to calculate the time heuristic
    max_speed = 65
    # Pass travel time to child city as the cost
    g = child_node[3]
    # Evaluate heuristic as per the routing algorithm
    if algo == 'uniform':
        h = 0
    else:
        # Check for inaccuracy/unavailability of child city's GPS location
        if ( sld_between_nodes(parent_node, child_node) > road_dist(child_node) ) or ( not sld_between_nodes(parent_node, child_node) ):
            inaccuracy_factor = road_dist(child_node)
            min_dist = min( sld_to_goal(child_node), max(0, ( sld_to_goal(parent_node) -  inaccuracy_factor ) ) )
        else:
            min_dist = sld_to_goal(child_node)
        h = min_dist / max_speed

    return g + h

def estimate(child_node, parent_node, algo):
    global cost_func

    # Call appropriate evaluation function
    if cost_func == 'segments':
        f = segments(child_node, algo)
    elif cost_func == 'distance':
        f = travel_distance(child_node, parent_node, algo)
    elif cost_func == 'time':
        f = travel_time(child_node, parent_node, algo)
    else:
        f = -1

    return f

def successors(parent_node):
    global road_seg
    # Unpack parent node
    parent_city, parent_route, parent_dist, parent_time, parent_seg = parent_node
    child_nodes = []
    # Initialize a var for cities already visited on the current route
    visited = [node for node in parent_route.split()]
    # Begin traversing the road segments
    for key, value in road_seg.items():
        # Unpack road segment
        city1, city2, dist, speed, highway = key[0], key[1], value[0], value[1], value[2]
        # Check if parent city is in the road segment
        if parent_city in (city1, city2):
            child_city = city2 if parent_city == city1 else city1
            # Check of the child city has already been visited
            if child_city not in visited:
                child_route = parent_route + " " + child_city
                child_dist = dist + parent_dist
                child_time = parent_time + (dist/speed)
                child_seg = parent_seg + 1
                # Pack child node
                child_node = (child_city, child_route, child_dist, child_time, child_seg)
                child_nodes += [child_node]

    return child_nodes

def is_goal(city):
    global end_city
    return end_city == city

def bfs():
    global start_city
    # Structure of node: (current_city, route_to_city, journey_distance, journey_time, number_of_road_segments)
    # Pack & initialize initial node
    initial_node = (start_city, start_city, 0, 0, 0)
    # Initialize fringe with initial node
    fringe = [initial_node]
    while len(fringe) > 0:
        parent_node = fringe.pop(0)
        city, route, dist, time, seg = parent_node
        if is_goal(city):
            if cost_func == 'segments':
                optimal = 'yes'
            else:
                optimal = 'no'
            print("Total number of turns on the route:", seg)
            return (optimal, dist, round(time, 4), route)
        for child_node in successors(parent_node):
            fringe += [child_node]

    return False

def dfs():
    global start_city
    # Structure of node: (current_city, route_to_city, journey_distance, journey_time, number_of_road_segments)
    # Pack & initialize initial node
    initial_node = (start_city, start_city, 0, 0, 0)
    # Initialize fringe with initial node
    fringe = [initial_node]
    while len(fringe) > 0:
        parent_node = fringe.pop()
        city, route, dist, time, seg = parent_node
        if is_goal(city):
            optimal = 'no'
            print("Total number of turns on the route:", seg)
            return (optimal, dist, round(time, 4), route)
        for child_node in successors(parent_node):
            fringe += [child_node]

    return False

def ids():
    global start_city
    # Structure of node: (current_city, route_to_city, journey_distance, journey_time, number_of_road_segments)
    # Pack & initialize initial node
    initial_node = (start_city, start_city, 0, 0, 0)
    # Initialize depth limit
    limit = 40
    for l in range(1, limit+1):
        # Initialize fringe with initial node
        fringe = [initial_node]
        while len(fringe) > 0:
            parent_node = fringe.pop()
            city, route, dist, time, seg = parent_node
            if is_goal(city):
                if cost_func == 'segments':
                    optimal = 'yes'
                else:
                    optimal = 'no'
                print("Total number of turns on the route:", seg)
                return (optimal, dist, round(time, 4), route)
            # Expand parent node if current depth is within limit
            if seg < l:
                for child_node in successors(parent_node):
                    fringe += [child_node]

    return False

def uniform():
    global start_city
    # Structure of node: (current_city, route_to_city, journey_distance, journey_time, number_of_road_segments)
    # Pack & initialize initial node
    initial_node = (start_city, start_city, 0, 0, 0)
    # Initialize fringe with initial node
    fringe = PriorityQueue()
    fringe.put( (0,  initial_node) )
    while not fringe.empty():
        (f, parent_node) = fringe.get()
        city, route, dist, time, seg = parent_node
        if is_goal(city):
            optimal = 'yes'
            print("Total number of turns on the route:", seg)
            return (optimal, dist, round(time, 4), route)
        for child_node in successors(parent_node):
            f = estimate(child_node, parent_node, 'uniform')
            if f == -1:
                print("Please punch-in one of the following costs:\nsegments, distance, time")
                return False
            else:
                fringe.put( (f, child_node) )

    return False

def astar():
    global start_city
    global end_city
    # Check for end city's GPS location availability
    if end_city not in list(city_gps.keys()):
        print("Cannot locate destination city on the GPS. Running Uniform-cost algorithm instead...")
        solution = uniform()
        return solution
    else:
        # Structure of node: (current_city, route_to_city, journey_distance, journey_time, number_of_road_segments)
        # Pack & initialize initial node
        initial_node = (start_city, start_city, 0, 0, 0)
        # Initialize fringe with initial node
        fringe = PriorityQueue()
        fringe.put( (0,  initial_node) )
        while not fringe.empty():
            (f, parent_node) = fringe.get()
            city, route, dist, time, seg = parent_node
            if is_goal(city):
                # Check noise flag and assign optimality
                optimal = 'yes'
                print("Total number of turns on the route:", seg)
                return (optimal, dist, round(time, 4), route)
            for child_node in successors(parent_node):
                f = estimate(child_node, parent_node, 'astar')
                if f == -1:
                    print("Please punch-in one of the following costs:\nsegments, distance, time")
                    return False
                else:
                    fringe.put( (f, child_node) )

    return False

def solve():
    global routing_algo
    global road_seg
    global start_city
    global end_city

    # Unpack cities in road segments
    cities = set( city for key in list( road_seg.keys() ) for city in key )

    # Check if start and end cities are available in the dataset
    if start_city in cities and end_city in cities:
        if routing_algo == 'bfs':
            solution = bfs()
        elif routing_algo == 'dfs':
            solution = dfs()
        elif routing_algo == 'ids':
            solution = ids()
        elif routing_algo == 'uniform':
            solution = uniform()
        elif routing_algo == 'astar':
            solution = astar()
        else:
            print("Please punch-in one of the following routing algorithms:\nbfs, dfs, ids, uniform, astar")
            solution = False
    else:
        print("Please punch-in a valid start/end city")
        solution = False

    return solution

city_gps = {}
with open('city-gps.txt', 'r') as file:
    for line in file:
        (city, lat, long) = line.split()
        city_gps[city] = (float(lat), float(long))

road_seg = {}
with open('road-segments.txt', 'r') as file:
    for line in file:
        for i in [line.split()]:
            # Populate missing/zero speed limits as 45mph (average speed limit as per the dataset)
            if len(i) == 5:
                (city1, city2, distance, speed, highway) = i
                if not int(speed):
                    speed = 45
                road_seg[ (city1, city2) ] = ( int(distance), int(speed), highway )
            else:
                (city1, city2, distance, highway) = i
                road_seg[ (city1, city2) ] = ( int(distance), 45, highway )

start_city, end_city, routing_algo, cost_func = sys.argv[1:]

solution = solve()

if not solution:
    print("No route found\n")
else:
    print("Directions:")
    route = solution[-1].split()
    for i in range(0, len(route) - 1 ):
        print("Travel from", route[i], "to", route[i+1], "for", end=" ")
        for key, value in road_seg.items():
            if route[i] in key and route[i+1] in key:
                print(value[0], "miles on", value[2], "highway. At", value[1], "mph it will take about", round( 60 * value[0] / value[1]), "minutes.")
                break
    for item in solution:
        print(item, end=" ")
print('')
