# Routing-and-Navigation
Implementation of graph search algorithms to navigate from one city to another. Uses BFS, DFS, IDS, Uniform-cost (Dijkstra), and A-star routing algorithms with distance, time, or number of turns as costs.

The program runs as per the following command line format:
./route.py [start-city] [end-city] [routing-algorithm] [cost-function]
where:
• start-city and end-city are the cities we need a route between. • routing-algorithm is one of:
  – bfs uses breadth-first search (which ignores edge weights in the state graph)
  – uniform is uniform cost search (the variant of bfs that takes edge weights into consideration)
  – dfs uses depth-first search
  – ids uses iterative deepening search
  – astar uses A* search, with a suitable heuristic function
• cost-function is one of:
  – segments tries to find a route with the fewest number of “turns” (i.e. edges of the graph)
  – distance tries to find a route with the shortest total distance
  – time tries to find the fastest route, for a car that always travels at the speed limit

The output of the program is of the following format:
[optimal?] [total-distance-in-miles] [total-time-in-hours] [start-city] [city-1] [city-2] ... [end-city]
where:
•[optimal?] is either yes or no to indicate whether the program can guarantee that the solution found is one with the lowest cost
