'''
Reference: https://leetcode.com/problems/shortest-path-in-binary-matrix/discuss/313347/a-search-in-python
Ideas and insights were adapted from the reference above in implementing the shortest path based on A* algorithm
'''

from heapq import heappop, heappush
from math import sqrt

def distance(x1, x2):
    dist = (x1[0] - x2[0])**2 + (x1[1] - x2[1])**2
    eucl_dist = sqrt(dist)
    
    return eucl_dist

def reconstruct_path(came_from, start, end):
    
    reversed_path = [end]
    
    while end != start:
        end = came_from[end]
        reversed_path.append(end)
        new_path = list(reversed(reversed_path))
        
    return new_path


def shortest_path(M, start, goal):
    
    if start == goal:
        return [start]
    
    paths_x_y = M.intersections
    connections_of_nodes = M.roads
    
    frontier = []
    frontier = [(0, start)]
    
    came_from = {}
    came_from[start] = None
    
    g_cost = {}
    g_cost[start] = 0
    

    while len(frontier) >= 1:
        next_node = heappop(frontier)[1]

        if next_node == goal:
            return reconstruct_path(came_from, start, goal)

        for neighbor in connections_of_nodes[next_node]:
            updated_g_cost = g_cost[next_node] + distance(paths_x_y[next_node], paths_x_y[neighbor])

            if neighbor not in g_cost or updated_g_cost < g_cost[neighbor]:
                came_from[neighbor] = next_node
                g_cost[neighbor] = g_cost[next_node] + distance(paths_x_y[next_node], paths_x_y[neighbor])
                heappush(frontier, (updated_g_cost, neighbor))

    return reconstruct_path(came_from, start, goal)




