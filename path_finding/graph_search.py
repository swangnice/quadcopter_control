from heapq import heappush, heappop  # Recommended.
import numpy as np

from rotorpy.world import World

from path_finding.occupancy_map import OccupancyMap # Recommended.

def get_neighbors(index, occ_map):
    x, y, z = index
    neighbors = []
    costs = []

    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            for dz in [-1, 0, 1]:
                # Skip the center point (0, 0, 0)
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                nx, ny, nz = x + dx, y + dy, z + dz
                # Check if the node is not occupied
                if not occ_map.is_occupied_index((nx, ny, nz)):
                    neighbors.append((nx, ny, nz))
                    changes = abs(dx)+abs(dy)+abs(dz)
                    if changes == 1:
                        cost = 1
                    elif changes ==2:
                        cost = 1.41
                    else:
                        cost = 1.73                
                    costs.append(cost)
    return neighbors, costs

def dijkstra(start, goal, start_index, goal_index, occ_map):
    frontier = []
    heappush(frontier,(0,start_index))
    previous_nodes = {start_index:None}
    cost = {start_index:0}
    nodes_expanded = 0

    while frontier:
        current = heappop(frontier)[1]
        nodes_expanded += 1

        if current == goal_index:
            break

        next_nodes, next_costs = get_neighbors(current, occ_map)

        for next in next_nodes:
             new_cost = cost[current]+next_costs[next_nodes.index(next)]


             if next not in cost or new_cost < cost[next]:
                  cost[next] = new_cost
                  heappush(frontier,(new_cost, next))                 
                  previous_nodes[next] = current


    if goal_index not in previous_nodes:
         return None, nodes_expanded
    
    path = []
    current = goal_index
    while current != start_index:
        path.append(occ_map.index_to_metric_center(current))
        current = previous_nodes[current]
    path.append(start)
    path.reverse() 
    path[-1] = goal

    return np.array(path), nodes_expanded


def heuristic(next, goal):
    euclidean_distance = np.sqrt((next[0] - goal[0])**2 + (next[1] - goal[1])**2 + (next[2] - goal[2])**2)
    manhattan_distance = (goal[0]-next[0])+(goal[1]-next[1])+(goal[2]-next[2])
    return euclidean_distance

def astar_algorithm(start, goal, start_index, goal_index, occ_map):
    frontier = []
    heappush(frontier,(0,start_index))
    previous_nodes = {start_index:None}
    cost = {start_index:0}
    nodes_expanded = 0

    while frontier:
        current = heappop(frontier)[1]
        nodes_expanded += 1

        if current == goal_index:
            break

        next_nodes, next_costs = get_neighbors(current, occ_map)

        for next in next_nodes:
             new_cost = cost[current]+next_costs[next_nodes.index(next)]
             if next not in cost or new_cost < cost[next]:
                cost[next] = new_cost
                total_cost = new_cost + heuristic(next, goal_index)
                heappush(frontier,(total_cost, next))
                previous_nodes[next] = current


    if goal_index not in previous_nodes:
        return None, nodes_expanded
    
    path = []
    current = goal_index
    while current != start_index:
        path.append(occ_map.index_to_metric_center(current))
        current = previous_nodes[current]
    path.append(start)
    path.reverse() 
    path[-1] = goal

    return np.array(path), nodes_expanded



def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        return a tuple (path, nodes_expanded)
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
        nodes_expanded, the number of nodes that have been expanded
    """

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))
    
    # MY CODE STARTS HERE
    if astar == False:
        return dijkstra(start, goal, start_index, goal_index, occ_map)
    
    if astar == True:
        return astar_algorithm(start, goal, start_index, goal_index, occ_map)
        


    
    
    
