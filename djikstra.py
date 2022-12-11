import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors

def create_empty_plot(max_val_x,max_val_y):

    fig, ax = plt.subplots(figsize=(7,7))
    
    major_ticks_x = np.arange(0, max_val_x+1, 5)
    minor_ticks_x = np.arange(0, max_val_x+1, 1)
    major_ticks_y = np.arange(0, max_val_y+1, 5)
    minor_ticks_y = np.arange(0, max_val_y+1, 1)
    ax.set_xticks(major_ticks_x)
    ax.set_xticks((minor_ticks_x-1)+0.5, minor=True)
    ax.set_yticks(major_ticks_y)
    ax.set_yticks((minor_ticks_y-1)+0.5, minor=True)
    ax.grid(which='minor', alpha=0.5)
    ax.grid(which='major', alpha=0)
    ax.set_ylim([-1,max_val_y])
    ax.set_xlim([-1,max_val_x])
    ax.invert_yaxis()
    ax.grid(True)
    
    return fig, ax

def create_plot(max_val_x,max_val_y,data,start,goal):
    limit = 15 
    occupancy_grid = data.copy()
    occupancy_grid[data>limit] = 1
    occupancy_grid[data<=limit] = 0
    occupancy_grid[start] = 0
    occupancy_grid[goal] = 0
    return occupancy_grid

def _get_movements_4n():
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0)]

def _get_movements_8n():
    s2 = math.sqrt(2)
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (1, 1, s2),
            (-1, 1, s2),
            (-1, -1, s2),
            (1, -1, s2)]


def reconstruct_path(cameFrom, current):
    """
    Recurrently reconstructs the path from start node to the current node
    :param cameFrom: map (dictionary) containing for each node n the node immediately 
                     preceding it on the cheapest path from start to n 
                     currently known.
    :param current: current node (x, y)
    :return: list of nodes from start to current node
    """
    total_path = [current]
    while current in cameFrom.keys():
        # Add where the current node came from to the start of the list
        total_path.insert(0, cameFrom[current]) 
        current=cameFrom[current]
    return total_path

def A_Star(start, goal, h, coords, occupancy_grid, max_val_x, max_val_y):
    """
    A* for 2D occupancy grid. Finds a path from start to goal.
    h is the heuristic function. h(n) estimates the cost to reach goal from node n.
    :param start: start node (x, y)
    :param goal_m: goal node (x, y)
    :param occupancy_grid: the grid map
    :param movement: select between 4-connectivity ('4N') and 8-connectivity ('8N', default)
    :return: a tuple that contains: (the resulting path in meters, the resulting path in data array indices)
    """
    
    # Check if the start and goal are within the boundaries of the map
    for point in [start, goal]:
        assert point[0]>=0 and point[0]<max_val_x, "start or end goal not contained in the map"
        assert point[1]>=0 and point[1]<max_val_y, "start or end goal not contained in the map"

    # check if start and goal nodes correspond to free spaces
    if occupancy_grid[start[0], start[1]]:
        raise Exception('Start node is not traversable')

    if occupancy_grid[goal[0], goal[1]]:
        raise Exception('Goal node is not traversable')
    
    # get the possible movements corresponding to the selected connectivity
    # if movement_type == '4N':
    movements = _get_movements_4n()
    # elif movement_type == '8N':
    #     movements = _get_movements_8n()
    # else:
    #     raise ValueError('Unknown movement')
    

    
    # The set of visited nodes that need to be (re-)expanded, i.e. for which the neighbors need to be explored
    # Initially, only the start node is known.
    openSet = [start]
    
    # The set of visited nodes that no longer need to be expanded.
    closedSet = []

    # For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start to n currently known.
    cameFrom = dict()

    # For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    gScore = dict(zip(coords, [np.inf for x in range(len(coords))]))
    gScore[start] = 0

    # For node n, fScore[n] := gScore[n] + h(n). map with default value of Infinity
    fScore = dict(zip(coords, [np.inf for x in range(len(coords))]))
    fScore[start] = h[start]

    # while there are still elements to investigate
    while openSet != []:
        
        #the node in openSet having the lowest fScore[] value
        fScore_openSet = {key:val for (key,val) in fScore.items() if key in openSet}
        current = min(fScore_openSet, key=fScore_openSet.get)
        del fScore_openSet
        
        #If the goal is reached, reconstruct and return the obtained path
        if current == goal:
            return reconstruct_path(cameFrom, current), closedSet

        openSet.remove(current)
        closedSet.append(current)
        
        #for each neighbor of current:
        for dx, dy, deltacost in movements:
            
            neighbor = (current[0]+dx, current[1]+dy)
            
            # if the node is not in the map, skip
            if (neighbor[0] >= occupancy_grid.shape[0]) or (neighbor[1] >= occupancy_grid.shape[1]) or (neighbor[0] < 0) or (neighbor[1] < 0):
                continue
            
            # if the node is occupied or has already been visited, skip
            if (occupancy_grid[neighbor[0], neighbor[1]]) or (neighbor in closedSet): 
                continue

            if deltacost == math.sqrt(2) :
                if (occupancy_grid[neighbor[0]-np.sign(dx), neighbor[1]]) or (occupancy_grid[neighbor[0], neighbor[1]-np.sign(dy)]): 
                    continue

            # d(current,neighbor) is the weight of the edge from current to neighbor
            # tentative_gScore is the distance from start to the neighbor through current
            tentative_gScore = gScore[current] + deltacost
            
            if neighbor not in openSet:
                openSet.append(neighbor)
                
            if tentative_gScore < gScore[neighbor]:
                # This path to neighbor is better than any previous one. Record it!
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = gScore[neighbor] + h[neighbor]

    # Open set is empty but goal was never reached
    print("No path found to goal")
    return [], closedSet


def djikstra_algo(data,start,goal):
    
    # List of all coordinates in the grid
    size = np.shape(data)
    x,y = np.mgrid[0:size[0]:1, 0:size[1]:1]
    dir = np.empty(x.shape + (2,))
    dir[:, :, 0] = x; dir[:, :, 1] = y
    dir = np.reshape(dir, (x.shape[0]*x.shape[1], 2))
    coords = list([(int(x[0]), int(x[1])) for x in dir])
    cmap = colors.ListedColormap(['white', 'grey', 'black']) # Select the colors with which to display obstacles and free cells

    # Define the heuristic, here = distance to goal ignoring obstacles
    h = np.linalg.norm(dir - goal, axis=-1)
    h = dict(zip(coords, h))
    # create grid to work on
    occupancy_grid = data
    # Run the A* algorithm
    path, visitedNodes = A_Star(start, goal, h, coords, occupancy_grid, size[0] ,size[1])
    path = np.array(path).reshape(-1, 2).transpose()
    visitedNodes = np.array(visitedNodes).reshape(-1, 2).transpose()

    # # Displaying the map
    # fig_astar, ax_astar = create_empty_plot(size[0],size[1])
    # ax_astar.imshow(occupancy_grid.transpose(), cmap=cmap)

    # # Plot the best path found and the list of visited nodes
    # ax_astar.scatter(visitedNodes[0], visitedNodes[1], marker="o", color = 'orange');
    # ax_astar.plot(path[0], path[1], marker="o", color = 'blue');
    # ax_astar.scatter(start[0], start[1], marker="o", color = 'green', s=200);
    # ax_astar.scatter(goal[0], goal[1], marker="o", color = 'purple', s=200);
    return path
