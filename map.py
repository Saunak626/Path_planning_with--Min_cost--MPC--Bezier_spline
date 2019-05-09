import numpy as np

def mapGenerator(start, obstacle_type, mapsize):
    if obstacle_type == 'static_obstacle':
        obstacle = np.array([start, [10, 27], [12, 25], [14, 23], [16, 21], [18, 26], [20, 17],[22,20],[27,22],[30,20],[35,30]])  # Indices of obstacles

    else:
        obstacle = np.array([start])  # Indices of obstacles

    safe_distance = [ [1, 0], [0, 1], [-1, 0], [0, -1], [1, 1], [1, -1], [-1, 1], [-1, -1] ]
    occupy_grid = obstacle                                      # Obstacle map taking safe distance into consideration
    freeGrid_num = int( mapsize * mapsize - len(obstacle) )     # Open grid number
    #print("len(obstacle)",len(obstacle))

    for i in range(1, len(occupy_grid)):
        temp = np.add(occupy_grid[i], safe_distance)
        # print("temp",temp)
        occupy_grid = np.append(occupy_grid, temp, axis=0)

    occupy_grid = np.unique(occupy_grid, axis=0)
    #print("occupy_grid",occupy_grid)

    return freeGrid_num, obstacle, occupy_grid