import math
import pdb

class Tile:
    def __init__(self, location, parent=None):
        self.location = location
        self.parent = parent

        self.distance = 0
        self.heuristic = 0
        self.cost = 0

#calculate heuristic
def heuristic(tile_location, goal_location):
    return abs(tile_location[0] - goal_location[0]) + abs(tile_location[1] - goal_location[1])

def pathfind(map, start, goal):
    # Open fringe and closed list
    fringe = []
    closed_list = []

    # Create the start tile and goal location
    start_tile = Tile(start)
    goal_location = goal  # We only need the goal location, not the Tile object

    # Add the start tile to the fringe
    fringe.append(start_tile)

    # pdb.set_trace()
    straight_move_done = False

    while fringe:
        # Find the tile with the lowest cost
        working_tile = fringe[0]
        for tile in fringe:
            if tile.cost < working_tile.cost:
                working_tile = tile

        # Remove the current tile from the fringe
        fringe.remove(working_tile)

        # Goal test: if we reached the goal, backtrack to reconstruct the path
        if working_tile.location == goal_location:

            path = []

            while working_tile:

                path.append(working_tile.location)
                working_tile = working_tile.parent

            # Gives path in forward format 
            return path[::-1]  

        # Add the current tile to the closed list
        closed_list.append(working_tile.location)

        #Look for succesors in only 4 directions 90 degree turns
        #requires more moves 
        cross_pattern = [(0,1),(0,-1),(1,0),(-1,0)]

        #Look for succesors in 8 directions 45 degree turns 
        #requires less moves 
        sqaure_pattern = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        
        straight_direction = [(1,0)]

        successors = []

        if not straight_move_done:
            # Make the first move in a straight line (one direction)
            new_location = (working_tile.location[0] + 1,
                            working_tile.location[1] + 0)

            if 0 <= new_location[1] < len(map) and 0 <= new_location[0] < len(map[0]):
                if map[new_location[1]][new_location[0]] == 0 and new_location not in closed_list:
                    new_tile = Tile(new_location, working_tile)
                    new_tile.distance = working_tile.distance + 1
                    new_tile.heuristic = heuristic(new_tile.location, goal_location)
                    new_tile.cost = new_tile.distance + new_tile.heuristic
                    successors.append(new_tile)
            straight_move_done = True  # Set flag once the straight movement is done
        else:

            for direction in cross_pattern:
                new_location = (working_tile.location[0] + direction[0], working_tile.location[1] + direction[1])

                # Check if the new location is within bounds
                if 0 <= new_location[1] < len(map) and 0 <= new_location[0] < len(map[0]):
                
                    # Check if the new location is not an obstacle and not already closed
                    if map[new_location[1]][new_location[0]] == 0 and new_location not in closed_list:
                    
                        new_tile = Tile(new_location, working_tile)
                        
                        new_tile.distance = working_tile.distance + 1
                        # Pass only the locations (not the Tile objects) to the heuristic function
                        new_tile.heuristic = heuristic(new_tile.location, goal_location)
                        new_tile.cost = new_tile.distance + new_tile.heuristic
                        
                        successors.append(new_tile)

        # Add valid successors to the fringe
        for succ_tile in successors:
            
            # Check if a tile with the same location is already in the fringe with a lower cost
            existing_tile = next((t for t in fringe if t.location == succ_tile.location), None)
            
            if not existing_tile or succ_tile.cost < existing_tile.cost:
                fringe.append(succ_tile)

    # Return an empty path if no solution is found
    return []

def print_map(map):
    for row in map:
        print(' '.join(f'{cell}' for cell in row))

def add_path(path,map):
    for item in path:
        map[item[1]][item[0]] = '-'

def map_gen(obstical_locations):
    rows , col = 10, 16
    map = [[0 for j in range(col)] for i in range(rows)]

    for obstical in obstical_locations:
        map[9-obstical[1]][obstical[0]] = 1

    return(map)

    

def main():
    this_obsticals = [(1,3),(2,3),(3,3),(4,8),(6,5),(7,5)]

    assgn_obsticals = [(0.61,2.743),(0.915,2.743),(1.219,2.743),(1.829,1.219),
        (1.829,1.524),(1.829,1.829), (1.829,2.134),(2.743,0.305),
        (2.743,0.61),(2.743,0.915),(2.743,2.743),(3.048,2.743),
        (3.353,2.743)]

    new_obsticals = []
    for item in assgn_obsticals:
        new_obsticals.append((round(item[0] * 3.2804),round(item[1] * 3.2804)))



    start = (1,8)
    goal = (9,3)
    
    new_map = map_gen(this_obsticals)

    print('\n')

    print_map(new_map)

    print("\nStarting pathfinding...\n")

    path = pathfind(new_map, start, goal)

    print("Path found: \n", path, '\n')

    add_path(path, new_map)

    print_map(new_map)
    
    current_orientation = 0

    movement_angles = []

# creates turn angles 
    for i in range(len(path) - 1):
        x1 = path[i][0]
        x2 = path[i+1][0]
        y1 = path[i][1]
        y2 = path[i+1][1]

        target_orientation  = 0

        x_tot = x2 - x1 
        y_tot = y2 - y1 

        # print(x2 - x1 , y2 - y1)
       

        if x_tot == 1 and y_tot == 1 :
            target_orientation = 40
        if x_tot == 1 and y_tot == 0:
            target_orientation = 0
        if x_tot == 1 and y_tot == -1:
            target_orientation = -45
        if x_tot == 0 and y_tot == 1:
            target_orientation = -90
        if x_tot == 0 and y_tot == -1:
            target_orientation = 90
        if x_tot == -1 and y_tot == 1:
            target_orientation =135
        if x_tot == -1 and y_tot == 0:
            target_orientation = 180
        if x_tot == -1 and y_tot == -1:
            target_orientation = 135
        if current_orientation != target_orientation:    

            move_required = target_orientation - current_orientation
            # print('move req:',move_required)
            current_orientation = target_orientation
            movement_angles.append(current_orientation)


        movement_angles.append(current_orientation)

        # print('new angle',current_orientation)

    # print(movement_angles)


# converts path from ft -> mm and adds turn angle 

    final_path = []
    count = 0
    for item in path:

        final_path.append(((item[0] * 310),(9*310) - (item[1] * 310)))
    
    print("\nPath for input: \n")   
    print(final_path)
    print('\n')

    last_path= []
    prev_x = final_path[0][0]
    prev_y = final_path[0][1]

     
if __name__ == '__main__':
    main()

