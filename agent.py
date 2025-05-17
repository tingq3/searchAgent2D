#!/usr/bin/python3
# ^^ note the python directive on the first line
# COMP3411/9814 agent initiation file 
# requires the host to be running before the agent
# typical initiation would be (file in working directory, port = 31415)
#        python3 agent.py -p 31415
# created by Leo Hoare
# with slight modifications by Alan Blair

# Brief summary of program
# This program uses bfs to explore unknown tiles, once target is found, 
# it also uses bfs to find the shortest possible path to reach the target. 
# Important notes: 
# local map is created centered by the starting position, then expand by the view of the agent
# transform_offset is used to change the perspective view of agent to real coordinate based on direction
# if explored tiles are visited mroe than 5 times, then program will prefer to go to less visited tiles

import sys
import socket
from collections import deque

# Declaring visible grid to agent
view = [['' for _ in range(5)] for _ in range(5)]

DIRECTIONS = {'^': (-1, 0), '>': (0, 1), 'v': (1, 0), '<': (0, -1)}
TURN_LEFT = {'^': '<', '<': 'v', 'v': '>', '>': '^'}
TURN_RIGHT = {'^': '>', '>': 'v', 'v': '<', '<': '^'}

OBSTACLES = {'T', '-', '*', '˜', '~'}
TOOLS = {'a', 'k', 'd'}
TREASURE = '$'
RAFT = 'r' 

agent_pos = (0, 0)
agent_dir = '>'
world_map = { (x, y): '?' for x in range(-50, 50) for y in range(-50, 50) }
inventory = set()
target_pos = None
planned_path = deque()
start_pos = None
visit_count = {}
VISIT_THRESHOLD = 5
tool_positions = {}
has_treasure = False

# changes the coordinates as the direction of agent change
def transform_offset(i, j, agent_dir):
    # set x and y to centre of view 5x5 grid
    r, c = i - 2, j - 2
    if agent_dir == '^': return (r, c)
    if agent_dir == '>': return (c, -r)
    if agent_dir == 'v': return (-r, -c)
    if agent_dir == '<': return (-c, r)

# Updating the world map
def update_world_map(view):
    global agent_pos, world_map, target_pos, has_treasure, start_pos

    if start_pos is None:
        start_pos = agent_pos
        world_map[start_pos] = 'S'

    new_map = {}

    # update every tile
    for i in range(5):
        for j in range(5):
            # coord form agent pos
            dx, dy = transform_offset(i, j, agent_dir)
            # real coord
            global_x, global_y = agent_pos[0] + dx, agent_pos[1] + dy
            pos = (global_x, global_y)

            old_tile = world_map.get(pos, '?')
            new_tile = view[i][j]

            if new_tile in TOOLS:
                tool_positions[pos] = new_tile

            if old_tile != new_tile and old_tile != 'S':  
                new_map[pos] = new_tile  

            if new_tile == TREASURE and not has_treasure:
                target_pos = pos
                has_treasure = True

    world_map.update(new_map)
    visit_count[agent_pos] = visit_count.get(agent_pos, 0) + 1
  
# BFS for pathfinding
def bfs(start, goal_condition):
    queue = deque([(start, [])])
    visited = set([start])

    while queue:
        (x, y), path = queue.popleft()

        if goal_condition(x, y):
            return path

        for d, (dx, dy) in DIRECTIONS.items():
            nx, ny = x + dx, y + dy
            tile = world_map.get((nx, ny), '?')

            if (nx, ny) not in visited:
                # make path if foudn treasure
                if tile == TREASURE:
                    return path + [(nx, ny)]
                
                elif tile == ' ':
                    has_valid_neighbor = False
                    for dx, dy in DIRECTIONS.values():
                        neighbor_tile = world_map.get((nx + dx, ny + dy), '?')
                        
                        # stop if there is unkown tile next to it
                        if neighbor_tile != '?':
                            has_valid_neighbor = True
                            break
                    
                    if has_valid_neighbor:
                        visited.add((nx, ny))
                        queue.append(((nx, ny), path + [(nx, ny)]))

                elif tile == 'S' or tile in TOOLS or (tile in OBSTACLES and has_required_tool(tile)):
                    visited.add((nx, ny))
                    queue.append(((nx, ny), path + [(nx, ny)]))

    return []

def has_required_tool(obstacle):
    return (obstacle == 'T' and 'a' in inventory) or \
           (obstacle == '-' and 'k' in inventory) or \
           (obstacle == '*' and 'd' in inventory) or \
           (obstacle == '~' and RAFT in inventory)

def get_tool_action(tile):
    if tile == 'T' and 'a' in inventory:
        inventory.add(RAFT)
        return 'C'  
    elif tile == '-' and 'k' in inventory:
        return 'U'  
    elif tile == '*' and 'd' in inventory:
        return 'B'  
    return None

# Determine the next movement
def move_towards(next_pos):
    global agent_pos, agent_dir

    dx, dy = next_pos[0] - agent_pos[0], next_pos[1] - agent_pos[1]

    # change agent_pos as returning the correct output that goes closer to target
    for d, (dx_d, dy_d) in DIRECTIONS.items():
        if dx_d == dx and dy_d == dy:
            if agent_dir == d:
                tile = world_map.get(next_pos, '?')
                tool_action = get_tool_action(tile)
                if tool_action:
                    return tool_action  # Use tool instead of trying to walk through
                agent_pos = next_pos
                return 'F'
            
            left_turn = TURN_LEFT[agent_dir]
            right_turn = TURN_RIGHT[agent_dir]

            if left_turn == d:
                agent_dir = left_turn
                return 'L'
            elif right_turn == d:
                agent_dir = right_turn
                return 'R'
    
    agent_dir = TURN_LEFT[agent_dir]
    return 'L'

def try_alternative_moves():
    global agent_dir, agent_pos

    dir_scores = []

    for d, (dx, dy) in DIRECTIONS.items():
        nx, ny = agent_pos[0] + dx, agent_pos[1] + dy
        tile = world_map.get((nx, ny), '?')
        visits = visit_count.get((nx, ny), 0)

        # Now allows moving through passable obstacles
        if tile != '?' and (tile not in OBSTACLES or has_required_tool(tile)):
            dir_scores.append((visits, d, (nx, ny)))

    if not dir_scores:
        # All directions are invalid → try turning both ways to escape
        agent_dir = TURN_RIGHT[agent_dir]
        return 'R'

    dir_scores.sort()
    _, best_dir, next_pos = dir_scores[0]

    if best_dir == agent_dir:
        agent_pos = next_pos
        return 'F'
    elif TURN_LEFT[agent_dir] == best_dir:
        agent_dir = TURN_LEFT[agent_dir]
        return 'L'
    elif TURN_RIGHT[agent_dir] == best_dir:
        agent_dir = TURN_RIGHT[agent_dir]
        return 'R'
    else:
        agent_dir = TURN_LEFT[agent_dir]
        return 'L'
    
# explore all possible tiles that is unknown/until target is found
def explore():
    global planned_path, agent_pos, agent_dir

    if planned_path:
        return move_towards(planned_path.popleft())

    unexplored_path = bfs(agent_pos, lambda x, y: world_map.get((x, y), '?') == '?')
    if unexplored_path:
        planned_path = deque(unexplored_path)
        return move_towards(planned_path.popleft())

    # go to less visited tiles if visit same tiles for more than 5 times
    if visit_count.get(agent_pos, 0) > VISIT_THRESHOLD:
        return try_alternative_moves()

    # Try to move forward if not blocked
    dx, dy = DIRECTIONS[agent_dir]
    forward_pos = (agent_pos[0] + dx, agent_pos[1] + dy)
    tile = world_map.get(forward_pos, '?')
    if tile not in OBSTACLES:
        agent_pos = forward_pos
        return 'F'

    # Turn if stuck
    agent_dir = TURN_LEFT[agent_dir]
    return 'L'

# find any obstacle to use tools
def target_obstacle_position():
    for (x, y), tile in world_map.items():
        if tile in OBSTACLES and has_required_tool(tile):
            return (x, y)
    return None

# function to take get action from AI or user
def get_action(view):
    global planned_path, target_pos, agent_pos, inventory, has_treasure, start_pos

    update_world_map(view)

    # since view is made with 5x5 grid, its centre should be [2,2]
    if agent_pos in tool_positions:
        tool = tool_positions[agent_pos]
        inventory.add(tool)
        del tool_positions[agent_pos]

    if has_treasure:
        planned_path.clear()
        target_pos = start_pos

    # use bfs to go to target (e.g. treasure or start)
    if target_pos:
        new_path = bfs(agent_pos, lambda x, y: (x, y) == target_pos)
        if new_path:
            planned_path = deque(new_path)
        else:
            planned_path.clear()

    if not planned_path and inventory:
        obstacle_pos = target_obstacle_position()
        if obstacle_pos:
            new_path = bfs(agent_pos, lambda x, y: (x, y) == obstacle_pos)
            if new_path:
                planned_path = deque(new_path)

    if planned_path:
        return move_towards(planned_path.popleft())

    return explore()

# helper function to print the grid
def print_grid(view):
    print('+-----+')
    for ln in view:
        print("|"+str(ln[0])+str(ln[1])+str(ln[2])+str(ln[3])+str(ln[4])+"|")
    print('+-----+')

if __name__ == "__main__":

    # checks for correct amount of arguments 
    if len(sys.argv) != 3:
        print("Usage Python3 "+sys.argv[0]+" -p port \n")
        sys.exit(1)

    port = int(sys.argv[2])

    # checking for valid port number
    if not 1025 <= port <= 65535:
        print('Incorrect port number')
        sys.exit()

    # creates TCP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
         # tries to connect to host
         # requires host is running before agent
         sock.connect(('localhost',port))
    except (ConnectionRefusedError):
         print('Connection refused, check host is running')
         sys.exit()

    # navigates through grid with input stream of data
    i=0
    j=0
    while True:
        data=sock.recv(100)
        if not data:
            exit()
        for ch in data:
            if (i==2 and j==2):
                view[i][j] = '^'
                view[i][j+1] = chr(ch)
                j+=1 
            else:
                view[i][j] = chr(ch)
            j+=1
            if j>4:
                j=0
                i=(i+1)%5
        if j==0 and i==0:
            #print_grid(view) # COMMENT THIS OUT ON SUBMISSION
            action = get_action(view) # gets new actions
            sock.send(action.encode('utf-8'))

    sock.close()
