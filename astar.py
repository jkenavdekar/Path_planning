from math import sqrt
zeroes =  0
start = [0, 0]
goal = [5, 9]
grid =  [[0, 0, 0, 0, 0, 0, 0, 1, 0, 0],  
         [0, 1, 1, 0, 0, 0, 0, 1, 0, 0],  
         [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],  
         [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],  
         [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],  
         [0, 1, 1, 0, 0, 0, 0, 0, 0, 0], 
         [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],  
         [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],  
         [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],  
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]] 

path = [[zeroes for _ in range(len(grid))] for _ in range(len(grid))]
best_path = [[zeroes for _ in range(len(grid))] for _ in range(len(grid))]
h_grid = [[zeroes for _ in range(len(grid))] for _ in range(len(grid))]


class BreadthFirstSearch:
    def __init__(self, start, goal, grid, path):
        self.pos = start
        self.pos_str = str(start)
        self.pos_depth = 0
        self.direction = 1
        self.goal_str = str(goal)
        self.explored = {}
        self.not_explored = {}
        self.not_explored[str(start)] = 0
        self.grid = grid
        self.path = path
        self.h_grid = h_grid

    # START

    def get_possible_moves(self):
        potential_moves = self.generate_potential_moves(self.pos)
        for move in potential_moves:
            # Check if potential move is valid.
            if not self.valid_move(move):
                continue
            # Check if move has already been explored.
            if (str(move) not in self.explored) and (str(move) not in self.not_explored):
                self.not_explored[str(move)] = self.pos_depth + 1 + self.heuristic(move)
                
                self.h_grid[move[0]][move[1]] = self.pos_depth + 1 + self.heuristic(move)     
        self.explored[self.pos_str] = 0      
        # Since all next possible moves have been determined,
        # consider current location explored.

        return True

    def goal_found(self):
        if self.goal_str in self.not_explored:
            # Add goal to path.
            self.pos = self.string_to_array(self.goal_str)
            self.pos_depth = self.not_explored.pop(self.goal_str)
            self.path[self.pos[0]][self.pos[1]] = self.pos_depth
            return True
        return False

    def explore_next_move(self):
        # explore next move.
        sorted_not_explored = sorted(self.not_explored, key = self.not_explored.get, reverse=False)
        
        # Determine the pos and depth of next move.
        
        self.pos_str = sorted_not_explored[0]
        self.pos = self.string_to_array(self.pos_str)
        self.pos_depth = self.not_explored.pop(self.pos_str) - self.heuristic(self.pos)
        self.path[self.pos[0]][self.pos[1]] = round(self.pos_depth, 1)

        # Write depth of next move onto path.

        return True
    
    def heuristic(self, move):
        d1 = move[0] - goal[0]
        d2 = move[1] - goal[1]
        d1 = d1**2
        d2 = d2**2
        return sqrt(d1 + d2)

    # END


    def generate_potential_moves(self, pos):
        potential_moves = [[0, 0],[0, 0],[0, 0],[0, 0]]
        
        
        potential_moves[0] = [pos[0] - 1, pos[1]] #up
        potential_moves[1] = [pos[0] + 1, pos[1]] #down
        potential_moves[2] = [pos[0], pos[1] - 1] #left
        potential_moves[3] = [pos[0], pos[1] + 1] #right
        
        '''
        if pos[0] != 0 and pos[1] != 9:
            potential_moves[0] = [pos[0] - 1, pos[1] + 1] #ur
        if pos[0] != 0 and pos[1] != 0:
            potential_moves[1] = [pos[0] - 1, pos[1] - 1] #ul
        if pos[0] != 9 and pos[1] != 9:
            potential_moves[2] = [pos[0] + 1, pos[1] + 1] #dr
        if pos[0] != 9 and pos[1] != 0:
            potential_moves[3] = [pos[0] + 1, pos[1] - 1] #dl
        '''    
        return potential_moves

    def valid_move(self, move):
        # Check for boundary.
        if (move[0] < 0) or (move[0] > 9):
            return False
        if (move[1] < 0) or (move[1] > 9):
            return False
        # Check for obstacle 
        if self.grid[move[0]][move[1]]:
            return False
        return True

    def string_to_array(self, string):
        array = [int(string[1]), int(string[4])]
        return array


# Init
bfs = BreadthFirstSearch(start, goal, grid, path)

while True:
    # Determine next possible moves.
    bfs.get_possible_moves()
    if bfs.goal_found():
        break
    bfs.explore_next_move()

print('')
print('Heuristic Grid')
print('-------------')
print(h_grid)
print('')
print('')
print('Explored Path')
print('-------------')
print("\n",path[0],"\n",path[1],"\n",path[2],"\n",path[3],"\n",path[4],"\n",path[5],"\n",path[6],"\n",path[7],"\n",path[8],"\n",path[9])
print('')
#print('Fully explored count ' + str(path[0].count(0)))


def find_best_path(pos):
    best_path[pos[0]][pos[1]] = 1
    h_pos = path[pos[0]][pos[1]]
    if h_pos == 1:
        return 1

    potential_moves = bfs.generate_potential_moves(pos)
    for move in potential_moves:
        if not bfs.valid_move(move):
            continue
        h_move = path[move[0]][move[1]]
        if h_move == (h_pos - 1):
            return find_best_path(move) + 1


goal_count = find_best_path(goal)
best_path[start[0]][start[1]] = 88
print('')
print('Best Path To Goal')
print('-----------------')
print("\n",best_path[0],"\n",best_path[1],"\n",best_path[2],"\n",best_path[3],"\n",best_path[4],"\n",best_path[5],"\n",best_path[6],"\n",best_path[7],"\n",best_path[8],"\n",best_path[9])
print('')
print('Moves to Goal: ' + str(goal_count))
print('')

