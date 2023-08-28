
import math
from queue import PriorityQueue
import time

def bresenham(x0, y0, x1, y1):
    # from default bresenham in python

    dx = x1 - x0
    dy = y1 - y0

    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx, xy, yx, yy = xsign, 0, 0, ysign
    else:
        dx, dy = dy, dx
        xx, xy, yx, yy = 0, ysign, xsign, 0

    D = 2*dy - dx
    y = 0

    for x in range(dx + 1):
        yield x0 + x*xx + y*yx, y0 + x*xy + y*yy
        if D >= 0:
            y += 1
            D -= 2*dx
        D += 2*dy

class Spot:
    def __init__(self, row, col, total_rows):
        # for fidning coordinates on computer screen
        self.x = row
        self.y = col
        self.state = "empty"
        self.neighbours = []
        self.total_rows = total_rows

    # getting position of nodes
    def get_pos(self):
        return self.x, self.y

    # whether a node has been looked at
    def is_closed(self):
        return self.state == "closed"

    # whether node hasnt been looked at
    def is_open(self):
        return self.state == "open"

    def is_barrier(self):
        return self.state == "barrier"

    # if node is end node
    def is_end(self):
        return self.state == "end"

    # if node is linetraceable line
    def is_line(self):
        return self.state == "line"

    # to change the colour back to white
    def reset(self):
        self.state = "empty"

    # check if adjacent nodes exist and add to neighbours list
    def update_neighbours(self, grid):
        self.neighbours = []
        
        if self.x < self.total_rows - 1 and not grid[self.x + 1][self.y].is_barrier():
            self.neighbours.append(grid[self.x + 1][self.y]) # go down

        if self.x > 0 and not grid[self.x - 1][self.y].is_barrier():
            self.neighbours.append(grid[self.x - 1][self.y]) # go up

        if self.y < self.total_rows - 1 and not grid[self.x][self.y + 1].is_barrier():
            self.neighbours.append(grid[self.x][self.y + 1]) # go right

        if self.y > 0 and not grid[self.x][self.y - 1].is_barrier():
            self.neighbours.append(grid[self.x][self.y - 1]) # go left
        


def h(p1, p2):  # heuristic distance
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)


# trace back by following parent nodes to find the shortest path after finding end
def reconstruct_path(came_from, current):
    global path_cords
    path_cords = []
    while current in came_from:  # came_from is a dictionary of parent nodes
        current = came_from[current]
        current.state = "purple"
        path_cords.append((current.x, current.y))
    
    path_cords = path_cords[::-1]
    print(path_cords)


def algorithm(grid, start, end):
    count = 0
    open_set = PriorityQueue()  # gets the minimum element from the queue
    open_set.put((0, count, start))
    came_from = {}
    # g_score = distance of start to current node
    # g_score of every node (initialise to infinity)
    g_score = {spot: float("inf") for row in grid for spot in row}
    g_score[start] = 0
    # distance of current node to end node
    # f_score of every node (initialise to infinity)
    f_score = {spot: float("inf") for row in grid for spot in row}
    f_score[start] = h(start.get_pos(), end.get_pos())  # heuristic distance

    open_set_hash = {start}  # a set of open_set list

    while not open_set.empty():
        # getting the corresponding node (im also not very sure but this basically gets the current node)
        current = open_set.get()[2]
        # removes current node from open_set_hash as it has been discovered alr
        open_set_hash.remove(current)

        if current == end:
            # creates shortest path back
            reconstruct_path(came_from, end)
            set_map(grid)
            start.state = "start"  # recolours start and end cuz the path will colour over them
            end.state = "end"
            # making path
            return True

        for neighbour in current.neighbours:  # for each neighbour of each current node

            if neighbour.is_line():  # encourages algorithm to use line as increase in g_score is lower when crossing line
                temp_g_score = g_score[current] + 0.8
            else:
                temp_g_score = g_score[current] + 1

            # if this path is better than the previous path
            if temp_g_score < g_score[neighbour]:
                came_from[neighbour] = current  # updates parent
                g_score[neighbour] = temp_g_score  # updates g_score
                f_score[neighbour] = temp_g_score + \
                    h(neighbour.get_pos(), end.get_pos())  # updates f_score
                if neighbour not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbour], count, neighbour))
                    # adding neighbour into open_set_hash
                    open_set_hash.add(neighbour)
                    neighbour.state = "open"

        # calls draw function to change colours of open and closed nodes

        if current != start:
            current.state = "closed"  # turns open to closed

    return False  # if there is no path from start to end


def make_grid(rows):
    grid = []  # grid will be a list of empty lists
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            # (row, col, width(of spot), no_of_rows)     creating nodes
            spot = Spot(i, j, rows)
            grid[i].append(spot)  # adding nodes

    return grid


def draw_line(grid, x1, y1, x2, y2):
    line = list(bresenham(x1, y1, x2, y2))
    for i in range(len(line)):
        spot = grid[line[i][0]][line[i][1]]
        spot.state = "line"


def set_map(grid):  
    # for recreating nrc map in window (work in progress)
    # barriers
    rect_barrier(grid, 0, 24, 49, 24)
    rect_barrier(grid, 19, 0, 20, 6)
    rect_barrier(grid, 41, 0, 43, 2)
    rect_barrier(grid, 41, 9, 44, 11)
    rect_barrier(grid, 29, 16, 36, 23)
    rect_barrier(grid, 14, 18, 21, 23)
    rect_barrier(grid, 2, 15, 5, 18)

    # lines (left to right)
    draw_line(grid, 5, 16, 8, 16)
    draw_line(grid, 8, 16, 13, 11)
    draw_line(grid, 13, 11, 13, 5)
    draw_line(grid, 13, 11, 17, 15)
    draw_line(grid, 14, 12, 31, 12)
    draw_line(grid, 17, 15, 17, 18)
    draw_line(grid, 24, 4, 24, 7)
    draw_line(grid, 24, 7, 26, 8)
    draw_line(grid, 26, 8, 31, 8)
    draw_line(grid, 31, 8, 34, 10)
    draw_line(grid, 31, 12, 42, 3)
    draw_line(grid, 31, 12, 33, 16)
    draw_line(grid, 33, 16, 33, 17)
    draw_line(grid, 34, 10, 41, 10)


def rect_barrier(grid, x1, y1, x2, y2):
    for i in range(x1, x2 + 1):
        for j in range(y1, y2 + 1):
            spot = grid[i][j]
            spot.state = "barrier"

def simplify(path):
    # simplify the path to shorten
    x_changing = False
    repeat_list = []
    for i in range(1, len(path)):
        if i < (len(path) - 1): 
            if x_changing:
                if path[i - 1][1] == path[i][1]: repeat_list.append(i - 1)
                else: x_changing = False
            else: 
                if path[i - 1][0] == path[i][0]: repeat_list.append(i - 1)
                else: x_changing = True
        
    for j in repeat_list[::-1]: del path[j]

    return path

def diagonal_simplify(path):
    # simplify diagonally (take the indices of the diagonal segment, remove them.)
    repeat_list = []
    for i in range(len(path) - 1):
        if (abs(path[i + 1][0] - path[i][0]) == 1) or \
            (abs(path[i + 1][1] - path[i][1]) == 1): 
            if (abs(path[i - 1][0] - path[i][0]) == 1) or \
                (abs(path[i - 1][1] - path[i][1]) == 1): repeat_list.append(i)
        
    print(repeat_list)
    for j in repeat_list[::-1]: del path[j]

    return path

def main(start_end_cords):

    rows = 50
    grid = make_grid(rows)
    run = True

    set_map(grid)  # add map

    for i in start_end_cords:
        start_cord = i[0]
        end_cord = i[1]
        run = True
        start_time = time.time()
        while run:
            start = grid[start_cord[0]][start_cord[1]]
            end = grid[end_cord[0]][end_cord[1]]
            start.state = "start"
            end.make_end = "start"

            for row in grid:
                for spot in row:
                    # creates list of neighbours for every spot
                    spot.update_neighbours(grid)
            if algorithm(grid, start, end): #performs algorithm
                end_time = time.time()
                print(end_time - start_time)
                run = False

            start = None
            end = None
            grid = make_grid(rows)
            set_map(grid)

    new_path = simplify(path_cords)
    print(new_path)
    new_path = diagonal_simplify(new_path)
    print(new_path)
    print((time.time() - start_time))


main((((3, 24), (7, 30)), ((3, 21), (20, 10))))
