

import math
import time
import heapq
import tracemalloc
import gc


tracemalloc.start()

line = [(5, 16), (6, 16), (7, 16), (8, 16), (8, 16),
        (9, 15), (10, 14), (11, 13), (12, 12), (13, 11),
        (13, 11), (13, 10), (13, 9), (13, 8), (13, 7), (13, 6),
        (13, 5), (13, 11), (14, 12), (15, 13), (16, 14), (17, 15),
        (14, 12), (15, 12), (16, 12), (17, 12), (18, 12), (19, 12),
        (20, 12), (21, 12), (22, 12), (23, 12), (24, 12), (25, 12),
        (26, 12), (27, 12), (28, 12), (29, 12), (30, 12), (31, 12),
        (17, 15), (17, 16), (17, 17), (17, 18), (24, 4), (24, 5),
        (24, 6), (24, 7), (24, 7), (25, 8), (26, 8), (26, 8),
        (27, 8), (28, 8), (29, 8), (30, 8), (31, 8), (31, 8), (32, 9),
        (33, 9), (34, 10), (31, 12), (32, 11), (33, 10), (34, 10),
        (35, 9), (36, 8), (37, 7), (38, 6), (39, 5), (40, 5), (41, 4),
        (42, 3), (31, 12), (32, 13), (32, 14), (33, 15), (33, 16), (33, 16),
        (33, 17), (34, 10), (35, 10), (36, 10), (37, 10), (38, 10), (39, 10),
        (40, 10), (41, 10)]



class Spot:
    def __init__(self, row, col):
        # for fidning coordinates on computer screen
        self.x = row
        self.y = col
        self.state = 0
        self.neighbours = []

    # getting position of nodes
    def get_pos(self):
        return self.x, self.y

    # whether a node has been looked at
    def is_closed(self):
        return self.state == 1

    # whether node hasnt been looked at
    def is_open(self):
        return self.state == 2

    def is_barrier(self):
        return self.state == 3

    # if node is end node
    def is_end(self):
        return self.state == 4

    # if node is linetraceable line
    def is_line(self):
        return self.state == 6

    # to change the colour back to white
    def reset(self):
        self.state = 0

    # making the node the colours
    def make_closed(self):
        self.state = 1

    def make_open(self):
        self.state = 2

    def make_barrier(self):
        self.state = 3

    def make_start(self):
        self.state = 4

    def make_end(self):
        self.state = 4

    def make_path(self):
        self.state = 5

    def make_line(self):
        self.state = 6
        line.append(self.get_pos())

    # check if adjacent nodes exist and add to neighbours list
    def update_neighbours(self, grid):
        self.neighbours = []

        if self.x < hx and not grid[self.x - lx + 1][self.y - ly].is_barrier():
            self.neighbours.append(
                grid[self.x - lx + 1][self.y - ly])  # go down

        if self.x > lx and not grid[self.x - lx - 1][self.y - ly].is_barrier():
            self.neighbours.append(grid[self.x - lx - 1][self.y - ly])  # go up

        if self.y < hy and not grid[self.x - lx][self.y + 1 - ly].is_barrier():
            self.neighbours.append(
                grid[self.x - lx][self.y - ly + 1])  # go right

        if self.y > ly and not grid[self.x - lx][self.y - ly - 1].is_barrier():
            self.neighbours.append(
                grid[self.x - lx][self.y - ly - 1])  # go left


def bresenham(x0, y0, x1, y1):
    # from default bresenham
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

    

class PriorityQueue:
    # self-made priority queue
    def __init__(self):
        self.queue = []

    def put(self, item: tuple):
        heapq.heappush(self.queue, item)

    def get(self):
        return heapq.heappop(self.queue)

    def empty(self):
        return not bool(len(self.queue))


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
        current.make_path()
        path_cords.append((current.x, current.y))
    
    print("reconstruct path", tracemalloc.get_traced_memory())
    tracemalloc.reset_peak()

    path_cords = path_cords[::-1]
    return path_cords


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
            start.make_start()  # recolours start and end cuz the path will colour over them
            end.make_end()
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
                    neighbour.make_open()

        # calls draw function to change colours of open and closed nodes

        if current != start:
            current.make_closed()  # turns open to closed

    print("algorithm", tracemalloc.get_traced_memory())
    tracemalloc.reset_peak()
    gc.collect()
    return False  # if there is no path from start to end


def make_grid():
    grid = []  # grid will be a list of empty lists
    for i in range(lx, hx + 1):
        grid.append([])
        for j in range(ly, hy + 1):
            # (row, col, width(of spot), no_of_rows)     creating nodes
            spot = Spot(i, j)
            grid[i - lx].append(spot)  # adding nodes
    
    gc.collect()
    print("make grid", tracemalloc.get_traced_memory())
    tracemalloc.reset_peak()
    return grid


def draw_line(grid, x1, y1, x2, y2):
    line = list(bresenham(x1, y1, x2, y2))
    for i in range(len(line)):
        if line[i][0] < lx or line[i][0] > hx or line[i][1] < ly or line[i][1] > hy:
            continue
        spot = grid[line[i][0] - lx][line[i][1] - ly]
        spot.make_line()


def set_map(grid):  # for recreating nrc map in window (work in progress)
    # for i in range(50):  # test line
    #     spot = grid[i][10]
    #     spot.make_line()
    # barriers
    rect_barrier(grid, 0, 23, 49, 23)
    rect_barrier(grid, 19, 0, 20, 6)
    rect_barrier(grid, 41, 0, 43, 2)
    rect_barrier(grid, 41, 9, 44, 11)
    rect_barrier(grid, 29, 16, 36, 23)
    rect_barrier(grid, 14, 18, 21, 23)
    rect_barrier(grid, 2, 15, 5, 18)

    print("set map", tracemalloc.get_traced_memory())
    tracemalloc.reset_peak()


def rect_barrier(grid, x1, y1, x2, y2):
    for i in range(x1, x2 + 1):
        if i < lx or i > hx:
            continue
        for j in range(y1, y2 + 1):
            if j < ly or j > hy:
                continue
            spot = grid[i - lx][j - ly]
            spot.make_barrier()
    

def simplify(path):
    # simplify the path to shorten
    x_changing = False
    repeat_list = []
    for i in range(1, len(path)):
        if i < (len(path) - 1):
            if x_changing:
                if path[i - 1][1] == path[i][1]:
                    repeat_list.append(i - 1)
                else:
                    x_changing = False
            else:
                if path[i - 1][0] == path[i][0]:
                    repeat_list.append(i - 1)
                else:
                    x_changing = True

    for j in repeat_list[::-1]:
        del path[j]

    gc.collect()
    print("simplify", tracemalloc.get_traced_memory())
    tracemalloc.reset_peak()

    return path


def diagonal_simplify(path):
    # simplify diagonally (take the indices of the diagonal segment, remove them.)
    repeat_list = []
    for i in range(len(path) - 1):
        if (abs(path[i + 1][0] - path[i][0]) == 1) or \
                (abs(path[i + 1][1] - path[i][1]) == 1):
            if (abs(path[i - 1][0] - path[i][0]) == 1) or \
                    (abs(path[i - 1][1] - path[i][1]) == 1):
                repeat_list.append(i)
    
    for j in repeat_list[::-1]:
        del path[j]

    print("diagonal simplify", tracemalloc.get_traced_memory())
    tracemalloc.reset_peak()

    return path


def main(start_cord, end_cord):

    global lx
    global hx
    global ly
    global hy

    lx = min([start_cord[0], end_cord[0]])
    hx = max([start_cord[0], end_cord[0]])
    ly = min([start_cord[1], end_cord[1]])
    hy = max([start_cord[1], end_cord[1]])

    grid = make_grid()
    run = True
    test = time.time()

    set_map(grid)  # add map

    run = True
    start_time = time.time()
    while run:
        start = grid[start_cord[0] - lx][start_cord[1] - ly]
        end = grid[end_cord[0] - lx][end_cord[1] - ly]
        start.make_start()
        end.make_end()

        for row in grid:
            for spot in row:
                # creates list of neighbours for every spot
                spot.update_neighbours(grid)
        gc.collect()

        if algorithm(grid, start, end):  # performs algorithm
            end_time = time.time()
            run = False

        start = None
        end = None
        # grid = make_grid()

    new_path = simplify(path_cords)
    print(new_path)
    new_path = diagonal_simplify(new_path)
    new_path.append(end_cord)
    print(new_path)
    print((time.time() - test))


main((13, 5), (23, 3))  


# displaying the memory
print(tracemalloc.get_traced_memory())
tracemalloc.reset_peak()
# stopping the library
tracemalloc.stop()
