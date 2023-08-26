# python -m pip install bresenham


from bresenham import bresenham
import pygame
import math
from queue import PriorityQueue

WIDTH = 800
# setting window of display
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Finding Algorithm")


# defining colours
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 255, 0)
yellow = (255, 255, 0)
white = (255, 255, 255)
black = (0, 0, 0)
purple = (128, 0, 128)
orange = (255, 165, 0)
grey = (128, 128, 128)
turquoise = (64, 224, 208)


class Spot:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        # for fidning coordinates on computer screen
        self.x = row * width  # width of each square in grid
        self.y = col * width
        self.color = white
        self.neighbours = []
        self.width = width
        self.total_rows = total_rows

    # getting position of nodes
    def get_pos(self):
        return self.row, self.col

    # whether a node has been looked at
    def is_closed(self):
        return self.color == red

    # whether node hasnt been looked at
    def is_open(self):
        return self.color == green

    def is_barrier(self):
        return self.color == black

    # if node is end node
    def is_end(self):
        return self.color == turquoise

    # if node is linetraceable line
    def is_line(self):
        return self.color == grey

    # to change the colour back to white
    def reset(self):
        self.color = white

    # making the node the colours
    def make_closed(self):
        self.color = red

    def make_open(self):
        self.color = green

    def make_barrier(self):
        self.color = black

    def make_start(self):
        self.color = orange

    def make_end(self):
        self.color = turquoise

    def make_path(self):
        self.color = purple

    def make_line(self):
        self.color = grey

    # where you want to draw             #pygame 0,0 is at the top left, y increases downwards and x increases rightwards
    def draw(self, win):
        pygame.draw.rect(
            win, self.color, (self.x, self.y, self.width, self.width))  # x and y

    # check if adjacent nodes exist and add to neighbours list
    def update_neighbours(self, grid):
        self.neighbours = []
        # check if can go down
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier():
            self.neighbours.append(grid[self.row + 1][self.col])

        # check if can go up
        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():
            self.neighbours.append(grid[self.row - 1][self.col])

        # check if can go right
        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier():
            self.neighbours.append(grid[self.row][self.col + 1])

        # check if can go left
        if self.row > 0 and not grid[self.row][self.col - 1].is_barrier():
            self.neighbours.append(grid[self.row][self.col - 1])


def h(p1, p2):  # heuristic distance
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)


# trace back by following parent nodes to find the shortest path after finding end
def reconstruct_path(came_from, current, draw):
    path_cords = []
    while current in came_from:  # came_from is a dictionary of parent nodes
        current = came_from[current]
        current.make_path()
        path_cords.append((current.x // 16, current.y // 16))
        draw()
    print(path_cords[::-1])


def algorithm(draw, grid, start, end):
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
        for event in pygame.event.get():
            # if user presses "X (button at top right of window)", closes the window
            if event.type == pygame.QUIT:
                pygame.quit()

        # getting the corresponding node (im also not very sure but this basically gets the current node)
        current = open_set.get()[2]
        # removes current node from open_set_hash as it has been discovered alr
        open_set_hash.remove(current)

        if current == end:
            # creates shortest path back
            reconstruct_path(came_from, end, draw)
            set_map(grid)
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

        draw()  # calls draw function to change colours of open and closed nodes

        if current != start:
            current.make_closed()  # turns open to closed

    return False  # if there is no path from start to end


def make_grid(rows, width):
    grid = []  # grid will be a list of empty lists
    gap = width // rows  # width of spot (node)
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            # (row, col, width(of spot), no_of_rows)     creating nodes
            spot = Spot(i, j, gap, rows)
            grid[i].append(spot)  # adding nodes

    return grid


def draw_grid(win, rows, width):
    gap = width // rows  # width of spot (node)
    for i in range(rows):
        pygame.draw.line(win, grey, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, grey, (j * gap, 0), (j * gap, width))


def draw(win, grid, rows, width):
    win.fill(white)

    for row in grid:
        for spot in row:
            # changes colour of spot at the correct x and y coords (.draw() method)
            spot.draw(win)

    draw_grid(win, rows, width)
    pygame.display.update()  # updates the entire window


def get_clicked_pos(pos, rows, width):
    gap = width // rows
    y, x = pos  # mouse position is in (y, x) for some reason

    row = y // gap  # rows = y cord // width of each node
    col = x // gap  # col = x cord // width of each node

    return row, col


def draw_line(grid, x1, y1, x2, y2):
    line = list(bresenham(x1, y1, x2, y2))
    for i in range(len(line)):
        spot = grid[line[i][0]][line[i][1]]
        spot.make_line()


def set_map(grid):  # for recreating nrc map in window (work in progress)

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
            spot.make_barrier()


def main(win, width, start_end_cords):
    rows = 50
    grid = make_grid(rows, width)

    run = True
    started = False

    set_map(grid)  # add map\

    for i in start_end_cords:
        start_cord = i[0]
        end_cord = i[1]
        run = True
        next = False
        while not next:
            while run:
                draw(win, grid, rows, width)  # draws grid
                start = grid[start_cord[0]][start_cord[1]]
                end = grid[end_cord[0]][end_cord[1]]
                start.make_start()
                end.make_end()

                for row in grid:
                    for spot in row:
                        # creates list of neighbours for every spot
                        spot.update_neighbours(grid)
                if algorithm(lambda: draw(win, grid, rows, width),  # performs algorithm
                             grid, start, end):
                    run = False

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:  # pressing "X" = quit
                        pygame.quit()

            start = None
            end = None
            grid = make_grid(rows, width)
            set_map(grid)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:  # pressing "X" = quit
                    pygame.quit()
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:  # if space is pressed and start and end exist
                        next = True

    pygame.quit()


main(WIN, WIDTH, (((3, 24), (7, 30)), ((3, 21), (20, 10))))
