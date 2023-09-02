from hub import port
import runloop
import color_sensor
import color
import motor
import motor_pair
from hub import motion_sensor
import math
import runloop
import device
import time
import heapq

# 1 September
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

#basic functions

def get_angle():
    return motion_sensor.tilt_angles()[0] / 10

def cm(cm):        #converts cm into motor degrees
    degrees = (cm / 4.5) * (180 / math.pi)    #radius to be set
    return degrees

# Pathfinding
class Spot:
    def __init__(self, row, col, total_rows):
        # for fidning coordinates on computer screen
        self.x = row
        self.y = col
        self.state = 0
        self.neighbours = []
        self.total_rows = total_rows

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
        
        if self.x < self.total_rows - 1 and not grid[self.x + 1][self.y].is_barrier():
            self.neighbours.append(grid[self.x + 1][self.y]) # go down

        if self.x > 0 and not grid[self.x - 1][self.y].is_barrier():
            self.neighbours.append(grid[self.x - 1][self.y]) # go up

        if self.y < self.total_rows - 1 and not grid[self.x][self.y + 1].is_barrier():
            self.neighbours.append(grid[self.x][self.y + 1]) # go right

        if self.y > 0 and not grid[self.x][self.y - 1].is_barrier():
            self.neighbours.append(grid[self.x][self.y - 1]) # go left
        
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
        spot.make_line()


def set_map(grid):  # for recreating nrc map in window (work in progress)
    # for i in range(50):  # test line
    #     spot = grid[i][10]
    #     spot.make_line()
    # barriers
    rect_barrier(grid, 0, 24, 49, 24)
    rect_barrier(grid, 19, 0, 20, 6)
    rect_barrier(grid, 41, 0, 43, 2)
    rect_barrier(grid, 41, 9, 44, 11)
    rect_barrier(grid, 29, 16, 36, 23)
    rect_barrier(grid, 14, 18, 21, 23)
    rect_barrier(grid, 2, 15, 5, 18)

    '''# lines (left to right)
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
    draw_line(grid, 34, 10, 41, 10)'''


def rect_barrier(grid, x1, y1, x2, y2):
    for i in range(x1, x2 + 1):
        for j in range(y1, y2 + 1):
            spot = grid[i][j]
            spot.make_barrier()

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

def astar(start_cord, end_cord):
    rows = 50
    grid = make_grid(rows)
    run = True
    test = time.time()

    set_map(grid)  # add map

    run = True
    start_time = time.time()
    while run:
        start = grid[start_cord[0]][start_cord[1]]
        end = grid[end_cord[0]][end_cord[1]]
        start.make_start()
        end.make_end()

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

    new_path = simplify(path_cords)
    print(new_path)
    new_path = diagonal_simplify(new_path)
    new_path.append(end_cord)
    print(new_path)
    print((time.time() - test))

def astar_pid(start_cord, end_cord, speed):
    path = astar(start_cord, end_cord)
    for i in range(1, len(path)):
        ptp(path[i - 1], path[i], speed)
            
    
#robot movements
def gyro_straight(distance, multiplier, speed):
    # robot move straight from where its facing
    # parameters:(distance in cm, magnitude of correction (should be pretty low), speed)
    motion_sensor.reset_yaw(0)
    target = 0
    avg_dist = (abs(motor.relative_position(port.A)) + abs(motor.relative_position(port.B))) / 2
    while avg_dist < cm(distance):
        avg_dist = (abs(motor.relative_position(port.A)) + abs(motor.relative_position(port.B))) / 2
        angle = get_angle() #similar to proportional line follower (part of PID)
        error = target - angle
        correct = error * multiplier
        motor_pair.move_tank(motor_pair.PAIR_1, round(speed - correct), round(speed + correct))


def turn_180(target, speed):
    degrees = round((target/180) * 180)
    motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, degrees, -speed, speed)


def ots_turn(degree, speed): #on the spot turn
    # parameters:(yaw degree(from where the robot is facing), speed)
    target = get_angle() - degree
    switch = False
    print(target)
    if (target > 170 and target < 190) or (target < -170 and target > -190):
        turn_180(target, speed)
        return
    elif target < -180:
        target += 360
        switch = True
    elif target > 180:
        target -= 360
        switch = True
    print(target)
    if switch == True:
        if degree > 0:
            while get_angle() < target:    #turn left
                motor_pair.move_tank(motor_pair.PAIR_1, -speed, speed)
        else:
            while get_angle() > target:    #turn left
                motor_pair.move_tank(motor_pair.PAIR_1, speed, -speed)
    else:
        if degree < 0:
            while get_angle() < target:    #turn left
                motor_pair.move_tank(motor_pair.PAIR_1, -speed, speed)
        else:
            while get_angle() > target:    #turn left
                motor_pair.move_tank(motor_pair.PAIR_1, speed, -speed)
    motor_pair.stop(motor_pair.PAIR_1)

def ow_turn(degree, speed): #one wheel turn    parameters:(yaw degree(from where the robot is facing), speed)
    # parameters:(yaw degree(from where the robot is facing), speed)
    target = get_angle() - degree
    switch = False
    print(target)
    if target < -180:
        target += 360
        switch = True
    elif target > 180:
        target -= 360
        switch = True
    print(target)
    if switch == True:
        if degree > 0:
            while get_angle() < target:    #turn left
                motor_pair.move_tank(motor_pair.PAIR_1, 0, speed)
                print(target, get_angle(), "1")
        else:
            while get_angle() > target:    #turn left
                motor_pair.move_tank(motor_pair.PAIR_1, speed, 0)
                print(target, get_angle(), "2")
    else:
        if degree < 0:
            while get_angle() < target:    #turn left
                motor_pair.move_tank(motor_pair.PAIR_1, 0, speed)
                print(target, get_angle(), "3")
        else:
            while get_angle() > target:    #turn left
                motor_pair.move_tank(motor_pair.PAIR_1, speed, 0)
                print(target, get_angle(), "4")


#more complex robot movements

def PID(target, kp, ki, kd, speed): #PID line follower (smooth line follower) go watch a vid about it or else its gonna be a bit confusing
    last_error = 0
    integral = 0
    while True:    #if other light sensor senses a different colour (eg. when the robot reaches the desired location)
        light = color_sensor.reflection(port.E)    #light intensity reading
        error = target - light    #difference of target light intensity and current
        proportional = error * kp            #kp: increase if robot corrects too little / undershoots line, vice versa
        integral += error * ki                #ki: usually keep to 0 as it affects robot stability in the long term
        derivative = (error - last_error) * kd#kd: increase to help stabilise the robot, decrease if robot very jerky (small jerks)
        sum = round(proportional + integral + derivative)                #movement
        motor_pair.move_tank(motor_pair.PAIR_1, speed + sum, speed - sum) #robot should automatically correct
        print(error, speed + sum, speed - sum)
        last_error = error #updating last error

def ptp(start, end, speed):
    # moves robot from one coordinate to another (assuming map is turned into a grid)
    # parameters:(start point(list), end point(list), speed)
    x_diff = end[0] - start[0]
    y_diff = end[1] - start[1]
    degree = math.atan(y_diff / x_diff) * (180 / math.pi)
    print(degree)
    if x_diff > 0:
        turndeg = 90 + degree
    else:
        turndeg = 270 + degree
    ots_turn(get_angle() - 180 + turndeg, speed)
    distance = round(((x_diff ** 2 + y_diff ** 2) ** 0.5) * 5)
    print("distance", distance)

    # selectively pid
    if start in line and end in line: PID(distance, -2, 0, 0.5, speed)
    else: gyro_straight(distance, 1, speed)    # multiplier to be tuned


#attachment functions


def open_pincer():
    motor.run_for_time(port.C, 1000, 200)
    time.sleep(1.5)
    motor.stop(port.C)

def close_pincer():
    motor.run_for_time(port.C, 1000, -200)
    time.sleep(1.5)
    motor.stop(port.C)

def open_side():
    motor.run_for_time(port.D, 1000, -200)
    time.sleep(1.5)
    motor.stop(port.D)

def close_side():
    motor.run_for_time(port.D, 1000, 200)
    time.sleep(1.5)
    motor.stop(port.D)

def grab_turbine():
    motor.run_for_time(port.C, 1000, 200)
    time.sleep(1.5)
    motor.stop(port.C)
    motor_pair.move_tank(motor_pair.PAIR_1, 50, 50)
    time.sleep(2)
    motor_pair.stop(motor_pair.PAIR_1)
    motor.run_for_time(port.C, 1000, -200)
    time.sleep(1.5)
    motor_pair.move_tank(motor_pair.PAIR_1, -100, -100)
    time.sleep(1)
    motor_pair.stop(motor_pair.PAIR_1)

def dispense_turbine():
    motor.run_for_time(port.C, 1000, 200)
    time.sleep(1.5)
    motor.stop(port.C)
    motor_pair.move_tank(motor_pair.PAIR_1, -100, -100)
    time.sleep(1)
    motor_pair.stop(motor_pair.PAIR_1)

def grab_tray():
    # temporary grab tray
    motor.run_for_time(port.D, 1000, -200)
    time.sleep(1.5)
    motor.stop(port.D)
    motor_pair.move_tank(motor_pair.PAIR_1, 100, 100)
    time.sleep(1)
    motor_pair.stop(motor_pair.PAIR_1)

    motor.run_for_time(port.D, 1000, 200)
    time.sleep(1.5)
    motor.stop(port.D)
    motor_pair.move_tank(motor_pair.PAIR_1, 100, 100)
    time.sleep(1)
    motor_pair.stop(motor_pair.PAIR_1)
    motor.run_for_time(port.D, 1000,-200)
    time.sleep(1.5)
    motor.stop(port.D)

    motor_pair.move_tank(motor_pair.PAIR_1, 100, 100)
    time.sleep(1)
    motor_pair.stop(motor_pair.PAIR_1)

    motor.run_for_time(port.D, 1000, 200)
    time.sleep(1.5)
    motor.stop(port.D)

    motor_pair.move_tank(motor_pair.PAIR_1, 100, 100)
    time.sleep(2)
    motor_pair.stop(motor_pair.PAIR_1)

    motor.run_for_time(port.D, 1000, -200)
    time.sleep(1.5)
    motor.stop(port.D)


#main
def main():
    print("start")
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)#defining our large/driving motors
        #set yaw angle to 180 (since robot is facing downwards when starting)
    motor.reset_relative_position(port.A, 0)
    motor.reset_relative_position(port.B, 0)
    motion_sensor.reset_yaw(0)
    runloop.sleep_ms(1000)

    PID(70, -2, 0, 0.5, 100)    #-3, 0, 0.1, 100
    # PID_l(78, 0, 0, 3, 100)      #2, 0, 1.5, 400
    # ots_turn(185, 200)
    # ptp((-5, 16), (-20, 16), 200)
    astar_pid((3, 24), (7, 20), 200)


main()