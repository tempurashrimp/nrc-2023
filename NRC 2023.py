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
import threading

# 26 Aug 2023

def get_angle():
    return motion_sensor.tilt_angles()[0] / 10

def PID_r(target, kp, ki, kd, speed, colour=None): 
    # PID line follower (smooth line follower) go watch a vid about it or else its gonna be a bit confusing

    last_error = 0
    integral = 0

    while not color_sensor.color(port.C) == colour:    
        # if other light sensor senses a different colour (eg. when the robot reaches the desired location)
        light = color_sensor.reflection(port.D)    # light intensity reading
        error = target - light    # difference of target light intensity and current

        proportional = error * kp            # kp: increase if robot corrects too little / undershoots line, vice versa
        integral += error * ki                # ki: usually keep to 0 as it affects robot stability in the long term
        derivative = (error - last_error) * kd # kd: increase to help stabilise the robot, decrease if robot very jerky (small jerks)

        sum = round(proportional + integral + derivative)                # movement
        motor_pair.move_tank(motor_pair.PAIR_1, speed + sum, speed - sum) # robot should automatically correct
        print(error, speed + sum, speed - sum)

        last_error = error # updating last error

def PID_l(target, kp, ki, kd, speed, colour=None):
     # PID line follower (smooth line follower) go watch a vid about it or else its gonna be a bit confusing

    last_error = 0
    integral = 0

    while not color_sensor.color(port.D) == colour:    # if other light sensor senses a different colour (eg. when the robot reaches the desired location)
        light = color_sensor.reflection(port.C)    # light intensity reading
        error = target - light    # difference of target light intensity and current

        proportional = error * kp            # kp: increase if robot corrects too little / undershoots line, vice versa
        integral += error * ki                # ki: usually keep to 0 as it affects robot stability in the long term
        derivative = (error - last_error) * kd # kd: increase to help stabilise the robot, decrease if robot very jerky (small jerks)

        sum = round(proportional + integral + derivative)                # movement
        motor_pair.move_tank(motor_pair.PAIR_1, speed + sum, speed - sum) # robot should automatically correct
        print(error, speed + sum, speed - sum)

        last_error = error # updating last error


def gyro_straight(distance, multiplier, speed): 
    # robot move straight from where its facing    
    # parameters:(distance in cm, magnitude of correction (should be pretty low), speed)
    motion_sensor.reset_yaw(0)
    # print(cm(distance))
    target = 0
    avg_dist = (abs(motor.relative_position(port.A)) + abs(motor.relative_position(port.B))) / 2
    # print(target)

    while avg_dist < cm(distance):
        avg_dist = (abs(motor.relative_position(port.A)) + abs(motor.relative_position(port.B))) / 2
        angle = get_angle() # similar to proportional line follower (part of PID)
        error = target - angle
        correct = error * multiplier
        # motor_pair.move_for_degrees(motor_pair.PAIR_1, 1000, correct, speed)
        motor_pair.move_tank(motor_pair.PAIR_1, round(speed - correct), round(speed + correct))
        # print(angle, error)
        # print(target, angle, error, motor.relative_position(port.A), motor.relative_position(port.B), "dist", avg_dist)

    motor_pair.stop(motor_pair.PAIR_1)


def turn_180(target, speed):
    degrees = round((target/180) * 180)
    motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, degrees, -speed, speed)


def ots_turn(degree, speed): 
    # on the spot turn    
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
            while get_angle() < target:       #turn left
                motor_pair.move_tank(motor_pair.PAIR_1, -speed, speed)
                # print(target, get_angle())
                # print("1")
            print("2")
        else:
            while get_angle() > target:    #turn left
                motor_pair.move_tank(motor_pair.PAIR_1, speed, -speed)
                # print(target, get_angle())

    else:
        if degree < 0:
            while get_angle() < target:    #turn left
                motor_pair.move_tank(motor_pair.PAIR_1, -speed, speed)
                # print(target, get_angle())
                # print("1")
            print("2")
        else:
            while get_angle() > target:    #turn left
                motor_pair.move_tank(motor_pair.PAIR_1, speed, -speed)
                # print(target, get_angle())

    motor_pair.stop(motor_pair.PAIR_1)


def ow_turn(degree, speed): 
    # one wheel turn 
    # parameters:(yaw degree(from where the robot is facing), speed)

    target = get_angle() - degree
    switch = False
    print(target)
    if target < -180: # turn left
        target += 360
        switch = True
    elif target > 180: # turn right
        target -= 360
        switch = True

    print(target)
    if switch == True:
        if degree > 0:
            while get_angle() < target:
                motor_pair.move_tank(motor_pair.PAIR_1, 0, speed)
                print(target, get_angle(), "1")
        else:
            while get_angle() > target:
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


def cm(cm):        
    # converts cm into motor degrees
    degrees = (cm / 4.5) * (180 / math.pi)    # radius to be set
    return degrees


def ptp(start, end, speed):
    # moves robot from one coordinate to another (assuming map is turned into a grid)       
    # parameters:(start point(list), end point(list), speed)
    x_diff = end[0] - start[0]
    y_diff = end[1] - start[1]
    degree = math.atan(y_diff / x_diff) * (180 / math.pi)
    print(degree)
    if x_diff > 0:
        turndeg = 90 - degree
    else:
        turndeg = -90 - degree

    ots_turn(turndeg, speed)

    distance = round(((x_diff ** 2 + y_diff ** 2) ** 0.5) * 5)
    print("distance", distance)
    gyro_straight(distance, 1, speed)    # multiplier to be tuned



def main():
    print("start")
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)# defining our large/driving motors
        # set yaw angle to 180 (since robot is facing downwards when starting)
    motor.reset_relative_position(port.A, 0)
    motor.reset_relative_position(port.B, 0)
    motion_sensor.reset_yaw(0)
    runloop.sleep_ms(1000)

    # PID_r(70, -2, 0, 0.1, 200)    #-3, 0, 0.1, 100
    # PID_l(78, 0, 0, 3, 100)      #2, 0, 1.5, 400
    # ots_turn(185, 200)
    ptp((-5, 16), (-20, 16), 200)

main()