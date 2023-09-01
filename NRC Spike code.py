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

# 26 Aug

#basic functions

def get_angle():
    return motion_sensor.tilt_angles()[0] / 10

def cm(cm):        #converts cm into motor degrees
    degrees = (cm / 4.5) * (180 / math.pi)    #radius to be set
    return degrees



#robot movements


def gyro_straight(distance, multiplier, speed):
    #robot move straight from where its facing
    #parameters:(distance in cm, magnitude of correction (should be pretty low), speed)
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
    gyro_straight(distance, 1, speed)    # multiplier to be tuned


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
    print("fuck you 1")
    motor_pair.move_tank(motor_pair.PAIR_1, 50, 50)
    time.sleep(2)
    motor_pair.stop(motor_pair.PAIR_1)
    print("fuck you24")
    motor.run_for_time(port.C, 1000, -200)
    time.sleep(1.5)
    print("fuck you2")
    motor_pair.move_tank(motor_pair.PAIR_1, -100, -100)
    time.sleep(1)
    motor_pair.stop(motor_pair.PAIR_1)
    print("fuck yo3u")

def dispense_turbine():
    motor.run_for_time(port.C, 1000, 200)
    time.sleep(1.5)
    motor.stop(port.C)
    motor_pair.move_tank(motor_pair.PAIR_1, -100, -100)
    time.sleep(1)
    motor_pair.stop(motor_pair.PAIR_1)

def grab_tray():
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


main()