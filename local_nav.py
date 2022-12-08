import Motion_control as mc
import numpy as np
import time as time

TURN_90 = -1
TURN_M90 = 1

OBST_AVOID = 1
OBST_HORIZONTAL_MOVE = 2
OBST_VERTICAL_MOVE = 3
OBST_TO_PATH = 4

# Detect Obstacle using the left, middle and right proximity sensors in the front of the Thymio
def obstacle_detect():

    obstThrL = 10      # low obstacle threshold to switch state 1->0
    obstThrH = 20      # high obstacle threshold to switch state 0->1

    state = 1          # 0=shortest path, 1=obstacle avoidance
    obst = [0,0,0]     # measurements from left, middle and right prox sensors

    if state == 0:
        if (obst[0] > obstThrH):
            state = 1
        elif (obst[1] > obstThrH):
            state = 1
        elif (obst[2] > obstThrH):
            state = 1
    elif state == 1:
        if obst[0] < obstThrL:
            if obst[1] < obstThrL:
                if obst[2] < obstThrL:
                    state = 0
    return state

#Avoid obstacles by doing a half circle to the right, triggered when obstacle_detect() returns 1
def obstacle_avoid_simple(x,y,orientation, index, shortest_path):

    obst_avoid_state = 1
    complete = 0
    going_left = 0

    #rotate 90 deg relative
    turn(TURN_90)
    if obst_avoid_state == OBST_AVOID_STEP1:
        if  not obstacle_detect(): # STEP 1
            #TODO move forward 1 cell
            #rotate -90 deg relative
            if going_left:
                turn(TURN_90)
            else:
                turn(TURN_M90)
            obst_avoid_state = OBST_AVOID_STEP2
        else:
            # add shift to x and y
            turn(TURN_90)
            turn(TURN_90)
            going_left = 1
    
    elif obst_avoid_state == OBST_AVOID_STEP2:
        if  not obstacle_detect(): # STEP 2
            #TODO move forward 1 cell
            obst_avoid_state = OBST_AVOID_STEP3
        else:
            if going_left:
                turn(TURN_M90)
            else:
                turn(TURN_90)
            coordinate_shift(orientation, 1)

    elif obst_avoid_state == OBST_AVOID_STEP3:
        if  not obstacle_detect(): # STEP 3
            #TODO move forward 1 cell
            #rotate -90 deg relative
            if going_left:
                turn(TURN_90)
            else:
                turn(TURN_M90)
            obst_avoid_state = OBST_AVOID_STEP4
        else:
            # add shift to x and y
            turn(TURN_M90)
            coordinate_shift(orientation, 1)

    elif obst_avoid_state == OBST_AVOID_STEP4:
        if  not obstacle_detect(): # STEP 4
            #TODO move forward 1 cell
            #rotate 90 deg relative
            turn(TURN_90)
            complete = 1
        else:
            # add shift to x and y
            coordinate_shift(orientation, 1)
    
    coordinate_shift(orientation, 2)

    return complete,x,y
    
def turn(angle):
    turn = mc.get_turn_rel(angle,orientation)
    orientation = orientation + turn
    turn_speed = int(100*np.sign(turn))
    mc.motors(-turn_speed, turn_speed, verbose=True)
    time.sleep(5*abs(turn)//2)
    return

def coordinate_increment(orientation, value):
    if orientation == 0:
        x = x+value
    elif orientation == 1:
        y = y-value
    elif orientation == 2:
        x = x-value
    elif orientation == 3:
        y = y+value

def obstacle_avoid(x,y,orientation, index, shortest_path):

    obst_avoid_state = 1
    complete = 0
    going_left = 0

    #TODO Find closer to shortest path
    [x_obj,y_obj] = shortest_path[index+2]

    if orientation = 0:
        if y_obj > y:
            going_left = 0
        elif y_obj < y:
            going_left = 1
    elif orientation = 1:
        if x_obj > x:
            going_left = 0
        elif x_obj < x:
            going_left = 1
    elif orientation = 2:
        if y_obj > y:
            going_left = 1
        elif y_obj < y:
            going_left = 0
    elif orientation = 3:
        if x_obj > x:
            going_left = 1
        elif x_obj < x:
            going_left = 0

    if going_left:
        turn(TURN_M90)
    else:
        turn(TURN_90)
    if obst_avoid_state == OBST_AVOID:
        if  not obstacle_detect():
            #TODO move forward 1 cell
            coordinate_increment(orientation, 1)
            #rotate -90 deg relative
            if going_left:
                turn(TURN_90)
            else:
                turn(TURN_M90)
            obst_avoid_state = OBST_VERTICAL_MOVE
        else:
            # add shift to x and y
            turn(TURN_90)
            turn(TURN_90)
            going_left = 1

    elif obst_avoid_state == OBST_VERTICAL_MOVE:
        if  not obstacle_detect():
            #TODO move forward 1 cell
            coordinate_increment(orientation, 1)
            #rotate -90 deg relative
            obst_avoid_state = OBST_TO_PATH
        else:
            if going_left:
                turn(TURN_M90)
            else:
                turn(TURN_90)
            obst_avoid_state = OBST_HORIZONTAL_MOVE
            # add shift to x and y

    elif obst_avoid_state == OBST_HORIZONTAL_MOVE:
        if  not obstacle_detect():
            #TODO move forward 1 cell
            coordinate_increment(orientation, 1)
            #rotate -90 deg relative
            if going_left:
                turn(TURN_90)
            else:
                turn(TURN_M90)
            obst_avoid_state = OBST_VERTICAL_MOVE
        else:
            going_left = 1
            # add shift to x and y

    elif obst_avoid_state == OBST_TO_PATH:

        if  not obstacle_detect():
                #TODO move forward 1 cell
                coordinate_increment(orientation, 1)
                #rotate -90 deg relative
                obst_avoid_state = OBST_TO_PATH
        else:
            if going_left:
                turn(TURN_90)
            else:
                turn(TURN_M90)
            obst_avoid_state = OBST_HORIZONTAL_MOVE
            # add shift to x and y
return x,y,orientation,complete

# TODO Ajouter fonction move 1 case

# TODO Ajouter fonction tourne relatif

# TODO check out of bound

# TODO check if avoidance is on path -> if yes: release

