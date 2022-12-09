import Motion_control as mc
import numpy as np
import time as time
from tdmclient import aw

TURN_90 = -1
TURN_M90 = 1

OBST_AVOID = 1
OBST_HORIZONTAL_MOVE = 2
OBST_VERTICAL_MOVE = 3
OBST_TO_PATH = 4

# Detect Obstacle using the left, middle and right proximity sensors in the front of the Thymio
def obstacle_detect(node):

    obstThrL = 10      # low obstacle threshold to switch state 1->0
    obstThrH = 3000      # high obstacle threshold to switch state 0->1

    state = 0          # 0=shortest path, 1=obstacle avoidance
    obst = [0,0,0]     # measurements from left, middle and right prox sensors
    aw(node.wait_for_variables({"prox.horizontal"}))
    #print(list(node.v.prox.horizontal))
    obst = [node.v.prox.horizontal[1],node.v.prox.horizontal[2],node.v.prox.horizontal[3]]
    if state == 0:
        if (obst[0] > obstThrH) or (obst[1] > obstThrH) or (obst[2] > obstThrH) :
            state = 1
    return state



#Avoid obstacles by doing a half circle to the right, triggered when obstacle_detect() returns 1
def obstacle_avoid(x,y,orientation, obj_x,obj_y, speed, step_duration, speed_conversion, node, client):
    obstThrL = 20 
    obst_avoid_state = 1
    complete = False
    going_left = False
    Ts = 0.01
    x_robot = x
    y_robot = y
    o_robot = orientation
    print("obj x,y",obj_x,obj_y)
    print("act x,y",x,y,orientation)
    #rotate 90 deg relative
    o_robot = turn(o_robot,x_robot,y_robot,speed[0],speed_conversion, node, client,TURN_90)
    obst_avoid_state = OBST_HORIZONTAL_MOVE
    while not complete:
        if obst_avoid_state == OBST_HORIZONTAL_MOVE:
            if  not obstacle_detect(node): # STEP 1
                time_spent,obst_forw = move_forward(node, client, speed, Ts, obstThrL, going_left)
                distance = time_spent/step_duration
                if obst_forw:
                    move_adjust(node, client, speed, Ts, obst_forw, (distance%1)*step_duration)
                    x_robot,y_robot = coordinate_increment(o_robot, np.floor(distance),x_robot,y_robot)
                    print("x,y",x_robot,y_robot, o_robot)
                    o_robot = turn(o_robot,x_robot,y_robot,speed[0],speed_conversion, node, client,TURN_90)
                else:
                    print(distance)
                    move_adjust(node, client, speed, Ts, obst_forw, (1-distance%1)*step_duration)
                    x_robot,y_robot = coordinate_increment(o_robot, np.floor(1+distance),x_robot,y_robot)
                    print("x,y",x_robot,y_robot, o_robot)
                    obst_avoid_state = OBST_VERTICAL_MOVE
                    #complete = 1
            else:
                # add shift to x and y
                if not going_left:
                    o_robot = turn(o_robot,x_robot,y_robot,speed[0],speed_conversion, node, client,TURN_M90)
                    o_robot = turn(o_robot,x_robot,y_robot,speed[0],speed_conversion, node, client,TURN_M90)
                    going_left = True
                else: 
                    o_robot = turn(o_robot,x_robot,y_robot,speed[0],speed_conversion, node, client,TURN_M90)
                    going_left = False
        
        elif obst_avoid_state == OBST_VERTICAL_MOVE:
            o_robot = turn(o_robot,x_robot,y_robot,speed[0],speed_conversion, node, client,TURN_M90)
            going_left = False
            time_spent,obst_forw = move_forward(node, client, speed, Ts, obstThrL, going_left)
            distance = time_spent/step_duration
            if obst_forw:
                move_adjust(node, client, speed, Ts, obst_forw, (distance%1)*step_duration)
                x_robot,y_robot = coordinate_increment(o_robot, np.floor(distance),x_robot,y_robot)
                print("x,y",x_robot,y_robot, o_robot)
                o_robot = turn(o_robot,x_robot,y_robot,speed[0],speed_conversion, node, client,TURN_90)
                obst_avoid_state = OBST_HORIZONTAL_MOVE
                if (x_robot == obj_x) and (y_robot == obj_y):
                    complete = 1
            else:
                print("vertical",distance)
                move_adjust(node, client, speed, Ts, obst_forw, (1-distance%1)*step_duration)
                x_robot,y_robot = coordinate_increment(o_robot, np.floor(1+distance),x_robot,y_robot)
                print("x,y",x_robot,y_robot, o_robot)
                obst_avoid_state = OBST_TO_PATH
                if (x_robot == obj_x) and (y_robot == obj_y):
                    complete = 1                

        elif obst_avoid_state == OBST_TO_PATH:
            obst_forward = False
            counter_stuck = 0
            o_robot = turn(o_robot,x_robot,y_robot,speed[0],speed_conversion, node, client,TURN_M90)
            if not obstacle_detect(node):
                move_adjust(node, client, speed, Ts, obst_forw, step_duration)
                x_robot,y_robot = coordinate_increment(o_robot, 1,x_robot,y_robot)
                print("x,y",x_robot,y_robot, o_robot)
            while (x_robot!=obj_x) or (y_robot!=obj_y):
                counter_stuck += 1
                if x_robot>obj_x:
                    o_robot = turn(o_robot,-1,0,speed[0],speed_conversion, node, client)
                    if not obstacle_detect(node):
                        move_adjust(node, client, speed, Ts, obst_forw, step_duration)
                        x_robot,y_robot = coordinate_increment(o_robot, 1,x_robot,y_robot)
                        print("x,y",x_robot,y_robot, o_robot)
                        counter_stuck = 0
                if x_robot<obj_x:
                    o_robot = turn(o_robot,1,0,speed[0],speed_conversion, node, client)
                    if not obstacle_detect(node):
                        move_adjust(node, client, speed, Ts, obst_forw, step_duration)
                        x_robot,y_robot= coordinate_increment(o_robot, 1,x_robot,y_robot)
                        print("x,y",x_robot,y_robot, o_robot)
                        counter_stuck = 0
                if y_robot>obj_y:
                    o_robot = turn(o_robot,0,-1,speed[0],speed_conversion, node, client)
                    if not obstacle_detect(node):
                        move_adjust(node, client, speed, Ts, obst_forw, step_duration)
                        x_robot,y_robot = coordinate_increment(o_robot, 1,x_robot,y_robot)
                        print("x,y",x_robot,y_robot, o_robot)
                        counter_stuck = 0
                if y_robot<obj_y:
                    o_robot = turn(o_robot,0,1,speed[0],speed_conversion, node, client)
                    if not obstacle_detect(node):
                        move_adjust(node, client, speed, Ts, obst_forw, step_duration)
                        x_robot,y_robot = coordinate_increment(o_robot, 1,x_robot,y_robot)
                        print("x,y",x_robot,y_robot, o_robot)
                        counter_stuck = 0
                if counter_stuck > 5:
                    o_robot = turn(o_robot,x_robot,y_robot,speed[0],speed_conversion, node, client,TURN_90)
            complete = 1

    return complete,x_robot,y_robot,o_robot
    
def turn(o_robot,x,y,SPEED_X,speed_conversion, node, client,turn = 10):
    if turn == 10:
        turn = mc.get_turn(x,y,o_robot)
    o_robot_new = (o_robot + turn)%4
    if o_robot_new<0:
        o_robot_new = o_robot_new+4
    for i in range(abs(turn)):
        mc.robot_turn(np.sign(turn),SPEED_X,speed_conversion, node, client)
    return o_robot_new

def move_adjust(node, client, speed, Ts, obst_forw, step_duration):
    start_move = time.time()
    step_done = False
    while (step_done == False):
        current = time.time()
        if((current - start_move) > step_duration):
            step_done = True 
        else:   
            if obst_forw:
                speed_x = int(-speed[0])
                speed_y = int(-speed[1]) 
            else :
                speed_x = int(speed[0])
                speed_y = int(speed[1]) 
            mc.motors(node, speed_x, speed_y)
            aw(client.sleep(Ts))
    mc.motors(node,0, 0)

def move_forward(node, client, speed, Ts, obstThrL,going_left):
    start_move = time.time()
    obstacle_front = False
    aw(node.wait_for_variables({"prox.horizontal"}))
    if going_left:
        i = 4
    else:
        i = 0
    prox_sensor = node.v.prox.horizontal[i]
    while(prox_sensor > obstThrL):
        aw(node.wait_for_variables({"prox.horizontal"}))
        if node.v.prox.horizontal[2] > obstThrL:
            obstacle_front = True
            break
        prox_sensor = node.v.prox.horizontal[i]
        speed[0] = int(speed[0]) #je reutilise juste les variables
        speed[1] = int(speed[1]) 
        mc.motors(node, speed[0], speed[1])
        aw(client.sleep(Ts))
    time_spent = time.time() - start_move   
    mc.motors(node,0, 0)
    return time_spent,obstacle_front

def coordinate_increment(o_robot, value,x,y):
    x_new = x
    y_new = y
    if o_robot == 0:
        x_new = x+value
    elif o_robot == 1:
        y_new = y-value
    elif o_robot == 2:
        x_new = x-value
    elif o_robot == 3:
        y_new = y+value
    return x_new,y_new
