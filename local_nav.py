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
    obstThrH = 1000      # high obstacle threshold to switch state 0->1

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
def obstacle_avoid(vision, mc, x,y, shortest_path, index, node, client):
    obstThrL = 20 
    obst_avoid_state = 1
    complete = False
    going_left = False
    wrong_move = False
    Ts = 0.1
    obj_x = shortest_path[0][index+1]
    obj_y = shortest_path[1][index+1]
    #shortest_path = np.transpose(shortest_path)
    shortest_path_local = shortest_path #[:index]
    print("shortest_path_local", shortest_path_local)
    x_robot = x
    y_robot = y
    print("obj x,y",obj_x,obj_y)
    print("act x,y",x,y,mc.orientation)
    #rotate 90 deg relative
    mc.orientation = turn(mc,x_robot,y_robot,TURN_90)
    mc.adjust_angle(vision)
    obst_avoid_state = OBST_HORIZONTAL_MOVE
    while not complete:
        vision.update_coordinates()
        if obst_avoid_state == OBST_HORIZONTAL_MOVE:
            if  not obstacle_detect(node): # STEP 1
                time_spent,obst_forw = move_forward(mc, node, client, Ts, obstThrL, going_left)
                distance = time_spent/mc.step_duration
                if obst_forw:
                    move_adjust(mc, client, Ts, obst_forw, (distance%1)*mc.step_duration)
                    x_robot,y_robot = coordinate_increment(mc, np.floor(distance),x_robot,y_robot)
                    print("x,y",x_robot,y_robot)
                    mc.orientation = turn(mc,x_robot,y_robot,TURN_90)
                    mc.adjust_angle(vision)
                else:
                    move_adjust(mc, client, Ts, obst_forw, (1-distance%1)*mc.step_duration)
                    x_robot,y_robot = coordinate_increment(mc,np.floor(1+distance),x_robot,y_robot)
                    print("x,y",x_robot,y_robot)
                    obst_avoid_state = OBST_VERTICAL_MOVE
                    #complete = 1
            else:
                if not going_left:
                    mc.orientation = turn(mc,x_robot,y_robot,TURN_M90)
                    mc.orientation = turn(mc,x_robot,y_robot,TURN_M90)
                    mc.adjust_angle(vision)
                    going_left = True
                # else: 
                #     mc.orientation = turn(x_robot,y_robot,speed[0],speed_conversion, node, client,TURN_M90)
                #     going_left = False
        
        elif obst_avoid_state == OBST_VERTICAL_MOVE:
            if going_left and wrong_move:
                mc.orientation = turn(mc,x_robot,y_robot,TURN_M90)
            else:
                if going_left or wrong_move:
                    mc.orientation = turn(mc,x_robot,y_robot,TURN_90)
                else:   
                    mc.orientation = turn(mc,x_robot,y_robot,TURN_M90)
            mc.adjust_angle(vision)
            wrong_move = False
            time_spent,obst_forw = move_forward(mc, node, client, Ts, obstThrL, going_left)
            distance = time_spent/mc.step_duration
            if obst_forw:
                move_adjust(mc, client, Ts, obst_forw, (distance%1)*mc.step_duration)
                x_robot,y_robot = coordinate_increment(mc,np.floor(distance),x_robot,y_robot)
                print("x,y",x_robot,y_robot)
                if going_left:
                    mc.orientation = turn(mc,x_robot,y_robot,TURN_M90)
                else:
                    mc.orientation = turn(mc,x_robot,y_robot,TURN_90)
                mc.adjust_angle(vision)
                obst_avoid_state = OBST_HORIZONTAL_MOVE
                if (x_robot == obj_x) and (y_robot == obj_y):
                    complete = 1
            else:
                move_adjust(mc, client, Ts, obst_forw, (1-distance%1)*mc.step_duration)
                x_robot,y_robot = coordinate_increment(mc, np.floor(1+distance),x_robot,y_robot)
                print("x,y",x_robot,y_robot)
                obst_avoid_state = OBST_TO_PATH
                if (x_robot,y_robot) in shortest_path_local:
                    complete = 1
                    return complete,x_robot,y_robot         

        elif obst_avoid_state == OBST_TO_PATH:
            obst_forward = False
            counter_stuck = 0
            if going_left:
                mc.orientation = turn(mc,x_robot,y_robot,TURN_90)
            else:
                mc.orientation = turn(mc, x_robot,y_robot,TURN_M90)
            mc.adjust_angle(vision)
            if obstacle_detect(node):
                obst_avoid_state = OBST_VERTICAL_MOVE
                wrong_move = True
            else:
                move_adjust(mc, client, Ts, obst_forw, mc.step_duration)
                x_robot,y_robot = coordinate_increment(mc,1, x_robot,y_robot)
                print("x,y, ORIENTATION",x_robot,y_robot, mc.orientation)
                while (x_robot!=obj_x) or (y_robot!=obj_y):
                    counter_stuck += 1
                    print("not complete")
                    print(" x robot : ", x_robot)
                    print(" y robot : ", y_robot)
                    print(" x obj : ", obj_x)
                    print(" y obj : ", obj_y)
                    if (x_robot,y_robot) in shortest_path_local:
                        complete = 1
                        return complete,x_robot,y_robot
                    if x_robot>obj_x:
                        mc.orientation = turn(mc,-1,0)
                        mc.adjust_angle(vision)
                        if not obstacle_detect(node):
                            move_adjust(mc, client, Ts, obst_forw, mc.step_duration)
                            x_robot,y_robot = coordinate_increment(mc,1,x_robot,y_robot)
                            print("x,y",x_robot,y_robot)
                            counter_stuck = 0
                    if x_robot<obj_x:
                        mc.orientation = turn(mc,1,0)
                        mc.adjust_angle(vision)
                        if not obstacle_detect(node):
                            move_adjust(mc, client, Ts, obst_forw, mc.step_duration)
                            x_robot,y_robot= coordinate_increment(mc, 1,x_robot,y_robot)
                            print("x,y",x_robot,y_robot)
                            counter_stuck = 0
                    if y_robot>obj_y:
                        mc.orientation = turn(mc,0,-1)
                        mc.adjust_angle(vision)
                        if not obstacle_detect(node):
                            move_adjust(mc, client, Ts, obst_forw, mc.step_duration)
                            x_robot,y_robot = coordinate_increment(mc,1,x_robot,y_robot)
                            print("x,y",x_robot,y_robot)
                            counter_stuck = 0
                    if y_robot<obj_y:
                        mc.orientation = turn(mc,0,1)
                        mc.adjust_angle(vision)
                        if not obstacle_detect(node):
                            move_adjust(mc, client, Ts, obst_forw, mc.step_duration)
                            x_robot,y_robot = coordinate_increment(mc,  1,x_robot,y_robot)
                            print("x,y",x_robot,y_robot)
                            counter_stuck = 0
                    # if counter_stuck > 5:
                    #     mc.orientation = turn(mc,x_robot,y_robotTURN_90)
                    #     mc.adjust_angle(vision)
                complete = 1
    return complete,x_robot,y_robot
    
def turn(mc,x,y,turn = 10):
    if turn == 10:
        turn = mc.get_turn(x,y,mc.orientation)
    orientation_new = (mc.orientation + turn)%4
    if orientation_new<0:
        orientation_new = orientation_new+4
    for i in range(abs(turn)):
        mc.robot_turn(np.sign(turn))
    return orientation_new

def move_adjust(mc, client, Ts, obst_forw, step_duration):
    start_move = time.time()
    step_done = False
    while (step_done == False):
        current = time.time()
        if((current - start_move) > step_duration):
            step_done = True 
        else:   
            if obst_forw:
                speed_l = int(-mc.speed[0])
                speed_r = int(-mc.speed[1]) 
            else :
                speed_l = int(mc.speed[0])
                speed_r = int(mc.speed[1]) 
            mc.motors(speed_l, speed_r)
            aw(client.sleep(Ts))
    mc.motors(0, 0)

def move_forward(mc, node, client, Ts, obstThrL,going_left):
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
        speed_l = int(mc.speed[0]) #je reutilise juste les variables
        speed_r = int(mc.speed[1]) 
        mc.motors(speed_l, speed_r)
        aw(client.sleep(Ts))
    time_spent = time.time() - start_move   
    mc.motors(0, 0)
    return time_spent,obstacle_front

def coordinate_increment(mc, value,x,y):
    x_new = x
    y_new = y
    if mc.orientation == 0:
        x_new = x+value
    elif mc.orientation == 1:
        y_new = y-value
    elif mc.orientation == 2:
        x_new = x-value
    elif mc.orientation == 3:
        y_new = y+value
    return x_new,y_new
