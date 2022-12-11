# Import tdmclient Notebook environment:
import numpy as np
from tdmclient import aw
import math
import time

class MotionControl(object):
    def __init__(self, node, client, dt, SPEED_L, SPEED_R):
        self.node = node                    
        self.client = client
        self.dt = dt                        # Sampling time in [s]
        self.orientation1 = 0
        self.speed_conversion = 0           # Speed conversion from the thymio's speed to [mm/s]. This value is set by our calibration process
        self.kp = 1.5                       # Proportionnal gain, used for our orientation correction. This has been set experimentally
        self.speed = np.array([SPEED_L, SPEED_R])   # speed for our thymio's command, set in thymio's unit
        self.SPEED_AVG = (SPEED_L + SPEED_R)/2      # average of the speed, used for computation of the locomotion time
        self.turn_speed = np.array([0, 0])
        self.MARGIN = 45                    # Margin for the orientation's angle's treatment, in [°]
        self.step_duration = 0
        self.turn_duration = 0
        self.angle_threshold = 7            # Threshold set experimentally, above which we do not correct the angle



    def motors(self, left, right):
        #####################################################
        # left  : desired speed for the left motor
        # right :  desired speed for the right motor
        #####
        # Sets the desired speed for the thymio
        #####################################################
        v =  {
            "motor.left.target": [left],
            "motor.right.target": [right],
        }
        aw(self.node.set_variables(v))



    def correct_orientation(self, orientation):
        #####################################################
        # orientation :
        #####
        # 
        #####################################################
        if((orientation > 90 - self.MARGIN) & (orientation < 90 + self.MARGIN)):
            self.orientation1 = 1
            return self.orientation1 # a enlever une fois que le reste sera dans le même fichier
        elif((orientation > 180 - self.MARGIN) & (orientation < 180 + self.MARGIN)):
            self.orientation1 = 2
            return self.orientation1 # a enlever une fois que le reste sera dans le même fichier
        elif((orientation > 270 - self.MARGIN) & (orientation < 270 + self.MARGIN)):
            self.orientation1 = 3
            return self.orientation1 # a enlever une fois que le reste sera dans le même fichier
        else:
            self.orientation1 = 0
            return self.orientation1 # a enlever une fois que le reste sera dans le même fichier



    def get_turn(self, x,y, orientation): # à renommer par direction pour éviter les confusions ? ou direction désirée  
        #####################################################
        #  
        #####
        # computes the turn 
        #####################################################
        if((x==1) & (y==0)):
            dir = 0
        elif((x==0) & (y==-1)):
            dir = 1
        elif((x==-1) & (y==0)):
            dir = 2
        elif((x==0) & (y==1)):
            dir = 3
        else:
            dir = orientation
        new_orientation = dir - orientation
        if abs(new_orientation) > 2:
            return (new_orientation) - 4*np.sign(new_orientation)
        else:
            return new_orientation



    def kalman_adjust(self, next_target_x, next_target_y, kalman_pos, angle):  
        #####################################################
        # next_target_x : coordinate of the next cell the thymio needs to reach, on the x axis, in terms of cells.
        # next_target_y : coordinate of the next cell the thymio needs to reach, on the y axis, in terms of cells.
        # kalman_pos    : estimated position of the thymio from the Kalman's filter, in [mm]
        # angle         : angle of the thymio from the camera's perspective
        #####
        # computes required angle for the thymio to reach the next cells, and compares its actual angle. A proportionnal controller 
        # will then be used to correct this and redirect the thymio
        #####################################################
        delta_x = next_target_x - kalman_pos[0]
        delta_y = next_target_y - kalman_pos[1]
        desired_angle = (math.degrees(math.atan2(delta_y,  delta_x))) % 360
        adjust_turn = 0
        delta_angle = desired_angle - angle
        
        if(np.abs(delta_angle) > 270):
            delta_angle = delta_angle - np.sign(delta_angle)*360 # A VERIFIER
        #    if(delta_angle < -270):
        #       delta_angle = delta_angle + 360
        
        if(np.abs(delta_angle) > self.angle_threshold): 
            adjust_turn = (int)(np.round(self.kp * delta_angle))
        return delta_x, delta_y, adjust_turn

    def adjust_angle(self, vision):
        # vision.update_coordinates()  
        # if(vision.thymio == True): # faire que si on voit le thymio
        #     angle = vision.thymio_orientation # angle of the robot
        #     if((orientation == 0) & (angle > 270)):  
        #         adjust_angle = angle - 360
        #     else:
        #         adjust_angle = angle - orientation*90 
        #     # if(DEBUG == True):
        #     #     print("adjust angle before moving :", adjust_angle)   
        #     self.turn_speed[0] = int(self.speed[0]*np.sign(adjust_angle))
        #     self.turn_speed[1] = int(-self.speed[1]*np.sign(adjust_angle))
        #     self.motors(self.turn_speed[0], self.turn_speed[1])
        #     aw(self.client.sleep(5*abs(adjust_angle)//180)) # a changer pour un check sur time ?
        #     self..motors(0, 0)
        a =1


    def robot_turn(self, signturn):  
        #####################################################
        # signturn : sign that tells if the thymio should turn left or right. (-) for left, (+) for right A VERIFIER
        #####
        #  
        #####################################################
        turn_duration2 = 55 / (self.SPEED_AVG * self.speed_conversion)
        turn_duration1 = 20 / (self.SPEED_AVG * self.speed_conversion)
        if signturn > 0:
            self.motors(20, 100)
            aw(self.client.sleep(turn_duration2)) #changer ca pour le tour
            self.motors(-120, 50)
            aw(self.client.sleep(turn_duration2)) #changer ca pour le tour
            self.motors(-60, -60)
            aw(self.client.sleep(turn_duration1)) #changer ca pour le tour
            self.motors(0, 0)
        elif signturn < 0:
            self.motors(100, 20)
            aw(self.client.sleep(turn_duration2)) #changer ca pour le tour
            self.motors(50, -120)
            aw(self.client.sleep(turn_duration2)) #changer ca pour le tour
            self.motors(-60, -60)
            aw(self.client.sleep(turn_duration1)) #changer ca pour le tour
            self.motors(0, 0)
        else:
            self.motors(0, 0)

