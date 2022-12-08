from tdmclient import aw
import time
import numpy as np
import Motion_control as mc
import computer_vision as av

class data(object):
    def __init__(self, dt, speed_x, speed_y, TRANSITION_THRESHOLD, client, node, speed_conversion = 0, r_nu = 0):
        self.dt = dt
        self.node = node
        self.client = client
        self.speed_x = speed_x
        self.speed_y = speed_y
        self.gnd_threshold = TRANSITION_THRESHOLD
        self.LINE_LENGTH = 300
        self.calibrated = 0
        self.speed_mms = 0

        # will get overridden if a calibration is made, otherwise, these are some safe values  
        self.speed_conversion = speed_conversion
        self.r_nu = r_nu
        self.q_nu = self.r_nu

    def calibration_mm(self):
        if (self.speed_conversion*self.r_nu) == 0:
            if(self.speed_conversion != 0):
                print("########################")
                print("r_nu and q_nu will be computed")
                print("#\n#\n#\n#\n")#ajouter des retour à la lignes
            elif(self.r_nu != 0):
                print("########################")
                print("speed_conversion will be computed")
                print("#\n#\n#\n#\n")
            else:
                print("########################")
                print("speed_conversion, r_nu and q_nu will be computed")
                print("#\n#\n#\n#\n")
            timerStarted = False
            iter = 0
            startIter = 0
            onLine = True
            start = 0
            end = 0
            thymio_data = []

            mc.motors(self.node, self.speed_x, self.speed_y)
            while(onLine == True):
                aw(self.node.wait_for_variables())
                self.get_data(thymio_data)
                aw(self.client.sleep(self.dt))
                #tenter avec un average des 2 capteurs
                # avg_gnd = np.mean(thymio_data[iter]["ground"])
                # print(avg_gnd)
                if(thymio_data[iter]["ground"][0] < self.gnd_threshold):
                    if(timerStarted == False):
                        timerStarted = True
                        start = time.time()
                        startIter = iter
                elif(timerStarted == False):
                    pass
                else:
                    end = time.time()
                    mc.motors(self.node,0,0)
                    aw(self.client.sleep(self.dt))
                    onLine = False
                    self.compute_data(start, end, startIter, thymio_data)
                    print("IF YOU DO NOW WANT TO RUN THIS CALIBRATION RUN OVER AND OVER, YOU CAN PASTE THE FOLLOWING COMMAND INSTEAD OF THE PREVIOUS ONE")
                    print(f"cal_data.calibration_mm({self.speed_conversion}, {self.r_nu})")
                    print("########################")
                iter = iter + 1
        else:
            self.speed_mms =self.speed_conversion * self.speed_y
            self.show_data()
            print("YOU ARE IN A NO CALIBRATION MODE. IF YOU WISH TO RERUN CALIBRATION, RERUN THE FOLLOWING COMMAND IF YOU WISH TO RE RUN THE CALIBRATION TEST")
            print("cal_data.calibration_mm()")
            print("########################")
        self.calibrated = 0 # A MODIFIER PLUS TARD POUR LA DEUXIèME COMMANDE DE CALIBARTION


    def get_data(self, thymio_data):
        thymio_data.append({"ground":list(self.node["prox.ground.reflected"]),
                        "left_speed":self.node["motor.left.speed"],
                        "right_speed":self.node["motor.right.speed"]})

    def compute_data(self, start, end, startIter, thymio_data):
        duration = end - start
        self.speed_mms = self.LINE_LENGTH / duration
        self.speed_conversion = self.speed_mms / self.speed_y
        avg_speed = [(x["left_speed"]+x["right_speed"])/2/self.speed_conversion for x in thymio_data]
        self.q_nu = np.std(avg_speed[startIter:])/2
        self.r_nu = self.q_nu
        self.calibrated = 1
        self.show_data()

    def show_data(self):
        print("########################")
        print(f"The conversion factor for the speed of the thymio to mm/sh is : {self.speed_conversion} ")
        print(f"With a desired speed of : {self.speed_y}, the thymio speed is : {self.speed_mms} mm/s")
        print(f"The standard deviation from the speed state (q_nu) and speed measurement (r_nu) is : {self.q_nu} ")
        print("########################")

    def cam_calibration(self):
        if (self.calibrated != 1):
            self.r_nu = 3.1773380067632053
            self.speed_conversion = 0.3210733473116764
            self.q_nu = self.r_nu
        threshold = 200 # faire fct pour calibrer ce machin
        vision = av.Vision(threshold)
        r_p_cam = 1 
        q_p_cam = 1 
        Q_cam = np.array([self.q_nu, q_p_cam])
        R_cam = np.array([self.q_nu, r_p_cam])
        r_p_gnd = 0.25 
        q_p_gnd = 0.04

        Q_gnd = np.array([self.q_nu, q_p_gnd])
        R_gnd = np.array([self.q_nu, r_p_gnd])
        return vision, Q_cam, R_cam, Q_gnd, R_gnd