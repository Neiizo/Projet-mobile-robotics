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
        self.q_p_cam = 0.25
        self.r_p_cam = 0.

    def calibration_mm(self, mc):
        if (self.speed_conversion*self.r_nu) == 0:
            if(self.speed_conversion != 0):
                print("########################")
                print("r_nu and q_nu will be computed")
                print("#\n#\n#\n#\n#")#ajouter des retour à la lignes
            elif(self.r_nu != 0):
                print("########################")
                print("speed_conversion will be computed")
                print("#\n#\n#\n#\n#")
            else:
                print("########################")
                print("speed_conversion, r_nu and q_nu will be computed")
                print("#\n#\n#\n#\n#")
            timerStarted = False
            iter = 0
            startIter = 0
            onLine = True
            start = 0
            end = 0
            thymio_data = []

            mc.motors(self.speed_x, self.speed_y)
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
                    mc.motors(0,0)
                    aw(self.client.sleep(self.dt))
                    onLine = False
                    self.compute_data(start, end, startIter, thymio_data)
                    print("YOU ARE IN A 'CALIBRATION' MODE. IF YOU WISH TO STOP THE CALIBRATION SEQUENCE, CHANGE THE CELL ABOVE WITH THIS CODE\n")
                    print(f"cal_data = data(Ts, SPEED_L, SPEED_R, GND_THRESHOLD, client, node, {self.speed_conversion}, {self.r_nu})")
                    print("cal_data.calibration_mm(mc)")
                    print("########################")
                iter = iter + 1
        else:
            self.speed_mms =self.speed_conversion * self.speed_y
            self.show_data()
            mc.speed_conversion = self.speed_conversion
            print("YOU ARE IN A 'NO CALIBRATION' MODE. IF YOU WISH TO RUN CALIBRATION SEQUENCE, CHANGE THE CELL ABOVE WITH THIS CODE\n")
            print(f"cal_data = data(Ts, SPEED_L, SPEED_R, GND_THRESHOLD, client, node)")
            print("cal_data.calibration_mm(mc)")
            print("########################")
        self.calibrated = 0 


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
        print(f"The conversion factor for the speed of the thymio to mm/sh is : {self.speed_conversion} ")
        print(f"With a desired speed of : {self.speed_y}, the thymio speed is : {self.speed_mms} mm/s")
        print(f"The standard deviation from the speed state (q_nu) and speed measurement (r_nu) is : {self.q_nu} ")
        print("########################")

    def cam_calibration(self, calibrate, nbAruco, test_threshold = 150):
        if (self.calibrated != 1):
            self.r_nu = 3.1773380067632053
            self.speed_conversion = 0.3210733473116764
            self.q_nu = self.r_nu
        cam_calibrated = False
        valueFound = False
        # gridDone = False 
        # threshold_grid = 170
        # while(gridDone != True):
        #     vision = av.Vision(threshold_grid)
        #     count  = np.count_nonzero(vision.grid)
        #     if(count < nbObstacles):
        #         if(threshold_grid > 250):
        #             cam_calibrated = True
        #             print("SATURATION")
        #         threshold_grid = threshold_grid + 10
        #         if(valueFound == True):
        #             cam_calibrated = True
        #             print("threshold_grid set to : ", threshold_grid)
        #     else:
        #         valueFound = True
        #         test_threshold = test_threshold - 10
                
        if(calibrate  == True):
            while(cam_calibrated != True):
                vision = av.Vision(test_threshold)
                corners, ids = vision.aruco()
                print(len(ids))
                print("test_threshold = ", test_threshold)
                if(ids is None):
                    cam_calibrated = True
                    valueFound = True
                    test_threshold = test_threshold + 10
                elif(len(ids) < nbAruco):
                    if(test_threshold > 250):
                        cam_calibrated = True
                        print("SATURATION")
                    test_threshold = test_threshold + 10
                    if(valueFound == True):
                        cam_calibrated = True
                        print("Threshold set to : ", test_threshold)
                else:
                    valueFound = True
                    test_threshold = test_threshold - 10
        else:
            test_threshold = 150
            vision = av.Vision(test_threshold)

        
        Q_cam = np.array([self.q_p_cam, self.q_nu])
        R_cam = np.array([self.r_p_cam, self.r_nu])
        r_p_gnd = 0.25 
        q_p_gnd = 0.04

        Q_gnd = np.array([q_p_gnd, self.q_nu])
        R_gnd = np.array([r_p_gnd, self.r_nu])
        return vision, Q_cam, R_cam, Q_gnd, R_gnd   