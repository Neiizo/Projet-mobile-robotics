from tdmclient import aw
import time
import numpy as np
import Motion_control as mc
import computer_vision as av

class data(object):
    def __init__(self, dt, speed_l, speed_r, TRANSITION_THRESHOLD, client, node, speed_conversion = 0, r_nu = 0):
        self.dt = dt                 # sampling time
        self.node = node
        self.client = client
        self.speed_l = speed_l       # speed of the left wheel, in thymio's coordinate
        self.speed_r = speed_r       # speed of the right wheel, in thymio's coordinate
        self.gnd_threshold = TRANSITION_THRESHOLD
        self.LINE_LENGTH = 300
        self.calibrated = 0
        self.speed_mms = 0

        # Will get overridden if a calibration is made, otherwise, these are some safe values  
        self.speed_conversion = speed_conversion
        self.r_nu = r_nu
        self.q_nu = self.r_nu
        self.q_p_cam = 0.25
        self.r_p_cam = 0.01

    def calibration_mm(self, mc):
        if (self.speed_conversion*self.r_nu) == 0:
            if(self.speed_conversion != 0):
                print("########################")
                print("r_nu and q_nu will be computed")
                print("#\n#\n#\n#\n#")
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

            mc.motors(self.speed_l, self.speed_r)
            while(onLine == True):
                aw(self.node.wait_for_variables())
                self.get_data(thymio_data)
                aw(self.client.sleep(self.dt))
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
            self.speed_mms =self.speed_conversion * self.speed_r
            self.show_data(mc)
            print("YOU ARE IN A 'NO CALIBRATION' MODE. IF YOU WISH TO RUN CALIBRATION SEQUENCE, CHANGE THE CELL ABOVE WITH THIS CODE\n")
            print(f"cal_data = data(Ts, SPEED_L, SPEED_R, GND_THRESHOLD, client, node)")
            print("cal_data.calibration_mm(mc)")
            print("########################")


    def get_data(self, thymio_data):
        thymio_data.append({"ground":list(self.node["prox.ground.reflected"]),
                        "left_speed":self.node["motor.left.speed"],
                        "right_speed":self.node["motor.right.speed"]})

    def compute_data(self, start, end, startIter, thymio_data):
        duration = end - start
        self.speed_mms = self.LINE_LENGTH / duration
        self.speed_conversion = self.speed_mms / self.speed_r
        avg_speed = [(x["left_speed"]+x["right_speed"])/2/self.speed_conversion for x in thymio_data]
        self.q_nu = np.std(avg_speed[startIter:])/2
        self.r_nu = self.q_nu
        self.show_data(mc)

    def show_data(self, mc):
        self.calibrated = 1
        speed_avg = (self.speed_r  + self.speed_l)/2
        print(f"The conversion factor for the speed of the thymio to mm/s is : {self.speed_conversion} ")
        print(f"With a desired speed of : {speed_avg}, the thymio speed is : {self.speed_mms} mm/s")
        print(f"The standard deviation from the speed state (q_nu) and speed measurement (r_nu) is : {self.q_nu} ")
        print("########################")

    def cam_calibration(self, calibrate, nbAruco, test_threshold = 100):
        if (self.calibrated != 1):
            self.r_nu = 3.1773380067632053
            self.speed_conversion = 0.3210733473116764
            self.q_nu = self.r_nu
        cam_calibrated = False
        valueFound = False
                
        if(calibrate  == True):
            while(cam_calibrated != True):
                vision = av.Vision(test_threshold)
                corners, ids = vision.aruco()
                if(ids is None):
                    test_threshold = test_threshold - 10
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
            vision = av.Vision(test_threshold)

        
        Q_cam = np.array([self.q_p_cam, self.q_nu])
        R_cam = np.array([self.r_p_cam, self.r_nu])
        return vision, Q_cam, R_cam 