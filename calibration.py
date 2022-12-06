from tdmclient import aw
import time
import numpy as np
import Motion_control as mc


# REFORMULER POUR AVOIR UNE CLASSE ?
def calibration_mm(node, Ts, thymio_data, LINE_LENGTH, TRANSITION_THRESHOLD, q_nu, speed_conversion, client, SPEED):
    timerStarted = False
    iter = 0
    startIter = 0
    onLine = True
    start = 0
    end = 0

    mc.motors(node, SPEED+1, SPEED)
    while(onLine == True):
        aw(node.wait_for_variables())
        get_data(thymio_data, node)
        aw(client.sleep(Ts))
        #tenter avec un average des 2 capteurs
        # avg_gnd = np.mean(thymio_data[iter]["ground"])
        # print(avg_gnd)
        if(thymio_data[iter]["ground"][0] < TRANSITION_THRESHOLD):
            if(timerStarted == False):
                timerStarted = True
                start = time.time()
                startIter = iter
        elif(timerStarted == False):
            pass
        else:
            end = time.time()
            mc.motors(node,0,0)
            aw(client.sleep(Ts))
            onLine = False
            q_nu, speed_conversion = compute_data(LINE_LENGTH, start, end, startIter, thymio_data, SPEED)
        iter = iter + 1

    return q_nu, speed_conversion


def get_data(thymio_data, node):
    thymio_data.append({"ground":list(node["prox.ground.reflected"]),
                    "left_speed":node["motor.left.speed"],
                    "right_speed":node["motor.right.speed"]})

def compute_data(LINE_LENGTH, start, end, startIter, thymio_data, SPEED):
    duration = end - start
    speed_mms = LINE_LENGTH / duration
    speed_conversion = speed_mms / SPEED
    avg_speed = [(x["left_speed"]+x["right_speed"])/2/speed_conversion for x in thymio_data]
    q_nu = np.std(avg_speed[startIter:])/2
    print(f"The conversion factor for the speed of the thymio to mm/sh is : {speed_conversion} ")
    print(f"With a desired speed of : {SPEED}, the thymio speed is : {speed_mms} mm/s")
    print(f"The standard deviation from the speed state (q_nu) and speed measurement (r_nu) is : {q_nu} ")
    return q_nu, speed_conversion