

from __future__ import division

import sys, getopt
sys.path.append('.')
import RTIMU
import os.path
import time
from smbus import SMBus
import Adafruit_PCA9685
import subprocess
import math
from math import pi
import psutil
import quadprog
import time
#from .quadrotor_qp.quadprog_solve import qp_q_dot_des
#import os
from numpy import array

# Global Variables
bus = SMBus(1)

p = psutil.Process(os.getpid())

p.nice(-19)
p.cpu_affinity([2])


if __name__ == '__main__':

    SETTINGS_FILE = "RTIMULib"

    print("Using settings file " + SETTINGS_FILE + ".ini")
    if not os.path.exists(SETTINGS_FILE + ".ini"):
        print("Settings file does not exist, will be created")

    s = RTIMU.Settings(SETTINGS_FILE)
    imu = RTIMU.RTIMU(s)

    print("IMU Name: " + imu.IMUName())

    if (not imu.IMUInit()):
        print("IMU Init Failed")
        sys.exit(1)
    else:
        print("IMU Init Succeeded")

    # this is a good time to set any fusion parameters

    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)

    poll_interval = imu.IMUGetPollInterval()
    print("Recommended Poll Interval: %dmS\n" % poll_interval)
    f = open('Black_dog_dIMUdt.txt','w')
    f.close()
    while 1:
        if imu.IMURead():
            data = imu.getIMUData()
            fusionPose = data["fusionPose"]
	    fusionVel = data["gyro"]
	    Acc = data["accel"]
	    print [fusionPose[0]*180/3.14, fusionPose[1]*180/3.14, fusionPose[2]*180/3.14]
            dIMUdt = [fusionVel[0]*180/3.14, fusionVel[1]*180/3.14, fusionVel[2]*180/3.14]
#	    print [Acc[0], Acc[1], Acc[2]]
	    f = open('Black_dog_dIMUdt.txt','a')
    	    f.write(str(dIMUdt[0]) + ',' +str(dIMUdt[1]) +',' + str(dIMUdt[2]) +'\n')
            f.close()
