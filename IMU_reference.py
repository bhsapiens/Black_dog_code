
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
pwm1 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
pwm2 = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
pwm1.set_pwm_freq(50)
pwm2.set_pwm_freq(50)
bus = SMBus(1)

p = psutil.Process(os.getpid())

p.nice(-19)
p.cpu_affinity([2, 3])


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

    # Functions
    #pwm1 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
    #imudevice = Adafruit_PCA9685.PCA9685(address=0x68, busnum=1)
    '''pwm1.set_pwm(0, 0, 227)
    pwm1.set_pwm(1, 0, 227)
    pwm1.set_pwm(2, 0, 227)
    pwm1.set_pwm(3, 0, 227)'''
    while 1:

        before=time.clock()
        if imu.IMURead():
            # x, y, z = imu.getFusionData()
            # print("%f %f %f" % (x,y,z))
            data = imu.getIMUData()
            fusionPose = data["fusionPose"]
	    fusionVel = data["gyro"]
	    Acc = data["accel"]
            #print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]), math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
#            time.sleep(poll_interval*1.0/1000.0)

#	    print [fusionPose[0]*180/3.14, fusionPose[1]*180/3.14, fusionPose[2]*180/3.14]
            print [fusionVel[0]*180/3.14, fusionVel[1]*180/3.14, fusionVel[2]*180/3.14]
#	    print [Acc[0], Acc[1], Acc[2]]
	    IMu_clock = time.clock()
	    after=time.clock()
#            print [int((after-before)*100000) ,int(( after - IMu_clock)*100000), int((IMu_clock - before)*100000)]

