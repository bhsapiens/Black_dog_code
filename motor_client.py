from __future__ import division
from smbus import SMBus
import Adafruit_PCA9685
import subprocess
import math
from math import pi as PI
import psutil
import os
import socket
import pickle
import time
from math import sin, cos
import numpy as np

# Global Variables
pwm1 = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
pwm2 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
pwm1.set_pwm_freq(50)
pwm2.set_pwm_freq(50)
bus = SMBus(1)

p = psutil.Process(os.getpid())
p.nice(-19)
p.cpu_affinity([3])

servomin = 160
servomax = 524

i = -2
while True:
    t0 =time.clock()
    s = socket.socket()
    host = '10.156.14.246'  # ip of raspberry pi
    port = 12345
    s.connect((host, port))
    data = s.recv(4096)
    Data = pickle.loads(data)
    s.close()
    # post processing
    [AA, HH, KK] = Data
    t_s = time.clock()

    mK = np.zeros([4, 1])
    mH = np.zeros([4, 1])
    mA = np.zeros([4, 1])

    mK[0] = int(150 + (KK[0] + HH[0]) * 300 / PI)
    mK[1] = int(150 + (KK[1] + HH[1]) * 300 / PI)
    mK[2] = int(150 + (KK[2] + HH[2]) * 300 / PI)
    mK[3] = int(150 + (KK[3] + HH[3]) * 300 / PI)

    mH[0] = int(150 - (HH[0]) * 600 / PI)
    mH[1] = int(150 - (HH[1]) * 600 / PI)
    mH[2] = int(150 - (HH[2]) * 600 / PI)
    mH[3] = int(150 - (HH[3]) * 600 / PI)

    mA[0] = int(150 + (AA[0]) * 300 / PI)
    mA[1] = int(150 + (AA[1]) * 300 / PI)
    mA[2] = int(150 + (AA[2]) * 300 / PI)
    mA[3] = int(150 + (AA[3]) * 300 / PI)

    motor = [int(mK[3][0]), int(mH[3][0]), int(mA[3][0]), int(mK[2][0]), int(mH[2][0]), int(mA[2][0]), int(mK[1][0]), int(mH[1][0]), int(mA[1][0]), int(mK[0][0]), int(mH[0][0]), int(mA[0][0])]
    #        HRK    HRH    HRA    HLK    HLH    HLA    FRK    FRH    FRA    FLK    FLH    FLA
    #print motor
    # Hind Right Leg
    pwm1.set_pwm(0, 0, motor[0])
    pwm1.set_pwm(1, 0, motor[1])
    pwm1.set_pwm(2, 0, motor[2])
    # Hind Left Leg
    pwm1.set_pwm(15, 0, motor[3])
    pwm1.set_pwm(14, 0, motor[4])
    pwm1.set_pwm(13, 0, motor[5])
    # Front Right Leg
    pwm2.set_pwm(0, 0, motor[6])
    pwm2.set_pwm(1, 0, motor[7])
    pwm2.set_pwm(2, 0, motor[8])
    # Front Left Leg
    pwm2.set_pwm(15, 0, motor[9])
    pwm2.set_pwm(14, 0, motor[10])
    pwm2.set_pwm(13, 0, motor[11])
    t1 = time.clock()
    diffT = t_s - t0
    print diffT
    
