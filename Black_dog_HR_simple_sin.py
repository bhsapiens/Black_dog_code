from __future__ import division
import time
from smbus import SMBus
import Adafruit_PCA9685
import subprocess
import math
from SherControl_Scripts_global import Inverse_Kinamatics as IK
from SherControl_Scripts_global import Jacobian_Inv as JInv
import numpy as np
PI = math.pi

#Global Variables

pwm2 = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
pwm2.set_pwm_freq(50)
bus = SMBus(1)

pwm_hip = 330
pwm_knee = 400

flag = 1

ts = 10
filename = 'Black_dog_smooth_motion_sinusoidial_st_'+str(ts) +'_ms.txt'
fid = open(filename, 'w')
fid.close()

l_count = 0
while 1:
    l_count += 1
    begin = time.time()
    '''if pwm_knee < 350:
	flag = 1
    elif pwm_knee > 450:
         flag =2

    if flag == 1:
    	pwm_hip += 0
    	pwm_knee += 1
    else:
         pwm_hip -= 0
	 pwm_knee -= 1'''
    pwm_knee = 400 + int(90 * math.sin(2*PI * l_count*ts/1000))
    if pwm_hip > 330:
	A_h = 60
    else:
        A_h = 30
    pwm_hip =  330 - int(A_h * math.sin(2*PI * l_count*ts/1000))
    # ------------------------ PWM update ------------------------------------
    motor = [0, 0, 244 , 0, 0, 244 , pwm_hip, pwm_knee, 244 , pwm_knee , pwm_hip , 244]

    #pwm2.set_pwm(12, 0, 495)
    #pwm2.set_pwm(14, 0, motor[10])
    pwm2.set_pwm(15, 0, motor[9])
    end = time.time()
    dT = (end- begin)
    print dT

    fid = open(filename,'a')
    fid.write(str(begin)+' , ' + str(pwm_knee) + ' , ' + str(pwm_hip) + '\n')
    fid.close()
