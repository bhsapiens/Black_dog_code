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

# Encoder
encoder = subprocess.check_output("/home/pi/encoder/encoder_driver_v1");
encoder = [int(i) for i in encoder.split()]


f = 1
# frequency in Hertz
w = 2*PI*f
theta0 = 0
e_HRH0 = 0
ie_HRH0 = 0
e_HRK0 = 0
ie_HRK0 = 0

kp = 0
kd = 0.0
ki = 0

# Integration time
dt = 0.01

# scaling of data and rotation parameter setting

file_name = "xydxdy_smooth_sample_centi_hz_" + str(f*100)+".txt"
text_file = open(file_name, "w")
text_file.close()

counter = 0
time_stamp = time.time()
print time_stamp

while 1:
    counter += 1
    begin_time =  time.time()
    theta = (w * dt + theta0) % (2*PI)

    encoder = subprocess.check_output("/home/pi/encoder/encoder_driver_v1");
    encoder = [int(i) for i in encoder.split()]
    flag = 0

    # Joint_name < Actual_angle_1 Actual_angle_2 | motor_pwm_1 motor_pwm_2 | encoder_value_1 encoder_value_2 >
    e_HRH = (668 - encoder[11])* PI/(2*(668 - 405)) # HRH < 0 90 | 130 300 | 668 405 >
    e_HRK = (encoder[10] - 363)* PI/(2*(615 - 363))    # HRK < 0 90 | 290 445 | 363 615>

    q_IK = [-PI/4 ,PI/4 + PI/6 * math.sin(theta)] #IK(Lh, Lk, r)
    dq_IK = [0, w * PI/6 * math.cos(theta)] # np.dot(invJ, dr)

    # Data format transform
    q_H = PI/2 + q_IK[0] # manual input("angles in degree:  ") * PI / 180
    q_K = PI/2 - q_IK[1]
    dq_H = dq_IK[0]
    dq_K = - dq_IK[1]

    # -------------- PID controller
    # de_HRH = (-e_HRH - 3 * e_HRH0 +4 * e_HRH1)/(2 * dt)
    de_HRH = (e_HRH - e_HRH0)/dt
    ie_HRH = (q_H -e_HRH)*dt + ie_HRH0

    # de_HRK = (e_HRK - e_HRK0)/dt
    de_HRK = (e_HRK - e_HRK0)/(dt)
    ie_HRK = (q_K -e_HRK)*dt + ie_HRK0

    # Generate target pwm
    pwm_knee_float = 290 + 2 * (q_K + ki * ie_HRK + kp *(q_K -e_HRK) + kd *(dq_K - de_HRK)) * (445 - 290) / PI
    pwm_hip = int(215 + 2 * (q_H + ki * ie_HRH + kp *(q_H -e_HRH) + kd *(dq_H - de_HRH)) * (300 - 135) / PI)
    pwm_knee = int(290 + 2 * (q_K + ki * ie_HRK + kp *(q_K -e_HRK) + kd *(dq_K - de_HRK)) * (445 - 290) / PI)
    #print [q_H, q_K],
    #print [e_HRH, e_HRK]
    print [dq_H, dq_K],
    print [de_HRH, de_HRK],
    #print de_HRH
    #print (q_H -e_HRH),
    print (dq_K - de_HRK)
    #print [pwm_hip, pwm_knee]

    # ------------------------ PWM update ------------------------------------
    motor = [0, 0, 244 , 0, 0, 244 , pwm_hip, pwm_knee, 244 , pwm_knee , pwm_hip , 244]

   # print " -------------------------------------------"
   # print [e_HRH, e_HRK, q_H, q_K]
   # print "encoder: ",
   # print "motor:   ",
   # print motor
   # print "Angles:  ",
   # print Angles
   # print "Angles software reference:  ",
   # print Angles_code,
   # print flag

    #          HRK   HRH   HRA   HLK   HLH   HLA   FRK   FRH   FRA   FLK   FLH   FLA
    # motor = [342 , 342 , 244 , 342 , 342 , 244 , 244 , 244 , 244 , 344 , 344 , 244]
    #          FRK   FRH   FRA   FLK   FLH   FLA   HLK   HLH   HLA   HRK   HRH   HRA

    # Hind Right Leg
    #pwm2.set_pwm(12, 0, 495)
    pwm2.set_pwm(14, 0, motor[10])
    pwm2.set_pwm(15, 0, motor[9])
    theta0 = theta % (2*PI)
    e_HRH0 = e_HRH
    ie_HRH0 = ie_HRH
    e_HRK0 = e_HRK
    ie_HRK0 = ie_HRK

    text_file = open(file_name, "a")
    text_file.write(str(begin_time)+ ',' +str(theta) + ',' + str(q_K)+','+str(dq_K)+','+str(motor[9]) + ',' + str(motor[10]) + ',' + str(de_HRK) + ',' + str(e_HRK) + ' , ' + str(pwm_knee_float) + '\n')
    text_file.close()

    end_time = time.time()
    dt = (end_time  - begin_time)


