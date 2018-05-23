from __future__ import division
import time
from smbus import SMBus
import Adafruit_PCA9685
import subprocess
import math
from SherControl_Scripts_global import Inverse_Kinamatics as IK
from SherControl_Scripts_global import Jacobian_Inv as JInv
import numpy as np
import psutil
import os

PI = math.pi
#Global Variables
pwm1 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
pwm2 = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
pwm1.set_pwm_freq(50)
pwm2.set_pwm_freq(50)
bus = SMBus(1)

p = psutil.Process(os.getpid())
p.nice(-19)
p.cpu_affinity([2])

FL_sensor = 0
FR_sensor = 0
HL_sensor = 0
HR_sensor = 0

#Functions

def adc_data_channel_1():
    global FR_sensor
    bus.write_i2c_block_data(0x48, 0x01,[0xC3,0x83])
    adc0= bus.read_i2c_block_data(0x48, 0x00,2)
    a= adc0[0] & 0xFFFF
    b= adc0[1] & 0xFF
    c= (a<<8)  | b
    FR_sensor = c

def adc_data_channel_2():
    global FL_sensor
    bus.write_i2c_block_data(0x48, 0x01,[0xD3,0x83])
    adc0= bus.read_i2c_block_data(0x48, 0x00,2)
    a= adc0[0] & 0xFFFF
    b= adc0[1] & 0xFF
    c= (a<<8)  | b
    FL_sensor = c

def adc_data_channel_3():
    global HL_sensor
    bus.write_i2c_block_data(0x48, 0x01,[0xE3,0x83])
    adc0= bus.read_i2c_block_data(0x48, 0x00,2)
    a= adc0[0] & 0xFFFF
    b= adc0[1] & 0xFF
    c= (a<<8)  | b
    HL_sensor = c

def adc_data_channel_4():
    global HR_sensor
    bus.write_i2c_block_data(0x48, 0x01,[0xF3,0x83])
    adc0= bus.read_i2c_block_data(0x48, 0x00,2)
    a= adc0[0] & 0xFFFF
    b= adc0[1] & 0xFF
    c= (a<<8)  | b
    HR_sensor = c

servomin=160
servomax=524
i1=1
b=0
b0=0
dkd=0

# Leg gemoetry data

Lh = [0, -0.12]
Lk = [0, -0.12]
count = 0

q_IK = [-PI/3, PI/6]
# Protection_checks
r_max = abs(Lh[1]+Lk[1])*0.75

# Upload trajectory data
Theta = np.loadtxt('Theta_data.txt')
Xdata = np.loadtxt('X_data_2.txt')
Ydata = np.loadtxt('Y_data_2.txt')
dXdata = np.loadtxt('dX_data_2.txt')
dYdata = np.loadtxt('dY_data_2.txt')

f = 0 # frequency in Hertz
w = 2*PI*f
theta0 = 0
e_HRH0 = 0; e_HLH0 = 0; e_FRH0 = 0; e_FLH0 = 0
e_HRK0 = 0; e_HLK0 = 0; e_FRK0 = 0; e_FLK0 = 0
e_HRA0 = 0; e_HLA0 = 0; e_FRA0 = 0; e_FLA0 = 0

ie_HRK0 = 0; ie_HLK0 = 0; ie_FRK0 = 0; ie_FLK0 = 0
ie_HRH0 = 0; ie_HLH0 = 0; ie_FRH0 = 0; ie_FLH0 = 0
ie_HRA0 = 0; ie_HLA0 = 0; ie_FRA0 = 0; ie_FLA0 = 0

kp = 0.25
kd = 0.0
ki = 0

dt = 0.1
# scaling of data and rotation parameter setting
fx = 0.015 * f
fy = 0.01 * f
H = -0.075
x_data = fx * Xdata
y_data = fy * Ydata
dx_data = fx * w * dXdata
dy_data = fy * w * dYdata
Dtheta = Theta[1] - Theta[0]

write_time_start = time.clock()
while 1:
    H -= 0.0025
    if H <=-0.2:
       H = -0.2

    write_time =time.clock() - write_time_start
    if write_time >= 0.005:
	write_pwm = 1
	write_time_start = time.clock()
	write_time = 0
    else:
	write_pwm = 0

    begin_time =  time.clock()
    theta = w * dt + (theta0 % (2*PI))
    ip = int(theta / Dtheta)
   # if ip>=20:
    #   ip = 1
    dtheta = theta - Theta[ip-1]

    data_x = x_data[ip-1] + (x_data[ip] - x_data[ip -1]) * dtheta/Dtheta
    data_y = y_data[ip-1] + (y_data[ip] - y_data[ip -1]) * dtheta/Dtheta
    data_dx = dx_data[ip-1] + (dx_data[ip] - dx_data[ip -1]) * dtheta/Dtheta
    data_dy = dy_data[ip-1] + (dy_data[ip] - dy_data[ip -1]) * dtheta/Dtheta
        # Provide target point input>> For Trajectory 1. r = [x, y], 2. r_target = np.dot(Rot(alpha), r)
    	#print "Target point"
    r = [data_x, data_y + H] #input("x, y values less than 0.175 [x, y]--> ")
    dr = [data_dx, data_dy]

    	# Modify and provide protection to the target point
            # Protection 1: Max length protection
    r_l = np.sqrt(r[0]**2 + r[1]**2)
    r_n = np.array(r)/r_l
    if r_l > r_max:
		r_l = r_max
		r = r_l * r_n
    	# Generate IK
    q_IK = IK(Lh, Lk, r)
    dq_IK = JInv(q_IK, Lh[1], Lk[1])

    # Data format transform
    q_H = PI/2 + q_IK[0] # manual input("angles in degree:  ") * PI / 180
    q_K = PI/2 - q_IK[1]
    dq_H = dq_IK[0][0]
    dq_K = - dq_IK[1][0]
    ''' adc_data_channel_1();
    time.sleep(0.01)
    adc_data_channel_2();
    time.sleep(0.01)
    adc_data_channel_3();
    time.sleep(0.01)
    adc_data_channel_4();
    print FL_sensor,
    print FR_sensor,
    print HL_sensor,
    print HR_sensor, '''

    encoder = subprocess.check_output("/home/pi/encoder/encoder_driver_v1");
    encoder = [int(i) for i in encoder.split()]

    #encoder[0]  FRK
    #encoder[1]  FRH
    #encoder[2]  FRA
    #encoder[3]  FLA
    #encoder[4]  FLH
    #encoder[5]  FLK
    #encoder[6]  HLK
    #encoder[7]  HLH
    #encoder[8]  HLA
    #encoder[9]  HRA
    #encoder[10] HRH
    #encoder[11] HRK

    flag = 0

    # Joint_name < Actual_angle_1 Actual_angle_2 | motor_pwm_1 motor_pwm_2 | encoder_value_1 encoder_value_2 >

    # ---------- Hind Right
    e_HRH = (558 - encoder[11])* PI/(2*(558 - 296)) # HRH < 0 90 | 200 380 | 558 296>
    e_HRK = (431 - encoder[10])* PI/(2*(431 - 668)) # HRK < 0 90 | 290 445 | 431 668>
    e_HRA = 0

    de_HRH = (e_HRH - e_HRH0)/dt
    ie_HRH = (q_H -e_HRH)*dt + ie_HRH0

    de_HRK = (e_HRK - e_HRK0)/dt
    ie_HRK = (q_K -e_HRK)*dt + ie_HRK0

    de_HRA = (e_HRA - e_HRA0)/dt
    ie_HRA = ( -e_HRA)*dt + ie_HRA0

    #---------- Hind Left
    e_HLH = (encoder[6] - 500)* PI/(2*(742 - 500)) # HLH < 0 90 | 200 380 | 500 742>
    e_HLK = (758 - encoder[7])* PI/(2*(758 - 500)) # HLK < 0 90 | 290 445 | 758 500>
    e_HLA = 0

    de_HLH = (e_HLH - e_HLH0)/dt
    ie_HLH = (q_H -e_HLH)*dt + ie_HLH0

    de_HLK = (e_HLK - e_HLK0)/dt
    ie_HLK = (q_K -e_HLK)*dt + ie_HLK0

    de_HLA = (e_HLA - e_HLA0)/dt
    ie_HLA = ( -e_HLA)*dt + ie_HLA0

    # ---------- Front Right
    e_FRH = (747 - encoder[1])* PI/(2*(747 - 490)) # FRH < 0 90 | 200 380 | 747 490>
    e_FRK = (encoder[0] - 455)* PI/(2*(709 - 455)) # FRK < 0 90 | 290 445 | 455 709>
    e_FRA = 0

    de_FRH = (e_FRH - e_FRH0)/dt
    ie_FRH = (q_H -e_FRH)*dt + ie_HRH0

    de_FRK = (e_FRK - e_FRK0)/dt
    ie_FRK = (q_K -e_FRK)*dt + ie_FRK0

    de_FRA = (e_FRA - e_FRA0)/dt
    ie_FRA = ( -e_FRA)*dt + ie_FRA0

    #---------- Front Left
    e_FLH = (encoder[4] - 361)* PI/(2*(608 - 361)) # FLH < 0 90 | 200 380 | 361 607 >
    e_FLK = (698 - encoder[5])* PI/(2*(698 - 431)) # FLK < 0 90 | 290 445 | 698 431>
    e_FLA = 0

    de_FLH = (e_HLH - e_HLH0)/dt
    ie_FLH = (q_H -e_HLH)*dt + ie_HLH0

    de_FLK = (e_HLK - e_HLK0)/dt
    ie_FLK = (q_K -e_HLK)*dt + ie_HLK0

    de_FLA = (e_FLA - e_FLA0)/dt
    ie_FLA = ( -e_FLA)*dt + ie_FLA0

    # #=============================================================== pwm ===========================================
    # Generate target pwm
    pwm_hip_hr = int(270 + 2 * (q_H + ki * ie_HRH + kp *(q_H -e_HRH) + kd *(dq_H - de_HRH)) * (435 - 270) / PI)  # <0  90 | 270 435>
    pwm_knee_hr = int(290 + 2 * (q_K + ki * ie_HRK + kp *(q_K -e_HRK) + kd *(dq_K - de_HRK)) * (460 - 290) / PI) # <0  90 | 290 460>

    pwm_hip_fr = int(140 + 4 * (q_H + ki * ie_FRH + kp *(q_H -e_FRH) + kd *(dq_H - de_FRH)) * (310 - 225) / PI)  # <45 90 | 225 310>
    pwm_knee_fr = int(240 + 2 * (q_K + ki * ie_FRK + kp *(q_K -e_FRK) + kd *(dq_K - de_FRK)) * (400 - 240) / PI) # <0  90 | 240 400>

    pwm_hip_hl = int(470 + 2 * (q_H + ki * ie_HLH + kp *(q_H -e_HLH) + kd *(dq_H - de_HLH)) * (300 - 470) / PI)  # <0  90 | 470 300>
    pwm_knee_hl = int(385 + 2 * (q_K + ki * ie_HLK + kp *(q_K -e_HLK) + kd *(dq_K - de_HLK)) * (210 - 385) / PI) # <0  90 | 385 210>

    pwm_hip_fl = int(530 + 4 * (q_H + ki * ie_FLH + kp *(q_H -e_FLH) + kd *(dq_H - de_FLH)) * (360 - 445) / PI)  # <45 90 | 445 360>
    pwm_knee_fl = int(415 + 2 * (q_K + ki * ie_FLK + kp *(q_K -e_FLK) + kd *(dq_K - de_FLK)) * (245 - 415) / PI) # <0  90 | 415 245>

    # ------------------------- Protection system ----------------------------
     # -------------------- Rad to degree ------------------
    E_FLK = e_FLK *180/PI; E_FRK = e_FRK *180/PI; E_HLK = e_HLK *180/PI; E_HRK = e_HRK *180/PI
    E_FLH = e_FLH *180/PI; E_FRH = e_FRH *180/PI; E_HLH = e_HLH *180/PI; E_HRH = e_HRH *180/PI;
    E_FLA = e_FLA *180/PI; E_FRA = e_FRA *180/PI; E_HLA = e_HLA *180/PI; E_HRA = e_HRA *180/PI;
    '''
    # ---------------------------------------
    if e_HRH < 0:
       e_HRH = 0; flag = 1

    if e_HRK < 0:
       e_HRK = 0; flag = 1

    if abs(PI - e_HRK - e_HRH) < PI/3:
           e_HRH = PI - e_HRK - PI/3; flag = 1
	   if e_HRH < 0:
		e_HRH = 0
		e_HRK = PI - e_HRH - PI/3
    elif abs(PI - e_HRK - e_HRH) > 2 * PI/3:
           e_HRH = PI - e_HRK - 2*PI/3; flag = 1
	   if e_HRH < 0:
                e_HRH = 0
                e_HRK = PI - e_HRH - 2 *PI/3
    if flag == 1:
    #	pwm_hip_hr = int(175 + 2 * e_HRH * (355 - 175) / PI)
    #	pwm_knee_hr = int(300 + 2 * e_HRK * (455 - 300) / PI)

    # -----------------------------------------
    if e_FRH < 0:
       e_FRH = 0; flag = 1

    if e_FRK < 0:
       e_FRK = 0; flag = 1

    if abs(PI - e_FRK - e_HRH) < PI/3:
           e_FRH = PI - e_HRK - PI/3; flag = 1
           if e_FRH < 0:
                e_FRH = 0
                e_FRK = PI - e_FRH - PI/3
    elif abs(PI - e_FRK - e_FRH) > 2 * PI/3:
           e_FRH = PI - e_FRK - 2*PI/3; flag = 1
           if e_FRH < 0:
                e_FRH = 0
                e_FRK = PI - e_FRH - 2 *PI/3

    if flag == 1:
    #    pwm_hip_fr = int(140 + 4 * e_FRH * (310 - 225) / PI)
   #     pwm_knee_fr = int(240 + 2 * e_FRK * (400 - 240) / PI)

    # -------------------------------------------
    if e_HLH < 0:
       e_HLH = 0; flag = 1

    if e_HLK < 0:
       e_HLK = 0; flag = 1

    if abs(PI - e_HLK - e_HLH) < PI/3:
           e_HLH = PI - e_HLK - PI/3; flag = 1
           if e_HLH < 0:
                e_HLH = 0
                e_HLK = PI - e_HLH - PI/3
    elif abs(PI - e_HLK - e_HLH) > 2 * PI/3:
           e_HLH = PI - e_HLK - 2*PI/3; flag = 1
           if e_HLH < 0:
                e_HLH = 0
                e_HLK = PI - e_HLH - 2 *PI/3
    if flag == 1:
#        pwm_hip_hl = int(485 + 2 * e_HLH * (325 - 485) / PI)
 #       pwm_knee_hl =  int(380 + 2 * e_HLK * (200 - 380) / PI)

    # -------------------------------------------
    if e_FLH < 0:
       e_FLH = 0; flag = 1

    if e_FLK < 0:
       e_FLK = 0; flag = 1

    if abs(PI - e_FLK - e_FLH) < PI/3:
           e_FLH = PI - e_FLK - PI/3; flag = 1
           if e_FLH < 0:
                e_FLH = 0
                e_FLK = PI - e_FLH - PI/3
    elif abs(PI - e_FLK - e_FLH) > 2 * PI/3:
           e_FLH = PI - e_FLK - 2*PI/3; flag = 1
           if e_FLH < 0:
                e_FLH = 0
                e_FLK = PI - e_FLH - 2 *PI/3

    if flag == 1:
    '''
  #      pwm_hip_fl = int(530 + 4 * e_FLH * (360 - 445) / PI)
   #     pwm_knee_fl = int(415 + 2 * e_FLK * (245 - 415) / PI)

    # ------------------------ PWM update ------------------------------------
    motor = [pwm_knee_fr, pwm_hip_fr, 210 , 244 , pwm_hip_fl, pwm_knee_fl , pwm_knee_hl, pwm_hip_hl, 325 , 327 , pwm_hip_hr , pwm_knee_hr]
    Angles_code = [ int(E_FRH) , int(E_FRK) , int(E_FRA) ,  int(E_FLA), int(E_FLK) , int(E_FLH) ,  int(E_HLH) , int(E_HLK) , int(E_HLA) , int(E_HRA) , int(E_HRK) , int(E_HRH)]

   # print " -------------------------------------------"
    print "encoder: ",
    print encoder
    print "motor:   ",
    print motor
    print "         ",
    print Angles_code

   # motor = [342 , 342 , 244 , 342 , 342 , 244 , 244 , 244 , 244 , 344 , 344 , 244]
   #          FRK   FRH   FRA   FLA   FLH   FLK   HLH   HLK   HLA   HRA   HRK   HRH

    if  write_pwm == 1:
    	pwm1.set_pwm(0, 0, motor[0])
    	pwm1.set_pwm(1, 0, motor[1])
    	pwm1.set_pwm(3, 0, motor[2])
    	pwm1.set_pwm(12, 0, motor[3])
    	pwm1.set_pwm(14, 0, motor[4])
    	pwm1.set_pwm(15, 0, motor[5])
    	pwm2.set_pwm(0, 0, motor[6])
    	pwm2.set_pwm(1, 0, motor[7])
    	pwm2.set_pwm(3, 0, motor[8])
    	pwm2.set_pwm(12, 0, motor[9])
    	pwm2.set_pwm(14, 0, motor[10])
    	pwm2.set_pwm(15, 0, motor[11])

    theta0 = theta

    e_HRH0 = e_HRH; e_HLH0 = e_HLH; e_FRH0 = e_FRH; e_FLH0 = e_FLH
    ie_HRH0 = ie_HRH; ie_HLH0 = ie_HLH; ie_FRH0 = ie_FRH; ie_FLH0 = ie_FLH

    e_HRK0 = e_HRK; e_HLK0 = e_HLK; e_FRK0 = e_FRK; e_FLK0 = e_FLK
    ie_HRK0 = ie_HRK; ie_HLK0 = ie_HLK; ie_FRK0 = ie_FRK; ie_FLK0 = ie_FLK

    e_HRA0 = e_HRA; e_HLA0 = e_HLA; e_FRA0 = e_FRA; e_FLA0 = e_FLA
    ie_HRA0 = ie_HRA; ie_HLA0 = ie_HLA; ie_FRA0 = ie_FRA; ie_FLA0 = ie_FLA

    end_time = time.clock()
    #dt = end_time  - begin_time
    #print dt

