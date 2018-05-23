from __future__ import division
import time
from smbus import SMBus
import Adafruit_PCA9685
import subprocess
import math
from SherControl_Scripts_global import Inverse_Kinamatics as IK
import numpy as np
PI = math.pi
#Global Variables
pwm1 = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
pwm2 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
pwm1.set_pwm_freq(50)
pwm2.set_pwm_freq(50)
bus = SMBus(1)


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
Lk = [0, -0.155]
count = 0

q_IK = [-PI/3, PI/6]
# Protection_checks
r_max =  0.25

# Upload trajectory data
Theta = np.loadtxt('Theta_data.txt')
Xdata = np.loadtxt('X_data_2.txt')
Ydata = np.loadtxt('Y_data_2.txt')

f = 1 # frequency in Hertz
w = 2*PI*f
theta0 = 0
dt = 0.2
# scaling of data and rotation parameter setting
fx = 0.02
fy = 0.01
H = -0.175
x_data = fx * Xdata
y_data = fy * Ydata
Dtheta = Theta[1] - Theta[0]

while 1:
    begin_time =  time.clock()
    theta = w * dt + (theta0 % (2*PI))
    ip = int(theta / Dtheta)
    if ip>=20:
       ip = 1
    dtheta = theta - Theta[ip-1]

    data_x = x_data[ip-1] + (x_data[ip] - x_data[ip -1]) * dtheta/Dtheta
    data_y = y_data[ip-1] + (y_data[ip] - y_data[ip -1]) * dtheta/Dtheta
        # Provide target point input>> For Trajectory 1. r = [x, y], 2. r_target = np.dot(Rot(alpha), r)
    	#print "Target point"
    r= [data_x, data_y + H] #input("x, y values less than 0.175 [x, y]--> ")

    	# Modify and provide protection to the target point
            # Protection 1: Max length protection
    r_l = np.sqrt(r[0]**2 + r[1]**2)
    r_n = np.array(r)/r_l
    if r_l > r_max:
		r_l = r_max
		r = r_l * r_n

	    # Protection 3: Internal protection < provided already>

            # Intelligent flag raising system
    	# Generate IK
    q_IK = IK(Lh, Lk, r)
    #print q_IK
            # Protection 2: Angle limit based protection
    if q_IK[0] < -PI/2:
		q_IK[0] = -PI/2
    if q_IK[1]  < 0:
		q_IK[1] = 0

    #print (PI/2 + q_IK[0]) * 180/PI,
    #print (PI/2 - q_IK[1]) * 180/PI

    # Data format transform
    q_H = PI/2 + q_IK[0] # manual input("angles in degree:  ") * PI / 180
    q_K = PI/2 - q_IK[1]
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

    #encoder[0]  HLK
    #encoder[1]  HLH
    #encoder[2]  HLA
    #encoder[3]  HRA
    #encoder[4]  HRH
    #encoder[5]  HRK
    #encoder[6]  FLK
    #encoder[7]  FLH
    #encoder[8]  FLA
    #encoder[9]  FRA
    #encoder[10] FRH
    #encoder[11] FRK

    flag = 0

    # Joint_name < Actual_angle_1 Actual_angle_2 | motor_pwm_1 motor_pwm_2 | encoder_value_1 encoder_value_2 >
    e_HRH = int((-PI/2 - (encoder[3] - 690)* PI/(2*(690 - 421)))*180/PI) # HRH < 0 90 | 200 380 | 690 421 >
    e_HRK = int((PI/2 - (encoder[4] - 340)* PI/(2*(590 - 340)))*180/PI)    # HRK < 0 90 | 290 445 | 340 590 >
    E_HRH = q_H * 180 / PI
    E_HRK = q_K * 180 / PI

    if abs(180 - E_HRK - E_HRH) < 60:
           E_HRH = 180 - E_HRK - 60
           flag = 1
    elif abs(180 - E_HRK - E_HRH) > 140:
           E_HRH = 180 - E_HRK - 140
           flag = 2

    # Generate target pwm
    if flag != 0:
        pwm_hip = int(200 + E_HRH * (380 - 200) / 90)
        pwm_knee = int(290 + E_HRK * (445 - 290) / 90)
    else:
	pwm_hip = int(200 + 2 * q_H * (380 - 200) / PI)
	pwm_knee = int(290 + 2 * q_K * (445 - 290) / PI)

    motor = [0, 0, 244 , 0, 0, 244 , pwm_hip, pwm_knee, 244 , pwm_knee , pwm_hip , 244]
    Angles = [0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , E_HRK , E_HRH , 0]
    Angles_code = [0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , e_HRK , e_HRH , 0]

   # print " -------------------------------------------"
   # print [ E_HRH, E_HRK]
   # print "encoder: ",
   # print encoder
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
    theta0 = theta
    end_time = time.clock()
    print end_time  - begin_time

