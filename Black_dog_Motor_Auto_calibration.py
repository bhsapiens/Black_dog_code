from __future__ import division
import time
from smbus import SMBus
import Adafruit_PCA9685
import subprocess
import math
import numpy as np
from ast import literal_eval

PI = 3.14
#Global Variables
pwm2 = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
pwm2.set_pwm_freq(50)
bus = SMBus(1)


FL_sensor = 0
FR_sensor = 0
HL_sensor = 0
HR_sensor = 0

#Functions

def adc(add, data):
    bus.write_i2c_block_data(add, 0x01,data)
    adc0= bus.read_i2c_block_data(add, 0x00,2)
    a= adc0[0] & 0xFFFF
    b= adc0[1] & 0xFF
    c= (a<<8)  | b
    return c

def robot_file_generate():
        file_name = 'Black_dog_reference.txt'
        f = open(file_name,'w')
        for i in range(0,24):
                f.write('line'+' '+str(i)+'\n')
        f.close()
        return file_name

def replace_line(file_name, line_num, text):
    lines = open(file_name, 'r').readlines()
    lines[line_num] = text
    out = open(file_name, 'w')
    out.writelines(lines)
    out.close()

data_map = open("Black_dog_contact_map.txt", "r")
data = data_map.read()
print data
servomin=160
servomax=524
i1=1
b=0
b0=0
dkd=0
counter = 0
Robot_file = robot_file_generate()

cc = np.zeros([12, 1])
while 1:
    # motor = [342 , 342 , 244 , 342 , 342 , 244 , 244 , 244 , 244 , 344 , 344 , 244]
    #          FRK   FRH   FRA   FLA   FLH   FLK   HLH   HLK   HLA   HRA   HRK   HRH
    flag = 0

    # Joint_name < Actual_angle_1 Actual_angle_2 | motor_pwm_1 motor_pwm_2 | encoder_value_1 encoder_value_2 >
    if counter == 0:
    	motor_i = input("Please enster the motor to be tested: <0 - 11>:  ")
    	mn = motor_i

    	pwm_input = input("Please input PWM values (150 - 525): ")
    	p_i = int(pwm_input)
	motor = p_i
	counter = 1
    elif counter == 100:
	counter = 0
    else:
	counter += 1

    for i in range(0, N_L):
        data_in = data[i, 1]
	add = data[i, 0]

	val = adc(add, data_in)
        time.sleep(0.002)
	if val >60000 or val < 10:
		cc[data[i, 3]] = data[i, 4]
		angle = data[i, 5]

    if cc[mn] > 0:
                motor +=5
    elif cc[mn] < 0:
                motor -= 5
                line_num = int(2*mn + 1)

    if cc[mn] > 0 or cc[mn] < 0:
		line_num = int(2*mn + 1)
                reference_txt = str(mn) +" , "+str(motor) + " , " + str(encoder[mn]) + " , " + str(angle)+ "\n"
                replace_line(Robot_file, line_num, reference_txt)
     # Leg
    if mn == 0:
	 pwm1.set_pwm(0, 0, motor)
    elif mn == 1:
	 pwm1.set_pwm(1, 0, motor)
    elif mn == 2:
	 pwm1.set_pwm(3, 0, motor)
    elif mn == 3:
	 pwm1.set_pwm(12, 0, motor)
    elif mn == 4:
	 pwm1.set_pwm(14, 0, motor)
    elif mn == 5:
	 pwm1.set_pwm(15, 0, motor)
    elif mn == 6:
	 pwm2.set_pwm(1, 0, motor)
    elif mn == 7:
	 pwm2.set_pwm(0, 0, motor)
    elif mn == 8:
	 pwm2.set_pwm(3, 0, motor)
    elif mn == 9:
	 pwm2.set_pwm(12, 0, motor)
    elif mn == 10:
	 pwm2.set_pwm(15, 0, motor)
    elif mn == 11:
	 pwm2.set_pwm(14, 0, motor)

    print " -------------------------------------------"
    print "motor:   ",
    print motor

    #time.sleep(1.5)
