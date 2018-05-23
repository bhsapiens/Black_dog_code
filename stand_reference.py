from __future__ import division
import time
from smbus import SMBus
import Adafruit_PCA9685
import subprocess
import math

PI = 3.14
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

while 1:
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
    E_HRH = int(((690 - encoder[3])* PI/(2*(690 - 421)))*180/PI)
    E_HRK = int(((encoder[4] - 340)* PI/(2*(590 - 340)))*180/PI)
    pwm_input = input("Please input PWM values Knee (150 - 525): ")
    p_i = int(pwm_input)
    pwm_input = input("Please input PWM values Hip (150 - 525): ")
    p_i_hip = int(pwm_input)

    if abs(e_HRK - e_HRH) < 50:
	E_HRK = 180 - E_HRH - 50
        flag = 1

    if E_HRK <0:
        E_HRK = 0
	if flag == 1:
		E_HRH = 130
    if E_HRH < 0:
	E_HRH = 0
	if flag ==  1:
		E_HRK = 130

    pwm_hip = int(200 + E_HRH * (375 - 200) / 90)
    pwm_knee = int(290 + E_HRK * (445 - 290) / 90)

    motor = [p_i, p_i, 244 , p_i, p_i, 244 , pwm_hip, pwm_knee, 244 , p_i , p_i_hip , 244]
    Angles = [0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , E_HRK , E_HRH , 0]
    Angles_code = [0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , e_HRK , e_HRH , 0]

    print " -------------------------------------------"
    print "encoder: ",
    print encoder
    print "motor:   ",
    print motor
    print "Angles:  ",
    print Angles
    print "Angles software reference:  ",
    print Angles_code

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

    time.sleep(0.25)
