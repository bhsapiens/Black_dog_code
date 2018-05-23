from __future__ import division
import time
from smbus import SMBus
import Adafruit_PCA9685
import subprocess
import math

PI = 3.14
#Global Variables
pwm1 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
pwm2 = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
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
    #pwm1 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
    #pwm2 = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
    #pwm1.set_pwm_freq(50)
    #pwm2.set_pwm_freq(50)
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

    #encoder = subprocess.check_output("/home/pi/encoder/encoder_driver_v1");
    #encoder = [int(i) for i in encoder.split()]
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

    # motor = [342 , 342 , 244 , 342 , 342 , 244 , 244 , 244 , 244 , 344 , 344 , 244]
    #          FRK   FRH   FRA   FLA   FLH   FLK   HLH   HLK   HLA   HRA   HRK   HRH
    flag = 0

    # Joint_name < Actual_angle_1 Actual_angle_2 | motor_pwm_1 motor_pwm_2 | encoder_value_1 encoder_value_2 >

    motor_i = input("Please enster the motor to be tested: <0 - 11>:  ")
    mn = motor_i

    pwm_input = input("Please input PWM values (150 - 525): ")
    p_i = int(pwm_input)

    #Angles = encoder[mn]

    motor = p_i

     # Hind Right Leg
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
	 pwm2.set_pwm(0, 0, motor)
    elif mn == 7:
	 pwm2.set_pwm(1, 0, motor)
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

    time.sleep(1.5)
