from __future__ import division
import time
from smbus import SMBus
import Adafruit_PCA9685
import subprocess

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

while 1:
    adc_data_channel_1();
    time.sleep(0.01)
    adc_data_channel_2();
    time.sleep(0.01)
    adc_data_channel_3();
    time.sleep(0.01)
    adc_data_channel_4();
    print FL_sensor,
    print FR_sensor,
    print HL_sensor,
    print HR_sensor,

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
    print encoder[0],
    print encoder[1],
    print encoder[2],
    print encoder[3],
    print encoder[4],
    print encoder[5],
    print encoder[6],
    print encoder[7],
    print encoder[8],
    print encoder[9],
    print encoder[10],
    print encoder[11]



##############ADD  CODE   HERE




    motor = [200 , 200 , 200 , 200 , 200 , 200 , 200 , 200 , 200 , 200 , 200 , 200]
    #        HRK   HRH   HRA   HLK   HLH   HLA   FRK   FRH   FRA   FLK   FLH   FLA



    pwm1.set_pwm(0, 0, motor[0])
    pwm1.set_pwm(1, 0, motor[1])
    pwm1.set_pwm(2, 0, motor[2])
    pwm1.set_pwm(15, 0, motor[3])
    pwm1.set_pwm(14, 0, motor[4])
    pwm1.set_pwm(13, 0, motor[5])
    pwm2.set_pwm(0, 0, motor[6])
    pwm2.set_pwm(1, 0, motor[7])
    pwm2.set_pwm(2, 0, motor[8])
    pwm2.set_pwm(15, 0, motor[9])
    pwm2.set_pwm(14, 0, motor[10])
    pwm2.set_pwm(13, 0, motor[11])





    time.sleep(0.01)
