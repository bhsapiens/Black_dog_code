from __future__ import division
import time
import Adafruit_PCA9685
import subprocess
import math
from smbus import SMBus
bus = SMBus(1)

#Global Variables
pwm1 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
pwm2 = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
pwm1.set_pwm_freq(50)
pwm2.set_pwm_freq(50)


def adc_data_channel_1():
    bus.write_i2c_block_data(0x48, 0x01,[0xC3,0xE3])
    adc0= bus.read_i2c_block_data(0x48, 0x00,2)
    a= adc0[0] & 0xFFFF
    b= adc0[1] & 0xFF
    c1 = (a<<8)  | b
    return c1

def adc_data_channel_2():
    bus.write_i2c_block_data(0x48, 0x01,[0xD3,0xE3])
    adc0= bus.read_i2c_block_data(0x48, 0x00,2)
    a= adc0[0] & 0xFFFF
    b= adc0[1] & 0xFF
    c2 = (a<<8)  | b
    return c2

def adc_data_channel_3():
    bus.write_i2c_block_data(0x48, 0x01,[0xE3,0xE3])
    adc0= bus.read_i2c_block_data(0x48, 0x00,2)
    a= adc0[0] & 0xFFFF
    b= adc0[1] & 0xFF
    c3 = (a<<8)  | b
    return c3

def adc_data_channel_4():
    bus.write_i2c_block_data(0x48, 0x01,[0xF3,0xE3])
    adc0= bus.read_i2c_block_data(0x48, 0x00,2)
    a= adc0[0] & 0xFFFF
    b= adc0[1] & 0xFF
    c4 = (a<<8)  | b
    return c4

pwm = 360
dpwm = -300
dt = 0
while True:
    pwm += dpwm*dt
    pwm_hrk = int(pwm)
    t1 = time.clock()
    HR_I = adc_data_channel_1();
    time.sleep(0.00025)
    HR_H = adc_data_channel_2();
    time.sleep(0.00025)
    HR_O = adc_data_channel_3();
    time.sleep(0.00025)
    HR_K = adc_data_channel_4();
    t2 = time.clock()
    print pwm
    if HR_K < 10 or HR_K > 60000:
	pwm = 300
        pwm_hrk = 300
        dpwm = 0

    #pwm1.set_pwm(0, 0, 300)
    #pwm1.set_pwm(1, 0, 300)
    #pwm1.set_pwm(3, 0, 300)
    #pwm1.set_pwm(12, 0, 300)
    #pwm1.set_pwm(14, 0, 300)
    #pwm1.set_pwm(15, 0, 300)
    #pwm2.set_pwm(0, 0, 300)
    #pwm2.set_pwm(1, 0, 300)
    #pwm2.set_pwm(3, 0, 300)
    #pwm2.set_pwm(12, 0, 300)
    #pwm2.set_pwm(14, 0, 300)
    pwm2.set_pwm(15, 0, pwm_hrk)
    t3 = time.clock()
    dt = t3 - t1
    #print [t3 - t1, t3- t2, t2 -t1]
