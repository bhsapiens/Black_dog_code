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


#Functions

while 1:
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
