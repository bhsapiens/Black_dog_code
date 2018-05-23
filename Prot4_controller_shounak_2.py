from __future__ import division 
import time 
from smbus import SMBus 
import Adafruit_PCA9685 
import subprocess 
import math 
from SherControl_Scripts_clean_2 import * 
from math import pi
import psutil
import os

# import matplotlib.pyplot as plt

# Global Variables
pwm1 = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
pwm2 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
pwm1.set_pwm_freq(50)
pwm2.set_pwm_freq(50)
bus = SMBus(1)

FL_sensor = 0
FR_sensor = 0
HL_sensor = 0
HR_sensor = 0
p = psutil.Process(os.getpid())
p.nice(-19)
p.cpu_affinity([3])
# Functions


def adc_data_channel_1():
    global FR_sensor
    bus.write_i2c_block_data(0x48, 0x01, [0xC3, 0x83])
    adc0 = bus.read_i2c_block_data(0x48, 0x00, 2)
    a = adc0[0] & 0xFFFF
    b = adc0[1] & 0xFF
    c = (a << 8) | b
    FR_sensor = c


def adc_data_channel_2():
    global FL_sensor
    bus.write_i2c_block_data(0x48, 0x01, [0xD3, 0x83])
    adc0 = bus.read_i2c_block_data(0x48, 0x00, 2)
    a = adc0[0] & 0xFFFF
    b = adc0[1] & 0xFF
    c = (a << 8) | b
    FL_sensor = c


def adc_data_channel_3():
    global HL_sensor
    bus.write_i2c_block_data(0x48, 0x01, [0xE3, 0x83])
    adc0 = bus.read_i2c_block_data(0x48, 0x00, 2)
    a = adc0[0] & 0xFFFF
    b = adc0[1] & 0xFF
    c = (a << 8) | b
    HL_sensor = c


def adc_data_channel_4():
    global HR_sensor
    bus.write_i2c_block_data(0x48, 0x01, [0xF3, 0x83])
    adc0 = bus.read_i2c_block_data(0x48, 0x00, 2)
    a = adc0[0] & 0xFFFF
    b = adc0[1] & 0xFF
    c = (a << 8) | b
    HR_sensor = c


servomin = 160
servomax = 524
i1 = 1
b = 0
b0 = 0
dkd = 0


# Code to run the robot
# Initial states

vf0 = 0.0
wf0 = 0.0

# Robot properties and dimensions

Gb = np.array([[0.25, 0, 0.1], [0.25, 0, -0.1], [-0.25, 0, 0.1],
              [-0.25, 0, -0.1]])
Gb = Gb.transpose()

Gl = np.array([
    [0, -0.15, 0],
    [0, -0.15, 0],
    [0, -0.15, 0],
    [0, -0.15, 0],
    [0, -0.15, 0],
    [0, -0.15, 0],
    [0, -0.15, 0],
    [0, -0.15, 0],
    ])
Gl = Gl.transpose()

X0 = np.zeros([16, 3])
X0_MO = np.zeros([48, 1])

theta = np.zeros([4, 1])  # Phase

Phi = np.zeros([4, 1])  # Phase difference

w = np.zeros([4, 1])  # Actual frequency

Phihk = pi / 2 * np.ones([4, 1])  # Hip- knee Phase difference

Hoff = np.array([-0.5, -0.5, -0.5, -0.5])
Koff = np.array([1, 1, 1, 1])

# - perform simulation steps until Webots is stopping the controller

qc = np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])

# Loading and pre-calculating trajectory values
sigmaf = np.loadtxt('sigmaf_data.txt')

lf = np.loadtxt('lf_data.txt')

Theta = np.loadtxt('Theta_data.txt')

Cq = Regression_kernel_C(Theta, sigmaf, lf)

f = open('Input.txt','w')
f.close()

f = open('Output.txt','w')
f.close()

f = open('Time.txt','w')
f.close()

T = 0
wt = 0.2
dt = 0.1

AA = np.zeros([4,1])
HH = np.zeros([4,1])
KK = np.zeros([4,1])

t0 = time.clock()
while 1:
    #adc_data_channel_1()
    #time.sleep(0.005)
    #adc_data_channel_2()
    #time.sleep(0.005)
    #adc_data_channel_3()
    #time.sleep(0.005)
    #adc_data_channel_4()
    # print FL_sensor,
    # print FR_sensor,
    # print HL_sensor,
    # print HR_sensor,

    encoder = subprocess.check_output("/home/pi/encoder/encoder_driver_v1")
    encoder = [int(i) for i in encoder.split()]
    # encoder[0]  HLK
    # encoder[1]  HLH
    # encoder[2]  HLA
    # encoder[3]  HRA
    # encoder[4]  HRH
    # encoder[5]  HRK
    # encoder[6]  FLK
    # encoder[7]  FLH
    # encoder[8]  FLA
    # encoder[9]  FRA
    # encoder[10] FRH
    # encoder[11] FRK
    before=time.clock()
    # %% Use the sensors:
    T = T + dt
    theta = 2 * PI * wt * T
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    qI = np.zeros([3, 4])

    fA = 2 * PI / 1024
    qI[0, 0] = (encoder[8]-515)* fA # 509 = 0 degree
    qI[0, 1] = (encoder[9]-517)* fA # 517 = 0 degree
    qI[0, 2] = (encoder[2]-525)* fA # 526 = 0 degree
    qI[0, 3] = (encoder[3]-560)* fA # 560 = 0 degree

    fH = PI / 1024
    qI[1, 0] = (encoder[7] -           120)   * fH # Offset recorded last: 20
    qI[1, 1] = (880       -  encoder[10])   * fH # Offset recorded last: 991
    qI[1, 2] = (encoder[1] -           110)   * fH
    qI[1, 3] = (900       -   encoder[4])   * fH

    fK = 2 * PI / 1024
    qI[2, 0] = (encoder[6] -  87) * fK
    qI[2, 1] = (890 - encoder[11]) * fK
    qI[2, 2] = (encoder[0] -  87) * fK
    qI[2, 3] = (910 -  encoder[5]) * fK

    q = np.zeros([3, 4])
    q[0, 0] = qI[0, 0]
    q[0, 1] = qI[0, 1]
    q[0, 2] = qI[0, 3]
    q[0, 3] = qI[0, 3]
    q[1, 0] = -qI[1, 0]
    q[1, 1] = -qI[1, 1]
    q[1, 2] = -qI[1, 2]
    q[1, 3] = -qI[1, 3]
    q[2, 0] = qI[2, 0]
    q[2, 1] = qI[2, 1]
    q[2, 2] = qI[2, 2]
    q[2, 3] = qI[2, 3]

    For3d = np.zeros([3, 4])
    F = For3d[2, :]
    F[0] = FL_sensor
    F[1] = FR_sensor
    F[2] = HL_sensor
    F[3] = HR_sensor

    if before < 1:
       V_input = 0
    else:
       V_input = 0

    mK = np.zeros([4, 1])
    mH = np.zeros([4, 1])
    mAA = np.zeros([4, 1])

    mK[0] = int(470 - (0) * 400 / 180)
    mK[1] = int(175 + (0) * 400 / 180)
    mK[2] = int(500 - (0) * 400 / 180)
    mK[3] = int(150 + (0) * 400 / 180)

    mH[0] = (150 + (0) * 700 / 180)
    mH[1] = (500 - (V_input) * 700 / 180)
    mH[2] = (175 + (0) * 700 / 180)
    mH[3] = (500 - (0) * 700 / 180)

    #mAA[0] = (150 + (AA[0]) * 300 / PI)
    #mAA[1] = (150 + (AA[1]) * 300 / PI)
    #mAA[2] = int(150 + (AA[2]) * 300 / PI)
    #mAA[3] = int(150 + (AA[3]) * 300 / PI)

    motor = [int(mK[3][0]), int(mH[3][0]), int(mAA[3][0]), int(mK[2][0]), int(mH[2][0]), int(mAA[2][0]), int(mK[1][0]), int(mH[1][0]), int(mAA[1][0]), int(mK[0][0]), int(mH[0][0]), int(mAA[0][0])]
    #        HRK    HRH    HRA    HLK    HLH    HLA    FRK    FRH    FRA    FLK    FLH    FLA
    after=time.clock()
    diff = after-before
    print diff
    print q[0,:]*180/PI

    f=open('Time.txt','a')
    f.write(str(before - t0) + '\n')
    f.close()

    f = open('Input.txt','a')
    f.write(str(V_input) + '\n')
    f.close()

    f = open('Output.txt','a')
    f.write(str(qI[1,1]) + '\n')
    f.close()

    #print q[0:3,2] * 180 / PI
    #print encoder
#    motor = [342 , 342 , 244 , 342 , 342 , 244 , 244 , 244 , 244 , 344 , 344 , 244]
    #        HRK   HRH   HRA   HLK   HLH   HLA   FRK   FRH   FRA   FLK   FLH   FLA
    #print [AA[1]*180/PI, HH[1]*180/PI, KK[1]*180/PI]

    # Hind Right Leg
    pwm1.set_pwm(0, 0, motor[0])
    pwm1.set_pwm(1, 0, motor[1])
    #pwm1.set_pwm(2, 0, motor[2])
    # Hind Left Leg
    pwm1.set_pwm(15, 0, motor[3])
    pwm1.set_pwm(14, 0, motor[4])
    #pwm1.set_pwm(13, 0, motor[5])
    # Front Right Leg
    pwm2.set_pwm(0, 0, motor[6])
    pwm2.set_pwm(1, 0, motor[7])
    #pwm2.set_pwm(2, 0, motor[8])
    # Front Left Leg
    pwm2.set_pwm(15, 0, motor[9])
    pwm2.set_pwm(14, 0, motor[10])
    #pwm2.set_pwm(13, 0, motor[11])
