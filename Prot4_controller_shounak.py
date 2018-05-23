from __future__ import division 
import time 
from smbus import SMBus 
import Adafruit_PCA9685 
import subprocess 
import math 
from SherControl_Scripts_global import * 
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
p.cpu_affinity([2])
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

Xdata = np.loadtxt('X_data_2.txt')
Ydata = np.loadtxt('Y_data_2.txt')

T = 0
wt = 1
dt = 0.05

AA = np.zeros([4,1])
HH = np.zeros([4,1])
KK = np.zeros([4,1])

mK = np.zeros([4, 1])
mH = np.zeros([4, 1])
mAA = np.zeros([4, 1])

qI = np.zeros([3, 4])
q = np.zeros([3, 4])

fA = 2 * PI / 1024
fH = PI / 1024
fK = 2 * PI / 1024

For3d = np.zeros([3, 4])
F = For3d[2, :]
C = np.zeros([4, 1])

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
    #print encoder
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

    e_HRH = int((-PI/2 - (encoder[3] - 741)* PI/(2*(741 - 478)))*180/PI)   # HRH < 0 90 | 165 340 | 741 478 >
    e_HRK = int((PI/2 - (encoder[4] - 340)* PI/(2*(590 - 340)))*180/PI)    # HRK < 0 90 | 290 445 | 340 590 >

    Tc0=time.clock()
    # %% Use the sensors:
    T = T + dt
    theta_t = wt * T
    #time.sleep(0.5)
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    qI[0, 0] = (encoder[8]-510)* fA # 509 = 0 degree
    qI[0, 1] = (encoder[9]-517)* fA # 517 = 0 degree
    qI[0, 2] = (encoder[2]-525)* fA # 526 = 0 degree
    qI[0, 3] = (encoder[3]-560)* fA # 560 = 0 degree

    qI[1, 0] = (encoder[7] -           16)   * fH # Offset recorded last: 20
    qI[1, 1] = (900       -  encoder[10])   * fH # Offset recorded last: 991
    qI[1, 2] = (encoder[1] -           10)   * fH
    qI[1, 3] = -PI/2 - (encoder[3] - 741)* PI/(2*(741 - 478))

    qI[2, 0] = (encoder[6] -     0) * fK
    qI[2, 1] = (900 - encoder[11]) * fK
    qI[2, 2] = (encoder[0] -     20) * fK
    qI[2, 3] = PI/2 - (encoder[4] - 340)* PI/(2*(590 - 340))

    q[0, 0] = qI[0, 0]
    q[0, 1] = qI[0, 1]
    q[0, 2] = qI[0, 2]
    q[0, 3] = qI[0, 3]
    q[1, 0] = qI[1, 0]
    q[1, 1] = qI[1, 1]
    q[1, 2] = qI[1, 1]
    q[1, 3] = qI[1, 3]
    q[2, 0] = qI[2, 0]
    q[2, 1] = qI[2, 1]
    q[2, 2] = qI[2, 2]
    q[2, 3] = qI[2, 3]

    #print [int(q[1, 3]*180/PI), int(q[2,3]*180/PI), e_HRH, e_HRK]

    F[0] = 0 #FL_sensor
    F[1] = 0 # FR_sensor
    F[2] = 0 #HL_sensor
    F[3] = 0 #HR_sensor

    Acc = np.zeros([3, 1])  # acc.getValues()
    PHI = np.zeros([3, 1])  # ori.getRollPitchYaw()

    k = 0  # keys.getKey()
    gP = 0  # gpad.getPressedButton()

    # Process sensor data here.

    #dq = (q - qc) / dt
    Tc1 = time.clock()
    [vf, vs, omega] = HigherControl(k, gP, vf0)

    vf0 = vf

    # # Higher level control
    Tc2 = time.clock()
    [parameters, X] = MidControl(X0, q, theta_t, F, C, Acc, PHI, dt, Gb, Gl, omega, vs, vf, theta, Phi, w, Phihk, qc,
                                 0, T, Xdata, Ydata, Theta, sigmaf, lf)

    X0 = X

    # Enter here functions to send actuator commands, like:
    #  led.set(1)
    # # Morphed oscillator

    [AA, HH, KK, X_MO, dXMO] = LowerControl(parameters, X0_MO, dt, q)

    X0_MO = X_MO

    w     = X_MO[0:4]
    theta = X_MO[4:8]
    Phi   = X_MO[8:12]

    #print [vf, vs, omega, gP]

    mK[0] = int(490 - (KK[0]) * 300 / PI)
    mK[1] = int(150 + (KK[1]) * 300 / PI)
    mK[2] = int(490 - (KK[2]) * 300 / PI)
    mK[3] = int(290 + (KK[3]) * (445 - 290) * 2 / PI)

    mH[0] = (490 + (HH[0]) * 600 / PI)
    mH[1] = (150 - (HH[1]) * 600 / PI)
    mH[2] = (480 + (HH[2]) * 600 / PI)
    mH[3] = (165 - (HH[3]) * (340 - 165) * 2 / PI)

    #mAA[0] = (150 + (AA[0]) * 300 / PI)
    #mAA[1] = (150 + (AA[1]) * 300 / PI)
    #mAA[2] = (150 + (AA[2]) * 300 / PI)
    #mAA[3] = (150 + (AA[3]) * 300 / PI)

    #if mAA[i]<155:
      #  mAA[i]= 155
     #elif mAA[i]> 450:
     #   mAA[i]=450

    motor = [int(mK[1][0]), int(mH[1][0]), 244, int(mK[0][0]), int(mH[0][0]), 244, int(mK[2][0]), int(mH[2][0]), 244, int(mK[3][0]), int(mH[3][0]), 244]
    #      FRK    FRH    FRA    FLK    FLH    FLA   HLK   HLH    HLA    HRK    HRH    HRA

    Tc5=time.clock()

    #print encoder
   # print [int(mK[1][0]), int(mH[1][0])]
   # print q[:,0:4]*180/PI
    #print motor
    #print KK.transpose()
    #print q*180/PI
    #print[KK[1], HH[1]]
#    motor = [342 , 342 , 244 , 342 , 342 , 244 , 244 , 244 , 244 , 344 , 344 , 244]
    #        HRK   HRH   HRA   HLK   HLH   HLA   FRK   FRH   FRA   FLK   FLH   FLA
    #print [AA[1]*180/PI, HH[1]*180/PI, KK[1]*180/PI]
    for i in range(0, 4):
	flag = 0
	e_HRH = int((-PI/2 - (encoder[3] - 741) * PI/(2 * (741 - 478))) * 180 / PI) # HRH < 0 90 | 165 340 | 741 478 >
        e_HRK = int((PI/2 - (encoder[4] - 340) * PI/(2 * (590 - 340))) * 180 / PI)    # HRK < 0 90 | 290 445 | 340 590 >
        E_HRH = int(((741 - encoder[3]) * PI / (2 * (741 - 478))) * 180 / PI)
        E_HRK = int(((encoder[4] - 340) * PI / (2 * (590 - 340))) * 180 / PI)

        if abs(180 - E_HRK - E_HRH) < 50:
           E_HRK = 180 - E_HRH - 50
           flag = 1
        elif abs(180 - E_HRK - E_HRH) > 120:
           E_HRK = 60 - E_HRH
           flag = 2

        if E_HRK <0:
           E_HRK = 0
           if flag == 1:
                E_HRH = 130
	   elif flag == 2:
		E_HRH = 60
        if E_HRH < 0:
           E_HRH = 0
           if flag ==  1:
                E_HRK = 130
	   elif flag == 2:
		E_HRK = 60

        motor[3 * i + 1 ] = int(165 + E_HRH * (340 - 165) / 90)
        motor[3 * i + 0 ] = int(290 + E_HRK * (445 - 290) / 90)
    print motor,
    print [ E_HRK, E_HRH, e_HRK, e_HRH],
    print q[:, 3]*180/PI
    # Hind Right Leg
    pwm1.set_pwm(0, 0, motor[0])
    pwm1.set_pwm(1, 0, motor[1])
    pwm1.set_pwm(2, 0, motor[2])
    # Hind Left Leg
    pwm1.set_pwm(15, 0, motor[3])
    pwm1.set_pwm(14, 0, motor[4])
    pwm1.set_pwm(13, 0, motor[5])
    # Front Right Leg
    pwm2.set_pwm(4, 0, motor[6])
    pwm2.set_pwm(5, 0, motor[7])
    pwm2.set_pwm(3, 0, motor[8])
    # Front Left Leg
    pwm2.set_pwm(15, 0, motor[9])
    pwm2.set_pwm(14, 0, motor[10])
    pwm2.set_pwm(12, 0, motor[11])
    Tc6 = time.clock()

    diff = Tc6 - Tc0
    #print diff*1000
