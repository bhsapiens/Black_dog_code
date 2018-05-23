#!/usr/bin/python
# -*- coding: utf-8 -*-

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor

"""Sher_control controller."""

from controller import Robot
from SherControl_Scripts_clean import *
from math import pi
import matplotlib.pyplot as plt

# create the Robot instance.

robot = Robot()

# get the time step of the current world.

timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  led = robot.getLED('ledname')
#  ds = robot.getDistanceSensor('dsname')

keys = robot.getKeyboard()
gpad = robot.getJoystick()

acc = robot.getAccelerometer('ACC')
ori = robot.getInertialUnit('IMU')

q11 = robot.getPositionSensor('FLAps')
q12 = robot.getPositionSensor('FLHps')
q13 = robot.getPositionSensor('FLKps')
q21 = robot.getPositionSensor('FRAps')
q22 = robot.getPositionSensor('FRHps')
q23 = robot.getPositionSensor('FRKps')
q31 = robot.getPositionSensor('HLAps')
q32 = robot.getPositionSensor('HLHps')
q33 = robot.getPositionSensor('HLKps')
q41 = robot.getPositionSensor('HRAps')
q42 = robot.getPositionSensor('HRHps')
q43 = robot.getPositionSensor('HRKps')

m11 = robot.getMotor('FLAm')
m12 = robot.getMotor('FLHm')
m13 = robot.getMotor('FLKm')
m21 = robot.getMotor('FRAm')
m22 = robot.getMotor('FRHm')
m23 = robot.getMotor('FRKm')
m31 = robot.getMotor('HLAm')
m32 = robot.getMotor('HLHm')
m33 = robot.getMotor('HLKm')
m41 = robot.getMotor('HRAm')
m42 = robot.getMotor('HRHm')
m43 = robot.getMotor('HRKm')

F1 = robot.getTouchSensor('TSFL')
F2 = robot.getTouchSensor('TSFR')
F3 = robot.getTouchSensor('TSHL')
F4 = robot.getTouchSensor('TSHR')

C1 = robot.getTouchSensor('TS_FL')
C2 = robot.getTouchSensor('TS_FR')
C3 = robot.getTouchSensor('TS_HL')
C4 = robot.getTouchSensor('TS_HR')

T11 = robot.getGPS('gps_FL')
T12 = robot.getGPS('gps_FLb')

Ka = 70
Kh = 70
Kk = 70
m11.setControlPID(Ka, 0, 0)
m12.setControlPID(Kh, 0, 0)
m13.setControlPID(Kk, 0, 0)

m21.setControlPID(Ka, 0, 0)
m22.setControlPID(Kh, 0, 0)
m23.setControlPID(Kk, 0, 0)

m31.setControlPID(Ka, 0, 0)
m32.setControlPID(Kh, 0, 0)
m33.setControlPID(Kk, 0, 0)

m41.setControlPID(Ka, 0, 0)
m42.setControlPID(Kh, 0, 0)
m43.setControlPID(Kk, 0, 0)


# Main loop:

dt = 0.05

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
print qc

m11.setPosition(qc[0, 0])
m12.setPosition(qc[1, 0])
m13.setPosition(qc[2, 0])

m21.setPosition(qc[0, 1])
m22.setPosition(qc[1, 1])
m23.setPosition(qc[2, 1])

m31.setPosition(qc[0, 2])
m32.setPosition(qc[1, 2])
m33.setPosition(qc[2, 2])

m41.setPosition(qc[0, 3])
m42.setPosition(qc[1, 3])
m43.setPosition(qc[2, 3])

sigmaf = np.loadtxt('sigmaf_data.txt')

lf = np.loadtxt('lf_data.txt')

Theta = np.loadtxt('Theta_data.txt')

Cq = Regression_kernel_C(Theta, sigmaf, lf)

T = 0
wt = 1

while robot.step(timestep) != -1:

    # %% Read the sensors:
    T = T + dt
    theta_t = wt * T
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    q = np.zeros([3, 4])
    q[0, 0] = q11.getValue()
    q[0, 1] = q21.getValue()
    q[0, 2] = q31.getValue()
    q[0, 3] = q41.getValue()
    q[1, 0] = q12.getValue()
    q[1, 1] = q22.getValue()
    q[1, 2] = q32.getValue()
    q[1, 3] = q42.getValue()
    q[2, 0] = q13.getValue()
    q[2, 1] = q23.getValue()
    q[2, 2] = q33.getValue()
    q[2, 3] = q43.getValue()

    For3d = np.zeros([3, 4])
    For3d[:, 0] = F1.getValues()
    For3d[:, 1] = F2.getValues()
    For3d[:, 2] = F3.getValues()
    For3d[:, 3] = F4.getValues()
    
    C = np.zeros([4, 1])
    # C[0] = C1.getValue()
    # C[1] = C2.getValue()
    # C[2] = C3.getValue()
    # C[3] = C4.getValue()
        
    Acc = np.zeros([3,1]) #acc.getValues()
    PHI = np.zeros([3,1]) #ori.getRollPitchYaw()

    k = keys.getKey()
    gP = gpad.getPressedButton()

    # Process sensor data here.

    dq = (q - qc)/dt

    F = For3d[2, :]
    H = For3d[0, :]

    [vf, vs, omega] = HigherControl(k, gP, vf0)

    vf0 = vf

    # # Higher level control

    [parameters, X] = MidControl(X0, q, theta_t, F, C, Acc, PHI, dt, Gb, Gl, omega, vs, vf, theta, Phi, w, Phihk, qc, Cq, T)

    X0 = X

    # Enter here functions to send actuator commands, like:
    #  led.set(1)
    # # Morphed oscillator

    [AA, HH, KK, X_MO, dXMO] = LowerControl(parameters, X0_MO, dt, q)

    X0_MO = X_MO

    w = X_MO[0:4]
    theta = X_MO[4:8]
    Phi = X_MO[8:12]

    print [vf, vs, omega, gP]

    # Send data to motor

    m11.setPosition(AA[0])
    m12.setPosition(HH[0])
    m13.setPosition(KK[0])

    m21.setPosition(AA[1])
    m22.setPosition(HH[1])
    m23.setPosition(KK[1])

    m31.setPosition(AA[2])
    m32.setPosition(HH[2])
    m33.setPosition(KK[2])

    m41.setPosition(AA[3])
    m42.setPosition(HH[3])
    m43.setPosition(KK[3])

    gpsFLe = T11.getValues()
    gpsFLb = T12.getValues()

    gpsFL = np.array(gpsFLe) - np.array(gpsFLb)

    f = open('gpsFL.txt', 'a')  # size: 3x1
    f.write(str(PHI[0]) + ',' + str(PHI[1]) + ',' + str(PHI[2]) + '\n')
    f.close()

    qc = q
# Enter here exit cleanup code.
