from __future__ import division
import time
from smbus import SMBus
import Adafruit_PCA9685
import subprocess
import math
from SherControl_Scripts_global import Inverse_Kinamatics as IK
from SherControl_Scripts_global import Jacobian_Inv as JInv
import numpy as np
PI = math.pi


#Global Variables
pwm2 = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
pwm2.set_pwm_freq(50)
bus = SMBus(1)

# Encoder
encoder = subprocess.check_output("/home/pi/encoder/encoder_driver_v1");
encoder = [int(i) for i in encoder.split()]

e_HRH = (668 - encoder[11])* PI/(2*(668 - 405)) # HRH < 0 90 | 130 300 | 668 405 >
e_HRK = (encoder[10] - 363)* PI/(2*(615 - 363))
#Functions

servomin=160
servomax=524

def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 50       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm2.set_pwm(channel, 0, pulse)

i1=1
b=0
b0=0
dkd=0

# Leg gemoetry data

Lh = [0, -0.12]
Lk = [0, -0.12]
count = 0

q_IK = [-PI/3, PI/6]
# Protection_checks
r_max = abs(Lh[1]+Lk[1])*0.9

# Upload trajectory data
Theta = np.loadtxt('Theta_data.txt')
Xdata = np.loadtxt('X_data_2.txt'); Xdata = -Xdata
Ydata = np.loadtxt('Y_data_2.txt')
dXdata = np.loadtxt('dX_data_2.txt')
dYdata = np.loadtxt('dY_data_2.txt')

f = 1
# frequency in Hertz
w = 2*PI*f
theta0 = 0
e_HRK1 = e_HRK
e_HRK0 = 0
ie_HRH0 = 0
e_HRH0 = 0
ie_HRK0 = 0

kp = 0
kd = 0.00
ki = 0.0

# Integration time
dt = 0.05

# scaling of data and rotation parameter setting
fx = 0.025
fy = 0.025
H = -0.175
x_data = fx * Xdata
y_data = fy * Ydata
dx_data = fx * w * dXdata
dy_data = fy * w * dYdata
Dtheta = Theta[1] - Theta[0]
Theta_min = min(Theta)

text_file = open("xydxdy.txt", "w")
text_file.close()


while 1:
    begin_time =  time.time()
    theta = (w * dt) % (2*PI) + (theta0 % (2*PI))
    ip = int((theta - Theta_min) / Dtheta)
    dtheta = theta - Theta[ip-1]
    data_x = x_data[ip-1] + (x_data[ip] - x_data[ip -1]) * dtheta/Dtheta
    data_y = y_data[ip-1] + (y_data[ip] - y_data[ip -1]) * dtheta/Dtheta
    data_dx = dx_data[ip-1] + (dx_data[ip] - dx_data[ip -1]) * dtheta/Dtheta
    data_dy = dy_data[ip-1] + (dy_data[ip] - dy_data[ip -1]) * dtheta/Dtheta

        # Provide target point input>> For Trajectory 1. r = [x, y], 2. r_target = np.dot(Rot(alpha), r)
    	#print "Target point"
    r = [data_x, data_y + H] #input("x, y values less than 0.175 [x, y]--> ")
    dr = [data_dx, data_dy]

    	# Modify and provide protection to the target point
            # Protection 1: Max length protection
    r_l = np.sqrt(r[0]**2 + r[1]**2)
    r_n = np.array(r)/r_l
    if r_l > r_max:
		r_l = r_max
		r = r_l * r_n


    encoder = subprocess.check_output("/home/pi/encoder/encoder_driver_v1");
    #encoder = [int(i) for i in encoder.split()]
    #print encoder
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

    # Joint_name < Actual_angle_1 Actual_angle_2 | motor_pwm_1 motor_pwm_2 | encoder_value_1 encoder_value_2 >
    #e_HRH = (555 - encoder[11])* PI/(2*(555 - 303)) # HRH < 0 90 | 130 300 | 668 405 >
    #e_HRK = (encoder[10] - 182)* PI/(2*(450 - 182))    # HRK < 0 90 | 290 445 | 363 615>

    # ------------- Generate IK ------------------------
    #q= np.array([e_HRH, e_HRK])
    q_IK = IK(Lh, Lk, r)
    #invJ = JInv(q, Lh[1], Lk[1])
    #dq_IK = np.dot(invJ, dr)

    # Data format transform
    q_H = PI/2 + q_IK[0] # manual input("angles in degree:  ") * PI / 180
    q_K = PI/2 - q_IK[1]
    #dq_H = dq_IK[0]
    #dq_K = - dq_IK[1]

    # -------------- PID controller
    #de_HRH = (e_HRH - e_HRH0)/ dt
    #ie_HRH = (q_H -e_HRH)*dt + ie_HRH0

    #de_HRK = (e_HRK - e_HRK1)/(dt)
    #ie_HRK = (q_K -e_HRK)*dt + ie_HRK0

    # Generate target pwm
    # pwm_hip = int(290 + 2 * (q_H + ki * ie_HRH + kp *(q_H -e_HRH) + kd *(dq_H - de_HRH)) * (450 - 290) / PI)
    # pwm_knee = int(285 + 2 * (q_K + ki * ie_HRK + kp *(q_K -e_HRK) + kd *(dq_K - de_HRK)) * (445 - 285) / PI)

    pwm_hip = int(290 + 2 * (q_H + kp *(q_H -e_HRH)) * (450 - 290) / PI)
    pwm_knee = int(285 + 2 * (q_K + kp *(q_K -e_HRK)) * (445 - 285) / PI)
	#print [q_H, q_K],
	#print [e_HRH, e_HRK]
	#print [dq_H, dq_K],
        #print [de_HRH, de_HRK]
        #print de_HRH
	#print (q_H -e_HRH),
	#print (dq_H - de_HRH)
    	#print [pwm_hip, pwm_knee]

    # ------------------------ PWM update ------------------------------------
    motor = [0, 0, 244 , 0, 0, 244 , pwm_hip, pwm_knee, 244 , pwm_knee , pwm_hip , 244]

    # Hind Right Leg
    #set_servo_pulse(14, motor[10])
    pwm2.set_pwm(14, 0, motor[10])
    pwm2.set_pwm(15, 0, motor[9])

    theta0 = theta
    e_HRH0 = e_HRH
    #ie_HRH0 = ie_HRH
    e_HRK1 = e_HRK
    #ie_HRK0 = ie_HRK

    #text_file = open("xydxdy.txt", "a")
    #text_file.write(str(begin_time)+ ',' + str(data_x)+','+str(data_dx)+','+str(theta)+','+str(dtheta)+',' + str(q_K)+ ','+str(q_H) +','+str(dq_K)+ ','+str(dq_H) +','+str(motor[9]) + ',' + str(motor[10]) + ',' + str(e_HRH) + ',' + str(e_HRK) + ',' + str(de_HRH) + ',' + str(de_HRK)+ '\n')
    #text_file.close()

    end_time = time.time()
    dt = end_time  - begin_time
    #time.sleep(dt - dT)
    print dt

