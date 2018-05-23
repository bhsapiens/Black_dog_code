from __future__ import division
import time
from smbus import SMBus
import Adafruit_PCA9685
import subprocess
import math
from SherControl_Scripts_global import Inverse_Kinamatics as IK
from SherControl_Scripts_global import Jacobian_Inv as JInv
import numpy as np
import pygame
import csv
PI = math.pi



#Global Variables
pwm1 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
pwm1.set_pwm_freq(50)
pwm2 = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
pwm2.set_pwm_freq(50)
bus = SMBus(1)

def adc(add, data):
    bus.write_i2c_block_data(add, 0x01,data)
    adc0= bus.read_i2c_block_data(add, 0x00,2)
    a= adc0[0] & 0xFFFF
    b= adc0[1] & 0xFF
    c= (a<<8)  | b
    return c

#Functions

servomin=160
servomax=524

# Leg geometry data

Lh = [0, -0.12]
Lk = [0, -0.13]
count = 0

q_IK = [-PI/3, PI/6]
# Protection_checks
r_max = 0.22

# Upload trajectory data
Theta = np.loadtxt('Theta_data.txt')
Xdata = np.loadtxt('X_data_2.txt'); #Xdata = -Xdata
Ydata = np.loadtxt('Y_data_2.txt')
dXdata = np.loadtxt('dX_data_2.txt')
dYdata = np.loadtxt('dY_data_2.txt')

f = 1
# frequency in Hertz
w = 2*PI*f
theta0 = [0, 0, 0, 0]
theta = [0, 0, 0, 0]
kp = 0
kd = 0.00
ki = 0.0

# Integration time
dt = 0.05
T_run = 200

# scaling of data and rotation parameter setting
fx = 0.025
fy = 0.025
H = -0.165
x_data = Xdata
y_data = Ydata
dx_data = w * dXdata
dy_data = w * dYdata
Dtheta = Theta[1] - Theta[0]
Theta_min = min(Theta)

text_file = open("openLoop.txt", "w")
text_file.close()

PWM = [300, 300, 325, 325, 300, 300, 400, 300, 325, 325, 300, 300]
PHI = [0, 0, PI, PI]
MR = [400, 530, 310, 250, 140, 275, 430, 380, 315, 325, 140, 160]
# [FLK FLH FLA FRA FRH FRK HLH HLK HLA HRA HRK HRH]
def filter(line):
  addr=int(line[0])
  data = eval(line[1])
  motor = int(line[3])
  dir = int(line[4])
  angle = int(line[5])
  return addr, data, motor, dir, angle
A = open('Black_dog_contact_map.csv', 'rb')
sp = csv.reader(A, delimiter='|' )

data = [[0 for i in range(7)] for j in range(16)]
i =0
for row in sp:
     row=map(lambda x: x.strip(),row)
     [addr, Data, motor, dir, angle] = filter(row)
     data[i][0] = addr
     data[i][1] = Data
     data[i][3] = motor
     data[i][5] = angle
     data[i][4] = dir
     i +=1

BD_ref = open("Black_dog_reference.txt", "r")
Ref = BD_ref.read()

T = 0
q = np.zeros([12, 1])
cc = np.zeros([12, 1])
H0 = -0.22
mn = 100

# Roll Pitch Yaw
print ("Initializing Black Dog open loop")
print ("Please enter the following inputs before running the robot")
flag = 0
while 1:
    begin_time =  time.time()
    if flag == 0:
	Leg_no = input("Which leg do you wish to operate ?  ( FL(0) / FR(1) / HL(2) / HR(3) / All(4) / To inspect motor(5) / Chassis orientation(6) / Fall recovery(7): \n:")
	flag = 1

    if Leg_no == 5:
	mn = 100
	print("Motor name (PWM number): FLK(0) FLH(1) FLA(2) FRA(3) FRH(4) FRK(5) HLH(6) HLK(7) HLA(8) HRA(9) HRK(10) HRH(11) | To return to menu (100)")
        mn = input("Which motor do you wish to operate ? : ")
	if mn == 100:
		flag = 0
	else:
	      Angle = input("Please input the motor angle in degrees: ")
	      if mn in [4, 5, 10, 11]: PWM[mn] = int(MR[mn] + Angle * 160 / 90)
	      else: PWM[mn] = int(MR[mn] - Angle * 160 / 90)
    elif Leg_no == 6:
	roll  = input("Please input roll angle in degrees (-180 to 180): ")
	pitch = input("Please input pitch angle in degrees (-60 to 60): ")
	while roll -5 < roll_ac < 5 + roll and pitch - 5 < pitch_ac < pitch + 5:
		# Body node positions 
		Rb = Rot(Phi, Gb)
		for i in range(0, 4):
			r = H_l + Rb[:, i]
			[qA, qH, qK] = InvK_AHK(r,Gl)
		flag = 0
	
    elif Leg_no == 7:
	print("Fall recovery Started")
	flag = 0
    else:
    	if T == 0:
		pwm1 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
		pwm1.set_pwm_freq(50)
		pwm2 = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
		pwm2.set_pwm_freq(50)

		protection = 1
		print("I can do | squats (0) | trot (1) | bound (2) | Amber (3) | walk (4) | stand (5) | couch(6) | sit(7) | sit to stand(8) | Return to menu (9)")
		V_type = input(" Please input the motion (number) you wish me to demonstrate: ")
		if V_type == 9:
			print("Retuning to menu")
			flag = 0
		else:
			protection = input("Do you wish me to keep the Auto-referencing and hardware protection system ON/OFF ? (0/1): \n:")
        		if V_type in [5, 6, 7, 8]:
				T_run = input("Please no of loops:(0 - 1000) :")
				if V_type in [5, 6, 7]: alpha = input("Please input speed parameter:(Maintain: 0.005 to 0.025) :\n")
				if V_type == 8: H0 = input("Please enter sitting height (min: -0.05, max: -0.15) : ")
				if H0 > 0: H0 = -H0
			else:
				T_run = 1000
				f =  input(" Please input the oscillation frequency(0 - 4 Hz): ")
				w = 2* PI *f
			print("Very well, here goes")

			T += 1
    	elif T == T_run:
		T = 0
    	else:
		T += 1
    	x_axis = 1

	H_os = -0.175

	if V_type == 1:
		PHI = [0, PI, PI, 0]; H = H_os

    	if V_type == 2:
		PHI = [0, 0, PI, PI]; H = H_os

    	if V_type == 3:
		PHI = [0, PI, 0, PI]; H = H_os

    	if V_type == 4:
		PHI = [0, PI/2, PI, 3*PI/2]; H = H_os

    	if V_type == 0 :
		PHI = [0, 0, 0, 0]; H = H_os; x_axis = 0


    	if V_type in [5, 6, 7]:
		f = 0; w = 2* PI * f
		if V_type == 5: HT = -0.2
		elif V_type == 6: HT = -0.14
		elif V_type == 7: HT = -0.08
		dH = alpha*(HT -H)
		if dH > 0.01: dH = 0.01
		elif dH <-0.01: dH = -0.01
		H = dH * dt + H0
        	H0 = H

	if V_type == 8:
		f = 0; w = 2* PI * f
		dHdt = -0.002
		H = dHdt + H0;
		if H <= -0.22: H = -0.22
		H0 = H

	factor_xy = 0.025
        max_amp = 0.035
    	fx = factor_xy * f;
    	if fx >= max_amp: fx = max_amp
    	fy = factor_xy * f;
    	if fy >= max_amp: fy = max_amp
    	user_1 = time.time()-begin_time
    	for i in range(0, 4):
		Hx = 0
		if i in [2, 3]: Hx = -0.05

    		theta[i] = w * dt +  theta0[i]
		theta_internal = (theta[i] + PHI[i]) % (2*PI)
    		ip = int((theta_internal - Theta_min) / Dtheta)
    		dtheta = theta_internal - Theta[ip-1]

		if i in [10, 11]: factor = 1
		else: factor = -1

    		data_x = factor * x_data[ip-1] + factor * (x_data[ip] - x_data[ip -1]) * dtheta/Dtheta
    		data_y = y_data[ip-1] +  (y_data[ip] - y_data[ip -1]) * dtheta/Dtheta

        # Provide target point input>> For Trajectory 1. r = [x, y], 2. r_target = np.dot(Rot(alpha), r)
    	# Print "Target point"
	    	r = [fx * x_axis*  data_x + Hx, fy * data_y + H] #input("x, y values less than 0.175 [x, y]--> ")

    	# Modify and provide protection to the target point
            # Protection 1: Max length protection
    		r_l = np.sqrt(r[0]**2 + r[1]**2)
    		r_n = np.array(r)/r_l
    		if r_l > r_max:
			r_l = r_max
			r = r_l * r_n
		q_IK = IK(Lh, Lk, r)
        # Data format transform
        	q[3*i] = PI/2 + q_IK[0] 	# Hip
        	q[3*i+1] = PI/2 - q_IK[1] 	# Knee
		q[3*i+2] = 0 			# Abduction

    	qM = [q[1], q[0], q[2], q[5], q[3], q[4], q[6], q[7], q[8], q[11], q[10], q[9]]
      #	     [FLK,  FLH,  FLA,  FRA,  FRH,  FRK,  HLH,  HLK,  HLA,  HRA,   HRK,   HRH ]
    	user_2 = time.time()-begin_time - user_1
    # Contact checking
    	if protection == 0:
    	 for i in range(0, 16):
		data_in = data[i][1]
        	add = data[i][0]
       		val = adc(add, data_in)
        	time.sleep(0.0001)
        	if val >60000 or val < 10:
                	cc[data[i][3]] = data[i][4]
                	angle = data[i][5]
		else: cc[data[i][3]] = 0
		i_pwm = data[i][3]
        	if cc[i_pwm] > 0:
                	PWM[i_pwm] +=5
#		MR[i_pwm] = PWM[i_pwm]
        	elif cc[i_pwm] < 0:
                	PWM[i_pwm] -= 5
#		MR[i_pwm] = PWM[i_pwm]
    	user_3 = time.time()-begin_time - user_1 - user_2
    # Generate target pwm
    	for i in range(0, 12):
 		if i in [2,3, 8, 9]:
 	     		if cc[i] == 0: PWM[i] = int(MR[i] + 2 * qM[i] * 160 / PI)
 	     		elif cc[i] > 0: PWM[i] +=15
 	     		elif cc[i] < 0: PWM[i] -=15
 		elif i in [4, 5, 10, 11]:
	     		if cc[i] == 0: PWM[i] = int(MR[i] + 2 * qM[i] * 160 / PI)
	     		elif cc[i] > 0: PWM[i] +=5
             		elif cc[i] < 0: PWM[i] -=5
		else:
             		if cc[i] == 0: PWM[i] = int(MR[i] - 2 * qM[i] * 160 / PI)
             		elif cc[i] > 0: PWM[i] -=5
             		elif cc[i] < 0: PWM[i] +=5

    	user_4 = time.time() - begin_time -user_3 -  user_1 - user_2
    # ------------------------ PWM update ------------------------------------
    if Leg_no == 4 or Leg_no == 3 or mn == 11: pwm2.set_pwm(14, 0, PWM[11])
    if Leg_no == 4 or Leg_no == 3 or mn == 10: pwm2.set_pwm(15, 0, PWM[10])
    if Leg_no == 4 or Leg_no == 3 or mn == 9: pwm2.set_pwm(12, 0, 325)
    if Leg_no == 4 or Leg_no == 2 or mn == 8: pwm2.set_pwm(3, 0, 315)
    if Leg_no == 4 or Leg_no == 2 or mn == 7: pwm2.set_pwm(1, 0, PWM[7])
    if Leg_no == 4 or Leg_no == 2 or mn == 6: pwm2.set_pwm(0, 0, PWM[6])
    if Leg_no == 4 or Leg_no == 0 or mn == 1: pwm1.set_pwm(15, 0, PWM[1])
    if Leg_no == 4 or Leg_no == 0 or mn == 0: pwm1.set_pwm(14, 0, PWM[0])
    if Leg_no == 4 or Leg_no == 0 or mn == 2: pwm1.set_pwm(12, 0, 310)
    if Leg_no == 4 or Leg_no == 1 or mn == 3: pwm1.set_pwm(3, 0, 250)
    if Leg_no == 4 or Leg_no == 1 or mn == 5: pwm1.set_pwm(1, 0, PWM[5])
    if Leg_no == 4 or Leg_no == 1 or mn == 4: pwm1.set_pwm(0, 0, PWM[4])

    theta0 = theta
    end_time = time.time()
    dt = end_time  - begin_time
    print dt, PWM

