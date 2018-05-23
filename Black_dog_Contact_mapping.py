from __future__ import division
import time
from smbus import SMBus
import Adafruit_PCA9685
import subprocess
import math

PI = 3.14
#Global Variables
pwm2 = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
pwm2.set_pwm_freq(50)
bus = SMBus(1)


FL_sensor = 0
FR_sensor = 0
HL_sensor = 0
HR_sensor = 0

#Functions
def address_ads1115():
	add=[0x48, 0x48, 0x48, 0x48, 0x49, 0x49, 0x49, 0x49, 0x4a, 0x4a, 0x4a, 0x4a, 0x4b, 0x4b, 0x4b, 0x4b]
        return add

def data_ads1115():
	data = [[0xC3, 0xE3], [0xD3, 0xE3], [0xE3, 0xE3], [0xF3, 0xE3],
		[0xC3, 0xE3], [0xD3, 0xE3], [0xE3, 0xE3], [0xF3, 0xE3],
		[0xC3, 0xE3], [0xD3, 0xE3], [0xE3, 0xE3], [0xF3, 0xE3],
		[0xC3, 0xE3], [0xD3, 0xE3], [0xE3, 0xE3], [0xF3, 0xE3]]
        return data

def adc(add, data):
    bus.write_i2c_block_data(add, 0x01,data)
    adc0= bus.read_i2c_block_data(add, 0x00,2)
    a= adc0[0] & 0xFFFF
    b= adc0[1] & 0xFF
    c= (a<<8)  | b
    return c

def robot_file_generate():
        file_name = 'Black_dog_contact_map.txt'
        f = open(file_name,'w')
        for i in range(0,24):
                f.write('line'+' '+str(i)+'\n')
        f.close()
        return file_name

def robot_file_modify():
        file_name = 'Black_dog_contact_map.txt'
        return file_name

def replace_line(file_name, line_num, text):
    lines = open(file_name, 'r').readlines()
    lines[line_num] = text
    out = open(file_name, 'w')
    out.writelines(lines)
    out.close()

print("Do you want to modify the main file or you wish to regenerate the main file?")
mod = input("Enter 1 to modify or 0 to regenerate the file ")
if mod == 1:
	Robot_file = robot_file_modify()
else:
	Robot_file = robot_file_generate()
address = address_ads1115()
data = data_ads1115()
N = len(data)

angle_ab_in = 45
angle_ab_out = 30
while 1:
    flag = 0

     # Hind Right Leg
    for i in range(0, N):
        val = adc(address[i], data[i])
        time.sleep(0.0002)
	print val
        dir = 0
        if val<10 or val>60000:
		line_num = i
		print("Front/Hind = 1/2")
		print("Left/Right = 1/2")
		print("Contact sensor: Knee/Abduction in/Hip/ Abduction out = 1/2/3/4")
		value = input("Input 'Front-Left-Hip' as 113: ")
		while value <= 99:
			print("Please check the Input")
			print("You have entered " + str(value))
			value = input("Input 'Front-Left-Hip' as 113: ")
		if value == 111:
			motor_ref = 0; dir = 1; angle = 0
		elif value == 113:
			motor_ref = 1; dir = 1; angle = 0
		elif value == 112 or value == 114:
			motor_ref = 2; dir = 1; angle = angle_ab_out
			if value == 112: dir = -1; angle = angle_ab_in
		elif value == 121:
                        motor_ref = 5; dir = 1; angle = 0
                elif value == 123:
                        motor_ref = 4; dir = 1; angle = 0
                elif value == 122 or value == 124:
                        motor_ref = 3; dir = 1; angle = angle_ab_out
                        if value == 122: dir = -1; angle = angle_ab_in
		elif value == 211:
                        motor_ref = 6; dir = 1; angle = 0
                elif value == 213:
                        motor_ref = 7; dir = 1; angle = 0
                elif value == 212 or value == 214:
                        motor_ref = 8; dir = 1; angle = angle_ab_out
                        if value == 212: dir = -1; angle = angle_ab_in
		elif value == 221:
                        motor_ref = 11; dir = 1; angle = 0
                elif value == 223:
                        motor_ref = 10; dir = 1; angle = 0
                elif value == 222 or value == 224:
                        motor_ref = 9; dir = 1; angle = angle_ab_out
                        if value == 222: dir = -1; angle = angle_ab_in

		reference_txt = str(address[i]) + " , " + str(data[i]) +" , "+str(value) +" , "+ str(motor_ref) + " , " + str(dir) + " , "+ str(angle) + "\n"
		replace_line(Robot_file, line_num, reference_txt)

