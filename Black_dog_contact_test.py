from __future__ import division
import time

from smbus import SMBus
bus = SMBus(1)


def adc_data_channel_1():
    bus.write_i2c_block_data(0x48, 0x01,[0xC3,0xE3])
    adc0= bus.read_i2c_block_data(0x48, 0x00,2)
    a= adc0[0] & 0xFFFF
    b= adc0[1] & 0xFF
    c= (a<<8)  | b
    print c,

def adc_data_channel_2():
    bus.write_i2c_block_data(0x48, 0x01,[0xD3,0xE3])
    adc0= bus.read_i2c_block_data(0x48, 0x00,2)
    a= adc0[0] & 0xFFFF
    b= adc0[1] & 0xFF
    c= (a<<8)  | b
    print c,

def adc_data_channel_3():
    bus.write_i2c_block_data(0x48, 0x01,[0xE3,0x83])
    adc0= bus.read_i2c_block_data(0x48, 0x00,2)
    a= adc0[0] & 0xFFFF
    b= adc0[1] & 0xFF
    c= (a<<8)  | b
    print c,

def adc_data_channel_4():
    bus.write_i2c_block_data(0x48, 0x01,[0xF3,0xE3])
    adc0= bus.read_i2c_block_data(0x48, 0x00,2)
    a= adc0[0] & 0xFFFF
    b= adc0[1] & 0xFF
    c= (a<<8)  | b
    print c

while True:
    adc_data_channel_1();
    time.sleep(0.01)
    c=0
    adc_data_channel_2();
    time.sleep(0.01)
    c=0
    adc_data_channel_3();
    time.sleep(0.01)
    c=0
    adc_data_channel_4();
    time.sleep(0.1)
    c=0
