from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685

# Alternatively specify a different address and/or bus:
pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)

# Configure min and max servo pulse lengths
servo_min = 200  # Min pulse length out of 4096
servo_max = 300  # Max pulse length out of 4096

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

pwm.set_pwm_freq(50)

while True:
    pwm.set_pwm(14, 0, 425)
    time.sleep(1)
    pwm.set_pwm(14, 0, 225)
    time.sleep(1)
