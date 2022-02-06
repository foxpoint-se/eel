#!/usr/bin/env python3
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep

factory = PiGPIOFactory()

servo = Servo(12, pin_factory=factory)

print("tjena")
servo.mid()
# sleep(2)
# servo.min()
# sleep(2)
# servo.max()
sleep(10)
servo.value = None


# =========================

#!/usr/bin/python3
# import RPi.GPIO as GPIO
# import pigpio
# import time
 
# servo = 13
 
# # more info at http://abyz.me.uk/rpi/pigpio/python.html#set_servo_pulsewidth
 
# pwm = pigpio.pi() 
# pwm.set_mode(servo, pigpio.OUTPUT)
 
# pwm.set_PWM_frequency( servo, 50 )
 
# print( "0 deg" )
# pwm.set_servo_pulsewidth( servo, 1000 )
# time.sleep( 3 )
 
# print( "90 deg" )
# pwm.set_servo_pulsewidth( servo, 1500 )
# time.sleep( 3 )
 
# print( "180 deg" )
# pwm.set_servo_pulsewidth( servo, 2000 )
# time.sleep( 3 )
 
# # turning off servo
# pwm.set_PWM_dutycycle(servo, 0)
# pwm.set_PWM_frequency( servo, 0 )
