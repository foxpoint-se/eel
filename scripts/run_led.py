from time import sleep

import RPi.GPIO as GPIO

LED_ON_LEVEL = GPIO.HIGH
LED_OFF_LEVEL = GPIO.LOW
LED_PIN = 16

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)

while(1):
    GPIO.output(LED_PIN, LED_ON_LEVEL)
    sleep(1.0)
    GPIO.output(LED_PIN, LED_OFF_LEVEL)
    sleep(1.0)
