from time import sleep

import RPi.GPIO as GPIO

LED_ON_LEVEL = GPIO.HIGH
LED_OFF_LEVEL = GPIO.LOW


class LEDControl:
    def __init__(self, led_pin=16) -> None:
        self.led_pin = led_pin
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.led_pin, GPIO.OUT)
    
    def sequence(self, nof_pulses=3, pulse_time=0.1):
        for _ in range(nof_pulses):
            self.on()
            sleep(pulse_time)
            self.off()

    def on(self):
        GPIO.output(self.led_pin, LED_ON_LEVEL)
    
    def off(self):
        GPIO.output(self.led_pin, LED_OFF_LEVEL)


if __name__ == "__main__":
    led_controller = LEDControl()

    while(1):
        sleep(1.0)
        led_controller.on()
        sleep(1.0)
        led_controller.off()
    #sleep(1.0)
    #led_controller.sequence(nof_pulses=5, pulse_time=0.1)
