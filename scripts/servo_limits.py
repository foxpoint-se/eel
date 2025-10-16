import pigpio
import time

# Run like this: 
# source source_me.sh
# python servo_limits.py

# SERVO_PIN = 13  # Change to your servo's GPIO pin
SERVO_PIN = 19  # Change to your servo's GPIO pin
MIN_PULSE = 800   # 500 µs is usually the default
MAX_PULSE = 2200  # 2500 µs is usually the default
STEP = 500         # Step size in µs
DELAY = 5         # Seconds to wait at each position

TOTAL = MIN_PULSE + MAX_PULSE
CENTER_PULSE = TOTAL // 2

pi = pigpio.pi()
if not pi.connected:
    print("Could not connect to pigpio daemon!")
    exit(1)

try:
    print("Sweeping servo from min to max pulse width...")

    time.sleep(5)
    
    pw = MIN_PULSE
    while pw <= MAX_PULSE:
        print(f"Pulse width: {pw} µs")
        pi.set_servo_pulsewidth(SERVO_PIN, pw)
        time.sleep(DELAY)
        pw += STEP

    # Catching the upper limit, in case the loop never goes there
    pw = MAX_PULSE
    print(f"Pulse width: {pw} µs")
    pi.set_servo_pulsewidth(SERVO_PIN, pw)
    time.sleep(DELAY)
    
    print("Sweep complete. Now returning to center.", CENTER_PULSE, "µs")
    pi.set_servo_pulsewidth(SERVO_PIN, CENTER_PULSE)
    time.sleep(2)
finally:
    pi.set_servo_pulsewidth(SERVO_PIN, 0)  # Turn off servo
    pi.stop()