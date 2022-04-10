import argparse
import logging
import serial
import time

logging.basicConfig(format='%(asctime)s %(message)s')

parser = argparse.ArgumentParser()
parser.add_argument("--baud", type=int, choices=[9600, 19200, 38400, 57600, 115200], default=19200)
parser.add_argument("--sleep", type=float, default=1.0)
parser.add_argument("-r", help="Configures the script to act as a receiver", action="store_true")
parser.add_argument("-s", help="Configures the script to act as a receiver", action="store_true")

args = parser.parse_args()

PORT = "/dev/ttyS0"
BAUD_RATE = args.baud
SLEEP_TIME = args.sleep

# RTSCTS - Request to send, Clear to send (Hardware handshaking) do we set this ourselves or is it handled?
# If we use rtscts I think we can keep timeout as 0, we then rely on non-blocking transmissions handled by the rtscts
# If that does not work at all we should use the timeout=1 approach
#hc_12 = serial.Serial(port=PORT, baudrate=BAUD_RATE, timeout=0, rtscts=1)


# Possibly modern equipment does not need the RTSCTS implementation at all, seems to be related to old modems
hc_12 = serial.Serial(port=PORT, baudrate=BAUD_RATE, timeout=1)
sample_msg = '{"n": {"c": {"lt": 59.0, "ln": 18.0}, "d": 100.0, "t": 5.0, "a": true}, "i": {"c": true, "s": 3, ' \
             '"g": 3, "a": 3, "m": 3, "h": 180.0}, "g": {"c": {"lt": 59.1, "ln": 18.1}}}\n'
msg_counter = 1

if args.s:
    while True:
        if hc_12.out_waiting():
            logging.debug(f"{hc_12.out_waiting}")
        msg_counter += 1
        hc_12.write(bytes(sample_msg, 'utf-8'))
        time.sleep(SLEEP_TIME)

if args.r:
    while True:
        if hc_12.inWaiting():
            logging.debug(f"{hc_12.inWaiting()} number of bytes in input buffer.")
        msg = hc_12.readline()
        logging.info(msg)
