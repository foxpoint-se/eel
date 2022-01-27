#!/usr/bin/python3
# Script to be put on device with the modem, forwards serial commands, i.e python forward_serial.py AT

import logging
import serial
import sys

logger = logging.getLogger(sys.argv[0])
logger.setLevel(logging.INFO)
ch = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
ch.setFormatter(formatter)
logger.addHandler(ch)

port = "/dev/ttyUSB3"
ser = serial.Serial(port, baudrate=115200, timeout=1)


if __name__ == "__main__":

    for arg in sys.argv[1:]:
        msg = f"{arg}\r".encode("utf-8")
        logger.info(f"Sending command: {msg}")
        ser.write(msg)
        resp = ser.read(128)
        logger.info(f"Got response {resp.decode()}")
        ser.close()
