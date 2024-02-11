#!/usr/bin/env python3
import re
from typing import Union
import serial
import time

from .modem_source import ModemSource


AT_COMMAND_TIMEOUT_MS = 5000


def get_time_with_ms():
    return int(time.time() * 1000)


class ModemSensor(ModemSource):
    def __init__(self, p="/dev/ttyUSB4", b=115200):
        self.serial_connection = serial.Serial(port=p, baudrate=b)
        self.serial_connection.parity = serial.PARITY_NONE
        self.serial_connection.stopbits = serial.STOPBITS_ONE
        self.serial_connection.bytesize = serial.EIGHTBITS

    def get_response(self, timeout_ms=AT_COMMAND_TIMEOUT_MS):
        """Reads a response from the modem, reading is done over serial and response can 
        take up 100 milliseconds to be read.
        
        :param timeout_ms: Time out in milliseconds for the serial read function
        :return: Response read from the modem in string format UTF-8 decoded
        """
        time.sleep(0.1)
        response = self.serial_connection.read_all().decode("utf-8")

        return response

    def send_at_command(self, command):
        """Sends a AT command over the serial port connection created by the parent class.
        
        :param command: AT command to be sent to the modem
        """
        composed_message = str(command) + "\r"

        self.serial_connection.reset_input_buffer()
        self.serial_connection.write(composed_message.encode())
    
    def get_registration_status(self) -> Union[int, None]:
        """Sends the AT+CREG command to the modem to read the network registration status.
        
            0,0 Not registered, ME is not currently searching a new operator to register to
            0,1 Registered, home network
            0,2 Not registered, but ME is currently searching a new operator to register to
            0,3 Registration denied
            0,4 Unknown
            0,5 Registered, Roaming

            What we are hoping for is the 0,1 status indicating that we are registered on the network.

            :return: int 0-5 indicating the registration status
        """
        at_command = "AT+CREG?"
        self.send_at_command(at_command)

        response = self.get_response()
        if response:
            # Regexp is almost never a clean solution, retrives the last digit in a 0,x combination
            match = re.search(r"0,\d", response)
            if match:
                first_group = match.group(0)
                last_digit = first_group[-1]
                return int(last_digit)

        return None

    def get_received_signal_strength_indicator(self) -> Union[int, None]:
        """Sends the AT+CSQ command to the modem to read the received signal strength indicator.
            The received signal strength indicator is a value 0-31 where 0 is the worst possible signal
            strength and 31 is the best. Possible values are listed in this table

            0 - (-113) dBm or less
            1 - (-111) dBm
            2..30 - (-109)dBm..(-53)dBm / 2 dBm per step
            31 - (-51)dBm or greater
            99 - not known or not detectable
        
            :return: int 0-31 or 99 indicating signal strength
        """
        at_command = "AT+CSQ"
        self.send_at_command(at_command)

        response = self.get_response()
        if response:
            # Regexp is almost never a clean solution, retrives the last digit in a 0,x combination
            match = re.search(r"\d+,\d+", response)
            if match:
                first_group = match.group(0)
                last_number = first_group.split(",")[0]
                return int(last_number)
        return None


if __name__ == "__main__":
    modem = ModemSensor()
    reg_status = modem.get_registration_status()
    signal_strength = modem.get_received_signal_strength_indicator()

    print(f"Registration status: {reg_status}\nSignal strength: {signal_strength}")
