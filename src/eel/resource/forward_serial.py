import serial

if __name__ == "__main__":
    

    port = "/dev/ttyUSB0"
    coms = serial.Serial(port, baudrate=115200)

    coms.write(msg)
