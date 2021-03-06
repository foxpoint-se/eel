# Eel

## Prerequisites

- Ubuntu 20.04
- ROS2 Foxy
- Python 3
- Colcon (`sudo apt install python3-colcon-common-extensions`)

## Getting started

Get the code, build ROS and install python dependencies:

```
git clone <this repo>
cd path/to/project
make install-py
source source_me.sh
make build-sym
```

This will be enough when running in simulation mode (without actual hardware). But when running in production mode (with actual hardware), you will have to install a few other things as well:

```
make install-depth-sensor
make install-pigpio
```

### Start in simulation mode

Terminal 1 (only required for navigation simulation):

```
make virtual-serial
```

Terminal 2:

You can either start depth simulation or navigation simulation.

```
source source_me.sh
make sim-nav
```

OR:

```
source source_me.sh
make sim-depth
```

### Start in production mode

```
make start-pigpio
source source_me.sh
make start
```

## Notes on running `imu` node

Plug in the BNO055 module to some pins (not sure which ones..? Possibly like this: https://learn.adafruit.com/bno055-absolute-orientation-sensor-with-raspberry-pi-and-beaglebone-black/hardware)

Check that you have permission to run I2C stuff.

Open a Python shell:

```python
import adafruit_bno055
import board
i2c = board.I2C()
```

This will probably give you a permission error. Do this to resolve:

```
sudo apt update
sudo apt upgrade -y
sudo apt install -y i2c-tools
sudo usermod -a -G i2c ubuntu // if ubuntu is your user
```

You should be able to run the `imu` node now:

```
source source_me.sh
ros2 run eel imu
```

## Notes on running `gnss` node

Plug in NEO 6M module. Possibly like this: https://medium.com/@kekreaditya/interfacing-u-blox-neo-6m-gps-module-with-raspberry-pi-1df39f9f2eba

See if it works:

```python
import serial
ser = serial.Serial("/dev/ttyUSB0")
print(ser.readline())
```

This will probably not work, since another service is using the same port. Follow these steps to solve (stolen from https://askubuntu.com/a/1338744). Or maybe do as the notes say in `config.txt` and edit the `usercfg.txt` file instead, but I haven't tried that.

1. Backup `config.txt` and `cmdline.txt` files:
   ```bash
   sudo cp -pr /boot/firmware/cmdline.txt /boot/firmware/cmdline.txt-orig
   sudo cp -pr /boot/firmware/config.txt /boot/firmware/config.txt-orig
   ```
1. Edit `/boot/firmware/config.txt` to comment out the `enable_uart=1`

   ```
   #enable_uart=1

   cmdline=cmdline.txt
   ```

1. Remove the console setting `console=serial0,115200` from `/boot/firmware/cmdline.txt`
1. Disable the Serial Service which used the miniUART
   ```
   sudo systemctl stop serial-getty@ttyS0.service
   sudo systemctl disable serial-getty@ttyS0.service
   sudo systemctl mask serial-getty@ttyS0.service
   ```
1. Add the user which will use the miniUART to tty and dialout group
   ```
   sudo adduser ${USER} tty
   sudo adduser ${USER} dialout
   ```
1. Reboot
