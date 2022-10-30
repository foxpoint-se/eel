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

## Notes on networking when using ethernet cable

We set `dhcp4` to false in both cases, since we want a specific address.

Connect through SSH to `192.168.0.101`.

- **For 1:1 connection (RPi to computer):** Uncomment `gateway4`. Should be IP for computer (host). Also, set a static IP on your computer, which will be the host that RPi connects to. Should be the IP of the `gateway4` (`192.168.0.100`).

- **For 1:m connection (RPi to router).** `gateway4` should be commented out (disabled).

Edit `/etc/netplan/50-cloud-init.yaml` and run `sudo netplan apply`.

**Example**

```
# This file is generated from information provided by the datasource.  Changes
# to it will not persist across an instance reboot.  To disable cloud-init's
# network configuration capabilities, write a file
# /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg with the following:
# network: {config: disabled}
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: false
      optional: true
      addresses:
        - 192.168.0.101/24
      #gateway4: 192.168.0.100   # <---- comment or uncomment
  wifis:
    wlan0:
      dhcp4: true
      optional: true
      access-points:
        mywifiname:
          password: mypassword
```
