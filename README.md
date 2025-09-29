# Eel

## Prerequisites

- Either:
  - Ubuntu 24.04 with ROS2 Jazzy
  - OR: Ubuntu 22.04 with ROS2 Humble

## Getting started

Get the code, install deps, build, run tests and run a node:

```bash
git clone <this repo>
cd path/to/project
make install-all
source source_me.sh
colcon build --symlink-install
make test
ros2 run eel imu --ros-args -p simulate:=true
```

This will be enough when running in simulation mode (without actual hardware). But when running in production mode (with actual hardware), you will have to install a few other things as well.

Check the `Makefile` for reference.

## Enable SPI access on Ubuntu

Create /etc/udev/rules.d/90-gpio-spi.rules with:

```
KERNEL=="spidev0.0", OWNER="root", GROUP="spi"
KERNEL=="spidev0.1", OWNER="root", GROUP="spi"
```

Create the group itself and assign it to an existing user "ubuntu":

```
sudo groupadd -f --system spi
sudo usermod -a -G spi ubuntu
```

Restart

sudo shutdown -r now

## I2C addresses

| Address | Device                          | Comment                        |
| ------- | ------------------------------- | ------------------------------ |
| 28      | IMU (BNO055)                    |                                |
| 29      | Front: Distance sensor          | Will be moved to 1d on startup |
| 29      | Rear: Distance sensor           | Will be moved to 16 on startup |
| 40      | Voltage sensor                  |                                |
| 48      | AD converter for potentiometers |                                |
| 76      | Pressure sensor                 |                                |

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

Reboot. You should be able to run the `imu` node now:

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

### GNSS serial ports

- On Ålen we use `/dev/ttyUSB1`.
- On Tvålen we use `/dev/ttyUSB0`.

Can't remember why this is the case.

## Notes on running `motor` node

`GPIO.setup()` will probably raise a runtime error saying `Not running on an RPi`. Do this to resolve:

```
sudo apt install rpi.gpio-common
sudo adduser ${USER} dialout
sudo reboot
```

## Notes on running `rudder` node

Rudder node can raise `OSError` saying `failed to connect to localhost:8888`. This means that you haven't started `pigpiod`. Resolve by running:

```
make start-pigpio
```

## Notes on networking when using ethernet cable

We set `dhcp4` to false in both cases, since we want a specific address.

Connect through SSH to `192.168.0.101`.

- **For 1:1 connection (RPi to computer):** Set a static IP on your computer, which will be the host that RPi connects to. Should be the IP of the `gateway4` (`192.168.0.100`). If it doesn't work: try setting the netmask on your machine to `255.255.255.0`.

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

## Installing modem

Installation scripts are provided by the provider and the guide can be found under:

https://docs.sixfab.com/page/qmi-interface-internet-connection-setup-using-sixfab-shield-hat

Connection script can then be found under /opt/qmi_files/quectel-CM
Run the connection script with

`sudo ./quectel-CM -s [YOUR APN]`

For easier access it is suggested to do a symlink into /usr/bin so that it is added to path

`ln /opt/qmi_files/quectel-CM/quectel-CM /urs/bin`

## Scripts and service files for auto modem/vpn usage

In a modem connect script we can search for the expected ipv4 address that we would get from a wifi connection.
If we do not find that address under ifconfig after 20 seconds then we start the modem.

```#!/bin/bash
quectel-CM -s 4g.tele2.se
```

Like wise for openvpn connections we could wait for the wwan0 interface (interface provided by modem) to show up under ifconfig.

```#!/bin/bash

MODEM_INTERFACE="wwan0"
NOF_TRYS=20
TRY_NR=1

echo "Waiting 20 seconds for modem to verify ip address initiate connection"
sleep 20

echo "Starting to scan for ${MODEM_INTERFACE} interface"
while [ $TRY_NR -le $NOF_TRYS ]
do
  IF_CONFIG=$(ifconfig | grep $MODEM_INTERFACE)

  if [[ "$IF_CONFIG" == *"$MODEM_INTERFACE"*  ]]; then
    break
  fi

  ((TRY_NR++))
  sleep 1
done

if [ $TRY_NR -ge $NOF_TRYS ]; then
  echo "Did not find interface ${MODEM_INTERFACE} will not initiate openvpn tunneling. Use local ip address"
  exit 0
fi

echo "Found interface ${MODEM_INTERFACE} initiating opnvpn tunneling"
openvpn3 session-start --config /home/ubuntu/pi-user.ovpn

while true
do
  openvpn3 session-stats --config /home/ubuntu/pi-user.ovpn
  sleep 15
done

```

Our systemd service file would want to know how to dissconnect from openvpn3 on shutdown. We create a script for that also.

```#!/bin/bash

openvpn3 session-manage --config /home/ubuntu/pi-user.ovpn --disconnect
```

We can then create two systemd service files one for modem connection and one for vpn connection. Start off with the modem service file.

`sudo touch modem.service && sudo vi modem.service`

```
[Unit]
Description=Service to connect to 4g network via modem

[Service]
Type=simple
ExecStart=/bin/bash /usr/bin/modem_connect

[Install]
WantedBy=multi-user.target

```

Move the file to the systemd service files
`sudo cp modem.service /etc/systemd/system`

Change the permissions for systemd to use the service
`sudo chmod 644 /etc/systemd/system/modem.service`

And then enable the service to run on boot
`sudo systemctl enable modem`

Now create a service file for the openvpn connection.

`sudo touch openvpn.service && sudo vi openvpn.service`

Add the following to the file

```[Unit]
Description=A service to automatically connect to openvpn with a given config
Wants=modem.service
After=network.target modem.service

[Service]
type=forking
ExecStart=/bin/bash /usr/bin/openvpn_connect
ExecStop=/bin/bash /usr/bin/openvpn_disconnect

[Install]
WantedBy=multi-user.target
```

Now do the same steps as for modem service with move, chmod and enable.
