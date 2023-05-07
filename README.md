# Eel

## Notes

starta om datorn och lägg detta i en annan fil

meddelande kommer in: 0 - sätt tankarna till 0 - ingen compute loop

---

check target loop:

om inget target:
return

om target surface och tankarna tömda:
return

om at target:
return

DepthController
target = "SURFACE" | "TARGET" | "NO_TARGET"

// stäng av vid is_at_target
// stäng av vid is_at_surface
// -- skicka tillbaka None. då kan jag låta bli att skicka meddelanden till tankarna

get_next_levels (curr_front, curr_rear) -> front, rear
om inget target:
return None,None

set_new_target(target1, target2, pid1, pid2)

merge_pids -> front, rear

\_\_is_at_target

\_\_go_to_neutral

init:
depth_velocity_limit_mps = 0.05
pitch_velocity_limit_degps = 2.0

input:
current depth
current pitch

    depth_velocity
    pitch_velocity

    target depth
    target pitch

    current t1 level
    current t2 level

    compute_pitch

output
next t1 level
next t2 level

---

vid init:
skapa en handler med --> merger, pitch_computer, depth_computer, target_checker

vid msg:
set new target

varje vända:

uppdatera handlern med current state

      --> uppdatera target_checker med --> current depth, pitch, velocity1, velocity2

get next state

(bonus)
vi kan visa i ui:t boundaries och typ grönt/rött för hastighet genom att

handler.target_checker.lower_boundary
handler.target_checker.upper_boundary
handler.target_checker.

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

## Setting up Openvpn

### Installing OpenVPN Access Server using AWS

First create a AWS account

Find correct image launch instance -> Market place -> OpenVpn Access Server
In launch meny create and download your SSL keys .pem file

Continue set up by connecting to your new instance by running the following command in command line:

`ssh -i "{path_to_pem_file}" root@{ip-address}.compute.amazonaws.com`

The variables can be found under connect to instance -> ssh client

Follow the installation instruction by basically choosing default options on every choice given.

Exit current shell and ssh onto the server with the openvpnas user instead

`ssh -i "{path_to_pem_file}" openvpnas@{ip-address}.compute.amazonaws.com`

Change the password for the openvpn user using the following command:

`sudo passwd openvpn`

Now connect to the admin GUI via a browser on address:

`https://{ip-address}/admin`

### Configuring the OpenVPN Access Server

First enable inter-client communication so that connected clients can reach eachother over the VPN.

Settings -> Advanced VPN Settings -> Inter-Client Communication set to yes

Create a static IP Address Network, this will create a subnet under your VPN allowing for clients to recieve a static IP address on that network.

For exampel
Network Address: 172.27.240.0 Netmark Bits: 20

Create two users for the OpenVPN server

User Management -> User Permisssions

Enter new user name, under more settings set the password for the user and also assign a static IP address under the same subnet as provided under create static IP Address Network.

### Installing the openvpn3 client

Unsure that apt supports the https transport

`sudo apt-get install apt-transport-https`

Install the OpenVPN repository key used by the OpenVPN 3 Linux packages

`sudo curl -fsSL https://swupdate.openvpn.net/repos/openvpn-repo-pkg-key.pub | gpg --dearmor > /etc/apt/trusted.gpg.d/openvpn-repo-pkg-keyring.gpg`

NOTE: There might be a issue with writing directly to the trusted.gpg.d folder. If so create the file in a less restricted file area and then move the file.

Find out the release name of your ubuntu distro by running

`lsb_release -a`

Copy the codename field

Add the package to the sources list by running, replace DISTRO with the codename copied

`curl -fsSL https://swupdate.openvpn.net/community/openvpn3/repos/openvpn3-$DISTRO.list >/etc/apt/sources.list.d/openvpn3.list`

Install the client

`sudo apt-get install openvpn3`

### Configuring client

First download the client configuration by visiting the Access Server via the user GUI. In a browser open:

`https://{ip-address}`

Log in with your newly created user credentials. Then download the configuration by pressing the link under Available connection profiles.

To be able to connect to the server without prompting for the credentials we will need to create a log in file. This file should contain two rows, row 1 contains the user name and row 2 the password.
Create this file under /etc/openvpn3/auth.txt.

Now edit your downloaded configuration file with for exampel vi.

`vi {connection_profile_path}`

Find the row that says auth-user-pass and edit that line by adding the full path to the auth file

`auth-user-pass /etc/openvpn3/auth.txt`

Now import the configuration profile into the openvpn3 client

`openvpn3 config-import -c {path} --name {new_name_for_profile}`

NOTE: The name is only a covenience for you so that you do not need to provide the full configuration path when connecting.

Configuration should now be done, connect to the server by running

`openvpn3 session-start -c {new_name_for_profile}`

To gracefully terminate a session run

`openvpn3 session-manage -c {new_name_for_profile} --disconnect`

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

LAN_IP_ADDRESS="192.168.0.222"
NOF_TRYS=20
TRY_NR=1

echo "Looking for ip ${LAN_IP_ADDRESS} among interfaces"
while [ $TRY_NR -le $NOF_TRYS ]
do
  IF_CONFIG=$(ifconfig | grep $LAN_IP_ADDRESS)

  if [[ "$IF_CONFIG" == *"$LAN_IP_ADDRESS"*  ]]; then
    break
  fi

  ((TRY_NR++))
  sleep 1
done

if [ $TRY_NR -le $NOF_TRYS ]; then
  echo "Found address ${LAN_IP_ADDRESS} no need for modem"
  exit 0
fi

echo "Did not find address ${LAN_IP_ADDRESS} starting modem"
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
