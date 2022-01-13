# Eel

## Prerequisites

- Ubuntu 20.04
- ROS2 Foxy
- Python 3

## Getting started

Get the code, build ROS and install python dependencies:

```
git clone <this repo>
cd path/to/project
colcon build --symlink-install
make install-py
```

Now you can either start everything in dev mode, or in production mode. Let's go with dev mode first.

### Start in dev mode

Terminal 1:

```
make virtual-serial
```

Terminal 2:

```
source source_me.sh
ros2 launch eel_bringup sim.launch.py
```

### Start in production mode

```
source source_me.sh
ros2 launch eel_bringup eel.launch.py
```

## Get started

In case you haven't already, do this:

1. Initialize virtual environment: `python3 -m venv .venv`
1. Activate python environment: `source source_me.sh`
1. Install python packages: `python -m pip install -r requirements.txt` or `make install-py`
1. Build ROS2 packages: `colcon build`
1. Source again, to make ROS2 packages available: `source source_me.sh`

Then start the Eel application with:

```
ros2 launch eel_bringup eel.launch.py
```

Or, start in simulation mode:

```
ros2 launch eel_bringup eel.launch.py simulate:=true
```

## How to create a simple Python package

1. `cd src`
1. `ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy`
1. Create a Python file: `touch my_py_pkg/my_py_pkg/my_first_node.py`
1. Paste contents from `templates/node_template.py`
1. Make the file executable: `chmod +x the_python_file.py`. Then you can execute by running `./the_python_file.py`
1. Add your node to `src/my_py_pkg/setup.py`
   ```python
   entry_points={
       'console_scripts': [
           'py_node = my_py_pkg.my_first_node:main' # <-- this line
       ],
   },
   ```
1. Go to the root of your workspace.
1. Build your package: `colcon build --packages-select my_py_pkg --symlink-install`
1. Source again: `source source_me.sh`
1. (Run your node manually: `./install/my_py_pkg/lib/my_py_pkg/py_node`)
1. Run your node through ROS2 commands: `ros2 run my_py_pkg py_node`

## Notes on running `imu`

NOTE: Maybe we can install this instead: `sudo pip3 install adafruit-circuitpython-bno055`

Plug in the BNO055 module to some pins (not sure which ones..? Possibly like this: https://learn.adafruit.com/bno055-absolute-orientation-sensor-with-raspberry-pi-and-beaglebone-black/hardware)

```
cd /some/path/outside/this/folder
git clone https://github.com/adafruit/Adafruit_CircuitPython_BNO055.git
cd /path/to/cloned-repo
sudo python3 setup.py install
```

Check that you have permission to run I2C stuff.

Open a Python shell: `python3`

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

Should `adafruit_bno055` be listed in package dependencies? No idea on how to handle dependencies that are resolved locally like this.

However, you should be able to run the `imu` node now:

```
cd /path/to/this/project
colcon build --packages-select rpi_test
source source_me.sh
ros2 run rpi_test imu
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
