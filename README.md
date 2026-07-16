# Eel

ROS 2 nodes and bringup for the AUV (simulation and production).

## Prerequisites

One of:

- Ubuntu 26.04 + ROS 2 Lyrical
- Ubuntu 24.04 + ROS 2 Jazzy
- Ubuntu 22.04 + ROS 2 Humble

Or skip host ROS and use Docker (see Getting started).

## Getting started

### Local development

Requires ROS 2 on the host (see Prerequisites).

**Once per machine** (after ROS is installed):

```bash
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update
```

**Per clone:**

```bash
git clone git@github.com:foxpoint-se/eel.git
cd eel
make install
source source_me.sh   # ROS + venv (+ install/ after the first build)
make build
make test
ros2 run eel imu --ros-args -p simulate:=true
```

`make install` sets up pip deps, rosdep, and `pi_ina226` (battery). Source `source_me.sh` in every new shell before `make build`, `make test`, or `ros2 run`. After sourcing, `make setup` re-runs install + build.

Enough for simulation. For hardware, see [Hardware bring-up](#hardware-bring-up) and the `Makefile`.

### Docker

From `docker/`:

```bash
cd docker
./build-image.sh jazzy   # or: humble, lyrical
```

Single node in simulation:

```bash
docker run --rm foxpoint/eel:jazzy ros2 run eel imu --ros-args -p simulate:=true
```

Full stacks: compose templates in `docker/` (e.g. `simulation-template.yml`, `alen-template.yml`).

## Hardware bring-up

### SPI (tank fill level)

On the Pi, the tank node reads ballast level over SPI (`MCP3208`). Without device permissions that fails. Run `make spidev-permissions`, then reboot.

### I2C addresses

| Address | Device |
| ------- | ------ |
| 28 | IMU (BNO055) |
| 40 | Battery / voltage (INA226) |

Detect devices: `make detect-i2c`.

### IMU

BNO055 over I2C. Needs I2C tools/permissions: `make install-i2c`, reboot, then `source source_me.sh && ros2 run eel imu`.

### GNSS

USB serial GPS. Pass the port (Ålen `/dev/ttyUSB1`, Tvålen `/dev/ttyUSB0`; also in compose templates). User should be in `dialout`.

```bash
ros2 run eel gnss --ros-args -p serial_port:=/dev/ttyUSB1
```

If the Pi serial console fights a UART GPS, see [docs/gnss-pi-uart-console.md](docs/gnss-pi-uart-console.md) (may be obsolete for USB-only GPS).

### Motor

Needs Pi GPIO access. If you see `Not running on an RPi`:

```bash
sudo apt install rpi.gpio-common
sudo adduser ${USER} dialout
sudo reboot
```

Then run without `simulate:=true`.

### Rudder

Needs `pigpiod` for servos. If you see connection errors to `localhost:8888`, run `make start-pigpio`, then `ros2 run eel rudder`.

### Modem (cellular)

The Pi needs internet at sea; use the Sixfab/Quectel 4G path. Install software + auto-connect service:

```bash
make install-modem
```

Vendor guide: [Sixfab QMI setup](https://docs.sixfab.com/page/qmi-interface-internet-connection-setup-using-sixfab-shield-hat).  
Wifi-then-modem fallback idea (not what the service does today): [docs/modem-wifi-fallback.md](docs/modem-wifi-fallback.md).

### Ethernet (rare)

Cable/UI setup (host `192.168.0.100`, Pi `192.168.0.101`): [How to use the UI and Eel together](https://github.com/foxpoint-se/promotion/blob/main/docs/src/content/docs/how-to-use-ui-and-eel-together.md) in the promotion docs. Day-to-day use is usually wifi/hotspot.

## More notes

Archived how-tos: [docs/](docs/README.md).
