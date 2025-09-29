SHELL = /bin/bash

.PHONY: help

help:
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

.DEFAULT_GOAL := help

clean: 		## clean workspace
	rm -rf .venv

# mkdir build install log
ros-clean:		## clean ROS build
	rm -rf build install log

install-py:
	python3 -m pip install -U --user -e src/eel

install-ros:
	rosdep install --from-paths src --ignore-src -r -y

install-depth-sensor:
	wget -nc https://github.com/bluerobotics/ms5837-python/archive/refs/heads/master.zip -O depth-lib.zip
	unzip -o depth-lib.zip
	python3 -m pip install -U --user ./ms5837-python-master/
	rm -rf ms5837-python-master depth-lib.zip

install-voltage-sensor:
	wget -nc https://github.com/e71828/pi_ina226/archive/refs/heads/main.zip -O voltage-lib.zip
	unzip -o voltage-lib.zip
	python3 -m pip install -U --user ./pi_ina226-main/
	rm -rf pi_ina226-main voltage-lib.zip

install-all: install-py install-ros install-depth-sensor install-voltage-sensor

start-pigpio:		## start pigpio
	sudo pigpiod

stop-pigpio:		## stop pigpio
	sudo killall pigpiod

install-i2c:		## install i2c stuff
	( \
		sudo apt update; \
		sudo apt upgrade -y; \
		sudo apt install -y i2c-tools; \
		sudo usermod -a -G i2c ubuntu; \
		echo "now restart your rpi"; \
	)

.PHONY: spidev-permissions
spidev-permissions:		## setup spidev permissions
	sudo ./scripts/spidev/spidev-permissions.sh

install-modem:		## Steps on how to install modem both software and service file for auto connecting
	@( \
		echo "Run these commands:"; \
		echo "cd ./scripts/modem"; \
		echo "sudo ./install_modem_software.sh"; \
		echo ""; \
		echo "Wait for reboot, then run"; \
		echo "cd ./scripts/modem"; \
		echo "sudo ./install_modem_service.sh"; \
	)

detect-i2c:		## detect i2c
	sudo i2cdetect -y 1

test-py:		## run tests specific to eel only
	source source_me.sh && pytest src/eel/test/eel

# NOTE: can't get colcon test to work with virtual environment??
# That's why we're only running pytest locally. However, colcon test
# works in Docker environment, since the packages are installed globally
# in the container
test:
	colcon test && colcon test-result --verbose
