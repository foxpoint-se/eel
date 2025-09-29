SHELL = /bin/bash

.PHONY: help

help:
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

.DEFAULT_GOAL := help

clean:		## Clean workspace
	rm -rf build install log

.PHONY: build
build:		## Colcon build but with python venv
	python3 -m colcon build --symlink-install

VENV_DIR := venv
.PHONY: venv
venv:
	@if [ ! -d $(VENV_DIR) ]; then \
		python3 -m venv $(VENV_DIR) --system-site-packages; \
		touch $(VENV_DIR)/COLCON_IGNORE; \
	fi

install-py: venv
	source $(VENV_DIR)/bin/activate; python3 -m pip install -U src/eel

install-rosdep:
	rosdep install --from-paths src --ignore-src -r -y

install-depth-sensor: venv
	wget -nc https://github.com/bluerobotics/ms5837-python/archive/refs/heads/master.zip -O depth-lib.zip
	unzip -o depth-lib.zip
	source $(VENV_DIR)/bin/activate; python3 -m pip install -U ./ms5837-python-master/
	rm -rf ms5837-python-master depth-lib.zip

install-voltage-sensor: venv
	wget -nc https://github.com/e71828/pi_ina226/archive/refs/heads/main.zip -O voltage-lib.zip
	unzip -o voltage-lib.zip
	source $(VENV_DIR)/bin/activate; python3 -m pip install -U ./pi_ina226-main/
	rm -rf pi_ina226-main voltage-lib.zip

install: install-py install-rosdep install-depth-sensor install-voltage-sensor		## Install all dependencies

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

test:		## Run colcon tests
	python3 -m colcon test; python3 -m colcon test-result --verbose
