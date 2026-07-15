SHELL = /bin/bash

.PHONY: help

help:
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

.DEFAULT_GOAL := help

# Pin pi_ina226 (not on PyPI) so local + Docker installs stay reproducible.
PI_INA226_GIT := git+https://github.com/e71828/pi_ina226.git@ccbaa21c1439f0c9728ef838162f4a18638e21dc

.PHONY: check-sourced
check-sourced:
	@if [ -z "$$ROS_DISTRO" ] || ! command -v ros2 >/dev/null 2>&1; then \
		echo "ROS is not sourced (need ROS_DISTRO and ros2 on PATH)."; \
		echo "  First time:  make install && source source_me.sh && make build"; \
		echo "  After that:  source source_me.sh  (then make build / make test / ros2 run)"; \
		exit 1; \
	fi; \
	if [ -z "$$VIRTUAL_ENV" ]; then \
		echo "Python venv is not active. Run: source source_me.sh"; \
		exit 1; \
	fi

clean:		## Clean workspace
	rm -rf build install log

.PHONY: build
build: check-sourced		## Colcon build but with python venv
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
	@if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
		echo "rosdep is not initialized. Run: sudo rosdep init"; \
		exit 1; \
	fi; \
	export ROS_DISTRO=$${ROS_DISTRO:-$$(ls /opt/ros | sort | tail -n 1)}; \
	rosdep update; \
	rosdep install --from-paths src --ignore-src -r -y

install: install-py install-rosdep install-depth-sensor install-voltage-sensor		## Install all dependencies

.PHONY: setup
setup: check-sourced		## Install deps and build (source source_me.sh first)
	$(MAKE) install
	$(MAKE) build

install-depth-sensor: venv
	wget -nc https://github.com/bluerobotics/ms5837-python/archive/refs/heads/master.zip -O depth-lib.zip
	unzip -o depth-lib.zip
	source $(VENV_DIR)/bin/activate; python3 -m pip install -U ./ms5837-python-master/
	rm -rf ms5837-python-master depth-lib.zip

install-voltage-sensor: venv
	source $(VENV_DIR)/bin/activate; python3 -m pip install -U "$(PI_INA226_GIT)"

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

test: check-sourced		## Run colcon tests
	python3 -m colcon test --python-testing pytest; python3 -m colcon test-result --verbose
