SHELL = /bin/bash

.PHONY: help

help:
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

.DEFAULT_GOAL := help

clean: 		## clean workspace
	rm -rf .venv

ros-clean:		## clean ROS build
	rm -rf build install log
	mkdir build install log

install-py:		## setup venv and install py dependencies
	( \
		python3 -m venv .venv; \
		touch .venv/COLCON_IGNORE; \
       	source .venv/bin/activate; \
       	pip install wheel; \
       	python -m pip install -r requirements.txt; \
    )

install: install-py		## install everything (not really, but should be)

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

install-depth-sensor:		## install stuff needed for depth senson
	( \
		wget https://github.com/bluerobotics/ms5837-python/archive/refs/heads/master.zip -O depth-lib.zip; \
		unzip depth-lib.zip; \
		touch ms5837-python-master/COLCON_IGNORE; \
		source .venv/bin/activate; \
		pip install ms5837-python-master/; \
	)

install-pigpiod:		## Steps on how to install pigpiod software and service
	@( \
		echo "Run these commands:"; \
		echo "cd ./scripts/pigpiod"; \
		echo "sudo ./install_pigpiod_software.sh"; \
		echo "sudo ./install_pigpiod_service.sh"; \
	)

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

install-voltage-sensor:		## install stuff needed for depth senson
	( \
		wget https://github.com/e71828/pi_ina226/archive/refs/heads/main.zip -O voltage-lib.zip; \
		unzip voltage-lib.zip; \
		touch pi_ina226-main/COLCON_IGNORE; \
		source .venv/bin/activate; \
		pip install pi_ina226-main/; \
	)

detect-i2c:		## detect i2c
	sudo i2cdetect -y 1

test-py:		## run tests specific to eel only
	source source_me.sh && pytest src/eel/test/eel

# NOTE: can't get colcon test to work with virtual environment??
# That's why we're only running pytest locally. However, colcon test
# works in Docker environment, since the packages are installed globally
# in the container
test: test-py		## run all tests
