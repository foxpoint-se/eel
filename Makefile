SHELL = /bin/bash

.PHONY: help

help:
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

.DEFAULT_GOAL := help

install-py:		## setup venv and install py dependencies
	( \
		python3 -m venv .venv; \
		touch .venv/COLCON_IGNORE; \
       	source .venv/bin/activate; \
       	python -m pip install -r requirements.txt; \
    )

# Run to create a virtual serial communication instead of HC12 radio link.
# Send and listen to /tmp/virtual_serial_eel on Eel side, and
# /tmp/virtual_serial_connect on the other (where you run ground-control application)
virtual-serial:		## /tmp/virtual_serial_eel <-> /tmp/virtual_serial_connect
	socat -d -d pty,raw,echo=0,link=/tmp/virtual_serial_eel pty,raw,echo=0,link=/tmp/virtual_serial_connect

start-pigpio:		## start pigpio
	sudo pigpiod

stop-pigpio:		## stop pigpio
	sudo killall pigpiod

# Stolen from https://abyz.me.uk/rpi/pigpio/download.html 
install-pigpio:		## install pigpio in this folder
	( \
		wget https://github.com/joan2937/pigpio/archive/master.zip; \
		unzip master.zip; \
		cd pigpio-master; \
		make; \
		sudo make install; \
	)

install-i2c:		## install i2c stuff
	( \
		sudo apt update; \
		sudo apt upgrade -y; \
		sudo apt install -y i2c-tools; \
		sudo usermod -a -G i2c ubuntu; \
		echo "now restart your rpi"; \
	)
