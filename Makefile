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
