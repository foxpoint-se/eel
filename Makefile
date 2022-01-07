install-py:
	python -m pip install -r requirements.txt

# Run to create a virtual serial communication instead of HC12 radio link.
# Send and listen to /tmp/virtual_serial_eel on Eel side, and
# /tmp/virtual_serial_connect on the other (where you run ground-control application)
serial-sim:
	socat -d -d pty,raw,echo=0,link=/tmp/virtual_serial_eel pty,raw,echo=0,link=/tmp/virtual_serial_connect
