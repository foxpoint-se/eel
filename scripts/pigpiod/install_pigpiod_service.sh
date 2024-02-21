#!/bin/bash

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

SERVICE_PATH=/etc/systemd/system/pigpiod.service

if [ -f "$SERVICE_PATH" ]; then
    echo "$SERVICE_PATH already exists, exiting."
    exit 1
fi

echo "[Unit]" > $SERVICE_PATH
echo "Description=Daemon required to control GPIO pins via pigpio" >> $SERVICE_PATH
echo "[Service]" >> $SERVICE_PATH
echo "ExecStart=/usr/local/bin/pigpiod" >> $SERVICE_PATH
echo "ExecStop=/bin/systemctl kill -s SIGKILL pigpiod" >> $SERVICE_PATH
echo "Type=forking" >> $SERVICE_PATH
echo "[Install]" >> $SERVICE_PATH
echo "WantedBy=multi-user.target" >> $SERVICE_PATH

echo "Created service file $SERVICE_PATH"
echo "Looks like this:"
echo
cat $SERVICE_PATH
echo

echo "Enabling service pigpiod"
systemctl enable pigpiod
echo
echo "Starting service pigpiod"
systemctl start pigpiod
echo
systemctl status pigpiod
echo
echo "Done!"
