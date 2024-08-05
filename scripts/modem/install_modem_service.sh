#!/bin/bash

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

echo "Generating modem connection script and service file"

FILE_PATH=/usr/local/bin/modem_connect.sh
SERVICE_PATH=/etc/systemd/system/modem.service
APN="4g.tele2.se"

if [ -f "$SERVICE_PATH" ]; then
    echo "$SERVICE_PATH exists, no need to create modem connect script, exiting."
    exit 1
fi

if [ -f "$FILE_PATH" ]; then
    echo "$FILE_PATH exists, no need to create modem connect script, exiting."
    exit 1
fi

echo "[Unit]" > $SERVICE_PATH
echo "Description=Service to connect to 4g network via modem" >> $SERVICE_PATH
echo "[Service]" >> $SERVICE_PATH
echo "Type=simple" >> $SERVICE_PATH
echo "ExecStart=/bin/bash $FILE_PATH" >> $SERVICE_PATH
echo "[Install]" >> $SERVICE_PATH
echo "WantedBy=multi-user.target" >> $SERVICE_PATH

systemctl enable modem
systemctl start modem

echo "#!/bin/bash" > $FILE_PATH
echo "/opt/qmi_files/quectel-CM/quectel-CM -s $APN" >> $FILE_PATH

echo "DONE!"
cat $FILE_PATH
cat $SERVICE_PATH
