#!/bin/bash

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

echo "Installing modem software from Sixfab"

wget https://raw.githubusercontent.com/sixfab/Sixfab_QMI_Installer/main/qmi_install.sh

chmod +x qmi_install.sh

./qmi_install.sh

ln -sf /opt/qmi_files/quectel-CM/quectel-CM /usr/bin/quectel-CM
