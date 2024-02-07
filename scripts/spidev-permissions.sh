#!/bin/bash

user=ubuntu
spi_rules_file_path=/etc/udev/rules.d/90-gpio-spi.rules

line1="KERNEL==\"spidev0.0\", OWNER=\"root\", GROUP=\"spi\""
line2="KERNEL==\"spidev0.1\", OWNER=\"root\", GROUP=\"spi\""

echo $line1 > $spi_rules_file_path
echo $line2 >> $spi_rules_file_path

echo "Wrote file $spi_rules_file_path"
cat $spi_rules_file_path

groupadd -f --system spi
usermod -a -G spi $user

echo "Created group spi and added user $user"
echo "All groups:"
groups $user
echo
echo "Remember to reboot!"
echo
echo "Done!"
