# GNSS / Pi UART console conflicts

> Archived from the eel README. **May be obsolete** if the GPS is USB-only (`/dev/ttyUSB*`). Kept for when serial console and a UART GPS fight over the same port.

When opening a serial GPS fails because another service holds the port, these steps were used (adapted from [Ask Ubuntu](https://askubuntu.com/a/1338744)). Editing `usercfg.txt` as mentioned in some Pi docs was not verified.

1. Backup firmware configs:

```bash
sudo cp -pr /boot/firmware/cmdline.txt /boot/firmware/cmdline.txt-orig
sudo cp -pr /boot/firmware/config.txt /boot/firmware/config.txt-orig
```

2. In `/boot/firmware/config.txt`, comment out `enable_uart=1` (and keep `cmdline=cmdline.txt` as needed).

3. Remove `console=serial0,115200` from `/boot/firmware/cmdline.txt`.

4. Disable the serial getty on miniUART:

```bash
sudo systemctl stop serial-getty@ttyS0.service
sudo systemctl disable serial-getty@ttyS0.service
sudo systemctl mask serial-getty@ttyS0.service
```

5. Add the user to `tty` and `dialout`:

```bash
sudo adduser ${USER} tty
sudo adduser ${USER} dialout
```

6. Reboot.

Current boats typically pass the GPS device via `serial_port` (Ålen `/dev/ttyUSB1`, Tvålen `/dev/ttyUSB0`) — see the main README and compose templates.
