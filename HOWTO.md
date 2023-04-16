# How to use the UI and Eel together

## Boot stuff

1. Start the Eel Raspberry PI.
1. Start a computer with Ubuntu installed.

## Connection

### Ethernet

1. Make sure you have a ethernet cable connected between the two.
1. Open `Wired network settings` on your computer:
   1. Top right corner.
   1. Click arrow.
   1. Click `Settings`.
   1. Click `Network`.
   1. Under `Wired`, click cog wheel.
1. Go to `IPv4` tab.
1. Select `Manual` under `IPv4 Method`.
1. In the first row under `Addresses`, enter:
   - Address: `192.168.0.100`
   - Netmask: `255.255.255.0`
1. Click `Apply` and close the `Network settings` dialog.
1. Open a terminal:
   - Use for example `Tabby`, either from the `Favorites` panel to the left, or
   - by searching for `Tabby` after pressing the `Windows` button on your keyboard.
1. Type `ssh eel` and press `enter`.
1. If you get something like
   ```
   ubuntu@ubuntu >
   ```
   you're good. If you get anything else, I can't help you :)

## Start applications

You will need to have three terminals running, which is why it's good to use `Tabby`, since you can see them all at once.

We will refer to the three terminals as:

- `Remote to RPi` (the one which you just connected to over SSH).
- `Ground control 1` (which will be running the user interface).
- `Ground control 2` (which will be running `ROS web bridge`, so that the user interace can communicate with ROS).

1. In the `Remote to RPi` terminal (which you just setup in the previous section), do the following:
   1. `cd eel`
   1. `git checkout main`
   1. `git pull`
   1. `source source_me.sh`
   1. `make build-sym`
   1. `make start-gunthix`
   1. You should now see some output saying that a few nodes have started. If you get any errors, I can't help you :)
1. In another terminal, let's say `Ground control 1`, do the following:
   1. `cd ~/code/ground-control`
   1. `make start-ros-ws`
   1. You should see something like `Rosbridge WebSocket server started...`.
1. In the third terminal, let's say `Ground control 2`, do the following:
   1. `cd ~/code/ground-control`
   1. `make web-dev`
   1. You should see something like `started server on...`.

At this point (assuming no errors), everything should be up and running, so you can start sending commands from the web user interface.

## Use the web user interface

1. Open a web browser.
1. Navigate to http://localhost:3000.

## Shut down

When you're done testing stuff, follow these steps to shut everything down properly:

1. In the `Ground control 1` terminal, press `Ctrl C`
1. In the `Ground control 2` terminal, press `Ctrl C`
1. In the `Remote to RPi` terminal, press `Ctrl C`
1. In the `Remote to RPi` terminal, run `sudo shutdown -h now`
1. Kill the power to the Eel.
1. Shut down your computer.
