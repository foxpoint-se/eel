from setuptools import find_packages, setup

package_name = "eel"

setup(
    name=package_name,
    version="1.2.12",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "pynmea2>=1.19.0",
        "pyserial>=3.5",
        "geopy>=2.4.0,<3",
        "awsiotsdk>=1.30.0,<2",
        "requests>=2.28,<3",
    ],
    extras_require={
        "pi": [
            "gpiozero>=2.0,<3",
            "pigpio>=1.78",
            "RPi.GPIO>=0.7.1",
            "spidev>=3.6",
            "adafruit-circuitpython-bno055>=5.4.22,<6",
        ],
        "dev": [
            "mypy>=2.3,<3",
            "pytest>=8.0",
            "ruff>=0.9,<1",
            "types-pyserial>=3.5.0",
            "types-requests",
        ],
    },
    zip_safe=True,
    maintainer="Foxpoint marinrobotik",
    maintainer_email="foxpoint.se@gmail.com",
    description="ROS 2 runtime nodes for the Eel AUV (simulation and Pi).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "dive = eel.dive.dive_action_server:main",
            "navigate = eel.navigation.navigation_action_server:main",
            "navigate_client = eel.navigation.navigation_action_client:main",
            "gnss = eel.gnss.gnss_node:main",
            "imu = eel.imu.imu_node:main",
            "rudder = eel.rudder.rudder_node:main",
            "motor = eel.motor.motor_node:main",
            "tank = eel.tank.tank_node:main",
            "pressure = eel.pressure.pressure_node:main",
            "depth_control = eel.depth_control.depth_control_node:main",
            "depth_control_rudder = eel.depth_control.depth_control_node_rudder:main",
            "battery = eel.battery.battery_node:main",
            "mqtt_bridge = eel.mqtt_bridge.node:main",
            "modem = eel.modem.modem_node:main",
            "localization = eel.localization.localization:main",
            "data_logger = eel.data_logger.data_logger_node:main",
            "leakage = eel.leakage.leakage_node:main",
            "led_control = eel.led_control.led_node:main",
        ],
    },
)
