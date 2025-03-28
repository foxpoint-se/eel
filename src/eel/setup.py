from setuptools import find_packages
from setuptools import setup

package_name = "eel"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bulingen",
    maintainer_email="adamlecorney@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dive = eel.dive.dive_action_server:main",
            "dive_client = eel.dive.dive_action_client:main",
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
            "led_control = eel.led_control.led_node:main"
        ],
    },
)
