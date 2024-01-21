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
            "navigation = eel.navigation.navigation_node:main",
            # "gnss = eel.gnss.gnss_node:main",
            "imu = eel.imu.imu_node:main",
            # "rudder = eel.rudder.rudder_node:main",
            "motor = eel.motor.motor_node:main",
            # "tank = eel.tank.tank_node:main",
            # "pressure = eel.pressure.pressure_node:main",
            # "depth_control = eel.depth_control.depth_control_node:main",
            # "battery = eel.battery.battery_node:main",
            # "depth_control_rudder = eel.depth_control.depth_control_node_rudder:main",
        ],
    },
)
