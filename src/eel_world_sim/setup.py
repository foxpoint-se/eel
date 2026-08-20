from setuptools import find_packages, setup

package_name = "eel_world_sim"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "dearpygui>=1.11,<3"],
    zip_safe=True,
    maintainer="Foxpoint marinrobotik",
    maintainer_email="foxpoint.se@gmail.com",
    description="Simple fake-world simulation for local eel development.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "world_sim = eel_world_sim.world_sim_node:main",
            "world_sim_gui = eel_world_sim.world_sim_gui:main",
        ],
    },
)
