from setuptools import find_packages, setup

package_name = "eel_world_sim"

setup(
    name=package_name,
    version="1.2.12",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Foxpoint marinrobotik",
    maintainer_email="foxpoint.se@gmail.com",
    description="Simple fake-world simulation for local eel development.",
    license="MIT",
)
