# Ålen

This repo contains everything to run Ålen.

## Get started

In case you haven't already, do this:

1. Initialize virtual environment: `python3 -m venv .venv`
1. Activate python environment: `source source_me.sh`
1. Install python packages: `python -m pip install -r requirements.txt`
1. Build ROS2 packages: `colcon build`
1. Source again, to make ROS2 packages available: `source source_me.sh`


## How to create a simple Python package

1. `cd src`
1. `ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy`
1. Create a Python file: `touch my_py_pkg/my_py_pkg/my_first_node.py`
1. Paste contents from `templates/node_template.py`
1. Make the file executable: `chmod +x the_python_file.py`. Then you can execute by running `./the_python_file.py`
1. Add your node to `src/my_py_pkg/setup.py`
   ```python
   entry_points={
       'console_scripts': [
           'py_node = my_py_pkg.my_first_node:main' # <-- this line
       ],
   },
   ```
1. Go to the root of your workspace.
1. Build your package: `colcon build --packages-select my_py_pkg --symlink-install`
1. Source again: `source source_me.sh`
1. (Run your node manually: `./install/my_py_pkg/lib/my_py_pkg/py_node`)
1. Run your node through ROS2 commands: `ros2 run my_py_pkg py_node`
