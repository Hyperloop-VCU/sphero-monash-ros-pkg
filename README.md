# Sphero-monash-ros-pkg
  This repository was forked from an existing sphero driver for use in the HyperLabs ARP Workshops. All the machines used in the workshop are clean, so it is OK to not use a virtual environment and mess with the system python packages.
  It was changed to be a standard ros package and work with ros2 jazzy, and also offer additional functionality.

# Setup Instructions
1. Make a new workspace (skip if you want to use an existing workspace):
  *```mkdir -p sphero_ws/src```
2. clone this repository into the workspace src folder:
  * ```cd sphero_ws/src```
  * ```git clone https://github.com/Hyperloop-VCU/sphero-monash-ros-pkg.git```
3. Move back to workspace root:
   *```cd ..```
4. install ROS dependencies:
   *```rosdep install --from-paths src --ignore-src -r -y```
   * Note - if you get something like "run sudo rosdep init" or "run sudo rosdep update" when you run that command, follow the instructions printed in the error, then try running the command again. It should print "all rosdeps installed successfully" when it works.
5. install python dependencies:
  *```pip install sphero-unsw==0.1.11 --break-system-packages```
  *```pip install gymnasium>=0.29 --break-system-packages```
  *Note - gymnasium installation may fail because of a conflict with typing_extensions, an apt-installed python package that pip cannot remove. Removing this package manually seemed to fix it; run    *```sudo apt remove python3-typing-extensions``` and try the gymnasium installation again.


