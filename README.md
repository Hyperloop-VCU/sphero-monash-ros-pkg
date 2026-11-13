# Sphero Monash ROS Package

This repository was forked from an existing Sphero driver for use in the **HyperLabs ARP Workshops**. 

> **Note:** Because all machines used in the workshop are clean environments, it is safe to bypass a virtual environment and install dependencies directly into the system Python packages.

This package has been updated to function as a standard ROS package compatible with **ROS 2 Jazzy**, along with offering additional functionality.

---

## Setup Instructions

### 1. Create a Workspace
Create a new ROS workspace *(skip this step if you want to use an existing workspace)*:
```bash
mkdir -p sphero_ws/src
```

### 2. Clone the Repository
Navigate into your workspace's `src` folder and clone the repository:
```bash
cd sphero_ws/src
git clone https://github.com/Hyperloop-VCU/sphero-monash-ros-pkg.git
```

### 3. Return to the Workspace Root
Move back up to the root directory of your workspace:
```bash
cd ..
```

### 4. Install ROS Dependencies
Run `rosdep` to install all required ROS packages:
```bash
rosdep install --from-paths src --ignore-src -r -y
```
> **Troubleshooting:** If you receive an error prompting you to run `sudo rosdep init` or `rosdep update`, follow the instructions printed in the terminal and run the `rosdep install` command again. When successful, it will output: `all rosdeps installed successfully`.

### 5. Install Python Dependencies
Install the required Python libraries using `pip`:
```bash
pip install sphero-unsw==0.1.11 --break-system-packages
pip install "gymnasium>=0.29" --break-system-packages
```
> **Troubleshooting:** The `gymnasium` installation may fail due to a version conflict with `typing_extensions` (an `apt`-installed Python package that `pip` cannot overwrite). If this happens, remove the system package manually and re-run the installation:
> ```bash
> sudo apt remove python3-typing-extensions
> pip install "gymnasium>=0.29" --break-system-packages
> ```
