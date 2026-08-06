# sphero-monash

This repository contains a codebase for working with Sphero robots. The code is built on a sphero-unsw interface but exposes a gym style wrapper.

## Robot Interface

Both the real robot and the provided simulator implement a [`gymnasium`](https://gymnasium.farama.org/index.html)-like interface.

### Action Space

The robot action is defined as:

```python
action = [speed, heading_angle]
```

| Index | Variable        | Description           |
| ----: | --------------- | --------------------- |
|   `0` | `speed`         | Desired robot speed   |
|   `1` | `heading_angle` | Desired heading angle |

### Observation Space

The robot observation is defined as:

```python
observation = [
    x_meas,
    y_meas,
    heading_meas,
    speed_meas,
    collision_flag,
]
```

| Index | Variable         | Description            |
| ----: | ---------------- | ---------------------- |
|   `0` | `x_meas`         | Measured x-position    |
|   `1` | `y_meas`         | Measured y-position    |
|   `2` | `heading_meas`   | Measured heading angle |
|   `3` | `speed_meas`     | Measured speed         |
|   `4` | `collision_flag` | Collision indicator    |

### Robot `info` dictionary

Each call to `robot.step(action)` returns an `info` dictionary with extra diagnostic details from the real Sphero robot.

```python
obs, reward, terminated, truncated, info = robot.step(action)
```

The main fields are:

| Field             |              Units | Description                                                                                                                            |
| ----------------- | -----------------: | -------------------------------------------------------------------------------------------------------------------------------------- |
| `state_true`      | `[m, m, rad, -]` | Current robot state `[x, y, heading, speed]`. For the real robot this is the same as odometry.                                         |
| `state_odom`      | `[m, m, rad, -]` | Odometry-based robot state `[x, y, heading, speed]`. Location from the Sphero API is converted from centimetres to metres.             |
| `collision`       |            boolean | Whether a collision was detected on this step.                                                                                         |
| `collision_flag`  |     `0.0` or `1.0` | Numeric collision flag. This is also included as the final observation value, `obs[4]`.                                                |
| `collision_debug` |              mixed | Extra values used by the software collision detector, useful for threshold tuning. See below for units.                                |
| `acceleration`    |                `g` | Latest acceleration vector from the Sphero accelerometer. `1 g = 9.80665 m/s²`; each axis is approximately in the range `-8` to `8 g`. |
| `orientation`     |            degrees | Latest robot orientation/attitude. `pitch` and `yaw` are approximately `-180°` to `180°`; `roll` is approximately `-90°` to `90°`.     |
| `gyro`            |          degrees/s | Latest gyroscope rate reading. Axes are approximately in the range `-2000°/s` to `2000°/s`.                                            |
| `velocity`        |               cm/s | Latest velocity vector from the motor encoders. `x` is right/left velocity and `y` is forward/back velocity.                           |
| `speed_cmd`       |           unitless | Speed command sent through the environment action interface. This is converted internally to the Sphero speed scale.                   |
| `heading_cmd`     |            radians | Heading command sent through the environment action interface. This is converted internally to degrees for the Sphero API.             |
| `setpoint_xy`     |           `[m, m]` | Current target/setpoint position used for logging and visualisation.

Example:

```python
obs, reward, terminated, truncated, info = robot.step(action)

if info["collision"]:
    print("Collision detected")
    print(info["collision_debug"])
```

The collision detector is software-based. It uses changes in acceleration, drops in velocity, and gyroscope spikes to estimate when the Sphero has hit an obstacle.

`collision_debug` contains values such as:

| Debug value          |                Units | Description                                                           |
| -------------------- | -------------------: | --------------------------------------------------------------------- |
| `accel_mag`          |                  `g` | Magnitude of the acceleration vector.                                 |
| `accel_jerk`         |           `g / step` | Change in acceleration magnitude since the previous control step.     |
| `speed_cm_s`         |               `cm/s` | Magnitude of the Sphero velocity vector.                              |
| `speed_drop`         |      `cm/s per step` | Drop in speed since the previous control step.                        |
| `gyro_mag`           |          `degrees/s` | Magnitude of the gyroscope rate vector.                               |
| `gyro_spike`         | `degrees/s per step` | Change in gyroscope magnitude since the previous control step.        |
| `moving_collision`   |              boolean | Collision condition based on acceleration jerk and speed drop.        |
| `stationary_impact`  |              boolean | Collision condition for impacts while the robot is nearly stationary. |
| `rotation_collision` |              boolean | Collision condition based on a sudden gyroscope spike.                |


### Goals

The environment can be configured with a 2d positional goal and goal tolerance:
* `env.goal_pos`
* `env.goal_tolerance`

### Teleoperation example

An example using the simulator and robot is available in:
`examples/teleop.py`

---
# Installation

This project requires Python 3.9 and below 3.13.

We recommend creating a virtual environment before installing the package and its dependencies.

## Option 1: Create an environment with `venv`

Create and activate a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

Upgrade `pip` and install the project:

```bash
python -m pip install --upgrade pip
pip install -e .
```

Install the dependencies from `requirements.txt`:

```bash
pip install -r requirements.txt
```

## Option 2: Create an environment with `uv`

Create a virtual environment, and install all dependencies and activate it:

```bash
uv sync
```

## Verify the installation

You can check that the package imports correctly with:

```bash
python -c "import sphero_env; print('sphero_env import OK')"
```

You can then run the teleoperation example with:

```bash
python examples/teleop.py
```

## Connecting to a real sphero

1. Gently shake the sphero robot - it should light up and show a unique bluetooth address on it.
2. Place it on the ground (it must not be moving when you connect)
3. When you run a robot connection (eg. using the teleop code), you will be asked to confirm the sphero bluetooth address to connect.

