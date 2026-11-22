# Getting started

This page takes you from a clone to a running server. The [README](../README.md) has the short
version.

## Requirements

- Linux with ROS 2 Jazzy installed. Jazzy is the only distribution CI tests against.
- Python 3.11 or newer. rclpy comes from the ROS 2 installation; it is not on PyPI and is not a
  dependency of the package.
- git, to clone the repository. The package version comes from its tags.

## Install from a clone

Clone the repository and install it into a virtual environment that can see the ROS 2 Python
packages:

```sh
. /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .venv
. .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -c constraints.txt -e .
```

Keep the `-c constraints.txt`. The package metadata only holds version ranges, and
`constraints.txt` pins the opensmi releases from PyPI the code is tested against. The
editable install (`-e`) also keeps the examples importable, since they live in the repository and
are not part of the package.

## Serve a Machine from a TOML file

A machine file declares one Machine and its Components. This one has a single Component with one
Binding that mirrors the `/odom` topic:

```toml
[machine]
name = "Robot"

[machine.identification]
serial_number = "0001"
manufacturer = "Example"
product_instance_uri = "urn:example:robot"

[[component]]
name = "Platform"

[component.identification]
serial_number = "0001-platform"
manufacturer = "Example"

[[component.monitoring]]
name = "Odom"
path = "/odom"
message_type = "nav_msgs/msg/Odometry"
```

Save it as `machine.toml` and start the server from a shell with ROS 2 sourced:

```sh
opensmi-ros2 serve machine.toml
```

`python -m opensmi.ros2 serve machine.toml` does the same. With opensmi's default configuration the
server listens on `opc.tcp://0.0.0.0:4841/server`, so a client on the same host connects to
`opc.tcp://localhost:4841/server`. Each of opensmi's default users has its name as password.
`visitor` may browse and read. `orchestrator` may also start Skills, call Methods and write Parameters,
each of them with the Machine's Lock held. `operator` may also use the introspection Methods that
change something. The [client quickstart](client-quickstart.md) logs in, takes the Lock, starts a Skill
and shows the errors you get when something is missing or refused.

Under Objects, Machines, Robot, Components you find `Platform` and `ROS2Introspection`. The
odometry is the variable `Platform/Monitoring/Odom`, with its fields below it, for example
`Platform/Monitoring/Odom/Pose/Pose/Position/X`. To see values arrive, publish odometry from another
shell:

```sh
ros2 topic pub --rate 1 /odom nav_msgs/msg/Odometry "{child_frame_id: base_link}"
```

## Serve the example Machine

[`examples/machine.toml`](../examples/machine.toml) declares a Nav2-style platform with one Binding
of every kind. [`examples/machine.py`](../examples/machine.py) and
[`examples/component.py`](../examples/component.py) declare the same Machine in Python. Several of
its types come from `nav2_msgs`, so that interface package has to be installed. A running Nav2
stack is not needed: the server starts either way, and its Bindings wait for their ROS 2
counterparts.

From the repository root:

```sh
opensmi-ros2 serve examples/machine.toml
# or the Python version, which reads its ROS2 settings from examples/ros2.toml
python -m examples.run_server
```

Both run on ROS 2 domain 42, as the example files set it. The [examples index](../examples/README.md)
also lists two complete simulated robots that run in Docker.

## Run and stop

`opensmi-ros2 serve` runs in the foreground until Ctrl+C or SIGTERM, then stops the server and the
Runtime. It imports rclpy only after the whole file has passed its checks, so `--help` and a file
with mistakes work on a machine without ROS 2.

| Exit status | Meaning |
| --- | --- |
| 0 | Clean stop |
| 1 | The machine file or the server configuration cannot be used. Every mistake is printed to stderr, one per line, and nothing was started |
| 2 | The command line was refused |

Whether a message type imports or a topic exists is only known once the Runtime runs, so those
errors show up when the server starts. A missing ROS 2 counterpart is not an error: the Binding waits
for it. A Binding marked `required = true` holds its Component's StartupSkill back until its
counterpart appears, for up to `startup_timeout` seconds.
