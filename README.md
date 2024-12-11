# dynaarm_description

This repository contains the xacro/urdf description file for the [Duatic DynaArms](https://duatic.com/robotic-arm/). The official ROS2 driver can be found in the [dynaarm_driver](https://github.com/Duatic/dynaarm_driver/) repository.

<img src="./doc/dynaarm.webp" width="300">\
Duatic DynaArm v2


# License

The contents are licensed under the BSD-3-Clause  [license](LICENSE).\
Images in this repository are to be licensed separately if you want to use them for any other usecase than forking/cloning this repository for your application. Please open an issue in order to get in touch with us.

# Usage

This package contains only the description files for the DynaArm. The [standalone](./urdf/dynaarm_standalone.urdf.xacro) and [standalone dual](./urdf/dynaarm_standalone_dual.urdf.xacro) demonstrate how to integrate the xacro files into your own description. For simple applications they can also be used directly.


## Supported Arms

This package current supports the following DynaArms:

| Name     | Description |
| ---      | ---         |
| arowana4 | Pre-release version of the arm with 4kg payload at maximum reach |
| baracuda12 | First publicly released version of the  DynaArm with 12kg payload at maximum reach |

## Show case

This repository integrates a simple show case you can view by running

```bash
ros2 launch dynaarm_description view.launch.py covers:=True dual:=True dof:=6 version:=baracuda12
```

![Example: baracuda12, no covers, 6 dof](./doc/example.png)
Example: baracuda12, no covers, 6 dof

# Contributing

We encourage community contributions to this repository. Feel free to open an issue or provide a pull request.
Please make sure for any pull request that it passes the `pre-commit` checks (see Tooling section).

## Tooling

### pre-commit

The [pre-commit](https://pre-commit.com/) tool is used for running certain checks and formatters every time before a commit is done.
Please use it on any pull requests for this repository.

__Installation:__

Ubuntu 24.04: `apt install pipx && pipx install pre-commit`. Open a new terminal afterwards.

__Usage:__
In the repository folder run: `pre-commit run --all`. This can be automated with `pre-commit install`, so all checks are run every time a commit is done.
