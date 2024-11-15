# dynaarm_description

This repository contains the xacro/urdf description file for the [Duatic DynaArms](https://duatic.com/robotic-arm/)

# License

The contents are licensed under the BSD-3-Clause  [license](LICENSE).\
Images in this repository are to be licensed separately if you want to use them for any other usecase than forking this repository. Please open an issue in order to get in touch with us.

# Usage

This package contains only the description files for the DynaArm. The [standalone](./urdf/dynaarm_standalone.urdf.xacro) and [standalone dual](./urdf/dynaarm_standalone_dual.urdf.xacro) demonstrate how to integrate the xacro files into your own description. For simple applications they can also be used directly. 


## Show case

This repository integrates a simple show case you can view by running 

```bash
ros2 launch dynaarm_description view.launch.py covers:=True dual:=False dof:=6 version:=v2
```

![Example: v1, no covers, 6 dof](./doc/example.png)
Example: v1, no covers, 6 dof

# Contributing

We encourage community contributions to this repository. Feel free to open an issue or provide a pull request.
Please make sure for any pull request that it passes the `pre-commit` checks (see Tooling section).

## Tooling

### pre-commit

The [pre-commit](https://pre-commit.com/) tool is used for running certain checks and formatters everytime before a commit is done. 
Please use it on any pull requests for this repository. 

__Installation:__

Ubuntu 24.04: `apt install pipx && pipx install pre-commit`. Open a new terminal afterwards.

__Usage:__ 
In the repository folder run: `pre-commit run --all`. This can be automated with `pre-commit install`, so all checks are run everytime a commit is done. 
