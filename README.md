# Hydra-ROS

This repository contains the ROS interface for [Hydra](https://github.com/MIT-SPARK/Hydra) and is based on the following papers:
  - ["Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization"](http://www.roboticsproceedings.org/rss18/p050.pdf)
  - ["Foundations of Spatial Perception for Robotics: Hierarchical Representations and Real-time Systems"](https://journals.sagepub.com/doi/10.1177/02783649241229725)

If you find this code relevant for your work, please consider citing us. A bibtex entry is provided below:
```bibtex
@article{hughes2022hydra,
    title={Hydra: A Real-time Spatial Perception System for {3D} Scene Graph Construction and Optimization},
    fullauthor={Nathan Hughes, Yun Chang, and Luca Carlone},
    author={N. Hughes and Y. Chang and L. Carlone},
    booktitle={Robotics: Science and Systems (RSS)},
    pdf={http://www.roboticsproceedings.org/rss18/p050.pdf},
    year={2022},
}

@article{hughes2024foundations,
    title={Foundations of Spatial Perception for Robotics: Hierarchical Representations and Real-time Systems},
    fullauthor={Nathan Hughes and Yun Chang and Siyi Hu and Rajat Talak and Rumaisa Abdulhai and Jared Strader and Luca Carlone},
    author={N. Hughes and Y. Chang and S. Hu and R. Talak and R. Abdulhai and J. Strader and L. Carlone},
    journal={The International Journal of Robotics Research},
    doi={10.1177/02783649241229725},
    url={https://doi.org/10.1177/02783649241229725},
    year={2024},
}
```

## Table of Contents

- [Getting Started](#getting-started)
  - [Installation](#installation)
  - [Dataset Setup](#dataset-setup)
  - [Running Hydra](#running-hydra)
- [Filing Issues](#filing-issues)
- [Additional Resources](#additional-resources)
- [Acknowledgements](#acknowledgements)

## Getting started

> :warning: **Warning** <br>
> The ROS2 version of Hydra is in active development and is unstable and may not fully be feature-complete.

Hydra has been tested on Ubuntu 22.04 & ROS2 Iron and Ubuntu 24.04 & ROS2 Jazzy. It **should** build on newer distributions. It **will not** build on older ROS2 distributions than Iron (it depends on the `NodeInterfaces` suite of classes that were introduced in Iron).

You can follow the instructions [here](https://docs.ros.org/en/jazzy/Installation.html) to install ROS2 if you haven't already. The link points to Jazzy currently, pay attention to the distribution!

### Installation

#### Dependencies

First, make sure you have some general requirements
```shell
# this is an optional package in the ROS2 installation directions
sudo apt install ros-dev-tools
```
and make sure you have set rosdep up:
```shell
sudo rosdep init
rosdep update
```

#### Building

To get started:

```shell
mkdir -p ~/hydra_ws/src
cd ~/hydra_ws
# you may find it convenient to set the build type, etc.
# you may also want to set `symlink-install: true`
echo "build: {cmake-args: [-DCMAKE_BUILD_TYPE=Release]}" > colcon_defaults.yaml

cd src
git clone git@github.com:MIT-SPARK/Hydra-ROS.git hydra_ros -b ros2
vcs import . < hydra_ros/install/ros2.yaml
rosdep install --from-paths . --ignore-src -r -y

cd ..
colcon build --continue-on-error
```

> **Note**<br>
> Depending on the amount of RAM available on your machine, you may run out of memory when compiling with `colcon build` directly (which will result in a `GCC killed` error). If this occurs, you can either specify fewer threads for colcon via `MAKEFLAGS="-j2" colcon build --parallel-workers 2` (which builds 2 packages at a time with 2 threads for 4 threads total) or compile certain larger packages directly first by building them specifically.

> :warning: **Warning**</br>
> In the `vcs import` step, GitHub may block too many concurrent requests. If you receive `kex_exchange_identification: read: Connection reset by peer` errors, try running `vcs import . < hydra/install/hydra.rosinstall --workers 1`.

### Dataset Setup

We've switched to the v2 version of uhumans2. Download the ROS1 bag for the office scene [here](https://drive.google.com/file/d/1awAzQ7R1hdS5O1Z2zOcpYjK7F4_APq_p/view?usp=drive_link).
Then install [rosbags](https://pypi.org/project/rosbags/) via pip (i.e., `pip install rosbags` into a virtual environment).

To convert the bag:
```shell
rosbags-convert --src path/to/office.bag --dst path/to/office
```

Make sure to create a override for latching static tf topics:
```shell
echo "/tf_static: {depth: 1, durability: transient_local}" > ~/.tf_overrides.yaml
```

### Running Hydra

> **:warning: Warning**<br>
> This guide is written assuming you use `zsh`. If you don't, source the right setup file for your shell

To start Hydra:
```shell
# this will break autocomplete, see group wiki for correct way to do this
source ~/hydra_ws/install/setup.zsh
ros2 launch hydra_ros uhumans2.launch.yaml
```

Then, start the rosbag in a separate terminal:
```shell
ros2 bag play path/to/rosbag --clock --qos-profile-overrides-path ~/.tf_overrides.yaml
```

#### Running with Semantic Segmentation

See [here](https://github.com/MIT-SPARK/semantic_inference/blob/main/docs/closed_set.md#getting-dependencies) for installation information for `semantic_inference`.

For the uHumans2 office scene, start Hydra via
```
source ~/hydra_ws/install/setup.zsh
ros2 launch hydra_ros uhumans2.launch.yaml use_gt_semantics:=false
```

You may need to wait for about a minute for TensorRT to compile the model the first time you run the launch file. Once Hydra initializes, then run
```
ros2 bag play path/to/rosbag --clock --qos-profile-overrides-path ~/.tf_overrides.yaml
```

## Filing Issues

Please understand that this is research code maintained by busy graduate students, **which comes with some caveats**:
  1. We do our best to maintain and keep the code up-to-date, but things may break or change occasionally
  2. We do not have bandwidth to help adapt the code to new applications
  3. The documentation, code-base and installation instructions are geared towards practitioners familiar with ROS and 3D scene graph research.

> **:warning: Warning**<br>
> We don't support other platforms. Issues requesting support on other platforms (e.g., Ubuntu 18.04, Windows) will be summarily closed.

Depending on the nature of the issue, it may be helpful to browse [this](doc/debugging.md) page about debugging Hydra first.

Thank you in advance for your understanding!

## Additional Resources

We maintain information on the ROS2 interfaces that Hydra uses [here](doc/hydra_ros_interfaces.md) that may be useful if you are trying to understand how to use Hydra with a new dataset.
We also have a preliminary guide to setting up Hydra with the Zed2i camera [here](doc/setting_up_a_new_sensor.md).

## Acknowledgements

**Acknowledgements:** This work was partially funded by the AIA CRA FA8750-19-2-1000, ARL DCIST CRA W911NF-17-2-0181, and ONR RAIDER N00014-18-1-2828.

**Disclaimer:** Research was sponsored by the United States Air Force Research Laboratory and the United States Air Force Artificial Intelligence Accelerator and was accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views and conclusions contained in this document are those of the authors and should not be interpreted as representing the official policies, either expressed or implied, of the United States Air Force or the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes notwithstanding any copyright notation herein.
