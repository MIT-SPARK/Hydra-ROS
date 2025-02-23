## Installation and Running

### Dependencies

Hydra has been tested on Ubuntu 22.04 and ROS Iron. It **should** build on newer distributions. It **will not** build on older ROS2 distributions than Iron (it depends on the `NodeInterfaces` suite of classes that were introduced in Iron).

You can follow the instructions [here](https://docs.ros.org/en/jazzy/Installation.html) to install ROS2 if you haven't already. The link points to Jazzy currently, pay attention to the distribution!
Then, make sure you have some general requirements:
```shell
# this is an optional package in the ROS2 installation directions
sudo apt install ros-dev-tools
```

Finally, if you haven't set up rosdep yet:
```shell
sudo rosdep init
rosdep update
```

### Building

To get started:

```shell
mkdir -p ~/hydra_ws/src
cd ~/hydra_ws
# you may find it convenient to set the build type, etc.
# you may also want to set `symlink-install: true`
echo "build: {cmake-args: [--no-warn-unused-cli, -DCONFIG_UTILS_ENABLE_ROS=OFF]}" > colcon_defaults.yaml

cd src
git clone git@github.mit.edu:SPARK/Hydra-ROS.git hydra_ros
vcs import . < hydra_ros/install/ros2.yaml
rosdep install --from-paths . --ignore-src -r -y

cd ..
colcon build --continue-on-error
```

> :warning: **Warning**</br>
> In the `vcs import` step, GitHub may block too many concurrent requests. If you receive `kex_exchange_identification: read: Connection reset by peer` errors, try running `vcs import . < hydra/install/hydra.rosinstall --workers 1`.

### Dataset

We've switched to the v2 version of uhumans2. Download the ROS1 bag for the office scene [here](https://drive.google.com/file/d/1awAzQ7R1hdS5O1Z2zOcpYjK7F4_APq_p/view?usp=drive_link).
Then install [rosbags](https://pypi.org/project/rosbags/) via pip (i.e., `pip install rosbags` into a virtual environment).

To convert the bag:
```shell
rosbags-convert --src path/to/office.bag --dest path/to/office
```

Make sure to create a override for latching static tf topics:
```shell
echo "/tf_static: {depth: 1, durability: transient_local}" > ~/.tf_overrides.yaml
```

### Running

> **:warning: Warning**<br>
> This guide is written assuming you use `zsh`. If you don't, source the right setup file for your shell

To start Hydra:
```shell
# this will break autocomplete, see group wiki for correct way to do this
source ~/hydra_ws/install/setup.zsh
ros2 launch hydra_ros uhumans2.launch
```

Then, start the rosbag in a separate terminal:
```shell
ros2 bag play path/to/rosbag --clock --qos-profile-overrides-path ~/.tf_overrides.yaml
```
