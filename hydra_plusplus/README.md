# Installation

To get started, set up your workspace and clone the Hydra-ROS repo. Then:

```shell
cd <path to ws>/src
vcs import . < hydra_ros/install/packages.yaml
vcs import . < hydra_ros/hydra_plusplus/install/system.repos
rosdep install --from-paths . --ignore-src -r -y

bash src/semantic_inference/install/setup.sh
bash src/hydra_ros/hydra_plusplus/install/setup.sh

cd ..
colcon build --continue-on-error
```


