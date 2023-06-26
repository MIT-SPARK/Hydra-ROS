## Hydra-ROS

This repository contains the ROS interface for [Hydra](https://github.com/MIT-SPARK/Hydra) and is based on the follwoing papers:
  - ["Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization"](http://www.roboticsproceedings.org/rss18/p050.pdf)
  - ["Foundations of Spatial Perception for Robotics: Hierarchical Representations and Real-time Systems"](https://arxiv.org/abs/2305.07154)

If you find this code relevant for your work, please consider citing us. A bibtex entry is provided below:
```
@article{hughes2022hydra,
    title={Hydra: A Real-time Spatial Perception System for {3D} Scene Graph Construction and Optimization},
    fullauthor={Nathan Hughes, Yun Chang, and Luca Carlone},
    author={N. Hughes and Y. Chang and L. Carlone},
    booktitle={Robotics: Science and Systems (RSS)},
    pdf={http://www.roboticsproceedings.org/rss18/p050.pdf},
    year={2022},
}

@article{hughes2023foundations,
         title={Foundations of Spatial Perception for Robotics: Hierarchical Representations and Real-time Systems},
         author={Nathan Hughes and Yun Chang and Siyi Hu and Rajat Talak and Rumaisa Abdulhai and Jared Strader and Luca Carlone},
         year={2023},
         eprint={2305.07154},
         archivePrefix={arXiv},
         primaryClass={cs.RO}
}
```

### Acknowledgements and Disclaimer

**Acknowledgements:** This work was partially funded by the AIA CRA FA8750-19-2-1000, ARL DCIST CRA W911NF-17-2-0181, and ONR RAIDER N00014-18-1-2828.

**Disclaimer:** Research was sponsored by the United States Air Force Research Laboratory and the United States Air Force Artificial Intelligence Accelerator and was accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views and conclusions contained in this document are those of the authors and should not be interpreted as representing the official policies, either expressed or implied, of the United States Air Force or the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes notwithstanding any copyright notation herein.

### General Requirements

Hydra has been tested on Ubuntu 20.04 and ROS Noetic.

:warning: Ubuntu 18.04 and ROS Melodic are no longer actively tested.

You can follow the instructions [here](http://wiki.ros.org/ROS/Installation) to install ROS if you haven't already.
Then, make sure you have some general requirements:
```
sudo apt install python3-rosdep python3-catkin-tools python3-vcstool
```

Finally, if you haven't set up rosdep yet:
```
sudo rosdep init
rosdep update
```

### Building Hydra

To get started:

```
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release -DGTSAM_TANGENT_PREINTEGRATION=OFF \
              -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
catkin config --skiplist hdf5_map_io mesh_msgs_hdf5 label_manager mesh_tools \
                         rviz_map_plugin minkindr_python

cd src
git clone git@github.com:MIT-SPARK/Hydra.git hydra
vcs import . < hydra/install/hydra.rosinstall

rosdep install --from-paths . --ignore-src -r -y
sudo apt install libprotobuf-dev protobuf-compiler

cd ..
catkin build
```

:warning: Depending on the amount of RAM available on your machine and whether or not you are compiling Kimera-VIO as well, you may run out of memory when compiling with `catkin build` directly (which will result in a `GCC killed` error). If this occurs, you can either specify fewer threads for catkin via `catkin build -j NUM_THREADS` or compile certain larger packages (e.g. gtsam) directly first by building them specifically, e.g. `catkin build gtsam`.

Please help us by creating new issues when you run into problems with these instructions!

### Running Hydra

See our documentation [here](doc/quickstart.md).

### Filing Issues

:warning: We don't support other platforms. Issues requesting support on other platforms (e.g. Ubuntu 16.04, Windows) will be summarily closed.

Depending on the nature of the issue, it may be helpful to browse [this](https://github.com/MIT-SPARK/Hydra/blob/main/doc/debugging.md) page about debugging Hydra first.
