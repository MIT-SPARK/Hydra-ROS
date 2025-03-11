## Hydra-ROS

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

### Acknowledgements and Disclaimer

**Acknowledgements:** This work was partially funded by the AIA CRA FA8750-19-2-1000, ARL DCIST CRA W911NF-17-2-0181, and ONR RAIDER N00014-18-1-2828.

**Disclaimer:** Research was sponsored by the United States Air Force Research Laboratory and the United States Air Force Artificial Intelligence Accelerator and was accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views and conclusions contained in this document are those of the authors and should not be interpreted as representing the official policies, either expressed or implied, of the United States Air Force or the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes notwithstanding any copyright notation herein.

### Getting started

> :warning: **Warning** <br>
> The ROS2 version of this package is in active development and is not guaranteed to build, run or have documentation. You have been warned!

Hydra has been tested on Ubuntu 22.04 and ROS2 Iron. See [here](doc/ros2_setup.md) to get started with Hydra and ROS2.

### Running Hydra

See our documentation [here](doc/quickstart.md).

### Design Information

We maintain information on the ROS2 interfaces that Hydra uses [here](doc/hydra_ros_interfaces.md) that may be useful if you are trying to understand how to use Hydra with a new dataset.

### Filing Issues

:warning: We don't support other platforms. Issues requesting support on other platforms (e.g. Ubuntu 16.04, Windows) will be summarily closed.

Depending on the nature of the issue, it may be helpful to browse [this](https://github.com/MIT-SPARK/Hydra/blob/main/doc/debugging.md) page about debugging Hydra first.
