### Running Hydra

The only dataset that is supported out-of-the-box is [uHumans2](http://web.mit.edu/sparklab/datasets/uHumans2/).
To test Hydra out, you can just download a single scene (the office scene without humans is recommended, and can be found [here](https://drive.google.com/uc?id=1CA_1Awu-bewJKpDrILzWok_H_6cOkGDb).
Make sure to decompress the rosbag (`rosbag decompress path/to/bagfile`) before running!

:warning: Also make sure to source the workspace before starting (typically `source path/to/catkin_ws/devel/setup.bash`, though if you use zsh you should use the correct setup file for that).

To start Hydra:
```
roslaunch hydra_ros uhumans2.launch
```

Then, start the rosbag in a separate terminal:
```
rosbag play path/to/rosbag --clock
```

### Using Kimera-VIO

You can configure your workspace to also include Kimera-VIO by:
```
roscd && cd ../src
vcs import . < hydra/install/vio_overlay.rosinstall

catkin build
```

#### Running Using VIO Only

First, start Kimera:

```
roslaunch kimera_vio_ros kimera_vio_ros_uhumans2.launch online:=true viz_type:=1 use_lcd:=false
```

and in a separate terminal, run:

```
# in separate terminal
roslaunch hydra_ros uhumans2.launch use_gt_frame:=false
```

#### Running Using VIO and External Visual Loop Closures

First, start Kimera:

```
roslaunch kimera_vio_ros kimera_vio_ros_uhumans2.launch online:=true viz_type:=1 \
    use_lcd:=true \
    lcd_no_optimize:=true
```

and in a separate terminal, run the same command for Hydra:

```
roslaunch hydra_ros uhumans2.launch use_gt_frame:=false
```

:warning: To achieve the best results with Kimera-VIO, you should wait for the LCD vocabulary to finish loading before starting the rosbag.

#### Running Using VIO and DSG Loop Closures

First, start Kimera:

```
roslaunch kimera_vio_ros kimera_vio_ros_uhumans2.launch online:=true viz_type:=1 \
     use_lcd:=true \
     lcd_no_detection:=true
```

and in a separate terminal, run the same command for Hydra:

```
roslaunch hydra_ros uhumans2.launch use_gt_frame:=false enable_dsg_lcd:=true
```

:warning: To achieve the best results with Kimera-VIO, you should wait for the LCD vocabulary to finish loading before starting the rosbag.
