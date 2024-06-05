#!/usr/bin/env python3
"""Script to set up a bag file and LAMP GT odom."""
import click
import pathlib
import shutil
import subprocess
import rosbag
import rospy
import sys
import tf2_ros
import warnings


INFO_LAUNCH_TEMPLATE = """<launch>
  <arg name="robot_frame" default="{robot_name}/lamp"/>
  <arg name="parent_frame" default="{robot_name}/map"/>
  <arg name="path_to_csv" default="{csv_file_path}"/>
  <arg name="label_topic" default="labels/image_raw"/>
  <arg name="color_info_topic" default="{robot_name}/forward/color/camera_info"/>
  <arg name="raw_depth_info_topic" default="{robot_name}/forward/depth/camera_info"/>
  <arg name="raw_depth_topic" default="{robot_name}/forward/depth/image_rect_raw"/>

  <arg name="color_map_name" default="ade20k_full"/>
  <arg name="color_map_file"
       default="$(find semantic_inference)/config/colors/$(arg color_map_name).yaml"/>

  <node pkg="nodelet" type="nodelet" name="info_nodelet_manager" args="manager">
  </node>

  <node pkg="nodelet" type="nodelet" name="recolor_nodelet"
        args="load semantic_inference/recolor info_nodelet_manager --no-bond">
    <rosparam file="$(arg color_map_file)" command="load" ns="colors"/>
    <remap from="labels/image_raw" to="$(arg label_topic)"/>
  </node>

  <node pkg="nodelet" type="nodelet" name="depth_rect"
        args="load depth_image_proc/register info_nodelet_manager --no-bond">
    <remap from="rgb/camera_info" to="$(arg color_info_topic)"/>
    <remap from="depth/image_rect" to="$(arg raw_depth_topic)"/>
    <remap from="depth/camera_info" to="$(arg raw_depth_info_topic)"/>
    <remap from="depth_registered/image_rect" to="$(arg depth_topic)"/>
  </node>

  <node pkg="tf2_ros" type="static_transform_publisher" name="lamp_bridge_tf"
        args="{lamp_bridge_tf} $(arg robot_frame) {robot_name}/realsense_base"/>

  <include file="{tf_file_path}"/>

  <include file="$(find hydra_ros)/launch/utils/gt_pose/robot_gt_pose.launch"
           pass_all_args="true">
    <arg name="load_static_tfs" value="false"/>
  </include>
</launch>
"""

DATA_LAUNCH_TEMPLATE = """<launch>
    <arg name="extra_bag_args" default=""/>

    <node pkg="rosbag" type="play" name="rosbag_player"
          args="{bag_path} --clock --topics {topic_list} $(arg extra_bag_args)"
          output="screen"/>
</launch>
"""


def _dump_static_tfs(bag_file, output):
    subprocess.run(["rosrun", "hydra_ros", "copy_static_tfs.py", bag_file, output])


def _make_odom_file(bag_file, gt_odom_topic, output):
    subprocess.run(
        ["rosrun", "hydra_ros", "dump_bag_odom.py", bag_file, gt_odom_topic, output]
    )


def _prep_output(output_path, bag_path, run_name, robot_name):
    bag_name = bag_path.stem
    run_location = output_path / "launch"
    run_location.mkdir(parents=True, exist_ok=True)

    include_path = run_location / f".{bag_name}"

    lamp_bridge_tf = _lookup_odom_link(str(bag_path), robot_name)

    setup_launch_path = run_location / f"{run_name}_info.launch"
    with setup_launch_path.open("w") as fout:
        fout.write(
            INFO_LAUNCH_TEMPLATE.format(
                robot_name=robot_name,
                csv_file_path=include_path / "traj_gt.csv",
                tf_file_path=include_path / "static_tfs.xml",
                lamp_bridge_tf=lamp_bridge_tf
            )
        )

    topic_list = [
        f"/{robot_name}/forward/color/camera_info",
        f"/{robot_name}/forward/color/image_raw/compressed",
        f"/{robot_name}/forward/depth/camera_info",
        f"/{robot_name}/forward/depth/image_rect_raw",
        "/oneformer/labels/image_raw",
    ]
    data_launch_path = run_location / f"{run_name}_data.launch"
    with data_launch_path.open("w") as fout:
        fout.write(
            DATA_LAUNCH_TEMPLATE.format(
                bag_path=bag_path, topic_list=" ".join(topic_list)
            )
        )

    return include_path


def _lookup_odom_link(bag_file, robot_name):
    tf_buffer = tf2_ros.BufferCore()

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        with rosbag.Bag(bag_file, "r") as bag:
            for topic, tf_msg, t in bag.read_messages(topics=["/tf_static"]):
                for msg in tf_msg.transforms:
                    tf_buffer.set_transform_static(msg, "rosbag")

    vel_T_base = tf_buffer.lookup_transform_core(
        f"{robot_name}/velodyne", f"{robot_name}/realsense_base", rospy.Time(0)
    )

    components = [
        vel_T_base.transform.translation.x,
        vel_T_base.transform.translation.y,
        vel_T_base.transform.translation.z,
        vel_T_base.transform.rotation.x,
        vel_T_base.transform.rotation.y,
        vel_T_base.transform.rotation.z,
        vel_T_base.transform.rotation.w,
    ]
    return " ".join([f"{x:7.4f}" for x in components])


@click.command()
@click.argument("bag_path", type=click.Path(exists=True))
@click.option("-t", "--odom_topic", default="locus/odometry")
@click.option("-n", "--name", default=None)
def main(bag_path, odom_topic, name):
    """Set up necessary files to run a sparkal dataset."""
    bag_path = pathlib.Path(bag_path).expanduser().absolute()
    robot_name = bag_path.parent.stem
    dataset_name = bag_path.parent.parent.stem
    bag_name = bag_path.stem
    output_path = bag_path.parent / "launch"
    gt_odom_topic = str(pathlib.Path(robot_name) / odom_topic)
    run_name = bag_name if name is None else name

    click.secho(f"making launch file for {bag_path}\n", fg="green")
    click.secho("input:", fg="green")
    click.secho(f"  - detected robot name: {robot_name}", fg="green")
    click.secho(f"  - detected dataset name: {dataset_name}", fg="green")
    click.secho(f"  - detected bag name: {bag_name}", fg="green")
    click.secho(f"  - detected gt odom topic: {gt_odom_topic}", fg="green")
    click.secho("output:", fg="green")
    click.secho(f"  - directory: {output_path}", fg="green")
    click.secho(f"  - launch file base name: {run_name}", fg="green")

    lamp_bag = bag_path.parent / f"{bag_name}_lamp" / "gt_odom.bag"
    if not lamp_bag.exists():
        click.secho(f"missing gt odom @ {lamp_bag}")
        sys.exit(1)

    include_path = _prep_output(bag_path.parent, bag_path, run_name, robot_name)
    if include_path.exists():
        click.secho(f"previous output detected @ {include_path}", fg="red")
        click.confirm("recreate?", abort=True, default=False)
        shutil.rmtree(include_path)

    include_path.mkdir(parents=True, exist_ok=False)

    odom_output = include_path / "traj_gt.csv"
    tf_output = include_path / "static_tfs.xml"

    click.echo("dumping odometry to trajectory file...")
    _make_odom_file(str(lamp_bag), gt_odom_topic, str(odom_output))
    click.echo("dumping static tfs to include file...")
    _dump_static_tfs(str(bag_path), str(tf_output))


if __name__ == "__main__":
    main()
