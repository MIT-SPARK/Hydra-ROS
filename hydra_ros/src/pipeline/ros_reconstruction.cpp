/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra_ros/pipeline/ros_reconstruction.h"

#include <config_utilities/printing.h>
#include <geometry_msgs/TransformStamped.h>
#include <hydra/common/hydra_config.h>
#include <hydra/places/gvd_integrator.h>
#include <hydra_msgs/QueryFreespace.h>
#include <kimera_semantics/color.h>
#include <tf2_eigen/tf2_eigen.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_ros/mesh_vis.h>

#include "hydra_ros/utils/image_receiver.h"
#include "hydra_ros/utils/pointcloud_adaptor.h"

namespace hydra {

using pose_graph_tools::PoseGraph;
using pose_graph_tools::PoseGraphEdge;
using pose_graph_tools::PoseGraphNode;
using sensor_msgs::PointCloud2;

inline geometry_msgs::Pose tfToPose(const geometry_msgs::Transform& transform) {
  geometry_msgs::Pose pose;
  pose.position.x = transform.translation.x;
  pose.position.y = transform.translation.y;
  pose.position.z = transform.translation.z;
  pose.orientation = transform.rotation;
  return pose;
}

RosReconstruction::RosReconstruction(const RosReconstructionConfig& config,
                                     const ros::NodeHandle& nh,
                                     const RobotPrefixConfig& prefix,
                                     const OutputQueue::Ptr& output_queue)
    : ReconstructionModule(
          config,
          prefix,
          output_queue ? output_queue : std::make_shared<OutputQueue>()),
      config_(config),
      nh_(nh) {
  buffer_.reset(new tf2_ros::Buffer(ros::Duration(config_.tf_buffer_size_s)));
  tf_listener_.reset(new tf2_ros::TransformListener(*buffer_));

  if (config_.use_image_receiver) {
    image_receiver_.reset(new ImageReceiver(
        nh,
        std::bind(&RosReconstruction::handlePointcloud, this, std::placeholders::_1),
        config_.image_queue_size));
    if (config_.publish_pointcloud) {
      pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud", 3);
    }
  } else {
    pcl_sub_ =
        nh_.subscribe("pointcloud", 10, &RosReconstruction::handlePointcloud, this);
  }

  if (!config_.make_pose_graph) {
    pose_graph_sub_ =
        nh_.subscribe("pose_graph", 1000, &RosReconstruction::handlePoseGraph, this);
  }

  pointcloud_thread_.reset(new std::thread(&RosReconstruction::pointcloudSpin, this));

  if (!config_.enable_output_queue && !output_queue) {
    // reset output queue so we don't waste memory with queued packets
    output_queue_.reset();
  }
}

RosReconstruction::~RosReconstruction() {
  stop();

  if (pointcloud_thread_) {
    VLOG(2) << "[Hydra Reconstruction] stopping pointcloud input thread";
    pointcloud_thread_->join();
    pointcloud_thread_.reset();
    VLOG(2) << "[Hydra Reconstruction] stopped pointcloud input thread";
  }

  VLOG(2) << "[Hydra Reconstruction] pointcloud queue: " << pointcloud_queue_.size();
}

std::string RosReconstruction::printInfo() const {
  std::stringstream ss;
  ss << config::toString(config_);
  return ss.str();
}

bool RosReconstruction::checkPointcloudTimestamp(const ros::Time& curr_time) {
  VLOG(1) << "[Hydra Reconstruction] Got raw pointcloud input @ " << curr_time.toNSec()
          << " [ns]";

  if (last_time_received_) {
    const auto separation_s = (curr_time - *last_time_received_).toSec();
    if (separation_s < config_.pointcloud_separation_s) {
      return false;
    }
  }

  last_time_received_.reset(new ros::Time(curr_time));
  VLOG(1) << "[Hydra Reconstruction] Got ROS input @ " << curr_time.toNSec() << " [ns]";
  return true;
}

void RosReconstruction::handlePointcloud(const PointCloud2::ConstPtr& msg) {
  if (!checkPointcloudTimestamp(msg->header.stamp)) {
    return;
  }

  pointcloud_queue_.push(msg);
}

void RosReconstruction::handlePoseGraph(const PoseGraph::ConstPtr& pose_graph) {
  if (pose_graph->nodes.empty()) {
    ROS_ERROR("Received empty pose graph!");
    return;
  }

  std::unique_lock<std::mutex> lock(pose_graph_mutex_);
  pose_graphs_.push_back(pose_graph);
}

void RosReconstruction::pointcloudSpin() {
  while (!should_shutdown_) {
    bool has_data = pointcloud_queue_.poll();
    if (!has_data) {
      continue;
    }

    const auto cloud = pointcloud_queue_.pop();
    const auto curr_time = cloud->header.stamp;
    if (config_.publish_pointcloud && config_.use_image_receiver) {
      pcl_pub_.publish(cloud);
    }

    VLOG(1) << "[Hydra Reconstruction] popped pointcloud input @ " << curr_time.toNSec()
            << " [ns]";

    ros::WallRate tf_wait_rate(1.0 / config_.tf_wait_duration_s);

    // note that this is okay in a separate thread from the callback queue because tf2
    // is threadsafe
    bool have_transform = false;
    std::string err_str;
    for (size_t i = 0; i < 5; ++i) {
      if (buffer_->canTransform(config_.world_frame,
                                config_.robot_frame,
                                curr_time,
                                ros::Duration(0),
                                &err_str)) {
        have_transform = true;
        break;
      }

      if (should_shutdown_) {
        return;
      }

      tf_wait_rate.sleep();
    }

    if (!have_transform) {
      LOG(WARNING) << "Failed to get tf from " << config_.robot_frame << " to "
                   << config_.world_frame << " @ " << curr_time.toNSec()
                   << " [ns]. Reason: " << err_str;
      continue;
    }

    geometry_msgs::TransformStamped transform;
    try {
      transform =
          buffer_->lookupTransform(config_.world_frame, config_.robot_frame, curr_time);
    } catch (const tf2::TransformException& ex) {
      LOG(ERROR) << "Failed to look up: " << config_.world_frame << " to "
                 << config_.robot_frame;
      continue;
    }

    ReconstructionInput::Ptr input(new ReconstructionInput());
    input->timestamp_ns = curr_time.toNSec();
    if (!fillVoxbloxPointcloud(*cloud,
                               input->pointcloud,
                               input->pointcloud_colors,
                               input->pointcloud_labels,
                               false)) {
      std::stringstream ss;
      for (const auto& field : cloud->fields) {
        ss << "  - " << field;
      }
      LOG(ERROR) << "Invalid pointcloud! fields:" << std::endl << ss.str();
      continue;
    }

    if (input->pointcloud_labels.size() != input->pointcloud.size()) {
      const auto colormap_ptr = HydraConfig::instance().getSemanticColorMap();
      if (!colormap_ptr || !colormap_ptr->isValid()) {
        LOG(ERROR)
            << "Label colormap not valid, but required for pointcloud conversion!";
      }

      const auto& label_colormap = *colormap_ptr;
      for (const auto& color : input->pointcloud_colors) {
        input->pointcloud_labels.push_back(label_colormap.getLabelFromColor(color));
      }
    }

    geometry_msgs::Pose curr_pose = tfToPose(transform.transform);
    tf2::convert(curr_pose.position, input->world_t_body);
    tf2::convert(curr_pose.orientation, input->world_R_body);

    {  // start pose graph critical section
      std::unique_lock<std::mutex> lock(pose_graph_mutex_);
      input->pose_graphs = pose_graphs_;
      pose_graphs_.clear();
    }  // end pose graph critical section

    queue_->push(input);
  }
}

}  // namespace hydra
