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
#include "hydra_ros/reconstruction/ros_reconstruction.h"

#include <config_utilities/printing.h>
#include <hydra/common/hydra_config.h>

#include "hydra_ros/reconstruction/image_receiver.h"
#include "hydra_ros/reconstruction/pointcloud_receiver.h"
#include "hydra_ros/utils/lookup_tf.h"

namespace hydra {

using pose_graph_tools::PoseGraph;
using sensor_msgs::PointCloud2;

RosReconstruction::RosReconstruction(const RosReconstructionConfig& config,
                                     const OutputQueue::Ptr& output_queue)
    : ReconstructionModule(
          config, output_queue ? output_queue : std::make_shared<OutputQueue>()),
      config_(config),
      nh_(ros::NodeHandle(config.reconstruction_ns)) {
  buffer_.reset(new tf2_ros::Buffer(ros::Duration(config_.tf_buffer_size_s)));
  tf_listener_.reset(new tf2_ros::TransformListener(*buffer_));
  data_queue_.reset(new DataQueue());

  if (config_.use_image_receiver) {
    data_receiver_.reset(new ImageReceiver(
        nh_, data_queue_, config_.input_separation_s, config_.image_queue_size));
  } else {
    data_receiver_.reset(new PointcloudReceiver(
        nh_, data_queue_, config_.input_separation_s, config_.image_queue_size));
  }

  if (!config_.pose_graphs.make_pose_graph) {
    pose_graph_sub_ =
        nh_.subscribe("pose_graph", 1000, &RosReconstruction::handlePoseGraph, this);

    agent_node_meas_sub_ =
        nh_.subscribe("agent_node_measurements",
                      1,
                      &RosReconstruction::handleAgentNodeMeasurements,
                      this);
  }

  data_thread_.reset(new std::thread(&RosReconstruction::dataSpin, this));

  if (!config_.enable_output_queue && !output_queue) {
    // reset output queue so we don't waste memory with queued packets
    output_queue_.reset();
  }
}

RosReconstruction::~RosReconstruction() {
  stop();

  if (data_thread_) {
    VLOG(2) << "[Hydra Reconstruction] stopping pointcloud input thread";
    data_thread_->join();
    data_thread_.reset();
    VLOG(2) << "[Hydra Reconstruction] stopped pointcloud input thread";
  }

  VLOG(2) << "[Hydra Reconstruction] data queue: " << data_queue_->size();
}

std::string RosReconstruction::printInfo() const {
  std::stringstream ss;
  ss << config::toString(config_);
  return ss.str();
}

void RosReconstruction::handlePoseGraph(const PoseGraph::ConstPtr& pose_graph) {
  if (pose_graph->nodes.empty()) {
    ROS_ERROR("Received empty pose graph!");
    return;
  }

  std::unique_lock<std::mutex> lock(pose_graph_mutex_);
  pose_graphs_.push_back(pose_graph);
}

void RosReconstruction::handleAgentNodeMeasurements(const PoseGraph::ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(pose_graph_mutex_);
  agent_node_measurements_ = msg;
}

void RosReconstruction::dataSpin() {
  while (!should_shutdown_) {
    bool has_data = data_queue_->poll();
    if (!has_data) {
      continue;
    }

    const auto packet = data_queue_->pop();
    const auto curr_time = packet->timestamp_ns;
    VLOG(1) << "[Hydra Reconstruction] popped input @ " << curr_time << " [ns]";

    ros::Time curr_ros_time;
    curr_ros_time.fromNSec(curr_time);
    const auto pose_status = lookupTransform(*buffer_,
                                             curr_ros_time,
                                             HydraConfig::instance().getFrames().odom,
                                             HydraConfig::instance().getFrames().robot,
                                             5,  // max tries
                                             config_.tf_wait_duration_s);
    if (!pose_status.is_valid) {
      LOG(WARNING) << "[Hydra Reconstruction] dropping input @ " << curr_time
                   << " [ns] due to missing pose";
      continue;
    }

    ReconstructionInput::Ptr input(new ReconstructionInput());
    input->timestamp_ns = curr_time;
    input->sensor_input = packet;
    input->world_t_body = pose_status.target_p_source;
    input->world_R_body = pose_status.target_R_source;

    {  // start pose graph critical section
      std::unique_lock<std::mutex> lock(pose_graph_mutex_);
      input->pose_graphs = pose_graphs_;
      input->agent_node_measurements = agent_node_measurements_;
      pose_graphs_.clear();
    }

    queue_->push(input);
  }
}

}  // namespace hydra
