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
#include "hydra_ros/backend/ros_backend_publisher.h"

#include <hydra/common/hydra_config.h>
#include <spark_dsg/zmq_interface.h>

namespace hydra {

using kimera_pgmo::DeformationGraph;
using kimera_pgmo::KimeraPgmoConfig;
using kimera_pgmo::KimeraPgmoMesh;
using mesh_msgs::TriangleMeshStamped;
using pose_graph_tools::PoseGraph;
using visualization_msgs::Marker;

RosBackendPublisher::RosBackendPublisher(const ros::NodeHandle& nh,
                                         const BackendConfig& config)
    : nh_(nh), config_(config), zmq_publish_mesh_(true), last_zmq_pub_time_(0) {
  mesh_mesh_edges_pub_ =
      nh_.advertise<Marker>("pgmo/deformation_graph_mesh_mesh", 10, false);
  pose_mesh_edges_pub_ =
      nh_.advertise<Marker>("pgmo/deformation_graph_pose_mesh", 10, false);
  pose_graph_pub_ = nh_.advertise<PoseGraph>("pgmo/pose_graph", 10, false);

  double min_mesh_separation_s = 0.0;
  nh_.getParam("min_mesh_separation_s", min_mesh_separation_s);
  nh_.getParam("zmq_publish_mesh", zmq_publish_mesh_);

  dsg_sender_.reset(new hydra::DsgSender(nh_,
                                         "backend",
                                         HydraConfig::instance().getFrames().map,
                                         false,
                                         min_mesh_separation_s));
  if (config_.use_zmq_interface) {
    zmq_sender_.reset(
        new spark_dsg::ZmqSender(config_.zmq_send_url, config_.zmq_num_threads));
  }
}

RosBackendPublisher::~RosBackendPublisher() {}

void RosBackendPublisher::publish(const DynamicSceneGraph& graph,
                                  const DeformationGraph& dgraph,
                                  size_t timestamp_ns) {
  ros::Time stamp;
  stamp.fromNSec(timestamp_ns);

  // TODO(nathan) consider serializing to bytes before sending
  dsg_sender_->sendGraph(graph, stamp);

  if (config_.use_zmq_interface) {
    // TODO(nathan) handle this better
    //&& timestamp_ns - last_zmq_pub_time_ > 9000000000
    zmq_sender_->send(graph, zmq_publish_mesh_);
    last_zmq_pub_time_ = timestamp_ns;
  }

  if (pose_graph_pub_.getNumSubscribers() > 0) {
    publishPoseGraph(graph, dgraph);
  }

  if (mesh_mesh_edges_pub_.getNumSubscribers() > 0 ||
      pose_mesh_edges_pub_.getNumSubscribers() > 0) {
    publishDeformationGraphViz(dgraph, timestamp_ns);
  }
}

void RosBackendPublisher::publishPoseGraph(const DynamicSceneGraph& graph,
                                           const DeformationGraph& dgraph) const {
  const auto& prefix = HydraConfig::instance().getRobotPrefix();
  const auto& agent = graph.getLayer(DsgLayers::AGENTS, prefix.key);

  std::map<size_t, std::vector<size_t>> id_timestamps;
  id_timestamps[prefix.id] = std::vector<size_t>();
  auto& times = id_timestamps[prefix.id];
  for (const auto& node : agent.nodes()) {
    times.push_back(node->timestamp.count());
  }

  const auto& pose_graph = dgraph.getPoseGraph(id_timestamps);
  pose_graph_pub_.publish(*pose_graph);
}

void RosBackendPublisher::publishDeformationGraphViz(const DeformationGraph& dgraph,
                                                     size_t timestamp_ns) const {
  ros::Time stamp;
  stamp.fromNSec(timestamp_ns);

  Marker mm_edges_msg;
  Marker pm_edges_msg;
  kimera_pgmo::fillDeformationGraphMarkers(dgraph, stamp, mm_edges_msg, pm_edges_msg);

  if (!mm_edges_msg.points.empty()) {
    mesh_mesh_edges_pub_.publish(mm_edges_msg);
  }
  if (!pm_edges_msg.points.empty()) {
    pose_mesh_edges_pub_.publish(pm_edges_msg);
  }
}

}  // namespace hydra
