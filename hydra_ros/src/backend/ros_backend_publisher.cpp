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

#include <hydra/common/global_info.h>
#include <kimera_pgmo_ros/visualization_functions.h>
#include <pose_graph_tools_msgs/PoseGraph.h>
#include <pose_graph_tools_ros/conversions.h>
#include <visualization_msgs/Marker.h>

namespace hydra {

using kimera_pgmo::DeformationGraph;
using kimera_pgmo::KimeraPgmoConfig;
using kimera_pgmo_msgs::KimeraPgmoMesh;
using pose_graph_tools_msgs::PoseGraph;
using visualization_msgs::Marker;

RosBackendPublisher::RosBackendPublisher(const ros::NodeHandle& nh) : nh_(nh) {
  mesh_mesh_edges_pub_ =
      nh_.advertise<Marker>("deformation_graph_mesh_mesh", 10, false);
  pose_mesh_edges_pub_ =
      nh_.advertise<Marker>("deformation_graph_pose_mesh", 10, false);
  pose_graph_pub_ = nh_.advertise<PoseGraph>("pose_graph", 10, false);

  double separation = 0.0;
  nh_.getParam("min_mesh_separation_s", separation);
  const auto map_frame = GlobalInfo::instance().getFrames().map;
  dsg_sender_.reset(new hydra::DsgSender(nh_, map_frame, "backend", false, separation));
}

void RosBackendPublisher::call(uint64_t timestamp_ns,
                               const DynamicSceneGraph& graph,
                               const DeformationGraph& dgraph) const {
  ros::Time stamp;
  stamp.fromNSec(timestamp_ns);
  dsg_sender_->sendGraph(graph, stamp);

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
  const auto& prefix = GlobalInfo::instance().getRobotPrefix();
  const auto& agent = graph.getLayer(DsgLayers::AGENTS, prefix.key);

  std::map<size_t, std::vector<size_t>> id_timestamps;
  id_timestamps[prefix.id] = std::vector<size_t>();
  auto& times = id_timestamps[prefix.id];
  for (const auto& node : agent.nodes()) {
    times.push_back(node->timestamp.value().count());
  }

  const auto& pose_graph = *dgraph.getPoseGraph(id_timestamps);
  pose_graph_pub_.publish(pose_graph_tools::toMsg(pose_graph));
}

void RosBackendPublisher::publishDeformationGraphViz(const DeformationGraph& dgraph,
                                                     size_t timestamp_ns) const {
  ros::Time stamp;
  stamp.fromNSec(timestamp_ns);

  Marker mm_edges_msg;
  Marker pm_edges_msg;
  kimera_pgmo::fillDeformationGraphMarkers(dgraph,
                                           stamp,
                                           mm_edges_msg,
                                           pm_edges_msg,
                                           GlobalInfo::instance().getFrames().map);

  if (!mm_edges_msg.points.empty()) {
    mesh_mesh_edges_pub_.publish(mm_edges_msg);
  }
  if (!pm_edges_msg.points.empty()) {
    pose_mesh_edges_pub_.publish(pm_edges_msg);
  }
}

}  // namespace hydra
