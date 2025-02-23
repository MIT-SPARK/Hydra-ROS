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
#include "hydra_ros/frontend/ros_frontend_publisher.h"

#include <hydra/common/global_info.h>

namespace hydra {

using kimera_pgmo::MeshDeltaTypeAdapter;
using pose_graph_tools::PoseGraphTypeAdapter;

RosFrontendPublisher::RosFrontendPublisher(ianvs::NodeHandle nh) {
  const auto odom_frame = GlobalInfo::instance().getFrames().odom;
  dsg_sender_.reset(new DsgSender(nh, odom_frame, "frontend", false));
  mesh_graph_pub_ = nh.create_publisher<PoseGraphTypeAdapter>(
      "mesh_graph_incremental", rclcpp::QoS(100).transient_local());
  mesh_update_pub_ = nh.create_publisher<MeshDeltaTypeAdapter>(
      "full_mesh_update", rclcpp::QoS(100).transient_local());
}

void RosFrontendPublisher::call(uint64_t timestamp_ns,
                                const DynamicSceneGraph& graph,
                                const BackendInput& backend_input) const {
  // TODO(nathan) make sure pgmo stamps the deformation graph
  mesh_graph_pub_->publish(backend_input.deformation_graph);
  if (backend_input.mesh_update) {
    backend_input.mesh_update->timestamp_ns = timestamp_ns;
    mesh_update_pub_->publish(*backend_input.mesh_update);
  }

  dsg_sender_->sendGraph(graph, rclcpp::Time(timestamp_ns));
}

}  // namespace hydra
