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
#include "hydra_ros/backend/ros_backend.h"

#include <config_utilities/printing.h>
#include <pose_graph_tools_ros/conversions.h>

namespace hydra {

using kimera_pgmo::DeformationGraph;
using kimera_pgmo::KimeraPgmoConfig;
using kimera_pgmo_msgs::KimeraPgmoMesh;
using pose_graph_tools_msgs::PoseGraph;

RosBackend::RosBackend(const Config& config,
                       const SharedDsgInfo::Ptr& dsg,
                       const SharedModuleState::Ptr& state,
                       const LogSetup::Ptr& log_setup)
    : BackendModule(config, dsg, state, log_setup), nh_("~") {
  pose_graph_sub_ = nh_.subscribe(
      "pose_graph_incremental", 10000, &RosBackend::poseGraphCallback, this);

  mesh_sub_.reset(new MeshSub(nh_, "pgmo/ful_mesh", 1));
  deformation_graph_sub_.reset(
      new PoseGraphSub(nh_, "pgmo/mesh_graph_incremental", 100));
  sync_.reset(new Sync(Policy(10), *mesh_sub_, *deformation_graph_sub_));
  sync_->registerCallback(boost::bind(&RosBackend::inputCallback, this, _1, _2));
}

RosBackend::~RosBackend() {}

std::string RosBackend::printInfo() const {
  std::stringstream ss;
  ss << config::toString(config);
  return ss.str();
}

void RosBackend::inputCallback(const KimeraPgmoMesh::ConstPtr& mesh,
                               const PoseGraph::ConstPtr& deformation_graph) {
  latest_mesh_msg_ = mesh;
  have_new_mesh_ = true;

  auto input = std::make_shared<BackendInput>();
  input->deformation_graph = std::make_shared<pose_graph_tools::PoseGraph>(
      pose_graph_tools::fromMsg(*deformation_graph));
  input->timestamp_ns = mesh->header.stamp.toNSec();
  for (const auto& graph : pose_graph_queue_) {
    input->agent_updates.pose_graphs.push_back(graph);
  }
  pose_graph_queue_.clear();

  state_->backend_queue.push(input);
}

void RosBackend::poseGraphCallback(const PoseGraph::ConstPtr& msg) {
  pose_graph_queue_.push_back(
      std::make_shared<pose_graph_tools::PoseGraph>(pose_graph_tools::fromMsg(*msg)));
}

}  // namespace hydra
