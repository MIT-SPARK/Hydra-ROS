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
#pragma once
#include <hydra/backend/backend_module.h>
#include <kimera_pgmo_msgs/KimeraPgmoMesh.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pose_graph_tools_msgs/PoseGraph.h>

namespace hydra {

class RosBackend : public BackendModule {
 public:
  using Policy =
      message_filters::sync_policies::ApproximateTime<kimera_pgmo_msgs::KimeraPgmoMesh,
                                                      pose_graph_tools_msgs::PoseGraph>;
  using Sync = message_filters::Synchronizer<Policy>;
  using PoseGraphSub = message_filters::Subscriber<pose_graph_tools_msgs::PoseGraph>;
  using MeshSub = message_filters::Subscriber<kimera_pgmo_msgs::KimeraPgmoMesh>;

  RosBackend(const Config& config,
             const SharedDsgInfo::Ptr& dsg,
             const SharedModuleState::Ptr& state,
             const LogSetup::Ptr& log_setup);

  ~RosBackend();

  std::string printInfo() const override;

  void inputCallback(
      const kimera_pgmo_msgs::KimeraPgmoMesh::ConstPtr& mesh,
      const pose_graph_tools_msgs::PoseGraph::ConstPtr& deformation_graph);

  void poseGraphCallback(const pose_graph_tools_msgs::PoseGraph::ConstPtr& msg);

 protected:
  void publishOutputs(const pcl::PolygonMesh& mesh, size_t timestamp_ns) const;

  void publishDeformationGraphViz() const;

  void publishPoseGraphViz() const;

  void publishUpdatedMesh(const pcl::PolygonMesh& mesh, size_t timestamp_ns) const;

 protected:
  ros::NodeHandle nh_;
  std::list<pose_graph_tools::PoseGraph::ConstPtr> pose_graph_queue_;
  kimera_pgmo_msgs::KimeraPgmoMesh::ConstPtr latest_mesh_msg_;

  ros::Subscriber pose_graph_sub_;
  std::unique_ptr<PoseGraphSub> deformation_graph_sub_;
  std::unique_ptr<MeshSub> mesh_sub_;
  std::unique_ptr<Sync> sync_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<BackendModule,
                                     RosBackend,
                                     Config,
                                     SharedDsgInfo::Ptr,
                                     SharedModuleState::Ptr,
                                     LogSetup::Ptr>("RosBackend");
};

}  // namespace hydra
