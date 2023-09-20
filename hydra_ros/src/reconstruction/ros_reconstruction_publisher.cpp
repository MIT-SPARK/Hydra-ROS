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
#include <geometry_msgs/TransformStamped.h>
#include <hydra/common/hydra_config.h>
#include <hydra/places/gvd_integrator.h>
#include <hydra_msgs/ActiveLayer.h>
#include <hydra_msgs/ActiveMesh.h>
#include <hydra_msgs/QueryFreespace.h>
#include <kimera_semantics/color.h>
#include <tf2_eigen/tf2_eigen.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_ros/mesh_vis.h>

#include "hydra_ros/reconstruction/ros_reconstruction.h"
#include "hydra_ros/utils/image_receiver.h"
#include "hydra_ros/utils/pointcloud_adaptor.h"

namespace hydra {

/*  output_callbacks_.push_back(*/
/*[this](const ReconstructionModule&, const ReconstructionOutput& output) {*/
/*this->visualize(output);*/
/*});*/

RosReconstructionPublisher::RosReconstructionPublisher(const ros::NodeHandle& nh)
    : nh_(nh) {
  if (config_.publish_mesh) {
    mesh_pub_ = nh_.advertise<voxblox_msgs::Mesh>("mesh", 10);
  }

  bool visualize_reconstruction = true;
  std::string topology_visualizer_ns = "~";
  field(conf.visualize_reconstruction, "visualize_reconstruction");
  field(conf.topology_visualizer_ns, "topology_visualizer_ns");

  if (config_.visualize_reconstruction) {
    visualizer_.reset(new TopologyServerVisualizer(config_.topology_visualizer_ns));
  }
}

RosReconstructionPublisher::~RosReconstructionPublisher() { stop(); }

void RosReconstructionPublisher::visualize(const ReconstructionOutput& output) {
  if (config_.publish_mesh && output.mesh) {
    hydra_msgs::ActiveMesh::ConstPtr msg(new hydra_msgs::ActiveMesh());
    auto mesh = const_cast<hydra_msgs::ActiveMesh&>(*msg);
    mesh_pub_.publish(msg);
  }

  if (visualizer_) {
    visualizer_->visualize(gvd_integrator_->getGraph(),
                           gvd_integrator_->getGvdGraph(),
                           *gvd_,
                           *tsdf_,
                           output.timestamp_ns,
                           mesh_->getVoxbloxMesh().get());
    if (config_.gvd.graph_extractor.use_compression_extractor) {
      visualizer_->visualizeExtractor(output.timestamp_ns,
                                      dynamic_cast<CompressionGraphExtractor&>(
                                          gvd_integrator_->getGraphExtractor()));
    }
  }
}

}  // namespace hydra
