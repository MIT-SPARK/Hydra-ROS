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
#include "hydra_ros/visualizer/object_visualizer.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <hydra/common/hydra_config.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

namespace hydra {

using VertexCloud = pcl::PointCloud<pcl::PointXYZRGBA>;

void declare_config(ObjectVisualizerConfig& conf) {
  using namespace config;
  name("ObjectVisualizerConfig");
  field(conf.module_ns, "module_ns");
  field(conf.enable_active_mesh_pub, "enable_active_mesh_pub");
  field(conf.enable_segmented_mesh_pub, "enable_segmented_mesh_pub");
}

ObjectVisualizer::ObjectVisualizer(const ObjectVisualizerConfig& config)
    : config_(config), nh_(config.module_ns) {
  if (config_.enable_active_mesh_pub) {
    active_vertices_pub_ = nh_.advertise<VertexCloud>("active_vertices", 1, true);
  }

  if (config_.enable_segmented_mesh_pub) {
    segmented_vertices_pub_.reset(new ObjectCloudPub("object_vertices", nh_));
  }
}

ObjectVisualizer::~ObjectVisualizer() { segmented_vertices_pub_.reset(); }

void ObjectVisualizer::start() {}

void ObjectVisualizer::stop() {}

void ObjectVisualizer::save(const LogSetup&) {}

std::string ObjectVisualizer::printInfo() const {
  std::stringstream ss;
  ss << config::toString(config_);
  return ss.str();
}

void ObjectVisualizer::visualize(const kimera_pgmo::MeshDelta& delta,
                                 const std::vector<size_t>& active,
                                 const LabelIndices& label_indices) const {
  publishActiveVertices(delta, active, label_indices);
  publishObjectClouds(delta, active, label_indices);
}

void ObjectVisualizer::publishActiveVertices(const kimera_pgmo::MeshDelta& delta,
                                             const std::vector<size_t>& active,
                                             const LabelIndices&) const {
  VertexCloud::Ptr active_cloud(new VertexCloud());
  active_cloud->reserve(active.size());
  for (const auto idx : active) {
    active_cloud->push_back(delta.vertex_updates->at(idx));
  }

  active_cloud->header.frame_id = HydraConfig::instance().getFrames().odom;
  pcl_conversions::toPCL(ros::Time::now(), active_cloud->header.stamp);
  active_vertices_pub_.publish(active_cloud);
}

void ObjectVisualizer::publishObjectClouds(const kimera_pgmo::MeshDelta& delta,
                                           const std::vector<size_t>&,
                                           const LabelIndices& label_indices) const {
  for (auto&& [label, indices] : label_indices) {
    VertexCloud label_cloud;
    label_cloud.reserve(indices.size());
    for (const auto idx : indices) {
      label_cloud.push_back(delta.vertex_updates->at(idx));
    }

    label_cloud.header.frame_id = HydraConfig::instance().getFrames().odom;
    pcl_conversions::toPCL(ros::Time::now(), label_cloud.header.stamp);
    segmented_vertices_pub_->publish(label, label_cloud);
  }
}

}  // namespace hydra
