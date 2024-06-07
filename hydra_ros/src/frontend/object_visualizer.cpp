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
#include "hydra_ros/frontend/object_visualizer.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/common/global_info.h>

namespace hydra {

using visualization_msgs::Marker;

void declare_config(ObjectVisualizer::Config& config) {
  using namespace config;
  name("ObjectVisualizerConfig");
  field(config.module_ns, "module_ns");
  field(config.enable_active_mesh_pub, "enable_active_mesh_pub");
  field(config.enable_segmented_mesh_pub, "enable_segmented_mesh_pub");
  field(config.point_scale, "point_scale");
  field(config.point_alpha, "point_alpha");
  field(config.use_spheres, "use_spheres");
}

ObjectVisualizer::ObjectVisualizer(const Config& config)
    : config(config::checkValid(config)), nh_(config.module_ns) {
  if (config.enable_active_mesh_pub) {
    active_vertices_pub_ = nh_.advertise<Marker>("active_vertices", 1, true);
  }

  if (config.enable_segmented_mesh_pub) {
    segmented_vertices_pub_.reset(new ObjectCloudPub("object_vertices", nh_));
  }
}

ObjectVisualizer::~ObjectVisualizer() { segmented_vertices_pub_.reset(); }

std::string ObjectVisualizer::printInfo() const {
  std::stringstream ss;
  ss << config::toString(config);
  return ss.str();
}

void ObjectVisualizer::call(uint64_t timestamp_ns,
                            const kimera_pgmo::MeshDelta& delta,
                            const std::vector<size_t>& active,
                            const LabelIndices& label_indices) const {
  publishActiveVertices(timestamp_ns, delta, active);
  publishObjectClouds(timestamp_ns, delta, label_indices);
}

void ObjectVisualizer::publishActiveVertices(uint64_t timestamp_ns,
                                             const kimera_pgmo::MeshDelta& delta,
                                             const std::vector<size_t>& active) const {
  if (active_vertices_pub_.getNumSubscribers() < 1) {
    return;
  }

  visualization_msgs::Marker msg;
  msg.header.stamp.fromNSec(timestamp_ns);
  msg.header.frame_id = GlobalInfo::instance().getFrames().odom;
  msg.ns = "active_vertices";
  msg.id = 0;
  fillMarkerFromCloud(delta, active, msg);
  active_vertices_pub_.publish(msg);
}

void ObjectVisualizer::publishObjectClouds(uint64_t timestamp_ns,
                                           const kimera_pgmo::MeshDelta& delta,
                                           const LabelIndices& label_indices) const {
  if (!segmented_vertices_pub_) {
    return;
  }

  for (auto&& [label, indices] : label_indices) {
    visualization_msgs::Marker msg;
    msg.header.stamp.fromNSec(timestamp_ns);
    msg.header.frame_id = GlobalInfo::instance().getFrames().odom;
    msg.ns = "label_vertices_" + std::to_string(label);
    msg.id = 0;
    fillMarkerFromCloud(delta, indices, msg);
    segmented_vertices_pub_->publish(label, msg);
  }
}

void ObjectVisualizer::fillMarkerFromCloud(const kimera_pgmo::MeshDelta& delta,
                                           const std::vector<size_t>& indices,
                                           Marker& msg) const {
  msg.type = config.use_spheres ? Marker::SPHERE_LIST : Marker::CUBE_LIST;
  msg.action = visualization_msgs::Marker::ADD;

  msg.scale.x = config.point_scale;
  msg.scale.y = config.point_scale;
  msg.scale.z = config.point_scale;
  msg.pose.orientation.w = 1.0;

  msg.points.reserve(indices.size());
  msg.colors.reserve(indices.size());
  for (const auto idx : indices) {
    const auto& p = delta.vertex_updates->at(idx);
    auto& point = msg.points.emplace_back();
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    auto& color = msg.colors.emplace_back();
    color.r = p.r / 255.0f;
    color.g = p.g / 255.0f;
    color.b = p.b / 255.0f;
    color.a = config.point_alpha;
  }
}

}  // namespace hydra
