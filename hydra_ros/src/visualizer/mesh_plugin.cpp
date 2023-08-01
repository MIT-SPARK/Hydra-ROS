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
#include "hydra_ros/visualizer/mesh_plugin.h"

#include <kimera_pgmo/utils/CommonFunctions.h>
#include <kimera_semantics/color.h>
#include <mesh_msgs/TriangleMeshStamped.h>

namespace hydra {

using VerticesPtr = MeshPlugin::Vertices::Ptr;

MeshPlugin::MeshPlugin(const ros::NodeHandle& nh, const std::string& name)
    : DsgVisualizerPlugin(nh, name), color_by_label_(false) {
  nh_.getParam("color_by_label", color_by_label_);

  std::string label_colormap = "";
  nh_.getParam("label_colormap", label_colormap);
  if (!label_colormap.empty()) {
    colormap_ = kimera::SemanticColorMap::fromFile(label_colormap);
    if (!colormap_) {
      ROS_WARN_STREAM("Unable to load colormap from " << label_colormap);
    }
  }

  // namespacing gives us a reasonable topic
  mesh_pub_ = nh_.advertise<mesh_msgs::TriangleMeshStamped>("", 1, true);
  toggle_service_ =
      nh_.advertiseService("color_by_label", &MeshPlugin::handleService, this);
}

MeshPlugin::~MeshPlugin() {}

bool MeshPlugin::handleService(std_srvs::SetBool::Request& req,
                               std_srvs::SetBool::Response& res) {
  color_by_label_ = req.data;
  res.success = true;
  return true;
}

VerticesPtr MeshPlugin::colorVerticesByLabel(const Vertices::Ptr& vertices,
                                             const LabelsPtr& labels) const {
  if (!colormap_ || !colormap_->isValid()) {
    ROS_WARN_STREAM("Invalid colormap; defaulting to original vertex color");
    return vertices;
  }

  if (!labels || labels->size() != vertices->size()) {
    ROS_WARN_STREAM("Mesh contains no labels; defaulting to original vertex color");
    return vertices;
  }

  Vertices::Ptr new_vertices(new Vertices(*vertices));
  for (size_t i = 0; i < new_vertices->size(); ++i) {
    auto& point = new_vertices->at(i);
    const auto color = colormap_->getColorFromLabel(labels->at(i));
    point.r = color.r;
    point.g = color.g;
    point.b = color.b;
    point.a = color.a;
  }

  return new_vertices;
}

void MeshPlugin::draw(const std_msgs::Header& header, const DynamicSceneGraph& graph) {
  if (!graph.hasMesh()) {
    return;
  }

  if (!graph.getMeshVertices()->size()) {
    return;
  }

  mesh_msgs::TriangleMeshStamped msg;
  msg.header = header;

  auto vertices = graph.getMeshVertices();
  if (color_by_label_) {
    vertices = colorVerticesByLabel(vertices, graph.getMeshLabels());
  }

  // vertices and meshes are guaranteed to not be null (from hasMesh)
  msg.mesh =
      kimera_pgmo::PolygonMeshToTriangleMeshMsg(*vertices, *graph.getMeshFaces());
  mesh_pub_.publish(msg);
}

void MeshPlugin::reset(const std_msgs::Header& header, const DynamicSceneGraph&) {
  mesh_msgs::TriangleMeshStamped msg;
  msg.header = header;
  mesh_pub_.publish(msg);
}

}  // namespace hydra
