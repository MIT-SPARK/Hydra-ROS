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

#include <hydra/common/semantic_color_map.h>
#include <kimera_pgmo/utils/CommonFunctions.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <spark_dsg/pgmo_mesh_traits.h>

namespace hydra {

struct ColoredLabelAdaptor {
  ColoredLabelAdaptor(const Mesh& mesh, const SemanticColorMap& cmap)
      : mesh(mesh), cmap(cmap) {}

  const Mesh& mesh;
  const SemanticColorMap& cmap;
};

Eigen::Vector3f pgmoGetVertex(const ColoredLabelAdaptor& mesh,
                              size_t i,
                              std::optional<kimera_pgmo::traits::Color>* color,
                              std::optional<uint8_t>* alpha,
                              std::optional<uint64_t>*,
                              std::optional<uint32_t>*) {
  CHECK(mesh.mesh.has_labels);
  const auto c = mesh.cmap.getColorFromLabel(mesh.mesh.label(i));
  *color = kimera_pgmo::traits::Color{{c.r, c.g, c.b}};
  *alpha = c.a;
  return mesh.mesh.pos(i);
}

size_t pgmoNumVertices(const ColoredLabelAdaptor& mesh) {
  return pgmoNumVertices(mesh.mesh);
}

size_t pgmoNumFaces(const ColoredLabelAdaptor& mesh) { return pgmoNumFaces(mesh.mesh); }

kimera_pgmo::traits::Face pgmoGetFace(const ColoredLabelAdaptor& mesh, size_t i) {
  return pgmoGetFace(mesh.mesh, i);
}

MeshPlugin::MeshPlugin(const ros::NodeHandle& nh, const std::string& name)
    : DsgVisualizerPlugin(nh, name), color_by_label_(false) {
  nh_.getParam("color_by_label", color_by_label_);

  std::string label_colormap = "";
  nh_.getParam("label_colormap", label_colormap);
  if (!label_colormap.empty()) {
    colormap_ = SemanticColorMap::fromCsv(label_colormap);
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

void MeshPlugin::draw(const std_msgs::Header& header, const DynamicSceneGraph& graph) {
  auto mesh = graph.mesh();
  if (!mesh || mesh->empty()) {
    return;
  }

  mesh_msgs::TriangleMeshStamped msg;
  msg.header = header;

  const auto invalid_colormap = !colormap_ || !colormap_->isValid();
  if (color_by_label_ && invalid_colormap) {
    ROS_WARN_STREAM("Invalid colormap; defaulting to original vertex color");
  }

  if (color_by_label_ && !invalid_colormap) {
    ColoredLabelAdaptor adaptor(*mesh, *colormap_);
    kimera_pgmo::fillTriangleMeshMsg(adaptor, msg.mesh);
  } else {
    kimera_pgmo::fillTriangleMeshMsg(*mesh, msg.mesh);
  }

  mesh_pub_.publish(msg);
}

void MeshPlugin::reset(const std_msgs::Header& header, const DynamicSceneGraph&) {
  mesh_msgs::TriangleMeshStamped msg;
  msg.header = header;
  mesh_pub_.publish(msg);
}

}  // namespace hydra
