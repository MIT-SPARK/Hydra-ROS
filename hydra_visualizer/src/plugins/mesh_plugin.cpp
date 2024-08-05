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
#include "hydra_visualizer/plugins/mesh_plugin.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <kimera_pgmo_msgs/KimeraPgmoMesh.h>
#include <tf2_eigen/tf2_eigen.h>

#include "hydra_visualizer/color/colormap_utilities.h"

namespace hydra {

using spark_dsg::DynamicSceneGraph;

void declare_config(MeshPlugin::Config& config) {
  using namespace config;
  name("MeshPlugin::Config");
  field(config.use_color_adaptor, "use_color_adaptor");
  config.coloring.setOptional();
  field(config.coloring, "coloring");
}

MeshPlugin::MeshPlugin(const Config& config,
                       const ros::NodeHandle& nh,
                       const std::string& name)
    : VisualizerPlugin(nh, name),
      config(config::checkValid(config)),
      need_redraw_(true),
      use_color_adaptor_(config.use_color_adaptor),
      mesh_coloring_(config.coloring.create()) {
  if (mesh_coloring_) {
    toggle_service_ =
        nh_.advertiseService("use_color_adaptor", &MeshPlugin::handleService, this);
  }

  // namespacing gives us a reasonable topic
  mesh_pub_ = nh_.advertise<kimera_pgmo_msgs::KimeraPgmoMesh>("", 1, true);
}

MeshPlugin::~MeshPlugin() {}

void MeshPlugin::draw(const std_msgs::Header& header, const DynamicSceneGraph& graph) {
  auto mesh = graph.mesh();
  if (!mesh || mesh->empty()) {
    return;
  }

  if (use_color_adaptor_ && !mesh_coloring_) {
    ROS_WARN_STREAM("Invalid colormap; defaulting to original vertex color");
  }

  kimera_pgmo_msgs::KimeraPgmoMesh msg;
  msg.header = header;
  msg.ns = getMsgNamespace();

  std::shared_ptr<const MeshColoring> default_func;
  if (!mesh->has_colors) {
    UniformMeshColoring::Config config{spark_dsg::Color::gray()};
    default_func = std::make_shared<UniformMeshColoring>(config);
  }

  MeshColorAdaptor adaptor(*mesh, use_color_adaptor_ ? mesh_coloring_ : default_func);
  msg.vertices.resize(mesh->points.size());
  msg.vertex_colors.resize(mesh->points.size());
  for (size_t i = 0; i < mesh->points.size(); ++i) {
    auto& vertex = msg.vertices[i];
    tf2::convert(mesh->points[i].cast<double>().eval(), vertex);
    auto& color = msg.vertex_colors[i];
    color = visualizer::makeColorMsg(adaptor.getVertexColor(i));
  }

  msg.triangles.resize(mesh->faces.size());
  for (size_t i = 0; i < mesh->faces.size(); ++i) {
    const auto& face = mesh->faces[i];
    auto& triangle = msg.triangles[i].vertex_indices;
    triangle[0] = face[0];
    triangle[1] = face[1];
    triangle[2] = face[2];
  }

  mesh_pub_.publish(msg);
}

void MeshPlugin::reset(const std_msgs::Header& header) {
  kimera_pgmo_msgs::KimeraPgmoMesh msg;
  msg.header = header;
  msg.ns = getMsgNamespace();
  mesh_pub_.publish(msg);
}

std::string MeshPlugin::getMsgNamespace() const {
  // TODO(lschmid): Hardcoded for now. Eventually read from scene graph or so.
  return "robot0/dsg_mesh";
}

bool MeshPlugin::hasChange() const { return need_redraw_; }

void MeshPlugin::clearChangeFlag() { need_redraw_ = false; }

bool MeshPlugin::handleService(std_srvs::SetBool::Request& req,
                               std_srvs::SetBool::Response& res) {
  use_color_adaptor_ = req.data;
  res.success = true;
  need_redraw_ = true;
  return true;
}

}  // namespace hydra
