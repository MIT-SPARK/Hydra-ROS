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

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/semantic_color_map.h>
#include <hydra/utils/pgmo_mesh_traits.h>
#include <kimera_pgmo_msgs/KimeraPgmoMesh.h>
#include <kimera_pgmo_ros/conversion/ros_conversion.h>

#include "hydra_ros/visualizer/mesh_color_adaptor.h"

namespace hydra {

void declare_config(MeshPlugin::Config& config) {
  using namespace config;
  name("MeshPlugin::Config");
  field(config.label_colormap, "label_colormap");
  field(config.color_by_label, "color_by_label");
}

MeshPlugin::MeshPlugin(const Config& config,
                       const ros::NodeHandle& nh,
                       const std::string& name)
    : DsgVisualizerPlugin(nh, name), config(config::checkValid(config)) {
  if (!config.label_colormap.empty()) {
    colormap_ = SemanticColorMap::fromCsv(config.label_colormap);
    if (!colormap_) {
      ROS_WARN_STREAM("Unable to load colormap from " << config.label_colormap);
    } else {
      color_by_label_ = config.color_by_label;
      toggle_service_ =
          nh_.advertiseService("color_by_label", &MeshPlugin::handleService, this);
      mesh_coloring_ = std::make_shared<SemanticMeshColoring>(*colormap_);
    }
  }

  // namespacing gives us a reasonable topic
  mesh_pub_ = nh_.advertise<kimera_pgmo_msgs::KimeraPgmoMesh>("", 1, true);
}

MeshPlugin::~MeshPlugin() {}

void MeshPlugin::draw(const ConfigManager&,
                      const std_msgs::Header& header,
                      const DynamicSceneGraph& graph) {
  auto mesh = graph.mesh();
  if (!mesh || mesh->empty()) {
    return;
  }

  const auto invalid_colormap = !colormap_ || !colormap_->isValid();
  if (color_by_label_ && invalid_colormap) {
    ROS_WARN_STREAM("Invalid colormap; defaulting to original vertex color");
  }

  kimera_pgmo_msgs::KimeraPgmoMesh msg;
  if (color_by_label_ && !invalid_colormap) {
    const MeshColorAdaptor adaptor(*mesh, mesh_coloring_);
    msg = kimera_pgmo::conversions::toMsg(adaptor);
  } else {
    msg = kimera_pgmo::conversions::toMsg(*mesh);
  }
  msg.header = header;
  msg.ns = getMsgNamespace();
  mesh_pub_.publish(msg);
}

void MeshPlugin::reset(const std_msgs::Header& header, const DynamicSceneGraph&) {
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
  color_by_label_ = req.data;
  res.success = true;
  need_redraw_ = true;
  return true;
}

}  // namespace hydra
