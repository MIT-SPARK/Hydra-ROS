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
#include "hydra_ros/visualizer/footprint_plugin.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/MarkerArray.h>

#include "hydra_ros/visualizer/colormap_utilities.h"
#include "hydra_ros/visualizer/polygon_utilities.h"
#include "hydra_ros/visualizer/visualizer_utilities.h"

namespace hydra {

struct LayerIdConversion {
  static std::string toIntermediate(const LayerId& layer_id, std::string&) {
    return spark_dsg::DsgLayers::LayerIdToString(layer_id);
  }

  static void fromIntermediate(const std::string& layer_name,
                               LayerId& layer_id,
                               std::string&) {
    layer_id = spark_dsg::DsgLayers::StringToLayerId(layer_name);
  }
};

void declare_config(FootprintPlugin::Config& config) {
  using namespace config;
  name("FootprintPlugin::Config");
  field(config.use_place_radius, "use_place_radius");
  field(config.draw_boundaries, "draw_boundaries");
  field(config.draw_boundary_vertices, "draw_boundary_vertices");
  field(config.line_width, "line_width");
  field(config.line_alpha, "line_alpha");
  field(config.mesh_alpha, "mesh_alpha");
  field(config.footprint_radius, "footprint_radius");
  field(config.num_samples, "num_samples");
  field<LayerIdConversion>(config.layer_id, "layer_id");

  check(config.line_width, GT, 0.0, "line_width");
  check(config.line_alpha, GT, 0.0, "line_alpha");
}

FootprintPlugin::FootprintPlugin(const Config& config,
                                 const ros::NodeHandle& nh,
                                 const std::string& name)
    : DsgVisualizerPlugin(nh, name), config(config::checkValid(config)) {
  // namespacing gives us a reasonable topic
  pub_ = nh_.advertise<visualization_msgs::MarkerArray>("", 1, true);
}

FootprintPlugin::~FootprintPlugin() {}

void FootprintPlugin::draw(const ConfigManager&,
                           const std_msgs::Header& header,
                           const DynamicSceneGraph& graph) {
  visualization_msgs::MarkerArray msg;
  msg.markers.resize(config.draw_boundaries ? (config.draw_boundary_vertices ? 3 : 2)
                                            : 1);

  std::string ns = "layer_" + std::to_string(config.layer_id) + "_footprints";
  msg.markers[0].header = header;
  msg.markers[0].ns = ns + "_polygons";
  msg.markers[0].id = 0;
  msg.markers[0].type = visualization_msgs::Marker::TRIANGLE_LIST;
  msg.markers[0].action = visualization_msgs::Marker::ADD;
  msg.markers[0].pose.orientation.w = 1.0;
  msg.markers[0].color.a = config.mesh_alpha;
  msg.markers[0].scale.x = 1.0;
  msg.markers[0].scale.y = 1.0;
  msg.markers[0].scale.z = 1.0;

  if (config.draw_boundaries) {
    msg.markers[1].header = header;
    msg.markers[1].ns = ns + "_boundaries";
    msg.markers[1].id = 0;
    msg.markers[1].type = visualization_msgs::Marker::LINE_LIST;
    msg.markers[1].action = visualization_msgs::Marker::ADD;
    msg.markers[1].pose.orientation.w = 1.0;
    msg.markers[1].scale.x = config.line_width;
    msg.markers[1].scale.y = 0.0;
    msg.markers[1].scale.z = 0.0;

    if (config.draw_boundary_vertices) {
      msg.markers[2].header = header;
      msg.markers[2].ns = ns + "_vertices";
      msg.markers[2].id = 0;
      msg.markers[2].type = visualization_msgs::Marker::SPHERE_LIST;
      msg.markers[2].action = visualization_msgs::Marker::ADD;
      msg.markers[2].pose.orientation.w = 1.0;
      msg.markers[2].scale.x = config.line_width;
      msg.markers[2].scale.y = config.line_width;
      msg.markers[2].scale.z = config.line_width;
    }
  }

  const auto& layer = graph.getLayer(config.layer_id);
  for (auto&& [id, node] : layer.nodes()) {
    const auto mean_z = getMeanNeighborHeight(layer, *node);
    const auto& attrs = node->attributes<SemanticNodeAttributes>();

    auto color = dsg_utils::makeColorMsg(attrs.color);
    color.a = config.line_alpha;

    double radius = config.footprint_radius;
    if (config.use_place_radius) {
      radius = node->attributes<PlaceNodeAttributes>().distance;
    }

    const auto footprint = getCirclePolygon(*node, radius, config.num_samples);

    auto mesh_color = color;
    mesh_color.a = config.mesh_alpha;
    makeFilledPolygon(footprint, mesh_color, msg.markers[0], mean_z);
    if (config.draw_boundaries) {
      makePolygonBoundary(footprint,
                          color,
                          msg.markers[1],
                          mean_z,
                          config.draw_boundary_vertices ? &msg.markers[2] : nullptr);
    }
  }

  for (const auto& marker : msg.markers) {
    namespaces_.insert(marker.ns);
  }

  pub_.publish(msg);
}

void FootprintPlugin::reset(const std_msgs::Header& header, const DynamicSceneGraph&) {
  visualization_msgs::MarkerArray msg;
  for (const auto& ns : namespaces_) {
    msg.markers.push_back(makeDeleteMarker(header, 0, ns));
  }
  namespaces_.clear();

  pub_.publish(msg);
}

}  // namespace hydra
