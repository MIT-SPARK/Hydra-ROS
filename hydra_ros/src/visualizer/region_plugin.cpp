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
#include "hydra_ros/visualizer/region_plugin.h"

#include <config_utilities/config.h>
#include <config_utilities/config_utilities.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/semantic_color_map.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>
#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/MarkerArray.h>

#include "hydra_ros/visualizer/colormap_utilities.h"
#include "hydra_ros/visualizer/polygon_utilities.h"

namespace hydra {

using spark_dsg::DsgLayers;
using spark_dsg::DynamicSceneGraph;
using spark_dsg::NodeSymbol;
using spark_dsg::SemanticNodeAttributes;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

void declare_config(RegionPlugin::Config& config) {
  using namespace config;
  name("RegionPlugin::Config");
  field(config.skip_unknown, "skip_unknown");
  field(config.draw_labels, "draw_labels");
  field(config.line_width, "line_width");
  field(config.line_alpha, "line_alpha");
  field(config.region_colormap, "region_colormap");

  check(config.line_width, GT, 0.0, "line_width");
  check(config.line_alpha, GT, 0.0, "line_alpha");
}

RegionPlugin::RegionPlugin(const Config& config,
                           const ros::NodeHandle& nh,
                           const std::string& name)
    : DsgVisualizerPlugin(nh, name), config(config::checkValid(config)) {
  // namespacing gives us a reasonable topic
  pub_ = nh_.advertise<visualization_msgs::MarkerArray>("", 1, true);
}

void RegionPlugin::draw(const std_msgs::Header& header,
                        const DynamicSceneGraph& graph) {
  visualization_msgs::MarkerArray markers;
  markers.markers.resize(3);

  markers.markers[0].header = header;
  markers.markers[0].ns = "region_plugin_mesh";
  markers.markers[0].id = 0;
  markers.markers[0].type = visualization_msgs::Marker::TRIANGLE_LIST;
  markers.markers[0].action = visualization_msgs::Marker::ADD;
  markers.markers[0].pose.orientation.w = 1.0;
  markers.markers[0].color.a = config.mesh_alpha;
  markers.markers[0].scale.x = 1.0;
  markers.markers[0].scale.y = 1.0;
  markers.markers[0].scale.z = 1.0;

  markers.markers[1].header = header;
  markers.markers[1].ns = "region_plugin_wireframe";
  markers.markers[1].id = 0;
  markers.markers[1].type = visualization_msgs::Marker::LINE_LIST;
  markers.markers[1].action = visualization_msgs::Marker::ADD;
  markers.markers[1].pose.orientation.w = 1.0;
  markers.markers[1].scale.x = config.line_width;
  markers.markers[1].scale.y = 0.0;
  markers.markers[1].scale.z = 0.0;

  markers.markers[2].header = header;
  markers.markers[2].ns = "region_plugin_wireframe_vertices";
  markers.markers[2].id = 0;
  markers.markers[2].type = visualization_msgs::Marker::SPHERE_LIST;
  markers.markers[2].action = visualization_msgs::Marker::ADD;
  markers.markers[2].pose.orientation.w = 1.0;
  markers.markers[2].scale.x = config.line_width;
  markers.markers[2].scale.y = config.line_width;
  markers.markers[2].scale.z = config.line_width;

  const auto& regions = graph.getLayer(DsgLayers::ROOMS);
  for (auto&& [id, node] : regions.nodes()) {
    const auto& attrs = node->attributes<SemanticNodeAttributes>();
    if (config.skip_unknown && attrs.name == "unknown") {
      continue;
    }

    auto color = visualizer::makeColorMsg(attrs.color);
    color.a = config.line_alpha;

    const double mean_z = getMeanChildHeight(graph, *node);
    auto hull_points = getChildrenConvexHull(graph, *node);

    if (config.draw_labels) {
      const auto& pos = attrs.position;
      visualization_msgs::Marker text_marker;
      text_marker.header = header;
      text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_marker.ns = "region_plugin_labels";
      text_marker.id = NodeSymbol(id).categoryId();
      text_marker.action = visualization_msgs::Marker::ADD;
      text_marker.text = attrs.name;
      text_marker.scale.z = config.label_scale;
      text_marker.color.a = 1.0;
      text_marker.pose.orientation.w = 1.0;
      tf2::convert(pos, text_marker.pose.position);
      text_marker.pose.position.z = mean_z;
      markers.markers.push_back(text_marker);
    }

    auto mesh_color = color;
    mesh_color.a = config.mesh_alpha;
    makeFilledPolygon(hull_points, mesh_color, markers.markers[0], mean_z);
    makePolygonBoundary(
        hull_points, color, markers.markers[1], mean_z, &markers.markers[2]);
  }

  MarkerArray msg;
  tracker_.add(markers, msg);
  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_.publish(msg);
  }
}

void RegionPlugin::reset(const std_msgs::Header& header) {
  visualization_msgs::MarkerArray msg;
  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_.publish(msg);
  }
}

}  // namespace hydra
