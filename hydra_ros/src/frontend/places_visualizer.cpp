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
#include "hydra_ros/frontend/places_visualizer.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <hydra/common/global_info.h>
#include <hydra/frontend/gvd_place_extractor.h>
#include <hydra_visualizer/color/color_parsing.h>
#include <hydra_visualizer/color/colormap_utilities.h>
#include <hydra_visualizer/drawing.h>

#include "hydra_ros/frontend/gvd_visualization_utilities.h"
#include <rclcpp/time.hpp>

#include "hydra_ros/common.h"
#include "hydra_ros/visualizer/voxel_drawing.h"

namespace hydra {

using places::GraphExtractor;
using places::GvdGraph;
using places::GvdLayer;
using places::GvdVoxel;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using visualizer::ContinuousPalette;
using visualizer::DivergentPalette;
using visualizer::RangeColormap;

void declare_config(PlacesVisualizer::Config& config) {
  using namespace config;
  name("PlacesVisualizerConfig");
  field(config.ns, "ns");
  field(config.colormap, "colormap");
  field(config.block_color, "block_color");
}

PlacesVisualizer::PlacesVisualizer(const Config& config)
    : config(config),
      nh_(getHydraNodeHandle(config.ns)),
      pubs_(nh_),
      gvd_config_("gvd"),
      layer_config_("graph"),
      colormap_(config.colormap) {}

std::string PlacesVisualizer::printInfo() const { return config::toString(config); }

void PlacesVisualizer::call(uint64_t timestamp_ns,
                            const Eigen::Isometry3d& pose,
                            const GvdLayer& gvd,
                            const GraphExtractor& extractor) const {
  std_msgs::msg::Header header;
  header.frame_id = GlobalInfo::instance().getFrames().map;
  header.stamp = rclcpp::Time(timestamp_ns);

  const RangeColormap sdf_cmap(RangeColormap::Config{});
  pubs_.publish("esdf_viz", header, [&]() -> Marker {
    return drawEsdf(gvd_config_.get(), sdf_cmap, pose, gvd, "esdf");
  });

  visualizeGvd(header, gvd);
  visualizeExtractor(header, extractor);
}

void PlacesVisualizer::visualizeGvd(const std_msgs::msg::Header& header,
                                    const GvdLayer& gvd) const {
  pubs_.publish("gvd_viz", header, [&]() -> Marker {
    return drawGvd(gvd_config_.get(), colormap_, gvd, "gvd");
  });

  pubs_.publish("surface_viz", header, [&]() -> Marker {
    return drawGvdSurface(gvd_config_.get(), colormap_, gvd, "surface");
  });

  ActiveBlockColoring block_cmap(config.block_color);
  pubs_.publish("voxel_block_viz", header, [&]() -> Marker {
    return drawSpatialGrid(gvd,
                           gvd_config_.get().block_outline_scale,
                           "blocks",
                           1.0,
                           block_cmap.getCallback<places::GvdBlock>());
  });
}

void PlacesVisualizer::visualizeExtractor(const std_msgs::msg::Header& header,
                                          const GraphExtractor& extractor) const {
  const auto& graph = extractor.getGraph();
  pubs_.publish("graph_viz", header, [&]() -> MarkerArray {
    const auto d_min = gvd_config_.get().gvd_min_distance;
    const auto d_max = gvd_config_.get().gvd_max_distance;

    visualizer::LayerInfo info(layer_config_.get());
    info.node_color = [&](const SceneGraphNode& node) {
      const auto dist = node.attributes<PlaceNodeAttributes>().distance;
      return colormap_(dist, d_min, d_max);
    };

    // TODO(nathan) work custom edge functor back into config
    /*    info.edge_color = [&](const auto&, const auto&, const auto& edge, bool) {*/
    /*const auto dist = edge.attributes().weight;*/
    /*return colormap_(dist, d_min, d_max);*/
    /*};*/

    MarkerArray msg;
    msg.markers.push_back(makeLayerNodeMarkers(header, info, graph, "places_nodes"));
    msg.markers.push_back(makeLayerEdgeMarkers(header, info, graph, "places_edges"));
    if (info.config.text.draw) {
      const auto text = makeLayerNodeTextMarkers(header, info, graph, "places_text");
      msg.markers.insert(msg.markers.end(), text.markers.begin(), text.markers.end());
    }

    return msg;
  });

  pubs_.publish("gvd_graph_viz", header, [&]() -> MarkerArray {
    return drawGvdGraph(
        extractor.getGvdGraph(), gvd_config_.get(), colormap_, "gvd_graph");
  });

  pubs_.publish("gvd_cluster_viz", header, [&]() -> MarkerArray {
    return drawGvdClusters(extractor.getGvdGraph(),
                           extractor.getCompressedNodeInfo(),
                           extractor.getCompressedRemapping(),
                           gvd_config_.get(),
                           "gvd_cluster_graph");
  });
}

}  // namespace hydra
