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
#include <config_utilities/parsing/ros.h>
#include <config_utilities/printing.h>
#include <hydra/common/global_info.h>
#include <hydra/frontend/gvd_place_extractor.h>
#include <hydra/places/compression_graph_extractor.h>

#include "hydra_ros/frontend/gvd_visualization_utilities.h"
#include "hydra_ros/visualizer/colormap_utilities.h"
#include "hydra_ros/visualizer/visualizer_utilities.h"

namespace hydra {

using places::CompressionGraphExtractor;
using places::GraphExtractorInterface;
using places::GvdGraph;
using places::GvdLayer;
using places::GvdVoxel;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

void declare_config(PlacesVisualizer::Config& config) {
  using namespace config;
  name("PlacesVisualizerConfig");
  field(config.ns, "ns");
}

PlacesVisualizer::PlacesVisualizer(const Config& config)
    : config(config),
      nh_(config.ns),
      pubs_(nh_),
      colormap_(nh_, "colormap"),
      gvd_config_(nh_, "gvd"),
      layer_config_(nh_, "graph") {}

std::string PlacesVisualizer::printInfo() const {
  std::stringstream ss;
  ss << std::endl << config::toString(config);
  return ss.str();
}

void PlacesVisualizer::call(uint64_t timestamp_ns,
                            const Eigen::Isometry3f&,
                            const GvdLayer& gvd,
                            const GraphExtractorInterface* extractor) const {
  std_msgs::Header header;
  header.frame_id = GlobalInfo::instance().getFrames().map;
  header.stamp.fromNSec(timestamp_ns);

  visualizeGvd(header, gvd);
  if (extractor) {
    visualizeExtractor(header, *extractor);
  }
}

void PlacesVisualizer::visualizeGvd(const std_msgs::Header& header,
                                    const GvdLayer& gvd) const {
  pubs_.publish("esdf_viz", header, [&]() -> Marker {
    return makeEsdfMarker(gvd_config_.get(), colormap_.get(), gvd, "esdf");
  });

  pubs_.publish("gvd_viz", header, [&]() -> Marker {
    return makeGvdMarker(gvd_config_.get(), colormap_.get(), gvd, "gvd");
  });

  pubs_.publish("surface_viz", header, [&]() -> Marker {
    return makeSurfaceVoxelMarker(gvd_config_.get(), colormap_.get(), gvd, "surface");
  });

  pubs_.publish("voxel_block_viz", header, [&]() -> Marker {
    return makeBlocksMarker(gvd, gvd_config_.get().block_outline_scale, "blocks");
  });
}

void PlacesVisualizer::visualizeExtractor(
    const std_msgs::Header& header, const GraphExtractorInterface& extractor) const {
  const auto& graph = extractor.getGraph();
  pubs_.publish("graph_viz", header, [&]() -> MarkerArray {
    const auto d_min = gvd_config_.get().gvd_min_distance;
    const auto d_max = gvd_config_.get().gvd_max_distance;
    const auto& cmap = colormap_.get();

    visualizer::StaticLayerInfo info{hydra_ros::VisualizerConfig(),
                                     layer_config_.get()};
    info.graph.collapse_layers = true;
    info.graph.layer_z_step = 0.0;
    info.node_color = [&](const SceneGraphNode& node) {
      const auto dist = node.attributes<PlaceNodeAttributes>().distance;
      return visualizer::interpolateColorMap(cmap, dist, d_min, d_max);
    };
    info.edge_color = [&](const auto&, const auto&, const auto& edge, bool) {
      const auto dist = edge.attributes().weight;
      return visualizer::interpolateColorMap(cmap, dist, d_min, d_max);
    };

    MarkerArray msg;
    msg.markers.push_back(makeLayerNodeMarkers(header, info, graph, "places_nodes"));
    msg.markers.push_back(makeLayerEdgeMarkers(header, info, graph, "places_edges"));
    if (info.layer.use_label) {
      const auto labels = makeLayerLabelMarkers(header, info, graph, "place_labels");
      msg.markers.insert(
          msg.markers.end(), labels.markers.begin(), labels.markers.end());
    }

    return msg;
  });

  pubs_.publish("freespace_viz", header, [&]() -> MarkerArray {
    // TODO(nathan) maybe add graph back
    return makePlaceSpheres(
        header, graph, "freespace", gvd_config_.get().freespace_sphere_alpha);
  });

  pubs_.publish("gvd_graph_viz", header, [&]() -> MarkerArray {
    return makeGvdGraphMarkers(
        extractor.getGvdGraph(), gvd_config_.get(), colormap_.get(), "gvd_graph");
  });

  const auto compression = dynamic_cast<const CompressionGraphExtractor*>(&extractor);
  if (!compression) {
    return;
  }

  pubs_.publish("gvd_cluster_viz", header, [&]() -> MarkerArray {
    return showGvdClusters(compression->getGvdGraph(),
                           compression->getCompressedNodeInfo(),
                           compression->getCompressedRemapping(),
                           gvd_config_.get(),
                           colormap_.get(),
                           "gvd_cluster_graph");
  });
}

}  // namespace hydra
