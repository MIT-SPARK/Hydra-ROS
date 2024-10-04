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
#include "hydra_visualizer/utils/visualizer_types.h"

#include <spark_dsg/node_attributes.h>

#include "hydra_visualizer/color/colormap_utilities.h"

namespace hydra::visualizer {

using namespace spark_dsg;

Color DefaultNodeColorFunction::operator()(const SceneGraphNode& node) const {
  try {
    return node.attributes<SemanticNodeAttributes>().color;
  } catch (const std::bad_cast&) {
    return {};
  }
}

std::string DefaultNodeLabelFunction::operator()(const SceneGraphNode& node) const {
  try {
    return node.attributes<SemanticNodeAttributes>().name;
  } catch (const std::bad_cast&) {
    return "";
  }
}

Color DefaultEdgeColorFunction::operator()(const SceneGraphNode&,
                                           const SceneGraphNode&,
                                           const SceneGraphEdge&,
                                           bool) const {
  return {};
}

template <>
double StaticLayerInfo::getZOffset() const {
  return graph.collapse_layers ? 0 : layer.z_offset_scale * graph.layer_z_step;
}

template <>
bool StaticLayerInfo::shouldVisualize(const spark_dsg::SceneGraphNode& node) const {
  if (!layer.visualize) {
    return false;
  }

  if (filter && !filter(node)) {
    return false;
  }

  return true;
}

template <>
double DynamicLayerInfo::getZOffset() const {
  return graph.collapse_layers ? 0 : layer.z_offset_scale * graph.layer_z_step;
}

template <>
bool DynamicLayerInfo::shouldVisualize(const spark_dsg::SceneGraphNode& node) const {
  if (!layer.visualize) {
    return false;
  }

  if (filter && !filter(node)) {
    return false;
  }

  return true;
}

// TODO(nathan) this is janky, but we should tackle this after the refactor
struct LayerEdgeInfo {
  LayerEdgeInfo()
      : visualize(false), visualize_edges(false), z_offset(0.0), num_to_skip(0) {}

  LayerEdgeInfo(const StaticLayerInfo& info)
      : visualize(info.layer.visualize),
        visualize_edges(info.layer.visualize),
        z_offset(info.getZOffset()),
        num_to_skip(info.layer.interlayer_edge_insertion_skip),
        node_color(info.node_color),
        edge_color(info.edge_color),
        filter(info.filter) {}

  LayerEdgeInfo(const DynamicLayerInfo& info)
      : visualize(info.layer.visualize),
        visualize_edges(info.layer.visualize_interlayer_edges),
        z_offset(info.getZOffset()),
        num_to_skip(info.layer.interlayer_edge_insertion_skip),
        node_color(info.node_color),
        edge_color(info.edge_color),
        filter(info.filter) {}

  bool visualize;
  bool visualize_edges;
  double z_offset;
  size_t num_to_skip;
  const NodeColorFunction node_color;
  const EdgeColorFunction edge_color;
  const FilterFunction filter;

  bool shouldVisualize(const SceneGraphNode& node) const {
    if (!filter) {
      return true;
    }

    return filter(node);
  }
};

LayerEdgeInfo getLayerInfo(const GraphInfo& info, const LayerKey& key) {
  if (key.dynamic) {
    auto iter = info.dynamic_layers.find(key.layer);
    if (iter == info.dynamic_layers.end()) {
      return {};
    }

    return iter->second;
  } else {
    auto iter = info.layers.find(key.layer);
    if (iter == info.layers.end()) {
      return {};
    }

    return iter->second;
  }
}

GraphInfo::EdgeInformation GraphInfo::getEdgeInfo(const LayerKey& source_layer,
                                                  const SceneGraphNode& source,
                                                  const LayerKey& target_layer,
                                                  const SceneGraphNode& target) const {
  const auto source_info = getLayerInfo(*this, source_layer);
  if (!source_info.visualize || !source_info.shouldVisualize(source)) {
    return {};
  }

  const auto target_info = getLayerInfo(*this, target_layer);
  if (!target_info.shouldVisualize(target)) {
    return {};
  }

  // dynamic layers have a toggle for interlayer edges
  if (!source_info.visualize_edges || !target_info.visualize_edges) {
    return {};
  }

  bool use_source = true;
  const StaticLayerInfo* info = nullptr;
  if (source_layer.dynamic) {
    if (!target_layer.dynamic) {
      use_source = false;
      info = &layers.at(target_layer.layer);
    }
  } else {
    info = &layers.at(source_layer.layer);
    if (!info->layer.interlayer_edge_use_source && !target_layer.dynamic) {
      use_source = false;
      info = &layers.at(target_layer.layer);
    }
  }

  Color color;
  if (!info || info->layer.interlayer_edge_use_color) {
    color =
        use_source ? source_info.node_color(source) : target_info.node_color(target);
  }

  const double alpha = info ? info->layer.interlayer_edge_alpha : 1.0;
  return {true,
          use_source ? source_info.num_to_skip : target_info.num_to_skip,
          info ? info->layer.interlayer_edge_scale : 0.01,
          makeColorMsg(color, alpha),
          source_info.z_offset,
          target_info.z_offset};
}

}  // namespace hydra::visualizer
