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
#include "hydra_visualizer/layer_info.h"

#include <config_utilities/config.h>
#include <spark_dsg/node_attributes.h>

namespace hydra::visualizer {

using namespace spark_dsg;

namespace {

inline Color getNodeColor(const SceneGraphNode& node) {
  auto attrs = node.tryAttributes<SemanticNodeAttributes>();
  return attrs ? attrs->color : Color{};
}

}  // namespace

// TODO(nathan) validity checks

void declare_config(LayerConfig::Nodes& config) {
  using namespace config;
  name("LayerConfig::Nodes");
  field(config.draw, "draw");
  field(config.scale, "scale");
  field(config.color, "color");
  field(config.alpha, "alpha");
  field(config.use_sphere, "use_sphere");

  check(config.scale, GT, 0.0, "scale");
  checkInRange(config.alpha, 0.0, 1.0, "alpha");
}

void declare_config(LayerConfig::Edges& config) {
  using namespace config;
  name("LayerConfig::Edges");
  field(config.draw, "draw");
  field(config.scale, "scale");
  field(config.alpha, "alpha");
  field(config.use_color, "use_color");
  field(config.draw_interlayer, "draw_interlayer");
  field(config.interlayer_use_source, "interlayer_use_source");
  field(config.interlayer_scale, "interlayer_scale");
  field(config.interlayer_alpha, "interlayer_alpha");
  field(config.interlayer_use_color, "interlayer_use_color");
  field(config.interlayer_insertion_skip, "interlayer_insertion_skip");
}

void declare_config(LayerConfig::Text& config) {
  using namespace config;
  name("LayerConfig::Text");
  field(config.draw, "draw");
  field(config.draw_layer, "draw_layer");
  field(config.collapse, "collapse");
  field(config.adapter, "adapter");
  field(config.height, "height");
  field(config.scale, "scale");
  field(config.add_jitter, "add_jitter");
  field(config.jitter_scale, "jitter_scale");
}

void declare_config(LayerConfig::BoundingBoxes& config) {
  using namespace config;
  name("LayerConfig::BoundingBoxes");
  field(config.draw, "draw");
  field(config.collapse, "collapse");
  field(config.scale, "scale");
  field(config.edge_scale, "edge_scale");
  field(config.alpha, "alpha");
  field(config.edge_break_ratio, "edge_break_ratio");
}

void declare_config(LayerConfig::Boundaries& config) {
  using namespace config;
  name("LayerConfig::Boundaries");
  field(config.draw, "draw");
  field(config.collapse, "collapse");
  field(config.wireframe_scale, "wireframe_scale");
  field(config.use_node_color, "use_node_color");
  field(config.alpha, "alpha");
  field(config.draw_ellipse, "draw_ellipse");
  field(config.ellipse_alpha, "ellipse_alpha");
}

void declare_config(LayerConfig& config) {
  using namespace config;
  name("LayerConfig");
  field(config.visualize, "visualize");
  field(config.z_offset_scale, "z_offset_scale");
  field(config.draw_frontier_ellipse, "draw_frontier_ellipse");
  field(config.draw_mesh_edges, "draw_mesh_edges");

  // subconfigs
  field(config.nodes, "nodes");
  field(config.edges, "edges");
  field(config.text, "text");
  field(config.bounding_boxes, "bounding_boxes");
  field(config.boundaries, "boundaries");
}

LayerInfo::LayerInfo(const LayerConfig config)
    : config(config),
      z_offset(0.0),
      node_color(&getNodeColor),
      node_text([](const SceneGraphNode&) { return ""; }) {}

LayerInfo& LayerInfo::offset(double offset_size, bool collapse) {
  z_offset = collapse ? 0.0 : offset_size * config.z_offset_scale;
  return *this;
}

LayerInfo& LayerInfo::graph(const DynamicSceneGraph& graph, LayerId layer) {
  color_adapter_ = config.nodes.color.create();
  if (color_adapter_) {
    color_adapter_->setGraph(graph, layer);
    node_color = [this, &graph](const SceneGraphNode& node) {
      return color_adapter_->getColor(graph, node);
    };
  }

  text_adapter_ = config.text.adapter.create();
  if (text_adapter_) {
    node_text = [this, &graph](const SceneGraphNode& node) {
      return text_adapter_->getText(graph, node);
    };
  }

  return *this;
}

bool LayerInfo::shouldVisualize(const spark_dsg::SceneGraphNode& node) const {
  if (!config.visualize) {
    return false;
  }

  if (filter && !filter(node)) {
    return false;
  }

  return true;
}

}  // namespace hydra::visualizer
