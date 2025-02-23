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
#include "hydra_visualizer/adapters/graph_color.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include "hydra_visualizer/color/color_parsing.h"

namespace hydra {

using namespace spark_dsg;

void declare_config(NodeColorAdapter::Config& config) {
  using namespace config;
  name("NodeColorAdapter::Config");
  field(config.default_color, "default_color");
}

NodeColorAdapter::NodeColorAdapter(const Config& config) : config(config) {}

Color NodeColorAdapter::getColor(const DynamicSceneGraph&,
                                 const SceneGraphNode& node) const {
  try {
    return node.attributes<SemanticNodeAttributes>().color;
  } catch (const std::bad_cast&) {
    return config.default_color;
  }
}

void declare_config(UniformColorAdapter::Config& config) {
  using namespace config;
  name("UniformColorAdapter::Config");
  field(config.color, "color");
}

UniformColorAdapter::UniformColorAdapter(const Config& config) : config(config) {}

Color UniformColorAdapter::getColor(const DynamicSceneGraph&,
                                    const SceneGraphNode& /* node */) const {
  return config.color;
}

LabelColorAdapter::LabelColorAdapter(const Config& config)
    : config(config::checkValid(config)), colormap_(config.colormap) {}

Color LabelColorAdapter::getColor(const DynamicSceneGraph&,
                                  const SceneGraphNode& node) const {
  SemanticLabel label = SemanticNodeAttributes::NO_SEMANTIC_LABEL;
  try {
    label = node.attributes<SemanticNodeAttributes>().semantic_label;
  } catch (const std::bad_cast&) {
    VLOG(5) << "Node " << NodeSymbol(node.id).str() << " has no label!";
  }

  return colormap_.getColor(label);
}

void declare_config(LabelColorAdapter::Config& config) {
  using namespace config;
  name("LabelColorAdapter::Config");
  field(config.colormap, "colormap");
}

IdColorAdapter::IdColorAdapter(const Config& config)
    : config(config::checkValid(config)), colormap_(config.colormap) {}

Color IdColorAdapter::getColor(const DynamicSceneGraph&,
                               const SceneGraphNode& node) const {
  return colormap_.getColor(NodeSymbol(node.id).categoryId());
}

void declare_config(IdColorAdapter::Config& config) {
  using namespace config;
  name("IdColorAdapter::Config");
  field(config.colormap, "colormap");
}

ParentColorAdapter::ParentColorAdapter(const Config& config)
    : config(config::checkValid(config)),
      parent_adapter_(config.parent_adapter.create()) {}

Color ParentColorAdapter::getColor(const DynamicSceneGraph& graph,
                                   const SceneGraphNode& node) const {
  auto parent = node.getParent();
  if (!parent || !parent_adapter_) {
    return config.default_color;
  }

  return parent_adapter_->getColor(graph, graph.getNode(*parent));
}

void declare_config(ParentColorAdapter::Config& config) {
  using namespace config;
  name("ParentColorAdapter::Config");
  config.parent_adapter.setOptional();
  field(config.parent_adapter, "parent_adapter");
  field(config.default_color, "default_color");
}

bool IsActiveFunctor::eval(const DynamicSceneGraph&, const SceneGraphNode& node) const {
  return node.attributes().is_active;
}

bool NeedsCleanupFunctor::eval(const DynamicSceneGraph&,
                               const SceneGraphNode& node) const {
  return node.attributes<Place2dNodeAttributes>().need_cleanup_splitting;
}

bool HasActiveMeshFunctor::eval(const DynamicSceneGraph&,
                                const SceneGraphNode& node) const {
  return node.attributes<Place2dNodeAttributes>().has_active_mesh_indices;
}

StatusColorAdapter::StatusColorAdapter(const Config& config)
    : config(config), functor_(config::create<StatusFunctor>(config.status_functor)) {
  CHECK(functor_) << "invalid functor type: " << config.status_functor;
}

Color StatusColorAdapter::getColor(const DynamicSceneGraph& graph,
                                   const SceneGraphNode& node) const {
  bool status = false;
  try {
    status = functor_->eval(graph, node);
  } catch (const std::bad_cast& e) {
    LOG_FIRST_N(ERROR, 1) << "Status functor unable to cast correctly: " << e.what();
  }

  return status ? config.true_color : config.false_color;
}

void declare_config(StatusColorAdapter::Config& config) {
  using namespace config;
  name("StatusColorAdapter::Config");
  field(config.true_color, "true_color");
  field(config.false_color, "false_color");
  field(config.status_functor, "status_functor");
}

PartitionColorAdapter::PartitionColorAdapter(const Config& config)
    : config(config), colormap_(config.colormap) {}

Color PartitionColorAdapter::getColor(const DynamicSceneGraph&,
                                      const SceneGraphNode& node) const {
  return colormap_.getColor(node.layer.partition);
}

void declare_config(PartitionColorAdapter::Config& config) {
  using namespace config;
  name("PartitionColorAdapter::Config");
  field(config.colormap, "colormap");
}

FrontierColorAdapter::FrontierColorAdapter(const Config& config) : config(config) {}

Color FrontierColorAdapter::getColor(const DynamicSceneGraph&,
                                     const SceneGraphNode& node) const {
  try {
    const auto& attrs = node.attributes<FrontierNodeAttributes>();
    if (attrs.real_place) {
      return Color();
    }

    if (attrs.is_predicted) {
      return config.predicted;
    }

    return attrs.active_frontier ? config.active : config.archived;
  } catch (const std::bad_cast&) {
    return Color();
  }
}

void declare_config(FrontierColorAdapter::Config& config) {
  using namespace config;
  name("FrontierColorAdapter::Config");
  field(config.real, "real");
  field(config.predicted, "predicted");
  field(config.active, "active");
  field(config.archived, "archived");
}

double DistanceFunctor::eval(const DynamicSceneGraph&,
                             const SceneGraphNode& node) const {
  return node.attributes<PlaceNodeAttributes>().distance;
}

double LastUpdatedFunctor::eval(const DynamicSceneGraph&,
                                const SceneGraphNode& node) const {
  return node.attributes().last_update_time_ns;
}

ValueColorAdapter::ValueColorAdapter(const Config& config)
    : config(config),
      min_value_(0.0),
      max_value_(1.0),
      functor_(config::create<ValueFunctor>(config.value_functor)),
      colormap_(config.colormap) {
  CHECK(functor_) << "invalid functor type: " << config.value_functor;
}

void ValueColorAdapter::setGraph(const DynamicSceneGraph& graph, LayerId layer) {
  if (!functor_) {
    return;
  }

  bool is_first = true;
  try {
    for (const auto& [node_id, node] : graph.getLayer(layer).nodes()) {
      const auto value = functor_->eval(graph, *node);
      if (is_first) {
        min_value_ = value;
        max_value_ = value;
        is_first = false;
      } else {
        min_value_ = std::min(value, min_value_);
        max_value_ = std::max(value, max_value_);
      }
    }
  } catch (const std::exception& e) {
    LOG_FIRST_N(ERROR, 1) << "Value functor unable to evaluate: " << e.what();
  }
}

Color ValueColorAdapter::getColor(const DynamicSceneGraph& graph,
                                  const SceneGraphNode& node) const {
  try {
    return colormap_.getColor(functor_->eval(graph, node), min_value_, max_value_);
  } catch (const std::exception& e) {
    LOG_FIRST_N(ERROR, 1) << "Value functor unable to evaluate: " << e.what();
    return Color();
  }
}

void declare_config(ValueColorAdapter::Config& config) {
  using namespace config;
  name("ValueColorAdapter::Config");
  field(config.colormap, "colormap");
  field(config.value_functor, "value_functor");
}

LabelDistributionAdapter::LabelDistributionAdapter(const Config& config)
    : config(config::checkValid(config)), colormap_(config.colormap) {}

Color LabelDistributionAdapter::getColor(const DynamicSceneGraph&,
                                         const SceneGraphNode& node) const {
  SemanticLabel label = SemanticNodeAttributes::NO_SEMANTIC_LABEL;
  auto attrs = node.tryAttributes<SemanticNodeAttributes>();
  if (attrs && attrs->hasFeature()) {
    Eigen::Index max_idx;
    attrs->semantic_feature.col(0).maxCoeff(&max_idx);
    label = static_cast<SemanticLabel>(max_idx);
  }

  return colormap_.getColor(label);
}

void declare_config(LabelDistributionAdapter::Config& config) {
  using namespace config;
  name("LabelDistributionAdapter::Config");
  field(config.colormap, "colormap");
}

}  // namespace hydra
