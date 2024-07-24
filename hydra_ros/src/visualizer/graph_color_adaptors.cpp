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
#include "hydra_ros/visualizer/graph_color_adaptors.h"

#include <config_utilities/config.h>
#include <glog/logging.h>
#include <hydra/common/config_utilities.h>
#include <spark_dsg/node_attributes.h>

namespace hydra {

using spark_dsg::Color;
using spark_dsg::DynamicSceneGraph;
using spark_dsg::FrontierNodeAttributes;
using spark_dsg::Place2dNodeAttributes;
using spark_dsg::PlaceNodeAttributes;
using spark_dsg::SceneGraphNode;
using spark_dsg::SemanticNodeAttributes;

void declare_config(NodeColorAdaptor::Config& config) {
  using namespace config;
  name("NodeColorAdaptor::Config");
  field(config.default_color, "default_color");
}

NodeColorAdaptor::NodeColorAdaptor(const Config& config) : config(config) {}

Color NodeColorAdaptor::getColor(const DynamicSceneGraph&,
                                 const SceneGraphNode& node) const {
  try {
    return node.attributes<SemanticNodeAttributes>().color;
  } catch (const std::bad_cast&) {
    return config.default_color;
  }
}

ParentColorAdaptor::ParentColorAdaptor(const Config& config) : config(config) {}

Color ParentColorAdaptor::getColor(const DynamicSceneGraph& graph,
                                   const SceneGraphNode& node) const {
  auto parent = node.getParent();
  if (!parent) {
    return {};
  }

  return graph.getNode(*parent).attributes<SemanticNodeAttributes>().color;
}

void declare_config(ParentColorAdaptor::Config& config) {
  using namespace config;
  name("ParentColorAdaptor::Config");
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

StatusColorAdaptor::StatusColorAdaptor(const Config& config)
    : config(config), functor_(config::create<StatusFunctor>(config.status_functor)) {
  CHECK(functor_) << "invalid functor type: " << config.status_functor;
}

Color StatusColorAdaptor::getColor(const DynamicSceneGraph& graph,
                                   const SceneGraphNode& node) const {
  bool status = false;
  try {
    status = functor_->eval(graph, node);
  } catch (const std::bad_cast& e) {
    LOG_FIRST_N(ERROR, 1) << "Status functor unable to cast correctly: " << e.what();
  }

  return status ? config.true_color : config.false_color;
}

void declare_config(StatusColorAdaptor::Config& config) {
  using namespace config;
  name("StatusColorAdaptor::Config");
  field(config.true_color, "true_color");
  field(config.false_color, "false_color");
  field(config.status_functor, "status_functor");
}

PrefixColorAdaptor::PrefixColorAdaptor(const Config& config) : config(config) {}

Color PrefixColorAdaptor::getColor(const DynamicSceneGraph&,
                                   const SceneGraphNode& node) const {
  // distance is measured from first relatively readable character prefix
  const auto prefix = spark_dsg::LayerPrefix::fromId(node.id).key();

  const auto dist = static_cast<int>(prefix - '0');
  return config.colors.empty() ? Color::rainbow(dist % 10)
                               : config.colors.at(dist % config.colors.size());
}

void declare_config(PrefixColorAdaptor::Config& config) {
  using namespace config;
  name("PrefixColorAdaptor::Config");
  field(config.colors, "colors");
}

FrontierColorAdaptor::FrontierColorAdaptor(const Config& config) : config(config) {}

Color FrontierColorAdaptor::getColor(const DynamicSceneGraph&,
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

void declare_config(FrontierColorAdaptor::Config& config) {
  using namespace config;
  name("FrontierColorAdaptor::Config");
  field(config.real, "real");
  field(config.predicted, "predicted");
  field(config.active, "active");
  field(config.archived, "archived");
}

double DistanceFunctor::eval(const DynamicSceneGraph&,
                             const SceneGraphNode& node) const {
  return node.attributes<PlaceNodeAttributes>().distance;
}

ValueColorAdaptor::ValueColorAdaptor(const Config& config)
    : config(config), functor_(config::create<ValueFunctor>(config.value_functor)) {
  CHECK(functor_) << "invalid functor type: " << config.value_functor;
}

Color ValueColorAdaptor::getColor(const DynamicSceneGraph& graph,
                                  const SceneGraphNode& node) const {
  try {
    const auto value = functor_->eval(graph, node);
    const auto temp = (value - config.vmin) / (config.vmax - config.vmin);
    return Color::ironbow(std::clamp(temp, 0.0, 1.0));
  } catch (const std::exception& e) {
    LOG_FIRST_N(ERROR, 1) << "Value functor unable to evaluate: " << e.what();
    return Color();
  }
}

void declare_config(ValueColorAdaptor::Config& config) {
  using namespace config;
  name("ValueColorAdaptor::Config");
  field(config.vmin, "vmin");
  field(config.vmax, "vmax");
  field(config.value_functor, "value_functor");
}

}  // namespace hydra
