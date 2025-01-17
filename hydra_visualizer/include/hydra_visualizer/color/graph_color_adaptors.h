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
#pragma once
#include <config_utilities/factory.h>
#include <spark_dsg/color.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include <memory>

#include "hydra_visualizer/color/colormap_utilities.h"

namespace hydra {

#define REGISTER_COLOR_ADAPTOR(adaptor)    \
  inline static const auto registration_ = \
      config::RegistrationWithConfig<GraphColorAdaptor, adaptor, Config>(#adaptor)

struct GraphColorAdaptor {
  using Ptr = std::shared_ptr<GraphColorAdaptor>;

  virtual ~GraphColorAdaptor() = default;

  /**
   * @brief Get color for a single node
   * @param graph Current scene graph node is from
   * @param node Node to get color for
   * @returns Visualizer color for node
   */
  virtual spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                                    const spark_dsg::SceneGraphNode& node) const = 0;

  /**
   * @brief Set any pre-draw information
   * @param graph Graph to get information for
   *
   * Allows color adaptors to gather statistics about the scene graph before generating
   * any node colors when drawing the scene graph
   */
  virtual void setGraph(const spark_dsg::DynamicSceneGraph& /* graph */,
                        spark_dsg::LayerId /* layer */) {}
};

struct NodeColorAdaptor : GraphColorAdaptor {
  struct Config {
    spark_dsg::Color default_color;
  } const config;

  explicit NodeColorAdaptor(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

 private:
  REGISTER_COLOR_ADAPTOR(NodeColorAdaptor);
};

void declare_config(NodeColorAdaptor::Config& config);

struct UniformColorAdaptor : GraphColorAdaptor {
  struct Config {
    spark_dsg::Color color;
  } const config;

  explicit UniformColorAdaptor(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

 private:
  REGISTER_COLOR_ADAPTOR(UniformColorAdaptor);
};

void declare_config(UniformColorAdaptor::Config& config);

struct LabelColorAdaptor : GraphColorAdaptor {
  struct Config {
    visualizer::CategoricalColormap::Config colormap;
  } const config;

  explicit LabelColorAdaptor(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

 private:
  const visualizer::CategoricalColormap colormap_;

  REGISTER_COLOR_ADAPTOR(LabelColorAdaptor);
};

void declare_config(LabelColorAdaptor::Config& config);

struct IdColorAdaptor : GraphColorAdaptor {
  struct Config {
    visualizer::DiscreteColormap::Config colormap;
  } const config;

  explicit IdColorAdaptor(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

 private:
  const visualizer::DiscreteColormap colormap_;

  REGISTER_COLOR_ADAPTOR(IdColorAdaptor);
};

void declare_config(IdColorAdaptor::Config& config);

struct ParentColorAdaptor : GraphColorAdaptor {
  struct Config {
    spark_dsg::Color default_color;
    config::VirtualConfig<GraphColorAdaptor> parent_adaptor{
        IdColorAdaptor::Config{visualizer::DiscretePalette::COLORBREWER}};
  } const config;

  explicit ParentColorAdaptor(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

 private:
  const GraphColorAdaptor::Ptr parent_adaptor_;

  REGISTER_COLOR_ADAPTOR(ParentColorAdaptor);
};

void declare_config(ParentColorAdaptor::Config& config);

struct StatusFunctor {
  virtual ~StatusFunctor() = default;
  virtual bool eval(const spark_dsg::DynamicSceneGraph& graph,
                    const spark_dsg::SceneGraphNode& node) const = 0;
};

struct IsActiveFunctor : StatusFunctor {
  virtual bool eval(const spark_dsg::DynamicSceneGraph& graph,
                    const spark_dsg::SceneGraphNode& node) const override;
  inline static const auto registration =
      config::Registration<StatusFunctor, IsActiveFunctor>("is_active");
};

struct NeedsCleanupFunctor : StatusFunctor {
  virtual bool eval(const spark_dsg::DynamicSceneGraph& graph,
                    const spark_dsg::SceneGraphNode& node) const override;
  inline static const auto registration =
      config::Registration<StatusFunctor, NeedsCleanupFunctor>("needs_cleanup");
};

struct HasActiveMeshFunctor : StatusFunctor {
  virtual bool eval(const spark_dsg::DynamicSceneGraph& graph,
                    const spark_dsg::SceneGraphNode& node) const override;
  inline static const auto registration =
      config::Registration<StatusFunctor, HasActiveMeshFunctor>("has_active_mesh");
};

struct StatusColorAdaptor : GraphColorAdaptor {
  struct Config {
    spark_dsg::Color true_color{0, 255, 0};
    spark_dsg::Color false_color;
    std::string status_functor{"is_active"};
  } const config;

  explicit StatusColorAdaptor(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

 private:
  std::unique_ptr<StatusFunctor> functor_;
  REGISTER_COLOR_ADAPTOR(StatusColorAdaptor);
};

void declare_config(StatusColorAdaptor::Config& config);

struct FrontierColorAdaptor : GraphColorAdaptor {
  struct Config {
    spark_dsg::Color real;
    spark_dsg::Color predicted{0, 0, 255};
    spark_dsg::Color active{0, 255, 0};
    spark_dsg::Color archived{255, 0, 0};
  } const config;

  explicit FrontierColorAdaptor(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

 private:
  REGISTER_COLOR_ADAPTOR(FrontierColorAdaptor);
};

void declare_config(FrontierColorAdaptor::Config& config);

struct PrefixColorAdaptor : GraphColorAdaptor {
  struct Config {
    visualizer::DiscreteColormap::Config colormap;
  } const config;

  explicit PrefixColorAdaptor(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

 private:
  const visualizer::DiscreteColormap colormap_;

  REGISTER_COLOR_ADAPTOR(PrefixColorAdaptor);
};

void declare_config(PrefixColorAdaptor::Config& config);

struct ValueFunctor {
  virtual ~ValueFunctor() = default;
  virtual double eval(const spark_dsg::DynamicSceneGraph& graph,
                      const spark_dsg::SceneGraphNode& node) const = 0;
};

struct DistanceFunctor : ValueFunctor {
  double eval(const spark_dsg::DynamicSceneGraph& graph,
              const spark_dsg::SceneGraphNode& node) const override;

  inline static const auto registration =
      config::Registration<ValueFunctor, DistanceFunctor>("place_distance");
};

struct LastUpdatedFunctor : ValueFunctor {
  double eval(const spark_dsg::DynamicSceneGraph& graph,
              const spark_dsg::SceneGraphNode& node) const override;

  inline static const auto registration =
      config::Registration<ValueFunctor, LastUpdatedFunctor>("last_updated");
};

struct ValueColorAdaptor : GraphColorAdaptor {
  struct Config {
    visualizer::RangeColormap::Config colormap;
    std::string value_functor{"place_distance"};
  } const config;

  explicit ValueColorAdaptor(const Config& config);
  void setGraph(const spark_dsg::DynamicSceneGraph& graph,
                spark_dsg::LayerId layer) override;
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

 private:
  double min_value_;
  double max_value_;
  std::unique_ptr<ValueFunctor> functor_;
  const visualizer::RangeColormap colormap_;
  REGISTER_COLOR_ADAPTOR(ValueColorAdaptor);
};

void declare_config(ValueColorAdaptor::Config& config);

#undef REGISTER_COLOR_ADAPTOR

}  // namespace hydra
