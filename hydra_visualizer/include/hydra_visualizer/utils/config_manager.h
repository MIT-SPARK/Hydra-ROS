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
#include <ros/ros.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <std_msgs/String.h>

#include "hydra_visualizer/color/graph_color_adaptors.h"
#include "hydra_visualizer/utils/config_wrapper.h"
#include "hydra_visualizer/utils/label_adaptors.h"
#include "hydra_visualizer/utils/visualizer_types.h"

namespace hydra::visualizer {

const std::string& getColorMode(const hydra_visualizer::LayerVisualizerConfig& config);
const std::string& getColorMode(
    const hydra_visualizer::DynamicLayerVisualizerConfig& config);

class ColorManager {
 public:
  using ColorFunc = std::function<spark_dsg::Color(const spark_dsg::SceneGraphNode&)>;
  ColorManager(const ros::NodeHandle& nh, spark_dsg::LayerId layer);
  ColorFunc get(const spark_dsg::DynamicSceneGraph& graph) const;
  void set(const std::string& mode);
  bool hasChange() const;
  void clearChangeFlag();

 private:
  void setAdaptor();
  void callback(const std_msgs::String& msg);

  bool has_change_;
  std::string mode_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  spark_dsg::LayerId layer_;
  std::string curr_contents_;
  std::unique_ptr<GraphColorAdaptor> adaptor_;
};

class LabelManager {
 public:
  using LabelFunc = std::function<std::string(const spark_dsg::SceneGraphNode&)>;
  explicit LabelManager(const ros::NodeHandle& nh);
  LabelFunc get() const;
  void set(const std::string& mode);
  bool hasChange() const;
  void clearChangeFlag();

 private:
  void setAdaptor();
  void callback(const std_msgs::String& msg);

  bool has_change_;
  std::string mode_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::string curr_contents_;
  GraphLabelAdaptor::Ptr adaptor_;
};

template <typename ConfigT>
class LayerConfig {
 public:
  LayerConfig(const ros::NodeHandle& nh,
              const std::string& ns,
              spark_dsg::LayerId layer)
      : color(std::make_unique<ColorManager>(ros::NodeHandle(nh, ns), layer)),
        label(std::make_unique<LabelManager>(ros::NodeHandle(nh, ns))),
        config(std::make_unique<ConfigWrapper<ConfigT>>(nh, ns)) {
    setCallback();
  }

  ~LayerConfig() = default;
  LayerConfig(const LayerConfig& other) = delete;
  LayerConfig& operator=(const LayerConfig& other) = delete;

  LayerConfig(LayerConfig&& other)
      : color(std::move(other.color)),
        label(std::move(other.label)),
        config(std::move(other.config)) {
    setCallback();
  }

  LayerConfig& operator=(LayerConfig&& other) {
    color = std::move(other.color);
    label = std::move(other.label);
    config = std::move(other.config);
    setCallback();
    return *this;
  }

  LayerInfo<ConfigT> getInfo(const spark_dsg::DynamicSceneGraph& graph) const;

  bool hasChange() const {
    return color->hasChange() || label->hasChange() || config->hasChange();
  }

  void clearChangeFlag() {
    color->clearChangeFlag();
    label->clearChangeFlag();
    config->clearChangeFlag();
  }

  std::unique_ptr<ColorManager> color;
  std::unique_ptr<LabelManager> label;
  std::unique_ptr<ConfigWrapper<ConfigT>> config;

 private:
  void setCallback() {
    const auto c = config->get();
    color->set(getColorMode(c));
    label->set(c.label_mode);
    config->setUpdateCallback([this](const auto& c) {
      color->set(getColorMode(c));
      label->set(c.label_mode);
    });
  }
};

using StaticLayerConfig = LayerConfig<hydra_visualizer::LayerVisualizerConfig>;
using DynamicLayerConfig = LayerConfig<hydra_visualizer::DynamicLayerVisualizerConfig>;

class ConfigManager {
 public:
  template <typename T>
  using LayerMap = std::map<spark_dsg::LayerId, T>;

  ~ConfigManager();

  static ConfigManager& instance();

  static void init(const ros::NodeHandle& nh);

  static void reset();

  static void reset(const spark_dsg::DynamicSceneGraph& graph);

  bool hasChange() const;

  void clearChangeFlags();

  const hydra_visualizer::VisualizerConfig& getVisualizerConfig() const;

  const StaticLayerConfig& getLayerConfig(spark_dsg::LayerId layer) const;

  const DynamicLayerConfig& getDynamicLayerConfig(spark_dsg::LayerId layer) const;

 private:
  ConfigManager();

  inline static std::unique_ptr<ConfigManager> s_instance_ = nullptr;

  ros::NodeHandle nh_;
  mutable ConfigWrapper<hydra_visualizer::VisualizerConfig>::Ptr visualizer_config_;
  mutable LayerMap<StaticLayerConfig> layers_;
  mutable LayerMap<DynamicLayerConfig> dynamic_layers_;
};

template <typename ConfigT>
LayerInfo<ConfigT> LayerConfig<ConfigT>::getInfo(
    const spark_dsg::DynamicSceneGraph& graph) const {
  LayerInfo<ConfigT> info;
  info.graph = ConfigManager::instance().getVisualizerConfig();
  info.layer = config->get();
  info.node_color = color->get(graph);
  info.node_label = label->get();
  return info;
}

}  // namespace hydra::visualizer
