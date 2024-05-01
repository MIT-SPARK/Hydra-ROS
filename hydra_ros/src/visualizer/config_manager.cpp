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
#include "hydra_ros/visualizer/config_manager.h"

#include <dynamic_reconfigure/server.h>
#include <glog/logging.h>

namespace hydra {

ConfigManager::ConfigManager(const ros::NodeHandle& nh) : nh_(nh) {}

ConfigManager::~ConfigManager() {
  visualizer_config_.reset();
  layer_configs_.clear();
  dynamic_layer_configs_.clear();
  colormaps_.clear();
}

void ConfigManager::reset() {
  visualizer_config_ = std::make_shared<ConfigWrapper<VisualizerConfig>>(nh_, "config");
  layer_configs_.clear();
  dynamic_layer_configs_.clear();
  colormaps_.clear();
}

void ConfigManager::reset(const DynamicSceneGraph& graph) {
  reset();
  for (const auto layer : graph.layer_ids) {
    const auto ns = "config/layer" + std::to_string(layer);
    layer_configs_.emplace(layer,
                           std::make_shared<ConfigWrapper<LayerConfig>>(nh_, ns));
  }

  for (const auto& id_layer_pair : graph.dynamicLayers()) {
    const auto ns = "config/dynamic_layers/" + std::to_string(id_layer_pair.first);
    dynamic_layer_configs_.emplace(
        id_layer_pair.first,
        std::make_shared<ConfigWrapper<DynamicLayerConfig>>(nh_, ns));
  }
}

bool ConfigManager::hasChange() const {
  bool has_changed = visualizer_config_->hasChange();

  for (const auto& id_config_pair : layer_configs_) {
    has_changed |= id_config_pair.second->hasChange();
  }

  for (const auto& id_config_pair : dynamic_layer_configs_) {
    has_changed |= id_config_pair.second->hasChange();
  }

  for (const auto& name_config_pair : colormaps_) {
    has_changed |= name_config_pair.second->hasChange();
  }

  return has_changed;
}

void ConfigManager::clearChangeFlags() {
  visualizer_config_->clearChangeFlag();
  for (auto& id_config_pair : layer_configs_) {
    id_config_pair.second->clearChangeFlag();
  }

  for (auto& id_config_pair : dynamic_layer_configs_) {
    id_config_pair.second->clearChangeFlag();
  }

  for (auto& name_config_pair : colormaps_) {
    name_config_pair.second->clearChangeFlag();
  }
}

const VisualizerConfig& ConfigManager::getVisualizerConfig() const {
  if (!visualizer_config_) {
    visualizer_config_ =
        std::make_shared<ConfigWrapper<VisualizerConfig>>(nh_, "config");
  }

  return visualizer_config_->get();
}

const LayerConfig* ConfigManager::getLayerConfig(LayerId layer) const {
  auto iter = layer_configs_.find(layer);
  if (iter == layer_configs_.end()) {
    const auto ns = "config/layer" + std::to_string(layer);
    iter = layer_configs_
               .emplace(layer, std::make_shared<ConfigWrapper<LayerConfig>>(nh_, ns))
               .first;
  }

  return &(iter->second->get());
}

const DynamicLayerConfig& ConfigManager::getDynamicLayerConfig(LayerId layer) const {
  auto iter = dynamic_layer_configs_.find(layer);
  if (iter == dynamic_layer_configs_.end()) {
    const std::string ns = "config/dynamic_layer/" + std::to_string(layer);
    iter = dynamic_layer_configs_
               .emplace(layer,
                        std::make_shared<ConfigWrapper<DynamicLayerConfig>>(nh_, ns))
               .first;
  }

  return iter->second->get();
}

const ColormapConfig& ConfigManager::getColormapConfig(const std::string& name) const {
  auto iter = colormaps_.find(name);
  if (iter == colormaps_.end()) {
    const std::string ns = "config/" + name;
    iter = colormaps_
               .emplace(name, std::make_shared<ConfigWrapper<ColormapConfig>>(nh_, ns))
               .first;
  }

  return iter->second->get();
}

}  // namespace hydra
