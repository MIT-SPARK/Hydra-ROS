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

#include "hydra_visualizer/dsg_visualizer.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

namespace hydra {

void declare_config(DsgVisualizer::Config& config) {
  using namespace config;
  name("HydraVisualizerConfig");
  field(config.ns, "ns");
  field(config.loop_period_s, "loop_period_s", "s");
  field(config.visualizer_frame, "visualizer_frame");
  field(config.graph, "graph");
  field(config.plugins, "plugins");
  checkCondition(!config.visualizer_frame.empty(), "visualizer_frame");
}

DsgVisualizer::DsgVisualizer(const Config& config)
    : config(config::checkValid(config)),
      nh_(config.ns),
      renderer_(new SceneGraphRenderer(nh_)) {
  for (auto&& [name, plugin] : config.plugins) {
    plugins_.push_back(plugin.create(nh_, name));
  }

  graph_ = config.graph.create();
  redraw_service_ = nh_.advertiseService("redraw", &DsgVisualizer::redraw, this);
  reset_service_ = nh_.advertiseService("reset", &DsgVisualizer::reset, this);
}

void DsgVisualizer::start() {
  loop_timer_ = nh_.createWallTimer(ros::WallDuration(config.loop_period_s),
                                    [this](const ros::WallTimerEvent&) { spinOnce(); });
}

void DsgVisualizer::reset() {
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = config.visualizer_frame;

  renderer_->reset(header);
  for (const auto& plugin : plugins_) {
    if (plugin) {
      plugin->reset(header);
    }
  }

  graph_ = config.graph.create();
}

void DsgVisualizer::addPlugin(VisualizerPlugin::Ptr plugin) {
  plugins_.push_back(std::move(plugin));
}

void DsgVisualizer::clearPlugins() { plugins_.clear(); }

void DsgVisualizer::spinOnce(bool force) {
  if (!graph_) {
    return;
  }

  bool has_change = false;
  has_change = graph_->hasChange();
  has_change |= renderer_->hasChange();
  for (const auto& plugin : plugins_) {
    if (plugin) {
      has_change |= plugin->hasChange();
    }
  }

  if (!has_change && !force) {
    return;
  }

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = config.visualizer_frame;

  const auto graph = graph_->get();
  if (!graph) {
    return;
  }

  renderer_->draw(header, *graph);
  for (const auto& plugin : plugins_) {
    if (plugin) {
      plugin->draw(header, *graph);
    }
  }

  graph_->clearChangeFlag();
  renderer_->clearChangeFlag();
  for (auto& plugin : plugins_) {
    if (plugin) {
      plugin->clearChangeFlag();
    }
  }
}

bool DsgVisualizer::redraw(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
  spinOnce(true);
  return true;
}

bool DsgVisualizer::reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
  reset();
  return true;
}

}  // namespace hydra
