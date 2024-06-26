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

#include <config_utilities/virtual_config.h>
#include <ros/ros.h>
#include <spark_dsg/zmq_interface.h>
#include <std_srvs/Empty.h>

#include <fstream>

#include "hydra_ros/utils/dsg_streaming_interface.h"
#include "hydra_ros/visualizer/dynamic_scene_graph_visualizer.h"
#include "hydra_ros/visualizer/mesh_plugin.h"

namespace hydra {

using DsgVisualizer = hydra::DynamicSceneGraphVisualizer;
using spark_dsg::getDefaultLayerIds;

struct HydraVisualizerConfig {
  bool load_graph = false;
  bool use_zmq = false;
  std::string scene_graph_filepath = "";
  std::string visualizer_ns = "/hydra_dsg_visualizer";
  std::string output_path = "";
  std::string zmq_url = "tcp://127.0.0.1:8001";
  size_t zmq_num_threads = 2;
  size_t zmq_poll_time_ms = 10;

  // Specify additional plugins that should be loaded <name, config>
  std::map<std::string, config::VirtualConfig<DsgVisualizerPlugin>> plugins;
};

void declare_config(HydraVisualizerConfig& config);

struct HydraVisualizer {
  HydraVisualizer(const ros::NodeHandle& nh);
  ~HydraVisualizer();

  void loadGraph();

  bool handleReload(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool handleRedraw(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  void addPlugin(DsgVisualizerPlugin::Ptr plugin);
  void clearPlugins();
  inline DynamicSceneGraphVisualizer& getVisualizer() { return *visualizer_; }

  void spinRos();
  void spinFile();
  void spinZmq();
  void spin();

  ros::NodeHandle nh_;
  std::shared_ptr<DsgVisualizer> visualizer_;
  std::unique_ptr<DsgReceiver> receiver_;
  std::unique_ptr<spark_dsg::ZmqReceiver> zmq_receiver_;
  HydraVisualizerConfig config_;
  std::unique_ptr<std::ofstream> size_log_file_;
  ros::ServiceServer reload_service_;
  ros::ServiceServer redraw_service_;
};

}  // namespace hydra
